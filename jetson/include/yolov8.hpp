//
// Created by ubuntu on 3/16/23.
//
#ifndef JETSON_DETECT_YOLOV8_HPP
#define JETSON_DETECT_YOLOV8_HPP
#include "NvInferPlugin.h"
#include "common.hpp"
#include <fstream>
#include <memory>
#include <cassert>
#include <iostream>
#include <vector>
#include <string>
using namespace det;

// TensorRT Logger
class Logger : public nvinfer1::ILogger {
public:
    explicit Logger(Severity severity = Severity::kERROR) : reportableSeverity(severity) {}
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= reportableSeverity)
            std::cout << msg << std::endl;
    }
    Severity reportableSeverity;
};

class YOLOv8 {
public:
    explicit YOLOv8(const std::string& engine_file_path);
    ~YOLOv8();

    void                 make_pipe(bool warmup = true);
    void                 copy_from_Mat(const cv::Mat& image);
    void                 copy_from_Mat(const cv::Mat& image, cv::Size& size);
    void                 letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size);
    void                 infer();
    void                 postprocess(std::vector<Object>& objs);
    static void          draw_objects(const cv::Mat&                                image,
                                      cv::Mat&                                      res,
                                      const std::vector<Object>&                    objs,
                                      const std::vector<std::string>&               CLASS_NAMES,
                                      const std::vector<std::vector<unsigned int>>& COLORS);

    int                  num_bindings;
    int                  num_inputs  = 0;
    int                  num_outputs = 0;
    std::vector<Binding> input_bindings;
    std::vector<Binding> output_bindings;
    std::vector<void*>   host_ptrs;
    std::vector<void*>   device_ptrs;
    std::vector<std::string> input_names;
    std::vector<std::string> output_names;

    PreParam pparam;

private:
    std::unique_ptr<nvinfer1::IRuntime>          runtime = nullptr;
    std::unique_ptr<nvinfer1::ICudaEngine>       engine = nullptr;
    std::unique_ptr<nvinfer1::IExecutionContext> context = nullptr;
    cudaStream_t                 stream  = nullptr;
    Logger gLogger{nvinfer1::ILogger::Severity::kERROR};
};

// --------- Implementation Inline (as in your original for completeness) -----------

YOLOv8::YOLOv8(const std::string& engine_file_path)
{
    std::ifstream file(engine_file_path, std::ios::binary);
    assert(file.good());
    file.seekg(0, std::ios::end);
    auto size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> trtModelStream(size);
    file.read(trtModelStream.data(), size);
    file.close();

    initLibNvInferPlugins(&this->gLogger, "");
    this->runtime.reset(nvinfer1::createInferRuntime(this->gLogger));
    assert(this->runtime != nullptr);

    this->engine.reset(this->runtime->deserializeCudaEngine(trtModelStream.data(), size));
    assert(this->engine != nullptr);

    this->context.reset(this->engine->createExecutionContext());
    assert(this->context != nullptr);

    cudaStreamCreate(&this->stream);

    // PATCH: TensorRT 10 API: getNbIOTensors(), getIOTensorName(i)
    this->num_bindings = this->engine->getNbIOTensors();

    for (int i = 0; i < this->num_bindings; ++i) {
        std::string tensor_name = this->engine->getIOTensorName(i);
        nvinfer1::DataType dtype = this->engine->getTensorDataType(tensor_name.c_str());
        nvinfer1::Dims dims = this->engine->getTensorShape(tensor_name.c_str());
        Binding binding;
        binding.name = tensor_name;
        binding.dsize = type_to_size(dtype);
        binding.dims = dims;
        binding.size = get_size_by_dims(dims);

        nvinfer1::TensorIOMode io_mode = this->engine->getTensorIOMode(tensor_name.c_str());
        if (io_mode == nvinfer1::TensorIOMode::kINPUT) {
            this->num_inputs += 1;
            this->input_bindings.push_back(binding);
            this->input_names.push_back(tensor_name);
        } else {
            this->num_outputs += 1;
            this->output_bindings.push_back(binding);
            this->output_names.push_back(tensor_name);
        }
    }
}

YOLOv8::~YOLOv8()
{
    cudaStreamDestroy(this->stream);
    for (auto& ptr : this->device_ptrs) {
        cudaFree(ptr);
    }
    for (auto& ptr : this->host_ptrs) {
        cudaFreeHost(ptr);
    }
}

void YOLOv8::make_pipe(bool warmup)
{
    for (auto& bindings : this->input_bindings) {
        void* d_ptr;
        assert(cudaMalloc(&d_ptr, bindings.size * bindings.dsize) == cudaSuccess);
        this->device_ptrs.push_back(d_ptr);
    }

    for (auto& bindings : this->output_bindings) {
        void * d_ptr, *h_ptr;
        size_t size = bindings.size * bindings.dsize;
        assert(cudaMalloc(&d_ptr, size) == cudaSuccess);
        assert(cudaHostAlloc(&h_ptr, size, 0) == cudaSuccess);
        this->device_ptrs.push_back(d_ptr);
        this->host_ptrs.push_back(h_ptr);
    }

    if (warmup) {
        for (int i = 0; i < 10; i++) {
            for (auto& bindings : this->input_bindings) {
                size_t size  = bindings.size * bindings.dsize;
                void*  h_ptr = malloc(size);
                memset(h_ptr, 0, size);
                assert(cudaMemcpyAsync(this->device_ptrs[0], h_ptr, size, cudaMemcpyHostToDevice, this->stream) == cudaSuccess);
                free(h_ptr);
            }
            this->infer();
        }
        printf("model warmup 10 times\n");
    }
}

void YOLOv8::letterbox(const cv::Mat& image, cv::Mat& out, cv::Size& size)
{
    const float inp_h  = size.height;
    const float inp_w  = size.width;
    float       height = image.rows;
    float       width  = image.cols;

    float r    = std::min(inp_h / height, inp_w / width);
    int   padw = std::round(width * r);
    int   padh = std::round(height * r);

    cv::Mat tmp;
    if ((int)width != padw || (int)height != padh) {
        cv::resize(image, tmp, cv::Size(padw, padh));
    }
    else {
        tmp = image.clone();
    }

    float dw = inp_w - padw;
    float dh = inp_h - padh;

    dw /= 2.0f;
    dh /= 2.0f;
    int top    = int(std::round(dh - 0.1f));
    int bottom = int(std::round(dh + 0.1f));
    int left   = int(std::round(dw - 0.1f));
    int right  = int(std::round(dw + 0.1f));

    cv::copyMakeBorder(tmp, tmp, top, bottom, left, right, cv::BORDER_CONSTANT, {114, 114, 114});

    out.create({1, 3, (int)inp_h, (int)inp_w}, CV_32F);

    std::vector<cv::Mat> channels;
    cv::split(tmp, channels);

    cv::Mat c0((int)inp_h, (int)inp_w, CV_32F, (float*)out.data);
    cv::Mat c1((int)inp_h, (int)inp_w, CV_32F, (float*)out.data + (int)inp_h * (int)inp_w);
    cv::Mat c2((int)inp_h, (int)inp_w, CV_32F, (float*)out.data + (int)inp_h * (int)inp_w * 2);

    channels[0].convertTo(c2, CV_32F, 1 / 255.f);
    channels[1].convertTo(c1, CV_32F, 1 / 255.f);
    channels[2].convertTo(c0, CV_32F, 1 / 255.f);

    this->pparam.ratio  = 1 / r;
    this->pparam.dw     = dw;
    this->pparam.dh     = dh;
    this->pparam.height = height;
    this->pparam.width  = width;
}

void YOLOv8::copy_from_Mat(const cv::Mat& image)
{
    cv::Mat  nchw;
    auto&    in_binding = this->input_bindings[0];
    int      width      = in_binding.dims.d[3];
    int      height     = in_binding.dims.d[2];
    cv::Size size{width, height};
    this->letterbox(image, nchw, size);

    // PATCH: TRT 10 - set shape by name
    this->context->setInputShape(this->input_names[0].c_str(), nvinfer1::Dims{4, {1, 3, height, width}});

    assert(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream) == cudaSuccess);
}

void YOLOv8::copy_from_Mat(const cv::Mat& image, cv::Size& size)
{
    cv::Mat nchw;
    this->letterbox(image, nchw, size);

    this->context->setInputShape(this->input_names[0].c_str(), nvinfer1::Dims{4, {1, 3, size.height, size.width}});
    assert(cudaMemcpyAsync(
        this->device_ptrs[0], nchw.ptr<float>(), nchw.total() * nchw.elemSize(), cudaMemcpyHostToDevice, this->stream) == cudaSuccess);
}

void YOLOv8::infer()
{
    // PATCH: Set TensorRT IO addresses by name (TRT 10 API)
    int offset = 0;
    // Input bindings
    for (size_t i = 0; i < input_bindings.size(); ++i)
        this->context->setTensorAddress(this->input_names[i].c_str(), this->device_ptrs[offset++]);
    // Output bindings
    for (size_t i = 0; i < output_bindings.size(); ++i)
        this->context->setTensorAddress(this->output_names[i].c_str(), this->device_ptrs[offset++]);

    // EnqueueV3 takes only the stream in TRT10
    this->context->enqueueV3(this->stream);

    for (int i = 0; i < this->num_outputs; i++) {
        size_t osize = this->output_bindings[i].size * this->output_bindings[i].dsize;
        assert(cudaMemcpyAsync(
            this->host_ptrs[i], this->device_ptrs[i + this->num_inputs], osize, cudaMemcpyDeviceToHost, this->stream) == cudaSuccess);
    }
    cudaStreamSynchronize(this->stream);
}

void YOLOv8::postprocess(std::vector<Object>& objs)
{
    objs.clear();
    int*  num_dets = static_cast<int*>(this->host_ptrs[0]);
    auto* boxes    = static_cast<float*>(this->host_ptrs[1]);
    auto* scores   = static_cast<float*>(this->host_ptrs[2]);
    int*  labels   = static_cast<int*>(this->host_ptrs[3]);
    auto& dw       = this->pparam.dw;
    auto& dh       = this->pparam.dh;
    auto& width    = this->pparam.width;
    auto& height   = this->pparam.height;
    auto& ratio    = this->pparam.ratio;
    for (int i = 0; i < num_dets[0]; i++) {
        float* ptr = boxes + i * 4;

        float x0 = *ptr++ - dw;
        float y0 = *ptr++ - dh;
        float x1 = *ptr++ - dw;
        float y1 = *ptr - dh;

        x0 = clamp(x0 * ratio, 0.f, width);
        y0 = clamp(y0 * ratio, 0.f, height);
        x1 = clamp(x1 * ratio, 0.f, width);
        y1 = clamp(y1 * ratio, 0.f, height);
        Object obj;
        obj.rect.x      = x0;
        obj.rect.y      = y0;
        obj.rect.width  = x1 - x0;
        obj.rect.height = y1 - y0;
        obj.prob        = *(scores + i);
        obj.label       = *(labels + i);
        objs.push_back(obj);
    }
}

void YOLOv8::draw_objects(const cv::Mat& image,
                          cv::Mat& res,
                          const std::vector<Object>& objs,
                          const std::vector<std::string>& CLASS_NAMES,
                          const std::vector<std::vector<unsigned int>>& COLORS) {
    res = image.clone();
    for (const auto& obj : objs) {
        // Set the same color for all detections
        cv::Scalar color = cv::Scalar(COLORS[0][0], COLORS[0][1], COLORS[0][2]);
        cv::rectangle(res, obj.rect, color, 2);

        // Create text label
        char text[256];
        snprintf(text, sizeof(text), "%s %.1f%%", CLASS_NAMES[0].c_str(), obj.prob * 100);

        // Calculate label size
        int baseLine = 0;
        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

        // Adjust position to avoid drawing outside the image
        int x = static_cast<int>(obj.rect.x);
        int y = static_cast<int>(obj.rect.y) - label_size.height - baseLine;
        y = std::max(y, 0);

        // Draw label background
        cv::rectangle(res, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                      color, -1);

        // Put label text
        cv::putText(res, text, cv::Point(x, y + label_size.height), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                    cv::Scalar(255, 255, 255), 1);
    }
}

#endif  // JETSON_DETECT_YOLOV8_HPP
