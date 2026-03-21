#ifndef DET_COMMON_HPP
#define DET_COMMON_HPP

#include <opencv2/opencv.hpp>
#include <NvInfer.h>
#include <cuda_runtime_api.h>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>

namespace det {

struct Binding {
    std::string name;
    int dsize = 0;
    size_t size = 0;
    nvinfer1::Dims dims;
};

struct Object {
    cv::Rect2f rect;
    float prob = 0.f;
    int label = 0;
};

struct PreParam {
    float ratio = 1.f;
    float dw = 0.f;
    float dh = 0.f;
    float height = 0.f;
    float width = 0.f;
};

inline int get_size_by_dims(const nvinfer1::Dims& dims)
{
    int size = 1;
    for (int i = 0; i < dims.nbDims; ++i)
        size *= dims.d[i];
    return size;
}

inline int type_to_size(const nvinfer1::DataType& dataType)
{
    switch (dataType)
    {
        case nvinfer1::DataType::kFLOAT: return 4;
        case nvinfer1::DataType::kHALF:  return 2;
        case nvinfer1::DataType::kINT8:  return 1;
        case nvinfer1::DataType::kINT32: return 4;
#if NV_TENSORRT_MAJOR >= 8
        case nvinfer1::DataType::kBOOL:  return 1;
#endif
        default: return 4;
    }
}

inline float clamp(float val, float min_val, float max_val)
{
    return std::max(min_val, std::min(val, max_val));
}

} // namespace det

#endif // DET_COMMON_HPP
