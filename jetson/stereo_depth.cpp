#include "stereo_depth.h"
#include "helpers.h"

#include <opencv2/opencv.hpp>
#include <opencv2/cudaimgproc.hpp>
#include <opencv2/cudastereo.hpp>
#include <opencv2/cudawarping.hpp>
#include <zmq.hpp>

#include <vector>
#include <iomanip>
#include <sstream>
#include <chrono>

using namespace cv;

// ── public ────────────────────────────────────────────────────────────────────

void StereoDepth::start(const std::string& calib_xml, int device_id, int zmq_port) {
    if (run_.load()) return;
    if (thread_.joinable()) thread_.join();  // reap any previous (failed) thread
    run_.store(true);
    thread_ = std::thread(&StereoDepth::loop, this, calib_xml, device_id, zmq_port);
}

void StereoDepth::stop() {
    run_.store(false);
    if (thread_.joinable()) thread_.join();
}

// ── private ───────────────────────────────────────────────────────────────────

void StereoDepth::loop(std::string calib_xml, int device_id, int zmq_port) {
  try {
    // Kill any leftover stereo_depth_zmq subprocess that may own the ZMQ port
    ::system("pkill -f stereo_depth_zmq 2>/dev/null; sleep 0.2");

    // ── ZMQ publisher ────────────────────────────────────────────────────────
    zmq::context_t ctx(1);
    zmq::socket_t  pub(ctx, ZMQ_PUB);
    pub.set(zmq::sockopt::sndhwm, 1);
    try {
        pub.bind("tcp://*:" + std::to_string(zmq_port));
        LOGI("StereoDepth: ZMQ PUB bound on port " << zmq_port);
    } catch (const zmq::error_t& e) {
        LOGE("StereoDepth: ZMQ bind failed: " << e.what());
        run_.store(false);
        return;
    }

    // ── Calibration ──────────────────────────────────────────────────────────
    // Load pre-computed rectification maps and camera parameters from XML.
    // The maps are confirmed float32 CV_32FC1 — load and use directly.
    FileStorage fs(calib_xml, FileStorage::READ);
    if (!fs.isOpened()) {
        LOGE("StereoDepth: cannot open calibration file: " << calib_xml);
        run_.store(false);
        return;
    }
    Mat map1x, map1y, map2x, map2y, P1, T;
    fs["map1x"] >> map1x;
    fs["map1y"] >> map1y;
    fs["map2x"] >> map2x;
    fs["map2y"] >> map2y;
    fs["P1"]    >> P1;
    fs["T"]     >> T;
    fs.release();

    if (map1x.empty() || map2x.empty() || P1.empty() || T.empty()) {
        LOGE("StereoDepth: missing maps or P1/T in calibration file");
        run_.store(false);
        return;
    }

    // Maps are float32 — ensure CV_32F for cuda::GpuMat upload
    for (Mat* m : {&map1x, &map1y, &map2x, &map2y})
        if (m->type() != CV_32F) m->convertTo(*m, CV_32F);

    cuda::GpuMat d_map1x(map1x), d_map1y(map1y);
    cuda::GpuMat d_map2x(map2x), d_map2y(map2y);

    float f  = static_cast<float>(P1.at<double>(0, 0));
    float B  = std::abs(static_cast<float>(T.at<double>(0, 0)));

    LOGI("StereoDepth: calibration loaded — f=" << f << " B=" << B);

    // ── Camera ───────────────────────────────────────────────────────────────
    // Use V4L2 backend directly — avoids GStreamer pipeline failures on Jetson.
    // MJPEG fourcc reduces USB bandwidth ~10x vs raw YUV.
    VideoCapture cap(device_id, CAP_V4L2);
    cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));
    cap.set(CAP_PROP_FRAME_WIDTH,  2560);
    cap.set(CAP_PROP_FRAME_HEIGHT,  720);
    cap.set(CAP_PROP_FPS, 10);   // cap at 10fps — reduces USB load and pipeline lag
    if (!cap.isOpened()) {
        LOGE("StereoDepth: cannot open camera device " << device_id);
        run_.store(false);
        return;
    }
    LOGI("StereoDepth: camera " << device_id << " opened at "
         << cap.get(CAP_PROP_FRAME_WIDTH) << "x" << cap.get(CAP_PROP_FRAME_HEIGHT)
         << " @ " << cap.get(CAP_PROP_FPS) << "fps");

    // ── CUDA StereoBM ────────────────────────────────────────────────────────
    const int numDisparities = 128;
    const int blockSize      = 9;
    const int blurKSize      = 3;
    auto stereo = cuda::createStereoBM(numDisparities, blockSize);

    // ── Histogram setup ──────────────────────────────────────────────────────
    const std::vector<float> hist_edges = {0.2f, 0.5f, 1.0f, 2.0f, 3.0f, 5.0f, 10.0f};
    std::vector<int> hist_bins(hist_edges.size(), 0);

    // ── Main loop ────────────────────────────────────────────────────────────
    int dbg_frame = 0;
    while (run_.load()) {
        Mat frame;
        if (!cap.read(frame)) {
            LOGW("StereoDepth: failed to grab frame — retrying");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        // Log frame dimensions on first frame
        if (dbg_frame == 0)
            LOGI("StereoDepth: first frame " << frame.cols << "x" << frame.rows
                 << "  map size " << map1x.cols << "x" << map1x.rows
                 << "  f=" << f << " B=" << B);

        // Split side-by-side frame into left / right halves.
        int w2    = frame.cols / 2;
        Mat left  = frame(Rect(0,  0, w2, frame.rows));
        Mat right = frame(Rect(w2, 0, w2, frame.rows));

        // Convert to greyscale and upload to GPU
        Mat grayL, grayR;
        cvtColor(left,  grayL, COLOR_BGR2GRAY);
        cvtColor(right, grayR, COLOR_BGR2GRAY);
        cuda::GpuMat d_grayL(grayL), d_grayR(grayR);

        // Rectify.
        // When swap_lr: cam2 (physical left) is in the LEFT frame half, cam1 in the RIGHT.
        // We swap maps so each half gets the maps for its actual camera, and d_rectL
        // ends up as cam2 (which must be the "left" input to StereoBM for positive disparity).
        const bool swap_lr = false;  // set true if physical left cam appears in the right frame half
        cuda::GpuMat d_rectL, d_rectR;
        cuda::remap(d_grayL, d_rectL,
                    swap_lr ? d_map2x : d_map1x,
                    swap_lr ? d_map2y : d_map1y, INTER_LINEAR);
        cuda::remap(d_grayR, d_rectR,
                    swap_lr ? d_map1x : d_map2x,
                    swap_lr ? d_map1y : d_map2y, INTER_LINEAR);

        // Compute disparity
        cuda::GpuMat d_disp;
        stereo->compute(d_rectL, d_rectR, d_disp);
        Mat disp;
        d_disp.download(disp);

        // Smooth and mask invalid pixels.
        // cuda::createStereoBM returns CV_8UC1 with pixel-disparity values directly —
        // NOT scaled ×16 like CPU StereoBM. Valid range is [1, numDisparities).
        if (blurKSize > 1) medianBlur(disp, disp, blurKSize);
        Mat mask = (disp > 0) & (disp < numDisparities);
        disp.setTo(0, ~mask);

        // Colourised disparity visualisation
        Mat disp_vis;
        disp.convertTo(disp_vis, CV_8U, 255.0 / numDisparities);
        applyColorMap(disp_vis, disp_vis, COLORMAP_JET);

        // Depth map — disp values are already in pixels, no ÷16 needed.
        Mat disp_f, depth;
        disp.convertTo(disp_f, CV_32F);
        depth = (f * B) / (disp_f + 1e-6f);
        depth.setTo(0, disp_f <= 0);

        // Centre-point depth annotation — geometric centre of the disparity image.
        int scx = disp.cols / 2, scy = disp.rows / 2;
        float dval    = depth.at<float>(scy, scx);
        float disp_px = disp_f.at<float>(scy, scx);
        if (dbg_frame % 30 == 0)
            LOGI("StereoDepth: centre disp=" << disp_px << "px  depth=" << dval << "m");
        ++dbg_frame;
        circle(disp_vis, Point(scx, scy), 7, Scalar(0, 255, 0), 2);
        std::ostringstream oss;
        if (dval > 0 && dval < 15.0f)
            oss << std::fixed << std::setprecision(2) << dval << " m";
        else
            oss << "No depth";
        putText(disp_vis, oss.str(), Point(10, 30),
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 255, 255), 2, LINE_AA);

        // Depth histogram bar chart
        std::fill(hist_bins.begin(), hist_bins.end(), 0);
        for (int y = 0; y < depth.rows; ++y) {
            const float* row = depth.ptr<float>(y);
            for (int x = 0; x < depth.cols; ++x) {
                float v = row[x];
                if (v > 0 && v < hist_edges.back()) {
                    for (size_t i = 0; i < hist_edges.size(); ++i) {
                        if (v < hist_edges[i]) { hist_bins[i]++; break; }
                    }
                }
            }
        }
        const int hist_h   = 80;
        const int bar_w    = disp_vis.cols / static_cast<int>(hist_bins.size());
        const int base_y   = disp_vis.rows - 10;
        int max_count = *std::max_element(hist_bins.begin(), hist_bins.end());
        for (size_t i = 0; i < hist_bins.size(); ++i) {
            int h = max_count > 0 ? hist_bins[i] * hist_h / max_count : 0;
            rectangle(disp_vis,
                      Point(static_cast<int>(i) * bar_w,        base_y - h),
                      Point(static_cast<int>(i+1) * bar_w - 5,  base_y),
                      Scalar(200, 200, 200), FILLED);
            std::ostringstream lbl;
            lbl << (i == 0 ? "<" : std::to_string(static_cast<int>(hist_edges[i-1])) + "-")
                << static_cast<int>(hist_edges[i]) << "m";
            putText(disp_vis, lbl.str(),
                    Point(static_cast<int>(i) * bar_w + 2, base_y + 20),
                    FONT_HERSHEY_PLAIN, 1.2, Scalar(240, 240, 240), 1, LINE_AA);
        }

        // Scale down 0.5x before encoding — reduces network traffic ~4x
        Mat disp_small;
        resize(disp_vis, disp_small, Size(), 0.5, 0.5, INTER_LINEAR);

        // Publish JPEG via ZMQ (quality 60 — adequate for depth visualisation)
        std::vector<uchar> buf;
        std::vector<int> enc_params = {IMWRITE_JPEG_QUALITY, 60};
        imencode(".jpg", disp_small, buf, enc_params);
        zmq::message_t msg(buf.size());
        std::memcpy(msg.data(), buf.data(), buf.size());
        try {
            pub.send(msg, zmq::send_flags::dontwait);
        } catch (const zmq::error_t&) {}
    }

    cap.release();
    pub.close();
    ctx.close();
    LOGI("StereoDepth: thread stopped");

  } catch (const std::exception& e) {
    LOGE("StereoDepth: fatal exception in thread: " << e.what());
    run_.store(false);
  } catch (...) {
    LOGE("StereoDepth: unknown fatal exception in thread");
    run_.store(false);
  }
}
