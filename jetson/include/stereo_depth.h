#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>

// Runs the CUDA stereo depth pipeline in a background thread.
// Opens the stereo camera independently (2560x720 side-by-side MJPEG),
// rectifies with precomputed calibration maps, computes disparity via
// CUDA StereoBM, and publishes colourised JPEG frames to a ZMQ PUB socket.
//
// share_left=true: also publishes the left half-frame at STEREO_LEFT_FPS
// for ObjectDetection to use when no separate detection camera is present.
class StereoDepth {
public:
    StereoDepth()  = default;
    ~StereoDepth() { stop(); }

    // device_index — V4L2 index of the stereo camera
    // calib_xml    — path to stereo_params_cuda.xml
    // zmq_port     — ZMQ PUB port (default 5558, mjpeg_bridge subscribes here)
    // share_left   — if true, publish left half-frame for detection fallback
    void start(int                device_index = 2,
               const std::string& calib_xml    = "/home/james/stereo_calib/stereo_params_cuda.xml",
               int                zmq_port     = 5558,
               bool               share_left   = false);
    void stop();

    bool running() const { return run_.load(); }

    // Returns true and copies the latest left half-frame when a new one is
    // available (only produces frames when started with share_left=true).
    // Thread-safe; clears the "new" flag on read.
    bool getLatestLeft(cv::Mat& out);

private:
    void loop(int device_index, std::string calib_xml, int zmq_port, bool share_left);

    std::thread       thread_;
    std::atomic<bool> run_{false};

    mutable std::mutex left_mtx_;
    cv::Mat            latest_left_;
    bool               left_new_ = false;
};
