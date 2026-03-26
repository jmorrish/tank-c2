#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>

struct RuntimeConfig;  // fwd

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

    // device — V4L2 device path (e.g. "/dev/video-stereo") or index as
    //          string (e.g. "2").  Path-based open is preferred (udev symlinks).
    // cfg    — runtime config for tunable parameters
    // share_left — if true, publish left half-frame for detection fallback
    void start(const std::string& device, const RuntimeConfig& cfg, bool share_left = false);
    void stop();

    bool running() const { return run_.load(); }

    // Returns true and copies the latest left half-frame when a new one is
    // available (only produces frames when started with share_left=true).
    // Thread-safe; clears the "new" flag on read.
    bool getLatestLeft(cv::Mat& out);

private:
    void loop(std::string device, const RuntimeConfig* cfg, bool share_left);

    std::thread       thread_;
    std::atomic<bool> run_{false};

    mutable std::mutex left_mtx_;
    cv::Mat            latest_left_;
    bool               left_new_ = false;
};
