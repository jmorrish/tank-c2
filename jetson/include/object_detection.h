#pragma once
#include <atomic>
#include <thread>
#include <opencv2/opencv.hpp>
#include "helpers.h"
#include "runtime_config.h"
#include <zmq.hpp>

class Comms; // fwd

class ObjectDetection {
public:
    ObjectDetection(const std::string& engine_path,
                    int cam1_index,
                    const std::string& cam2_rtsp,
                    bool headless,
                    AtomicLatest<TargetMsg>& bus,
                    Comms* comms,
                    const RuntimeConfig& cfg);
    ~ObjectDetection();

    bool start();
    void stop();
    bool isRunning() const { return run_.load(); }

private:
    void rtspThreadFunc();
    void mainLoop();

    std::string engine_path_;
    int cam1_index_;
    std::string cam2_rtsp_;
    bool headless_;
    AtomicLatest<TargetMsg>& bus_;
    Comms* comms_;
    RuntimeConfig cfg_;

    std::thread rtsp_thread_;
    std::thread main_thread_;
    std::atomic<bool> run_{false};

    // RTSP shared frame
    std::mutex m2_;
    cv::Mat frame2_;
    std::atomic<bool> rtsp_run_{false};

    // UI
    std::string window_ = "YOLOv8 + BoTSORT + PTU";

    // autofocus click
    int first_frame_width_ = 0;
    std::atomic<bool> secondCamFocusRequested_{false};

    // Tracking persistence: remember the last chosen track ID so we
    // re-acquire the same person rather than snapping to a new entrant
    int tracked_id_ = -1;

    // NEW: ZMQ for publishing frames to Python
    zmq::context_t zmq_ctx_{1};
    zmq::socket_t zmq_pub_{zmq_ctx_, ZMQ_PUB};
    bool zmq_ready_ = false;
};