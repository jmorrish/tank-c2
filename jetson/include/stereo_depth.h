#pragma once
#include <string>
#include <thread>
#include <atomic>

// Runs the CUDA stereo depth pipeline in a background thread.
// Reads from a side-by-side USB stereo camera, rectifies with precomputed
// calibration maps, computes disparity via CUDA StereoBM, and publishes
// colourised JPEG frames to a ZMQ PUB socket (same protocol as the old
// stereo_depth_zmq subprocess).
class StereoDepth {
public:
    StereoDepth()  = default;
    ~StereoDepth() { stop(); }

    // calib_xml  — path to stereo_params_cuda.xml (contains map1x/y, map2x/y, P1, T)
    // device_id  — V4L2 device index for the stereo camera (default 2)
    // zmq_port   — ZMQ PUB port (default 5558, mjpeg_bridge subscribes here)
    void start(const std::string& calib_xml  = "/home/james/stereo_calib/stereo_params_cuda.xml",
               int                device_id  = 2,
               int                zmq_port   = 5558);
    void stop();

    bool running() const { return run_.load(); }

private:
    void loop(std::string calib_xml, int device_id, int zmq_port);

    std::thread       thread_;
    std::atomic<bool> run_{false};
};
