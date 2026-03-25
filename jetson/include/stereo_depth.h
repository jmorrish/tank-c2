#pragma once
#include <string>
#include <thread>
#include <atomic>

// Runs the CUDA stereo depth pipeline in a background thread.
// Opens the stereo camera independently (2560x720 side-by-side MJPEG),
// rectifies with precomputed calibration maps, computes disparity via
// CUDA StereoBM, and publishes colourised JPEG frames to a ZMQ PUB socket.
class StereoDepth {
public:
    StereoDepth()  = default;
    ~StereoDepth() { stop(); }

    // device_index — V4L2 index of the stereo camera (default 2)
    // calib_xml    — path to stereo_params_cuda.xml (contains map1x/y, map2x/y, P1, T)
    // zmq_port     — ZMQ PUB port (default 5558, mjpeg_bridge subscribes here)
    void start(int                device_index = 2,
               const std::string& calib_xml    = "/home/james/stereo_calib/stereo_params_cuda.xml",
               int                zmq_port     = 5558);
    void stop();

    bool running() const { return run_.load(); }

private:
    void loop(int device_index, std::string calib_xml, int zmq_port);

    std::thread       thread_;
    std::atomic<bool> run_{false};
};
