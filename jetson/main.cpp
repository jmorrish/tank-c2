#include <iostream>
#include <limits>
#include <algorithm>
#include <vector>
#include <opencv2/opencv.hpp>

#include "config.h"
#include "helpers.h"
#include "comms.h"
#include "object_detection.h"
#include "movement.h"
#include "runtime_config.h"

// Probe each V4L2 camera device and return two indices:
//   stereo_cam — the device that actually delivers 2560×720 (unique to the stereo camera)
//   detect_cam — the first other device found
// Returns false if the stereo camera cannot be found.
static bool findCameras(int& detect_cam, int& stereo_cam, int maxTest = 8) {
    detect_cam = -1;
    stereo_cam = -1;
    for (int i = 0; i < maxTest; ++i) {
        cv::VideoCapture cap(i, cv::CAP_V4L2);
        if (!cap.isOpened()) continue;

        // Probe for stereo by negotiating 2560×720 MJPEG and reading back
        // the actual resolution the driver accepted.  If the driver rejects
        // 2560×720 it will clamp to its native size, so we can tell them apart
        // without permanently altering any device state — cap.release() clears
        // the V4L2 fd before either camera is opened for real.
        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,  720);
        int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        cap.release();

        if (w == 2560 && h == 720) {
            stereo_cam = i;
            LOGI("Camera auto-detect: device " << i << " → stereo (2560×720)");
        } else if (detect_cam < 0) {
            detect_cam = i;
            LOGI("Camera auto-detect: device " << i << " → detection (" << w << "×" << h << ")");
        }

        if (stereo_cam >= 0 && detect_cam >= 0) break;
    }
    return stereo_cam >= 0 && detect_cam >= 0;
}

// Helper: split "ip:port" into components; returns false if format is wrong.
static bool parseIpPort(const std::string& s, std::string& ip, int& port){
    auto colon = s.rfind(':');
    if (colon == std::string::npos) return false;
    ip   = s.substr(0, colon);
    try { port = std::stoi(s.substr(colon + 1)); } catch(...){ return false; }
    return true;
}

int main(int argc, char** argv){
    if (argc < 2){
        std::cerr << "Usage: " << argv[0]
                  << " <engine_path>"
                  << " [--headless]"
                  << " [--auto-continue]"
                  << " [--ptu-ip <ip:port>]  [--ip <ip>] [--port <n>]"
                  << " [--sensor-ip <ip:port>]"
                  << " [--detect-cam <n>] [--stereo-cam <n>]\n";
        return 1;
    }
    std::string engine_path = argv[1];
    bool headless      = DEFAULT_HEADLESS;
    bool auto_continue = false;
    bool no_hw         = false;
    std::string ip   = TEENSY_IP_DEFAULT;
    int         port = TEENSY_PORT_DEFAULT;
    std::string sensor_ip   = SENSOR_IP_DEFAULT;
    int         sensor_port = SENSOR_PORT_DEFAULT;
    int cam_detect = -1;   // -1 = auto-detect
    int cam_stereo = -1;   // -1 = auto-detect

    for (int i=2;i<argc;++i){
        std::string a = argv[i];
        if      (a == "--headless")      headless = true;
        else if (a == "--auto-continue") auto_continue = true;
        else if (a == "--no-hw")         no_hw = true;
        else if (a == "--ptu-ip"    && i+1 < argc) parseIpPort(argv[++i], ip, port);
        else if (a == "--sensor-ip" && i+1 < argc){
            std::string next = argv[++i];
            if (!parseIpPort(next, sensor_ip, sensor_port)) sensor_ip = next;
        }
        else if (a == "--ip"          && i+1 < argc) ip          = argv[++i];
        else if (a == "--port"        && i+1 < argc) port        = std::stoi(argv[++i]);
        else if (a == "--sensor-port" && i+1 < argc) sensor_port = std::stoi(argv[++i]);
        else if (a == "--detect-cam"  && i+1 < argc) cam_detect  = std::stoi(argv[++i]);
        else if (a == "--stereo-cam"  && i+1 < argc) cam_stereo  = std::stoi(argv[++i]);
    }

    // Auto-detect cameras if not overridden on command line.
    // The stereo camera is identified by its unique 2560×720 capability.
    // If stereo is missing: stereo depth is disabled (detection still runs).
    // If detect is missing: ObjectDetection scans at runtime.
    if (cam_detect < 0 || cam_stereo < 0) {
        int found_detect = -1, found_stereo = -1;
        findCameras(found_detect, found_stereo);  // sets what it finds; return ignored
        if (cam_detect < 0) cam_detect = found_detect;
        if (cam_stereo < 0) cam_stereo = found_stereo;
        if (cam_stereo < 0)
            LOGW("No stereo camera found — stereo depth disabled");
        if (cam_detect < 0)
            LOGW("No detection camera found at startup — will scan at runtime");
    }
    LOGI("Detection camera: " << (cam_detect >= 0 ? "device " + std::to_string(cam_detect) : "none (runtime scan)")
         << "   Stereo camera: " << (cam_stereo >= 0 ? "device " + std::to_string(cam_stereo) : "disabled"));

    // Comms — hardware connections are non-fatal; rx threads reconnect automatically
    Comms comms;
    if (!no_hw){
        if (!comms.connectControl(ip, port))
            LOGW("Control Teensy not reachable — will keep retrying in background");
        if (!comms.connectSensor(sensor_ip, sensor_port))
            LOGW("Sensor Teensy not reachable — will keep retrying in background");
    } else {
        LOGI("--no-hw: skipping hardware connections");
    }

    // Web IPC is always started — this is what the website talks to
    if (!comms.startWebIPC(9999)){
        LOGE("Failed to start Web IPC — cannot continue");
        return 1;
    }

    // Load runtime config (falls back to compiled defaults if missing)
    RuntimeConfig cfg = RuntimeConfig::load("config.json");

    // SLAM bridge — connects to slam_bridge.py:9997 in background (non-fatal)
    comms.startSlamBridge();

    // Stereo depth — opens its own camera independently (skipped if none found).
    // share_left=true when no separate detection camera: ObjectDetection will
    // use the left half-frame from StereoDepth instead of its own camera.
    if (cam_stereo >= 0) {
        bool share_left = (cam_detect < 0);
        comms.startStereoDepth(cam_stereo, share_left);
    } else {
        LOGI("Stereo depth not started (no stereo camera detected)");
    }

    // Shared bus: detection -> movement
    AtomicLatest<TargetMsg> bus;

    ObjectDetection det(engine_path, cam_detect, CAM2_RTSP, headless, bus, &comms, cfg);
    det.setStereoIndex(cam_stereo);          // skip stereo device during camera scan
    comms.setObjectDetection(&det);  // wire set_target: command dispatch
    if (!det.start()){
        LOGE("Failed to start ObjectDetection");
        comms.stopWebIPC();
        return 1;
    }
    Movement mov(comms, bus, cfg);
    if (!mov.start()){
        LOGE("Failed to start Movement");
        det.stop();
        comms.stopWebIPC();
        return 1;
    }

    LOGI("All subsystems running. Press Ctrl+C to exit.");

    while (true)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mov.stop();
    det.stop();
    comms.stopWebIPC();
    return 0;
}
