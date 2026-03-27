#include <iostream>
#include <limits>
#include <algorithm>
#include <vector>
#include <atomic>
#include <csignal>
#include <opencv2/opencv.hpp>

static std::atomic<bool> g_running{true};
static void sig_handler(int) { g_running.store(false); }

#include "helpers.h"
#include "comms.h"
#include "object_detection.h"
#include "behavior_coordinator.h"
#include "follow_behavior.h"
#include "mission_behavior.h"
#include "nav2_planner.h"
#include "runtime_config.h"
#include "config.h"  // DEFAULT_HEADLESS only

// Check for stable udev symlinks first, then fall back to index probing.
// Populates detect_dev and stereo_dev as device path strings:
//   "/dev/video-stereo" (udev symlink) or "2" (integer index) or "" (not found).
static bool findCameras(std::string& detect_dev, std::string& stereo_dev,
                        int maxTest = 8) {
    detect_dev.clear();
    stereo_dev.clear();

    // Try udev symlinks first (stable across reboots)
    {
        cv::VideoCapture cap("/dev/video-stereo", cv::CAP_V4L2);
        if (cap.isOpened()) {
            stereo_dev = "/dev/video-stereo";
            LOGI("Camera udev: /dev/video-stereo found");
            cap.release();
        }
    }
    {
        cv::VideoCapture cap("/dev/video-detect", cv::CAP_V4L2);
        if (cap.isOpened()) {
            detect_dev = "/dev/video-detect";
            LOGI("Camera udev: /dev/video-detect found");
            cap.release();
        }
    }

    if (!stereo_dev.empty() && !detect_dev.empty()) return true;

    // Fall back to probing each V4L2 device by resolution
    for (int i = 0; i < maxTest; ++i) {
        cv::VideoCapture cap(i, cv::CAP_V4L2);
        if (!cap.isOpened()) continue;

        cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  2560);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT,  720);
        int w = (int)cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int h = (int)cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        cap.release();

        if (w == 2560 && h == 720 && stereo_dev.empty()) {
            stereo_dev = std::to_string(i);
            LOGI("Camera auto-detect: device " << i << " -> stereo (2560x720)");
        } else if (detect_dev.empty()) {
            detect_dev = std::to_string(i);
            LOGI("Camera auto-detect: device " << i << " -> detection (" << w << "x" << h << ")");
        }

        if (!stereo_dev.empty() && !detect_dev.empty()) break;
    }
    return !stereo_dev.empty() && !detect_dev.empty();
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
                  << " [--detect-cam <dev>] [--stereo-cam <dev>]\n";
        return 1;
    }
    std::string engine_path = argv[1];

    // Load runtime config first so CLI defaults come from config.json
    RuntimeConfig cfg = RuntimeConfig::load("config.json");

    bool headless      = DEFAULT_HEADLESS;
    bool auto_continue = false;
    bool no_hw         = false;
    std::string ip   = cfg.teensy_ip;
    int         port = cfg.teensy_port;
    std::string sensor_ip   = cfg.sensor_ip;
    int         sensor_port = cfg.sensor_port;
    std::string cam_detect;   // empty = auto-detect
    std::string cam_stereo;   // empty = auto-detect

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
        else if (a == "--detect-cam"  && i+1 < argc) cam_detect  = argv[++i];
        else if (a == "--stereo-cam"  && i+1 < argc) cam_stereo  = argv[++i];
    }

    // Auto-detect cameras if not overridden on command line.
    // The stereo camera is identified by its unique 2560x720 capability.
    // If stereo is missing: stereo depth is disabled (detection still runs).
    // If detect is missing: ObjectDetection scans at runtime.
    if (cam_detect.empty() || cam_stereo.empty()) {
        std::string found_detect, found_stereo;
        findCameras(found_detect, found_stereo);
        if (cam_detect.empty()) cam_detect = found_detect;
        if (cam_stereo.empty()) cam_stereo = found_stereo;
        if (cam_stereo.empty())
            LOGW("No stereo camera found -- stereo depth disabled");
        if (cam_detect.empty())
            LOGW("No detection camera found at startup -- will scan at runtime");
    }
    LOGI("Detection camera: " << (cam_detect.empty() ? "none (runtime scan)" : cam_detect)
         << "   Stereo camera: " << (cam_stereo.empty() ? "disabled" : cam_stereo));

    // Comms — hardware connections are non-fatal; rx threads reconnect automatically
    Comms comms(cfg);
    if (!no_hw){
        if (!comms.connectControl(ip, port))
            LOGW("Control Teensy not reachable -- will keep retrying in background");
        if (!comms.connectSensor(sensor_ip, sensor_port))
            LOGW("Sensor Teensy not reachable -- will keep retrying in background");
    } else {
        LOGI("--no-hw: skipping hardware connections");
    }

    // Web IPC is always started — this is what the website talks to
    if (!comms.startWebIPC(cfg.web_ipc_port)){
        LOGE("Failed to start Web IPC -- cannot continue");
        return 1;
    }

    // SLAM bridge — connects to slam_bridge.py:9997 in background (non-fatal)
    comms.startSlamBridge();

    // Stereo depth — opens its own camera independently (skipped if none found).
    // share_left=true when no separate detection camera: ObjectDetection will
    // use the left half-frame from StereoDepth instead of its own camera.
    if (!cam_stereo.empty()) {
        bool share_left = cam_detect.empty();
        comms.startStereoDepth(cam_stereo, share_left);
    } else {
        LOGI("Stereo depth not started (no stereo camera detected)");
    }

    // Shared bus: detection -> movement
    AtomicLatest<TargetMsg> bus;

    ObjectDetection det(engine_path, cam_detect, cfg.cam2_rtsp, headless, bus, &comms, cfg);
    det.setStereoDevice(cam_stereo);      // skip stereo device during camera scan
    comms.setObjectDetection(&det);  // wire set_target: command dispatch
    if (!det.start()){
        LOGE("Failed to start ObjectDetection");
        comms.stopWebIPC();
        return 1;
    }

    // Nav2 path planner — requests obstacle-aware paths from slam_bridge/Nav2
    Nav2Planner nav2_planner(comms);
    comms.setNav2Planner(&nav2_planner);
    comms.mission().setPlanner(&nav2_planner);

    // Behavior coordinator replaces Movement — dispatches to registered behaviors
    FollowBehavior  follow(comms, bus, cfg);
    StoppedBehavior stopped;
    ManualBehavior  manual;
    MissionBehavior mission(comms.mission(), comms, cfg);

    BehaviorCoordinator coordinator(comms);
    coordinator.addBehavior(ControlMode::FOLLOW,  &follow);
    coordinator.addBehavior(ControlMode::STOPPED, &stopped);
    coordinator.addBehavior(ControlMode::MANUAL,  &manual);
    coordinator.addBehavior(ControlMode::MISSION, &mission);
    comms.setCoordinator(&coordinator);
    comms.setMissionBehavior(&mission);

    if (!coordinator.start()){
        LOGE("Failed to start BehaviorCoordinator");
        det.stop();
        comms.stopWebIPC();
        return 1;
    }

    LOGI("All subsystems running. Press Ctrl+C to exit.");

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    while (g_running.load())
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    LOGI("Shutting down...");
    coordinator.stop();
    det.stop();
    comms.stopWebIPC();
    return 0;
}
