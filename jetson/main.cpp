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

// enumerate local cameras
static std::vector<int> enumerateCams(int maxTest=10){
    std::vector<int> out;
    for (int i=0;i<maxTest;++i){
        cv::VideoCapture cap(i);
        if (cap.isOpened()){
            out.push_back(i);
            cap.release();
        }
    }
    return out;
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
                  << " [--sensor-ip <ip:port>]\n";
        return 1;
    }
    std::string engine_path = argv[1];
    bool headless      = DEFAULT_HEADLESS;
    bool auto_continue = false;
    bool no_hw         = false;   // skip all hardware — web IPC only (for testing)
    std::string ip   = TEENSY_IP_DEFAULT;
    int         port = TEENSY_PORT_DEFAULT;
    std::string sensor_ip   = SENSOR_IP_DEFAULT;
    int         sensor_port = SENSOR_PORT_DEFAULT;

    for (int i=2;i<argc;++i){
        std::string a = argv[i];
        if      (a == "--headless")      headless = true;
        else if (a == "--auto-continue") auto_continue = true;
        else if (a == "--no-hw")         no_hw = true;
        // Combined ip:port forms (e.g. --ptu-ip 192.168.0.177:23)
        else if (a == "--ptu-ip"    && i+1 < argc) parseIpPort(argv[++i], ip, port);
        else if (a == "--sensor-ip" && i+1 < argc){
            std::string next = argv[++i];
            if (!parseIpPort(next, sensor_ip, sensor_port)) sensor_ip = next;
        }
        // Legacy separate flags
        else if (a == "--ip"         && i+1 < argc) ip          = argv[++i];
        else if (a == "--port"       && i+1 < argc) port        = std::stoi(argv[++i]);
        else if (a == "--sensor-port"&& i+1 < argc) sensor_port = std::stoi(argv[++i]);
    }

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

    // Camera index: default 0, override with --cam <n>
    // ObjectDetection's internal loop waits for the camera to appear, so it's
    // safe to start even when nothing is plugged in yet.
    int cam1 = 0;
    if (!auto_continue && !headless && !no_hw){
        auto cams = enumerateCams();
        if (!cams.empty()){
            std::cout << "Available cameras: ";
            for (int c : cams) std::cout << c << " ";
            std::cout << "\nEnter device index for Camera 1: ";
            std::cin >> cam1;
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        }
    }

    // Load runtime config (falls back to compiled defaults if missing)
    RuntimeConfig cfg = RuntimeConfig::load("config.json");

    // Shared bus: detection -> movement
    AtomicLatest<TargetMsg> bus;

    ObjectDetection det(engine_path, cam1, CAM2_RTSP, headless, bus, &comms, cfg);
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

    // Run forever — subsystems manage their own reconnect loops.
    // ObjectDetection, Movement, and Comms rx threads all handle
    // hardware disconnect/reconnect internally.
    while (true)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));

    mov.stop();
    det.stop();
    comms.stopWebIPC();
    return 0;
}