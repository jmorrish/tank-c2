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
    std::string ip   = TEENSY_IP_DEFAULT;
    int         port = TEENSY_PORT_DEFAULT;
    std::string sensor_ip   = SENSOR_IP_DEFAULT;
    int         sensor_port = SENSOR_PORT_DEFAULT;

    for (int i=2;i<argc;++i){
        std::string a = argv[i];
        if      (a == "--headless")      headless = true;
        else if (a == "--auto-continue") auto_continue = true;
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

    // pick cam1
    auto cams = enumerateCams();
    if (cams.empty()){
        LOGE("No local cameras found.");
        return 1;
    }

    int cam1 = -1;
    if (auto_continue || headless){
        // Non-interactive: use first available camera automatically
        cam1 = cams[0];
        LOGI("Auto-selected camera " << cam1);
    } else {
        std::cout << "Available cameras: ";
        for (int c : cams) std::cout << c << " ";
        std::cout << "\nEnter device index for Camera 1: ";
        std::cin >> cam1;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
        if (std::find(cams.begin(), cams.end(), cam1) == cams.end()){
            LOGE("Invalid camera index");
            return 1;
        }
    }

    // Comms
    Comms comms;
    if (!comms.connectControl(ip, port)){
        LOGE("Failed to connect to control Teensy");
        return 1;
    }
    if (!comms.connectSensor(sensor_ip, sensor_port)){
        LOGE("Failed to connect to sensor Teensy");
        return 1;
    }

    // NEW: Start Web IPC
    if (!comms.startWebIPC(9999)){
        LOGE("Failed to start Web IPC");
        // Continue or handle as needed; not fatal
    }

    // Shared bus: detection -> movement
    AtomicLatest<TargetMsg> bus;

    // Load runtime config (falls back to compiled defaults if missing)
    RuntimeConfig cfg = RuntimeConfig::load("config.json");

    // Start modules
    ObjectDetection det(engine_path, cam1, CAM2_RTSP, headless, bus, &comms, cfg);
    if (!det.start()){
        LOGE("Failed to start ObjectDetection");
        return 1;
    }
    Movement mov(comms, bus, cfg);
    if (!mov.start()){
        LOGE("Failed to start Movement");
        det.stop();
        return 1;
    }

    // Wait until detection window closes (or 'q' pressed there), then cleanup.
    while (det.isRunning()){
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    mov.stop();
    det.stop();
    // NEW: Stop Web IPC
    comms.stopWebIPC();
    return 0;
}