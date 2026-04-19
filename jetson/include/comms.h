#pragma once
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>
#include <opencv2/opencv.hpp>
#include "helpers.h"         // For GPSData and EncoderData
#include "stereo_depth.h"    // Integrated stereo depth camera
#include "runtime_config.h"  // Runtime-tunable parameters
#include "sensor_store.h"    // Thread-safe sensor storage
#include "teensy_link.h"     // TCP client with auto-reconnect
#include "slam_link.h"       // SLAM bridge TCP client
#include "control_mode.h"    // ControlMode, MissionFault enums
#include "mission.h"         // GPS waypoint mission executor

class ObjectDetection;       // fwd
class BehaviorCoordinator;   // fwd
class Nav2Planner;           // fwd

class Comms {
public:
    explicit Comms(const RuntimeConfig& cfg);
    ~Comms();

    const RuntimeConfig& config() const { return cfg_; }

    // Control Teensy TCP (PTU + Wheels)
    bool connectControl(const std::string& ip, int port);
    void closeControl();

    // Sensor Teensy TCP (IMU + GPS + Encoders)
    bool connectSensor(const std::string& ip, int port);
    void closeSensor();

    bool sendPTUVelocity(float pan_sps, float tilt_sps);  // VPxxTyy
    bool sendWheels(int left_sps, int right_sps);         // LSxRSy
    bool imuOn();
    bool imuOff();
    bool imuRate(float hz);

    // IMU latest YPR (delegates to SensorStore)
    bool getLatestYPR(float& yaw, float& pitch, float& roll, double& age_ms) const {
        return sensors_.getLatestYPR(yaw, pitch, roll, age_ms);
    }
    bool getLatestPtuYPR(float& yaw, float& pitch, float& roll, double& age_ms) const {
        return sensors_.getLatestPtuYPR(yaw, pitch, roll, age_ms);
    }

    // GPS latest
    bool getLatestGPS(double& lat, double& lon, float& alt, float& speed_knots, float& course_deg, int& quality, int& sats, double& age_ms) const {
        return sensors_.getLatestGPS(lat, lon, alt, speed_knots, course_deg, quality, sats, age_ms);
    }

    // Encoders latest
    bool getLatestEncoders(int& left, int& right, double& age_ms) const {
        return sensors_.getLatestEncoders(left, right, age_ms);
    }

    float getLatestDistance(double* age_ms=nullptr) const {
        return sensors_.getLatestDistance(age_ms);
    }

    void  setDetectionFPS(float fps)   { sensors_.setDetectionFPS(fps); }
    int   getStreamQuality() const     { return sensors_.getStreamQuality(); }
    void  setTargetPersonId(int id)    { sensors_.setTargetPersonId(id); }

    // Direct access to SensorStore (for extracted subsystems)
    SensorStore&       sensors()       { return sensors_; }
    const SensorStore& sensors() const { return sensors_; }

    // Wire ObjectDetection so set_target: commands can be forwarded
    void  setObjectDetection(ObjectDetection* od) { od_ptr_ = od; }

    // Wire BehaviorCoordinator so mode transitions go through it
    void setCoordinator(BehaviorCoordinator* bc) { coordinator_ = bc; }

    // Access MissionRunner (for MissionBehavior wiring in main.cpp)
    MissionRunner& mission() { return mission_; }

    // Wire MissionBehavior so mission_start/resume commands go through it
    void setMissionBehavior(class MissionBehavior* mb) { mission_behavior_ = mb; }

    // Control mode — delegates to coordinator if wired, else falls back to atomic
    ControlMode getMode() const;
    void        setMode(ControlMode m);

    // NEW: Web IPC (localhost TCP server for Python web interface)
    bool startWebIPC(int port = 9999);  // Call this in main.cpp after connecting to Teensy
    void stopWebIPC();

    // Latest status as JSON string
    std::string getStatusJson() const;

    // Commands from web — returns custom response string, or "" to use getStatusJson()
    std::string handleWebCommand(const std::string& cmd);

    // Thread-safe push of a one-shot event line to the web client
    void sendWebEvent(const std::string& json_str);

    // SLAM bridge client (connects to slam_bridge.py:9997)
    bool startSlamBridge(const std::string& host = "127.0.0.1", int port = 9997);
    void stopSlamBridge();

    // Send a JSON line to slam_bridge (thread-safe, for Nav2 planner requests)
    bool sendSlamBridge(const std::string& json_str);

    // Wire Nav2Planner so planned_path responses are routed to it
    void setNav2Planner(Nav2Planner* p) { nav2_planner_ = p; }

    // Obstacle / lidar forward distance (populated by SLAM bridge)
    bool  hasObstacle() const { return sensors_.hasObstacle(); }
    float getLatestLidarFwdDist(double* age_ms = nullptr) const {
        return sensors_.getLatestLidarFwdDist(age_ms);
    }

    // SLAM-estimated pose (type alias delegates to SensorStore)
    using SlamPose = SensorStore::SlamPose;
    SlamPose getSlamPose() const { return sensors_.getSlamPose(); }

    // Stereo depth camera (integrated — runs its own VideoCapture independently)
    // device: V4L2 path ("/dev/video-stereo") or index as string ("2")
    // share_left=true: also publish left half-frames for detection fallback
    void startStereoDepth(const std::string& device, bool share_left = false);
    void stopStereoDepth();

    // Get the latest left half-frame from StereoDepth (only available when
    // startStereoDepth was called with share_left=true). Returns false if no
    // new frame since last call.
    bool getStereoLeftFrame(cv::Mat& out);

private:
    RuntimeConfig cfg_;
    SensorStore   sensors_;

    // Teensy TCP links
    TeensyLink control_link_;
    TeensyLink sensor_link_;

    // ObjectDetection pointer for set_target: command dispatch
    ObjectDetection* od_ptr_ = nullptr;

    // BehaviorCoordinator (optional — if set, mode transitions go through it)
    BehaviorCoordinator* coordinator_ = nullptr;

    // MissionBehavior pointer for web command dispatch
    MissionBehavior* mission_behavior_ = nullptr;

    // Nav2Planner pointer for routing planned_path responses
    Nav2Planner* nav2_planner_ = nullptr;

    // Control mode (fallback when no coordinator is wired)
    std::atomic<int> mode_{0};   // ControlMode::FOLLOW by default

    // Web IPC
    SocketFd          web_sock_;
    // web_client_ stays atomic<int> — it crosses thread boundaries atomically
    // and SocketFd is not trivially copyable, so atomic<SocketFd> isn't viable.
    std::atomic<int>  web_client_{-1};  // fd of the currently-connected VPS client (-1 = none)
    std::thread web_rxThread_;
    std::atomic<bool> web_rxRun_{false};
    std::mutex web_mtx_;

    void broadcastScan();
    void updateFromBridge(const std::string& line);

    // SLAM bridge
    SlamLink slam_link_;

    // Stereo depth camera instance
    StereoDepth stereo_depth_;
    std::string stereo_device_;  // remembered for restart via web command

    // Mission runner
    MissionRunner mission_;

    // NMEA parsing helpers (called from sensor rx callback)
    void parseGGA(const std::string& line);
    void parseRMC(const std::string& line);

    // PTU centre-to-level closed loop (tracked so repeated presses cancel old)
    std::atomic<uint32_t> level_session_{0};
    void runLevelTilt();
};