#pragma once
#include <string>
#include <atomic>
#include <thread>
#include <mutex>
#include <optional>
#include <cmath>
#include <limits>
#include <vector>
#include <utility>
#include "helpers.h"      // For GPSData and EncoderData
#include "stereo_depth.h" // Integrated stereo depth camera

class ObjectDetection;  // fwd

enum class ControlMode : int {
    FOLLOW  = 0,  // autonomous person-follow (default)
    MANUAL  = 1,  // web-driven manual control
    MISSION = 2,  // GPS waypoint mission running
    STOPPED = 3   // all motion halted
};

enum class MissionFault : int {
    NONE     = 0,
    GPS_LOST = 1,  // GPS signal absent for too long
    STUCK    = 2,  // physical movement not matching commanded motion
    WHEEL    = 3   // wheel command delivery failed (control socket down)
};

class Comms {
public:
    Comms();
    ~Comms();

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

    // IMU latest YPR
    bool getLatestYPR(float& yaw, float& pitch, float& roll, double& age_ms) const;

    // GPS latest
    bool getLatestGPS(double& lat, double& lon, float& alt, float& speed_knots, float& course_deg, int& quality, int& sats, double& age_ms) const;

    // Encoders latest
    bool getLatestEncoders(int& left, int& right, double& age_ms) const;

    float getLatestDistance(double* age_ms=nullptr) const; // meters (or <0 if invalid/none)

    void  setDetectionFPS(float fps)   { detection_fps_.store(fps); }
    int   getStreamQuality() const     { return stream_quality_.load(); }
    void  setTargetPersonId(int id)    { target_person_id_.store(id); }

    // Wire ObjectDetection so set_target: commands can be forwarded
    void  setObjectDetection(ObjectDetection* od) { od_ptr_ = od; }

    // Control mode
    ControlMode getMode() const { return static_cast<ControlMode>(mode_.load()); }
    void        setMode(ControlMode m) { mode_.store(static_cast<int>(m)); }

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

    // Obstacle / lidar forward distance (populated by SLAM bridge)
    bool  hasObstacle() const { return obstacle_.load(); }
    float getLatestLidarFwdDist(double* age_ms = nullptr) const;

    // SLAM-estimated pose
    struct SlamPose { float x=0, y=0, theta=0; bool valid=false; };
    SlamPose getSlamPose() const;

    // Stereo depth camera (integrated — replaces external stereo_depth_zmq subprocess)
    void startStereoDepth();
    void stopStereoDepth();

private:
    // Control TCP
    SocketFd    control_sock_;
    std::string control_ip_;
    int         control_port_ = 0;
    std::thread control_rxThread_;
    std::atomic<bool> control_rxRun_{false};
    std::mutex  control_send_mtx_;

    // Sensor TCP
    SocketFd    sensor_sock_;
    std::string sensor_ip_;
    int         sensor_port_ = 0;
    std::thread sensor_rxThread_;
    std::atomic<bool> sensor_rxRun_{false};
    std::mutex  sensor_send_mtx_;

    // IMU latest
    std::atomic<float> imu_yaw_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> imu_pitch_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> imu_roll_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<int64_t> imu_stamp_ns_{0}; // steady_clock::now().time_since_epoch().count()

    // GPS latest (mutex-protected: GPSData is too large for a lock-free atomic)
    mutable std::mutex gps_mtx_;
    GPSData            gps_{};
    std::atomic<int64_t> gps_stamp_ns_{0};

    // Encoders latest
    std::atomic<EncoderData> encoders_{};
    std::atomic<int64_t> enc_stamp_ns_{0};

    // TOF latest (from TCP now)
    std::atomic<float> tof_dist_{-1.0f};
    std::atomic<int64_t> tof_stamp_ns_{0};

    // Detection FPS (set by ObjectDetection)
    std::atomic<float> detection_fps_{0.0f};

    // Stream quality 1-100 (JPEG quality sent to web, tunable at runtime)
    std::atomic<int> stream_quality_{55};

    // Active follow target person ID (set by ObjectDetection each frame)
    std::atomic<int> target_person_id_{-1};

    // ObjectDetection pointer for set_target: command dispatch
    ObjectDetection* od_ptr_ = nullptr;

    // Control mode
    std::atomic<int> mode_{0};   // ControlMode::FOLLOW by default

    // Web IPC
    SocketFd          web_sock_;
    // web_client_ stays atomic<int> — it crosses thread boundaries atomically
    // and SocketFd is not trivially copyable, so atomic<SocketFd> isn't viable.
    std::atomic<int>  web_client_{-1};  // fd of the currently-connected VPS client (-1 = none)
    std::thread web_rxThread_;
    std::atomic<bool> web_rxRun_{false};
    std::mutex web_mtx_;

    // Lidar data (populated by SLAM bridge, not SDK)
    std::atomic<bool>     obstacle_{false};
    std::atomic<float>    lidar_fwd_dist_{-1.0f};
    std::atomic<int64_t>  lidar_stamp_ns_{0};
    mutable std::mutex    scan_mtx_;
    std::vector<std::pair<float,float>> latest_scan_;

    void broadcastScan();

    // SLAM bridge
    int               slam_sock_{-1};
    mutable std::mutex slam_sock_mtx_;
    std::thread       slam_rxThread_;
    std::thread       slam_txThread_;
    std::atomic<bool> slam_run_{false};
    mutable std::mutex slam_pose_mtx_;
    SlamPose          slam_pose_;

    void updateFromBridge(const std::string& line);

    // Stereo depth camera instance
    StereoDepth stereo_depth_;

    // Mission state
    mutable std::mutex mission_mtx_;
    std::string        mission_json_;
    std::atomic<int>   mission_wp_idx_{-1};
    std::atomic<bool>  mission_run_{false};
    std::thread        mission_thread_;
    std::atomic<int>   mission_fault_{0};   // MissionFault enum

    // helpers
    bool sendLine(int sock, std::mutex& mtx, const std::string& label, const std::string& line);
    static int  openTcpSocket(const std::string& ip, int port);
    static void controlRxLoop(Comms* self);
    static void sensorRxLoop(Comms* self);
    static void missionLoop(Comms* self);
    void saveMission(const std::string& id, const std::string& json_str);
    void saveMissionState(int wp_idx);
    int  loadMissionState(const std::string& id);  // returns saved idx or 0
    std::string listMissionsJson() const;
};