#pragma once
#include <atomic>
#include <mutex>
#include <shared_mutex>
#include <cmath>
#include <limits>
#include <chrono>
#include <vector>
#include <string>
#include <unordered_map>
#include <utility>
#include "helpers.h"  // GPSData, EncoderData

// One entry in the generic sensor slot table.
struct GenericReading { float value = 0.0f; int64_t stamp_ns = 0; };

// Thread-safe storage for all sensor readings.
// Written by TeensyLink/SlamLink rx threads; read by VpsLink, MissionRunner, Movement.
class SensorStore {
public:
    // ── IMU (body) ──────────────────────────────────────────────────────────
    void setIMU(float yaw, float pitch, float roll);
    bool getLatestYPR(float& yaw, float& pitch, float& roll, double& age_ms) const;

    // ── PTU IMU ─────────────────────────────────────────────────────────────
    void setPtuIMU(float yaw, float pitch, float roll);
    bool getLatestPtuYPR(float& yaw, float& pitch, float& roll, double& age_ms) const;

    // ── GPS ─────────────────────────────────────────────────────────────────
    void setGPS(const GPSData& gps);
    void setGPSSpeed(float speed_knots, float course_deg);
    bool getLatestGPS(double& lat, double& lon, float& alt, float& speed_knots,
                      float& course_deg, int& quality, int& sats, double& age_ms) const;

    // ── Encoders ────────────────────────────────────────────────────────────
    void setEncoders(int left, int right);
    bool getLatestEncoders(int& left, int& right, double& age_ms) const;

    // ── TOF distance ────────────────────────────────────────────────────────
    void setTOF(float dist_m);
    float getLatestDistance(double* age_ms = nullptr) const;

    // ── Lidar / obstacle ────────────────────────────────────────────────────
    void setLidar(bool obstacle, float fwd_dist,
                  std::vector<std::pair<float,float>>&& pts);
    bool  hasObstacle() const { return obstacle_.load(); }
    float getLatestLidarFwdDist(double* age_ms = nullptr) const;
    std::vector<std::pair<float,float>> getLatestScan() const;

    // ── SLAM pose ───────────────────────────────────────────────────────────
    struct SlamPose { float x=0, y=0, theta=0; bool valid=false; };
    void     setSlamPose(float x, float y, float theta);
    SlamPose getSlamPose() const;

    // ── Detection FPS (set by ObjectDetection, read by VpsLink) ─────────────
    void  setDetectionFPS(float fps) { detection_fps_.store(fps); }
    float getDetectionFPS() const    { return detection_fps_.load(); }

    // ── Stream quality ──────────────────────────────────────────────────────
    void setStreamQuality(int q)   { stream_quality_.store(q); }
    int  getStreamQuality() const  { return stream_quality_.load(); }

    // ── Active target person ID ─────────────────────────────────────────────
    void setTargetPersonId(int id) { target_person_id_.store(id); }
    int  getTargetPersonId() const { return target_person_id_.load(); }

    // ── Generic named sensor slots ───────────────────────────────────────────
    // Add any new sensor with setGeneric("name", value).
    // Auto-serialised into status JSON; no need to touch SensorStore interface.
    void setGeneric(const std::string& key, float value);
    bool getGeneric(const std::string& key, float& value, double* age_ms = nullptr) const;
    std::vector<std::pair<std::string, GenericReading>> getAllGeneric() const;

private:
    static int64_t now_ns() {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    // Body IMU
    std::atomic<float> imu_yaw_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> imu_pitch_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> imu_roll_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<int64_t> imu_stamp_ns_{0};

    // PTU IMU
    std::atomic<float> ptu_yaw_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> ptu_pitch_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<float> ptu_roll_{std::numeric_limits<float>::quiet_NaN()};
    std::atomic<int64_t> ptu_imu_stamp_ns_{0};

    // GPS
    mutable std::mutex gps_mtx_;
    GPSData            gps_{};
    std::atomic<int64_t> gps_stamp_ns_{0};

    // Encoders
    std::atomic<EncoderData> encoders_{};
    std::atomic<int64_t> enc_stamp_ns_{0};

    // TOF
    std::atomic<float> tof_dist_{-1.0f};
    std::atomic<int64_t> tof_stamp_ns_{0};

    // Lidar
    std::atomic<bool>     obstacle_{false};
    std::atomic<float>    lidar_fwd_dist_{-1.0f};
    std::atomic<int64_t>  lidar_stamp_ns_{0};
    mutable std::mutex    scan_mtx_;
    std::vector<std::pair<float,float>> latest_scan_;

    // SLAM pose
    mutable std::mutex slam_pose_mtx_;
    SlamPose           slam_pose_;

    // Detection state
    std::atomic<float> detection_fps_{0.0f};
    std::atomic<int>   stream_quality_{55};
    std::atomic<int>   target_person_id_{-1};

    // Generic named sensor slots
    mutable std::shared_mutex generic_mtx_;
    std::unordered_map<std::string, GenericReading> generic_;
};
