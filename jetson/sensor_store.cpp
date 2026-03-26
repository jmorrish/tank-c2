#include "sensor_store.h"
#include <cmath>

using std::isfinite;

// ── IMU ─────────────────────────────────────────────────────────────────────

void SensorStore::setIMU(float yaw, float pitch, float roll) {
    imu_yaw_.store(yaw);
    imu_pitch_.store(pitch);
    imu_roll_.store(roll);
    imu_stamp_ns_.store(now_ns());
}

bool SensorStore::getLatestYPR(float& yaw, float& pitch, float& roll, double& age_ms) const {
    yaw   = imu_yaw_.load();
    pitch = imu_pitch_.load();
    roll  = imu_roll_.load();
    int64_t stamp = imu_stamp_ns_.load();
    if (stamp == 0 || !isfinite(yaw) || !isfinite(pitch) || !isfinite(roll)) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

// ── PTU IMU ─────────────────────────────────────────────────────────────────

void SensorStore::setPtuIMU(float yaw, float pitch, float roll) {
    ptu_yaw_.store(yaw);
    ptu_pitch_.store(pitch);
    ptu_roll_.store(roll);
    ptu_imu_stamp_ns_.store(now_ns());
}

bool SensorStore::getLatestPtuYPR(float& yaw, float& pitch, float& roll, double& age_ms) const {
    yaw   = ptu_yaw_.load();
    pitch = ptu_pitch_.load();
    roll  = ptu_roll_.load();
    int64_t stamp = ptu_imu_stamp_ns_.load();
    if (stamp == 0 || !isfinite(yaw) || !isfinite(pitch) || !isfinite(roll)) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

// ── GPS ─────────────────────────────────────────────────────────────────────

void SensorStore::setGPS(const GPSData& gps) {
    { std::lock_guard<std::mutex> lk(gps_mtx_); gps_ = gps; }
    gps_stamp_ns_.store(now_ns());
}

void SensorStore::setGPSSpeed(float speed_knots, float course_deg) {
    { std::lock_guard<std::mutex> lk(gps_mtx_);
      gps_.speed_knots = speed_knots;
      gps_.course_deg  = course_deg; }
    gps_stamp_ns_.store(now_ns());
}

bool SensorStore::getLatestGPS(double& lat, double& lon, float& alt, float& speed_knots,
                               float& course_deg, int& quality, int& sats, double& age_ms) const {
    GPSData gps;
    { std::lock_guard<std::mutex> lk(gps_mtx_); gps = gps_; }
    if (!gps.valid) return false;
    lat = gps.lat;
    lon = gps.lon;
    alt = gps.alt;
    speed_knots = gps.speed_knots;
    course_deg  = gps.course_deg;
    quality     = gps.quality;
    sats        = gps.sats;
    int64_t stamp = gps_stamp_ns_.load();
    if (stamp == 0) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

// ── Encoders ────────────────────────────────────────────────────────────────

void SensorStore::setEncoders(int left, int right) {
    EncoderData enc; enc.left = left; enc.right = right; enc.valid = true;
    encoders_.store(enc);
    enc_stamp_ns_.store(now_ns());
}

bool SensorStore::getLatestEncoders(int& left, int& right, double& age_ms) const {
    EncoderData enc = encoders_.load();
    if (!enc.valid) return false;
    left  = enc.left;
    right = enc.right;
    int64_t stamp = enc_stamp_ns_.load();
    if (stamp == 0) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

// ── TOF ─────────────────────────────────────────────────────────────────────

void SensorStore::setTOF(float dist_m) {
    tof_dist_.store(dist_m);
    tof_stamp_ns_.store(now_ns());
}

float SensorStore::getLatestDistance(double* age_ms) const {
    float d = tof_dist_.load();
    if (d < 0.0f) return -1.0f;
    int64_t stamp = tof_stamp_ns_.load();
    if (stamp == 0) return -1.0f;
    if (age_ms) *age_ms = (now_ns() - stamp) / 1e6;
    return d;
}

// ── Lidar ───────────────────────────────────────────────────────────────────

void SensorStore::setLidar(bool obstacle, float fwd_dist,
                           std::vector<std::pair<float,float>>&& pts) {
    obstacle_.store(obstacle);
    lidar_fwd_dist_.store(fwd_dist);
    lidar_stamp_ns_.store(now_ns());
    { std::lock_guard<std::mutex> lk(scan_mtx_); latest_scan_ = std::move(pts); }
}

float SensorStore::getLatestLidarFwdDist(double* age_ms) const {
    float d = lidar_fwd_dist_.load();
    if (d < 0.0f) return -1.0f;
    int64_t stamp = lidar_stamp_ns_.load();
    if (stamp == 0) return -1.0f;
    if (age_ms) *age_ms = (now_ns() - stamp) / 1e6;
    return d;
}

std::vector<std::pair<float,float>> SensorStore::getLatestScan() const {
    std::lock_guard<std::mutex> lk(scan_mtx_);
    return latest_scan_;
}

// ── SLAM pose ───────────────────────────────────────────────────────────────

void SensorStore::setSlamPose(float x, float y, float theta) {
    std::lock_guard<std::mutex> lk(slam_pose_mtx_);
    slam_pose_ = {x, y, theta, true};
}

SensorStore::SlamPose SensorStore::getSlamPose() const {
    std::lock_guard<std::mutex> lk(slam_pose_mtx_);
    return slam_pose_;
}

// ── Generic named sensor slots ──────────────────────────────────────────────

void SensorStore::setGeneric(const std::string& key, float value) {
    std::unique_lock lk(generic_mtx_);
    generic_[key] = {value, now_ns()};
}

bool SensorStore::getGeneric(const std::string& key, float& value, double* age_ms) const {
    std::shared_lock lk(generic_mtx_);
    auto it = generic_.find(key);
    if (it == generic_.end()) return false;
    value = it->second.value;
    if (age_ms) *age_ms = (now_ns() - it->second.stamp_ns) / 1e6;
    return true;
}

std::vector<std::pair<std::string, GenericReading>> SensorStore::getAllGeneric() const {
    std::shared_lock lk(generic_mtx_);
    return {generic_.begin(), generic_.end()};
}
