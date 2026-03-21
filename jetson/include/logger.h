#pragma once
#include <string>
#include <fstream>
#include <mutex>
#include <chrono>

// Writes one CSV row per movement loop iteration.
// Columns: timestamp_ms, track_id, dx, dy, pan_vel, tilt_vel,
//          left_sps, right_sps, dist_m, yaw, pitch, roll
class RunLogger {
public:
    explicit RunLogger(const std::string& path);
    ~RunLogger();

    void log(int   track_id,
             float dx,       float dy,
             float pan_vel,  float tilt_vel,
             int   left_sps, int   right_sps,
             float dist_m,
             float yaw,      float pitch, float roll);

    bool isOpen() const { return file_.is_open(); }

private:
    std::ofstream file_;
    std::mutex    mtx_;
    std::chrono::steady_clock::time_point t0_;
};
