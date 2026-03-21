#include "logger.h"
#include "helpers.h"
#include <iomanip>

RunLogger::RunLogger(const std::string& path)
    : t0_(std::chrono::steady_clock::now())
{
    file_.open(path, std::ios::out | std::ios::app);
    if (!file_.is_open()) {
        LOGE("RunLogger: cannot open '" << path << "'");
        return;
    }
    file_ << "timestamp_ms,track_id,dx,dy,pan_vel,tilt_vel,"
             "left_sps,right_sps,dist_m,yaw,pitch,roll\n";
    LOGI("RunLogger: logging to '" << path << "'");
}

RunLogger::~RunLogger() {
    if (file_.is_open()) file_.close();
}

void RunLogger::log(int   track_id,
                    float dx,       float dy,
                    float pan_vel,  float tilt_vel,
                    int   left_sps, int   right_sps,
                    float dist_m,
                    float yaw,      float pitch, float roll)
{
    if (!file_.is_open()) return;
    auto now = std::chrono::steady_clock::now();
    long ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - t0_).count();
    std::lock_guard<std::mutex> lk(mtx_);
    file_ << ms         << ','
          << track_id   << ','
          << std::fixed << std::setprecision(2)
          << dx         << ',' << dy
          << ','        << pan_vel  << ',' << tilt_vel
          << ','        << left_sps << ',' << right_sps
          << ','        << dist_m
          << ','        << yaw << ',' << pitch << ',' << roll
          << '\n';
}
