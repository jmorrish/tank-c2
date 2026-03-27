#include "mission.h"
#include "comms.h"
#include "runtime_config.h"
#include "helpers.h"
#include "path_planner.h"

#include <nlohmann/json.hpp>

#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <sys/stat.h>
#include <dirent.h>

using Clock = std::chrono::steady_clock;

// ── GPS navigation helpers ─────────────────────────────────────────────────────

static double haversineM(double lat1, double lon1, double lat2, double lon2) {
    constexpr double R = 6371000.0;
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    double a = std::sin(dLat/2)*std::sin(dLat/2)
             + std::cos(lat1*M_PI/180)*std::cos(lat2*M_PI/180)
               *std::sin(dLon/2)*std::sin(dLon/2);
    return R * 2.0 * std::atan2(std::sqrt(a), std::sqrt(1.0 - a));
}

static double bearingDeg(double lat1, double lon1, double lat2, double lon2) {
    double dLon = (lon2 - lon1) * M_PI / 180.0;
    lat1 *= M_PI / 180.0;
    lat2 *= M_PI / 180.0;
    double y = std::sin(dLon) * std::cos(lat2);
    double x = std::cos(lat1)*std::sin(lat2) - std::sin(lat1)*std::cos(lat2)*std::cos(dLon);
    return std::fmod(std::atan2(y, x) * 180.0 / M_PI + 360.0, 360.0);
}

// ── Public API ───────────────────────────────────────────────────────────────

void MissionRunner::start(const std::string& json_str, Comms* comms, const RuntimeConfig& cfg) {
    abort();  // stop any running mission

    { std::lock_guard<std::mutex> lk(mtx_); json_ = json_str; }
    wp_idx_.store(0);
    fault_.store(0);
    run_.store(true);
    comms->setMode(ControlMode::MISSION);
    thread_ = std::thread(&MissionRunner::loop, this, comms, &cfg);
}

void MissionRunner::abort() {
    if (run_.exchange(false)) {
        if (thread_.joinable()) thread_.join();
    }
}

void MissionRunner::skipWaypoint() {
    if (run_.load()) {
        int nxt = wp_idx_.fetch_add(1) + 1;
        LOGI("Mission: skipping to WP " << nxt);
    }
}

void MissionRunner::resume(const std::string& id, Comms* comms, const RuntimeConfig& cfg) {
    std::string path = cfg.missions_dir + "/" + id + ".json";
    std::ifstream f(path);
    if (!f.is_open()) { LOGE("mission_resume: not found: " << id); return; }
    std::string json_str((std::istreambuf_iterator<char>(f)),
                          std::istreambuf_iterator<char>());
    int saved_idx = loadMissionState(id, cfg.missions_dir);

    abort();
    { std::lock_guard<std::mutex> lk(mtx_); json_ = json_str; }
    wp_idx_.store(saved_idx);
    fault_.store(0);
    run_.store(true);
    comms->setMode(ControlMode::MISSION);
    thread_ = std::thread(&MissionRunner::loop, this, comms, &cfg);
    LOGI("Mission resumed at WP " << saved_idx << ": " << id);
}

// ── Persistence ──────────────────────────────────────────────────────────────

void MissionRunner::saveMission(const std::string& id, const std::string& json_str,
                                const std::string& missions_dir) {
    ::mkdir(missions_dir.c_str(), 0755);
    std::string path = missions_dir + "/" + id + ".json";
    std::ofstream f(path);
    if (f.is_open()) {
        f << json_str;
        LOGI("Mission saved: " << path);
    } else {
        LOGE("Mission save failed: " << path);
    }
}

std::string MissionRunner::listMissionsJson(const std::string& missions_dir) {
    nlohmann::json list = nlohmann::json::array();
    DIR* dir = opendir(missions_dir.c_str());
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string fname = ent->d_name;
            if (fname.size() < 5 || fname.substr(fname.size()-5) != ".json") continue;
            std::string path = missions_dir + "/" + fname;
            std::ifstream f(path);
            if (!f.is_open()) continue;
            std::string content((std::istreambuf_iterator<char>(f)),
                                 std::istreambuf_iterator<char>());
            try {
                auto mj = nlohmann::json::parse(content);
                nlohmann::json entry;
                entry["id"]            = mj.value("id", "");
                entry["name"]          = mj.value("name", "");
                entry["type"]          = mj.value("type", "");
                entry["waypoint_count"] = mj.contains("waypoints")
                                         ? (int)mj["waypoints"].size() : 0;
                list.push_back(entry);
            } catch (...) {}
        }
        closedir(dir);
    }
    nlohmann::json resp;
    resp["type"]     = "missions";
    resp["missions"] = list;
    return resp.dump();
}

void MissionRunner::saveMissionState(int wp_idx, const std::string& missions_dir) {
    nlohmann::json state;
    { std::lock_guard<std::mutex> lk(mtx_); state["mission_json"] = json_; }
    state["waypoint_idx"] = wp_idx;
    std::string path = missions_dir + "/_state.json";
    std::ofstream f(path);
    if (f.is_open()) f << state.dump();
}

int MissionRunner::loadMissionState(const std::string& target_id, const std::string& missions_dir) {
    std::string path = missions_dir + "/_state.json";
    std::ifstream f(path);
    if (!f.is_open()) return 0;
    try {
        nlohmann::json state = nlohmann::json::parse(f);
        std::string json_str = state.value("mission_json", "");
        if (json_str.empty()) return 0;
        auto mj = nlohmann::json::parse(json_str);
        if (mj.value("id", "") != target_id) return 0;
        return state.value("waypoint_idx", 0);
    } catch (...) { return 0; }
}

// ── Mission executor thread ────────────────────────────────────────────────────

void MissionRunner::loop(Comms* comms, const RuntimeConfig* cfg) {
    LOGI("Mission executor started");

    std::string json_copy;
    { std::lock_guard<std::mutex> lk(mtx_); json_copy = json_; }
    nlohmann::json mission;
    try { mission = nlohmann::json::parse(json_copy); }
    catch (const std::exception& e) {
        LOGE("Mission: bad JSON: " << e.what());
        run_.store(false); comms->setMode(ControlMode::STOPPED); return;
    }
    if (!mission.contains("waypoints") || !mission["waypoints"].is_array()
        || mission["waypoints"].empty()) {
        LOGE("Mission: no waypoints");
        run_.store(false); comms->setMode(ControlMode::STOPPED); return;
    }

    auto& waypoints = mission["waypoints"];
    int n = (int)waypoints.size();
    for (int i = 0; i < n; i++) {
        double lat = waypoints[i].value("lat", 0.0);
        double lon = waypoints[i].value("lng", waypoints[i].value("lon", 0.0));
        if (std::abs(lat) < 0.001 && std::abs(lon) < 0.001) {
            LOGE("Mission: WP" << i << " has invalid coordinates (0,0) — aborting");
            run_.store(false); comms->setMode(ControlMode::STOPPED); return;
        }
    }

    bool do_loop   = mission.value("loop", false);
    bool do_return = mission.value("returnToStart", false);

    double heading_offset     = 0.0;
    bool   heading_calibrated = false;

    auto navigateTo = [&](double wp_lat, double wp_lon,
                          int wp_speed_sps, float arrive_r,
                          double timeout_s) -> bool
    {
        const double GPS_LOSS_ABORT_S  = cfg->mission_gps_loss_abort_s;
        const double STUCK_CHECK_S     = cfg->mission_stuck_check_s;
        const double STUCK_MIN_MOVE_M  = cfg->mission_stuck_min_move_m;
        const double STEER_GAIN        = cfg->mission_steer_gain;
        const double MAX_STEER_FRAC    = cfg->mission_max_steer_frac;
        const double MOVING_KNOTS      = cfg->mission_moving_knots;

        // ── Plan sub-waypoints via planner (if available) ────────────────
        struct SubWP { double lat; double lon; };
        std::vector<SubWP> sub_wps;
        int sub_idx = 0;
        auto plan_time = Clock::now();

        auto planSubWaypoints = [&](double from_lat, double from_lon) {
            if (planner_) {
                auto pts = planner_->planPath({from_lat, from_lon}, {wp_lat, wp_lon});
                sub_wps.clear();
                for (auto& gp : pts)
                    sub_wps.push_back({gp.lat, gp.lon});
                sub_idx = 0;
                plan_time = Clock::now();
                if (sub_wps.size() > 1) {
                    LOGI("Mission: planned " << sub_wps.size() << " sub-waypoints");
                }
            } else {
                sub_wps = {{wp_lat, wp_lon}};
                sub_idx = 0;
                plan_time = Clock::now();
            }
        };

        // Get initial GPS to plan from
        {
            double lat0, lon0; float alt0, spd0, crs0; int q0, s0; double ga0;
            if (comms->getLatestGPS(lat0, lon0, alt0, spd0, crs0, q0, s0, ga0) && q0 > 0)
                planSubWaypoints(lat0, lon0);
            else
                sub_wps = {{wp_lat, wp_lon}};  // fallback: direct
        }

        auto t_start     = Clock::now();
        auto gps_ok_last = Clock::now();
        auto stuck_t     = Clock::now();
        double stuck_ref_lat = wp_lat, stuck_ref_lon = wp_lon;
        bool   stuck_ref_init = false;

        while (run_.load()) {
            double lat, lon; float alt, spd, crs; int qual, sats; double gps_age;
            bool have_gps = comms->getLatestGPS(lat, lon, alt, spd, crs, qual, sats, gps_age);

            if (!have_gps || gps_age > 2000.0 || qual < 1) {
                double down_s = std::chrono::duration<double>(Clock::now() - gps_ok_last).count();
                fault_.store((int)MissionFault::GPS_LOST);
                comms->sendWheels(0, 0);
                if (down_s >= GPS_LOSS_ABORT_S) {
                    LOGE("Mission: GPS down " << (int)down_s << "s — aborting");
                    run_.store(false);
                    comms->setMode(ControlMode::STOPPED);
                    return false;
                }
                LOGW("Mission: GPS lost (" << (int)down_s << "/" << (int)GPS_LOSS_ABORT_S << "s)");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            gps_ok_last = Clock::now();
            fault_.store((int)MissionFault::NONE);

            if (!stuck_ref_init) {
                stuck_ref_lat = lat; stuck_ref_lon = lon;
                stuck_ref_init = true;
            }

            // Check arrival at final destination
            double dist_final = haversineM(lat, lon, wp_lat, wp_lon);
            if (dist_final <= arrive_r) return true;

            // ── Sub-waypoint advancement + replanning ─────────────────────
            if (!sub_wps.empty() && sub_idx < (int)sub_wps.size()) {
                double sub_dist = haversineM(lat, lon, sub_wps[sub_idx].lat, sub_wps[sub_idx].lon);
                if (sub_dist <= std::max(arrive_r, 2.0f)) {
                    sub_idx++;
                    if (sub_idx >= (int)sub_wps.size()) {
                        // All sub-waypoints done but not at final dest yet — replan
                        planSubWaypoints(lat, lon);
                    }
                }
            }

            // Replan if path invalidated or stale (>10s)
            if (planner_) {
                double plan_age = std::chrono::duration<double>(Clock::now() - plan_time).count();
                if (!planner_->isPathValid() || plan_age > 10.0) {
                    LOGI("Mission: replanning (age=" << (int)plan_age
                         << "s, valid=" << planner_->isPathValid() << ")");
                    planSubWaypoints(lat, lon);
                }
            }

            // Current steering target: current sub-waypoint or final destination
            double steer_lat = wp_lat, steer_lon = wp_lon;
            if (!sub_wps.empty() && sub_idx < (int)sub_wps.size()) {
                steer_lat = sub_wps[sub_idx].lat;
                steer_lon = sub_wps[sub_idx].lon;
            }

            double target = bearingDeg(lat, lon, steer_lat, steer_lon);

            double elapsed_s = std::chrono::duration<double>(Clock::now() - t_start).count();
            if (elapsed_s > timeout_s) {
                LOGW("Mission: nav timeout (" << (int)timeout_s << "s) — skipping WP");
                return false;
            }

            double stuck_elapsed = std::chrono::duration<double>(Clock::now() - stuck_t).count();
            if (stuck_elapsed >= STUCK_CHECK_S && wp_speed_sps > 500) {
                double moved = haversineM(stuck_ref_lat, stuck_ref_lon, lat, lon);
                if (moved < STUCK_MIN_MOVE_M) {
                    LOGE("Mission: STUCK — only moved " << moved << "m in "
                         << (int)STUCK_CHECK_S << "s");
                    fault_.store((int)MissionFault::STUCK);
                    comms->sendWheels(0, 0);
                    run_.store(false);
                    comms->setMode(ControlMode::STOPPED);
                    return false;
                }
                stuck_ref_lat = lat; stuck_ref_lon = lon;
                stuck_t = Clock::now();
            }

            if (spd > MOVING_KNOTS) {
                float yaw, pitch, roll; double imu_age;
                if (comms->getLatestYPR(yaw, pitch, roll, imu_age) && imu_age < 500.0) {
                    double raw_offset = std::fmod(crs - yaw + 360.0, 360.0);
                    if (!heading_calibrated) {
                        heading_offset = raw_offset;
                        heading_calibrated = true;
                    } else {
                        const double alpha = cfg->mission_heading_filter;
                        // Circular mean to avoid wrapping artefact near 0/360
                        double diff = raw_offset - heading_offset;
                        if (diff > 180.0)  diff -= 360.0;
                        if (diff < -180.0) diff += 360.0;
                        heading_offset = std::fmod(heading_offset + alpha * diff + 360.0, 360.0);
                    }
                }
            }

            double heading;
            if (spd > MOVING_KNOTS) {
                heading = std::fmod(crs + 360.0, 360.0);
            } else if (heading_calibrated) {
                float yaw, pitch, roll; double imu_age;
                if (comms->getLatestYPR(yaw, pitch, roll, imu_age) && imu_age < 500.0)
                    heading = std::fmod(yaw + heading_offset + 360.0, 360.0);
                else
                    heading = std::fmod(crs + 360.0, 360.0);
            } else {
                heading = std::fmod(crs + 360.0, 360.0);
            }

            double herr = target - heading;
            while (herr >  180.0) herr -= 360.0;
            while (herr < -180.0) herr += 360.0;
            double steer = std::clamp(herr * STEER_GAIN,
                                      -wp_speed_sps * MAX_STEER_FRAC,
                                       wp_speed_sps * MAX_STEER_FRAC);
            int L = std::clamp((int)(wp_speed_sps - steer), -cfg->wheel_max_sps, cfg->wheel_max_sps);
            int R = std::clamp((int)(wp_speed_sps + steer), -cfg->wheel_max_sps, cfg->wheel_max_sps);

            if (!comms->sendWheels(L, R)) {
                fault_.store((int)MissionFault::WHEEL);
                LOGW("Mission: wheel send failed — control link down? Waiting...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            fault_.store((int)MissionFault::NONE);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    };

    // ── Main mission loop ────────────────────────────────────────────────────
    do {
        int n_wps = (int)waypoints.size();
        while (run_.load()) {
            int idx = wp_idx_.load();
            if (idx >= n_wps) break;

            auto& wp = waypoints[idx];
            double wp_lat   = wp.value("lat", 0.0);
            double wp_lon   = wp.value("lng", wp.value("lon", 0.0));
            float  spd_pct  = wp.value("speed", 60.0f);
            int    wp_sps   = std::clamp((int)(spd_pct / 100.0f * cfg->wheel_max_sps),
                                         100, cfg->wheel_max_sps);
            float  radius   = wp.value("arrivalRadius", wp.value("radius_m", 3.0f));
            std::string action = wp.value("arrivalAction", wp.value("action", "continue"));

            LOGI("Mission: WP " << idx << "/" << (n_wps-1)
                 << " action=" << action
                 << " lat=" << wp_lat << " lon=" << wp_lon);

            bool arrived = navigateTo(wp_lat, wp_lon, wp_sps, radius, 120.0);
            comms->sendWheels(0, 0);

            if (!run_.load()) break;

            if (arrived) {
                if (action == "wait" || action == "stop") {
                    float wait_s = wp.value("waitSeconds", wp.value("dwell_s", 5.0f));
                    LOGI("Mission: WP" << idx << " wait " << wait_s << "s");
                    float el = 0.0f;
                    while (run_.load() && el < wait_s)
                        { std::this_thread::sleep_for(std::chrono::milliseconds(100)); el += 0.1f; }

                } else if (action == "scan") {
                    float from = wp.value("scanFrom", -90.0f);
                    float to   = wp.value("scanTo",    90.0f);
                    float sspd = wp.value("scanSpeed",  200.0f);
                    const float MS_PER_DEG = cfg->mission_ptu_ms_per_deg;
                    float sweep_ms = std::abs(to - from) * MS_PER_DEG;
                    float dir = (to > from) ? 1.0f : -1.0f;
                    LOGI("Mission: PTU scan " << from << "° to " << to << "°");
                    comms->sendPTUVelocity(dir * sspd, 0);
                    auto ts = Clock::now();
                    while (run_.load()) {
                        if (std::chrono::duration<float,std::milli>(Clock::now()-ts).count() >= sweep_ms) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    comms->sendPTUVelocity(-dir * sspd, 0);
                    ts = Clock::now();
                    while (run_.load()) {
                        if (std::chrono::duration<float,std::milli>(Clock::now()-ts).count() >= sweep_ms/2) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    comms->sendPTUVelocity(0, 0);

                } else if (action == "set_ptu") {
                    float pan  = wp.value("ptuPan",  0.0f);
                    float tilt = wp.value("ptuTilt", 0.0f);
                    LOGI("Mission: set PTU pan=" << pan << " tilt=" << tilt);
                    comms->sendPTUVelocity(pan, tilt);

                } else if (action == "follow") {
                    float dur = wp.value("followDuration", 0.0f);
                    LOGI("Mission: follow mode for " << (dur > 0 ? std::to_string((int)dur)+"s" : "unlimited"));
                    comms->setMode(ControlMode::FOLLOW);
                    if (dur > 0.0f) {
                        float el = 0.0f;
                        while (run_.load() && el < dur)
                            { std::this_thread::sleep_for(std::chrono::milliseconds(100)); el += 0.1f; }
                        comms->setMode(ControlMode::MISSION);
                    } else {
                        run_.store(false);
                        LOGI("Mission executor yielding to FOLLOW mode");
                        return;
                    }

                } else if (action == "custom") {
                    std::string ccmd = wp.value("customCommand", "");
                    if (!ccmd.empty()) {
                        LOGI("Mission: custom cmd: " << ccmd);
                        comms->handleWebCommand(ccmd);
                    }
                }
            }

            int next_idx = idx + 1;
            wp_idx_.store(next_idx);
            saveMissionState(next_idx, cfg->missions_dir);
        }

        if (!run_.load()) break;

        if (do_return && n > 0) {
            LOGI("Mission: returning to start");
            auto& sp    = waypoints[0];
            double slat = sp.value("lat", 0.0);
            double slon = sp.value("lng", sp.value("lon", 0.0));
            float  srad = sp.value("arrivalRadius", 3.0f);
            int ret_speed = (int)(cfg->wheel_max_sps * cfg->mission_return_speed_frac);
            navigateTo(slat, slon, ret_speed, srad, 120.0);
            comms->sendWheels(0, 0);
        }

        if (!run_.load()) break;

        if (do_loop) {
            LOGI("Mission: looping back to WP 0");
            wp_idx_.store(0);
            saveMissionState(0, cfg->missions_dir);
        }

    } while (do_loop && run_.load());

    // ── Complete ──────────────────────────────────────────────────────────────
    bool natural_completion = run_.load();
    if (natural_completion) {
        LOGI("Mission: complete");
        comms->sendWheels(0, 0);
        comms->sendPTUVelocity(0, 0);
        run_.store(false);
        comms->setMode(ControlMode::STOPPED);
        std::remove((cfg->missions_dir + "/_state.json").c_str());
    }

    // ── Fire mission event to VPS ─────────────────────────────────────────────
    {
        std::string mission_id, mission_name;
        { std::lock_guard<std::mutex> lk(mtx_);
          try {
              auto m = nlohmann::json::parse(json_);
              mission_id   = m.value("id",   "");
              mission_name = m.value("name", "");
          } catch (...) {} }

        nlohmann::json ev;
        ev["type"] = "event";
        ev["id"]   = mission_id;
        ev["name"] = mission_name;

        if (natural_completion) {
            ev["event"] = "mission_completed";
            LOGI("Mission event: completed — " << mission_name);
        } else {
            int f = fault_.load();
            if (f != (int)MissionFault::NONE) {
                static const char* faultNames[] = {"", "gps_lost", "stuck", "wheel_fail"};
                ev["event"] = "mission_faulted";
                ev["fault"] = faultNames[std::clamp(f, 0, 3)];
                LOGI("Mission event: faulted (" << ev["fault"] << ") — " << mission_name);
            }
        }

        if (ev.contains("event"))
            comms->sendWebEvent(ev.dump());
    }

    LOGI("Mission executor finished");
}
