#include "comms.h"
#include "helpers.h"
#include "object_detection.h"
#include "behavior_coordinator.h"
#include "mission_behavior.h"
#include "nav2_planner.h"

#include <nlohmann/json.hpp>

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <chrono>
#include <vector>
#include <algorithm>
#include <cerrno>
#include <cmath>
#include <sys/stat.h>
#include <dirent.h>

// POSIX / TCP (for Web IPC server)
#include <unistd.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

using Clock = std::chrono::steady_clock;

Comms::Comms(const RuntimeConfig& cfg) : cfg_(cfg) {}
Comms::~Comms(){
    closeControl();
    closeSensor();
    stopWebIPC();
    stopSlamBridge();
    stopStereoDepth();
}

// ── Control mode delegation ──────────────────────────────────────────────────

ControlMode Comms::getMode() const {
    if (coordinator_) return coordinator_->currentMode();
    return static_cast<ControlMode>(mode_.load());
}

void Comms::setMode(ControlMode m) {
    if (coordinator_) { coordinator_->requestTransition(m); return; }
    mode_.store(static_cast<int>(m));
}

bool Comms::connectControl(const std::string& ip, int port){
    closeControl();
    return control_link_.open("Control", ip, port,
        [this](const std::string& line) {
            if (line.rfind("ENC", 0) == 0) {
                int l, r;
                if (std::sscanf(line.c_str() + 3, "%d %d", &l, &r) == 2)
                    sensors_.setEncoders(l, r);
            } else if (line.rfind("PTU_YPR", 0) == 0) {
                float y, p, r;
                if (std::sscanf(line.c_str() + 7, "%f %f %f", &y, &p, &r) == 3)
                    sensors_.setPtuIMU(y, p, r);
            }
        });
}

bool Comms::connectSensor(const std::string& ip, int port){
    closeSensor();
    return sensor_link_.open("Sensor", ip, port,
        [this](const std::string& line) {
            if (line.rfind("YPR", 0) == 0 && line.rfind("PTU_YPR", 0) != 0) {
                float y, p, r;
                if (std::sscanf(line.c_str()+3, "%f %f %f", &y, &p, &r) == 3) {
                    sensors_.setIMU(y, p, r);
                    // Forward IMU to slam_bridge for ROS2 /imu/data topic
                    char buf[128];
                    std::snprintf(buf, sizeof(buf),
                        R"({"type":"imu_data","yaw":%.4f,"pitch":%.4f,"roll":%.4f})", y, p, r);
                    slam_link_.sendLine(buf);
                }
            } else if (line.rfind("PTU_YPR", 0) == 0) {
                float y, p, r;
                if (std::sscanf(line.c_str()+7, "%f %f %f", &y, &p, &r) == 3)
                    sensors_.setPtuIMU(y, p, r);
            } else if (line.rfind("TOF", 0) == 0) {
                float d;
                if (std::sscanf(line.c_str()+3, "%f", &d) == 1)
                    sensors_.setTOF(d);
            } else if (line.rfind("$GPGGA", 0) == 0 || line.rfind("$GNGGA", 0) == 0) {
                parseGGA(line);
            } else if (line.rfind("$GPRMC", 0) == 0 || line.rfind("$GNRMC", 0) == 0) {
                parseRMC(line);
            }
        },
        [this]() { imuOn(); imuRate(50.0f); });
}

void Comms::closeControl(){ control_link_.close(); }
void Comms::closeSensor() { sensor_link_.close(); }

bool Comms::sendPTUVelocity(float pan_sps, float tilt_sps){
    std::ostringstream oss;
    oss << "VP" << int(pan_sps) << "T" << int(tilt_sps);
    return control_link_.sendLine(oss.str());
}

bool Comms::sendWheels(int left_sps, int right_sps){
    std::ostringstream oss;
    oss << "LS" << left_sps << "RS" << right_sps;
    return control_link_.sendLine(oss.str());
}

bool Comms::imuOn(){
    return sensor_link_.sendLine("IMU_ON");
}

bool Comms::imuOff(){
    return sensor_link_.sendLine("IMU_OFF");
}

bool Comms::imuRate(float hz){
    std::ostringstream oss;
    oss << "IMU_RATE " << int(hz);
    return sensor_link_.sendLine(oss.str());
}

// ── NMEA parsing helpers ────────────────────────────────────────────────────

void Comms::parseGGA(const std::string& line) {
    size_t star_pos = line.find('*');
    if (star_pos == std::string::npos) return;
    std::string sentence = line.substr(0, star_pos);
    std::stringstream ss(sentence);
    std::string token;
    std::vector<std::string> fields;
    while (std::getline(ss, token, ',')) fields.push_back(token);
    if ((fields.size() < 14 || fields.size() > 15) ||
        (fields[0] != "$GPGGA" && fields[0] != "$GNGGA")) return;

    std::string lat_str = fields[2], ns = fields[3];
    std::string lon_str = fields[4], ew = fields[5];
    int quality = 0;
    try { quality = std::stoi(fields[6]); } catch (...) { return; }
    int sats = 0;
    try { sats = std::stoi(fields[7]); } catch (...) { return; }
    float alt = 0.0f;
    try { alt = std::stof(fields[9]); } catch (...) { return; }

    double lat_deg = 0.0;
    size_t dot = lat_str.find('.');
    if (dot != std::string::npos && dot >= 2) {
        try {
            int deg = std::stoi(lat_str.substr(0, dot-2));
            double min = std::stod(lat_str.substr(dot-2));
            lat_deg = deg + min / 60.0;
            if (ns == "S") lat_deg = -lat_deg;
        } catch (...) { return; }
    } else return;

    double lon_deg = 0.0;
    dot = lon_str.find('.');
    if (dot != std::string::npos && dot >= 3) {
        try {
            int deg = std::stoi(lon_str.substr(0, dot-2));
            double min = std::stod(lon_str.substr(dot-2));
            lon_deg = deg + min / 60.0;
            if (ew == "W") lon_deg = -lon_deg;
        } catch (...) { return; }
    } else return;

    GPSData gps;
    gps.lat = lat_deg; gps.lon = lon_deg; gps.alt = alt;
    gps.quality = quality; gps.sats = sats; gps.valid = (quality > 0);
    sensors_.setGPS(gps);

    // Forward GPS to slam_bridge for ROS2 /gps/fix topic
    char gbuf[256];
    std::snprintf(gbuf, sizeof(gbuf),
        R"({"type":"gps_data","lat":%.8f,"lon":%.8f,"alt":%.2f,"quality":%d,"sats":%d})",
        lat_deg, lon_deg, alt, quality, sats);
    slam_link_.sendLine(gbuf);
}

void Comms::parseRMC(const std::string& line) {
    size_t star_pos = line.find('*');
    if (star_pos == std::string::npos) return;
    std::string sentence = line.substr(0, star_pos);
    std::stringstream ss(sentence);
    std::string token;
    std::vector<std::string> fields;
    while (std::getline(ss, token, ',')) fields.push_back(token);
    if (fields.size() != 13 || (fields[0] != "$GPRMC" && fields[0] != "$GNRMC")) return;
    if (fields[2] != "A") return;

    float speed = 0.0f;
    try { speed = std::stof(fields[7]); } catch (...) { return; }
    float course = 0.0f;
    try { course = std::stof(fields[8]); } catch (...) { return; }

    sensors_.setGPSSpeed(speed, course);

    // Forward speed/course to slam_bridge for heading calibration
    char rbuf[128];
    std::snprintf(rbuf, sizeof(rbuf),
        R"({"type":"gps_rmc","speed_knots":%.2f,"course_deg":%.2f})", speed, course);
    slam_link_.sendLine(rbuf);
}

// Web IPC implementation
bool Comms::startWebIPC(int port) {
    web_sock_ = SocketFd(::socket(AF_INET, SOCK_STREAM, 0));
    if (!web_sock_.valid()) { LOGE("Web IPC socket failed"); return false; }

    int opt = 1;
    setsockopt(web_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = htonl(INADDR_ANY);  // Accept on all interfaces (Tailscale + LAN)

    if (::bind(web_sock_, (sockaddr*)&sa, sizeof(sa)) < 0) {
        LOGE("Web IPC bind failed: " << strerror(errno));
        web_sock_.close(); return false;
    }
    if (::listen(web_sock_, 1) < 0) {
        LOGE("Web IPC listen failed");
        web_sock_.close(); return false;
    }

    web_rxRun_.store(true);
    web_rxThread_ = std::thread([this]() {
        // Loop to accept reconnecting clients
        while (web_rxRun_.load()) {
            int client = ::accept(web_sock_, nullptr, nullptr);
            if (client < 0) {
                // web_sock_ was closed by stopWebIPC() — exit cleanly
                break;
            }
            LOGI("Web IPC client connected");
            web_client_.store(client);

            // Newline-delimited protocol: one command per line, one JSON response per line.
            // Socket has a 200ms recv timeout so we can push periodic status updates.
            char buf[4096];
            std::string partial;
            struct timeval tv{0, 200000};
            setsockopt(client, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
            auto lastPush = Clock::now();
            bool alive = true;
            while (web_rxRun_.load() && alive) {
                ssize_t n = ::recv(client, buf, sizeof(buf) - 1, 0);
                if (n > 0) {
                    partial.append(buf, n);
                    size_t pos;
                    while ((pos = partial.find('\n')) != std::string::npos) {
                        std::string cmd = partial.substr(0, pos);
                        partial.erase(0, pos + 1);
                        if (!cmd.empty() && cmd.back() == '\r') cmd.pop_back();
                        if (cmd.empty()) continue;
                        std::string response = handleWebCommand(cmd);
                        if (response.empty()) response = getStatusJson();
                        response += "\n";
                        if (::send(client, response.c_str(), response.size(), MSG_NOSIGNAL) < 0)
                            { alive = false; break; }
                        lastPush = Clock::now();
                    }
                } else if (n == 0) {
                    alive = false;  // clean disconnect
                } else {
                    if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                        alive = false;  // real error
                    // else: recv timeout — fall through to periodic push check
                }
                if (!alive) break;
                // Push status to server.js every 1 second (keeps telemetry live without commands)
                auto now = Clock::now();
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPush).count() >= cfg_.status_push_interval_ms) {
                    std::string s = getStatusJson() + "\n";
                    if (::send(client, s.c_str(), s.size(), MSG_NOSIGNAL) < 0) break;
                    lastPush = now;
                }
            }
            web_client_.store(-1);
            ::close(client);
            LOGI("Web IPC client disconnected");
        }
    });

    LOGI("Web IPC server started on localhost:" << port);
    return true;
}

void Comms::stopWebIPC() {
    mission_.abort();

    web_rxRun_.store(false);
    web_sock_.close();
    if (web_rxThread_.joinable()) web_rxThread_.join();
}

// Send a one-shot event JSON line to the connected VPS client (if any).
// server.js detects {type:"event"} lines and re-broadcasts them as mission_status
// without overwriting lastStatus.
void Comms::sendWebEvent(const std::string& json_str) {
    int fd = web_client_.load();
    if (fd < 0) return;
    std::string line = json_str + "\n";
    ::send(fd, line.c_str(), line.size(), MSG_NOSIGNAL);
}

std::string Comms::getStatusJson() const {
    nlohmann::json j;

    const double STALE_THRESHOLD_MS = cfg_.telemetry_stale_ms;

    float yaw = 0.0f, pitch = 0.0f, roll = 0.0f; double iage = 99999.0;
    getLatestYPR(yaw, pitch, roll, iage);
    j["imu"] = {{"yaw", yaw}, {"pitch", pitch}, {"roll", roll}, {"age_ms", iage},
                {"stale", iage > STALE_THRESHOLD_MS}};

    double lat = 0.0, lon = 0.0, gage = 99999.0; float alt = 0.0f, speed = 0.0f, course = 0.0f; int qual = 0, sats = 0;
    getLatestGPS(lat, lon, alt, speed, course, qual, sats, gage);
    j["gps"] = {{"lat", lat}, {"lon", lon}, {"alt", alt}, {"speed_knots", speed},
                {"course_deg", course}, {"quality", qual}, {"sats", sats}, {"age_ms", gage},
                {"stale", gage > STALE_THRESHOLD_MS}};

    int left = 0, right = 0; double eage = 99999.0;
    getLatestEncoders(left, right, eage);
    j["encoders"] = {{"left", left}, {"right", right}, {"age_ms", eage},
                     {"stale", eage > STALE_THRESHOLD_MS}};

    double dage = 99999.0;
    j["distance_m"]      = getLatestDistance(&dage);
    j["distance_age_ms"] = dage;

    double lage = 99999.0;
    j["lidar_fwd_m"]      = getLatestLidarFwdDist(&lage);
    j["lidar_fwd_age_ms"] = lage;
    j["lidar_obstacle"]   = sensors_.hasObstacle();

    {
        auto sp = sensors_.getSlamPose();
        j["slam_pose"] = {{"x",sp.x},{"y",sp.y},{"theta",sp.theta},{"valid",sp.valid}};
    }

    j["detection_fps"]    = sensors_.getDetectionFPS();
    j["stream_quality"]   = sensors_.getStreamQuality();
    j["target_person_id"] = sensors_.getTargetPersonId();

    // Generic sensor slots — auto-serialised, no code changes needed per sensor
    {
        auto generics = sensors_.getAllGeneric();
        if (!generics.empty()) {
            nlohmann::json gen;
            for (auto& [key, reading] : generics) {
                double age = (steady_now_ns() - reading.stamp_ns) / 1e6;
                gen[key] = {{"value", reading.value}, {"age_ms", age},
                            {"stale", age > STALE_THRESHOLD_MS}};
            }
            j["sensors"] = gen;
        }
    }

    // Control mode
    static const char* modeNames[] = {"follow", "manual", "mission", "stopped"};
    int m = static_cast<int>(getMode());
    j["mode"] = modeNames[std::clamp(m, 0, 3)];

    // Mission status
    static const char* faultNames[] = {"", "gps_lost", "stuck", "wheel_fail"};
    int fault = std::clamp(mission_.fault(), 0, 3);

    nlohmann::json ms;
    ms["running"]      = mission_.running();
    ms["waypoint_idx"] = mission_.waypointIdx();
    ms["fault"]        = faultNames[fault];
    {
        std::string mj_str = mission_.missionJson();
        if (!mj_str.empty()) {
            try {
                auto mj = nlohmann::json::parse(mj_str);
                ms["id"]             = mj.value("id", "");
                ms["name"]           = mj.value("name", "");
                ms["waypoint_count"] = mj.contains("waypoints")
                                       ? (int)mj["waypoints"].size() : 0;
                int idx = mission_.waypointIdx();
                if (mj.contains("waypoints") && idx >= 0 &&
                    idx < (int)mj["waypoints"].size()) {
                    auto& twp = mj["waypoints"][idx];
                    ms["target_lat"] = twp.value("lat", 0.0);
                    ms["target_lon"] = twp.value("lng", twp.value("lon", 0.0));
                }
            } catch (...) {}
        }
    }
    j["mission"] = ms;

    return j.dump();
}

std::string Comms::handleWebCommand(const std::string& cmd) {
    LOGI("Web cmd: " << cmd.substr(0, 80));   // truncate long mission JSON in log

    // ── Stop / Emergency ──────────────────────────────────────────────────────
    // Coordinator doTransition() zeros all actuators and calls onExit() on the
    // active behavior (which aborts the mission if in MISSION mode).
    if (cmd == "stop_follow" || cmd == "emergency_stop") {
        setMode(ControlMode::STOPPED);
        return "";
    }

    // ── Resume autonomous follow ───────────────────────────────────────────────
    if (cmd == "resume_follow") {
        setMode(ControlMode::FOLLOW);
        return "";
    }

    // ── Manual wheels: manual_wheel:L:R ───────────────────────────────────────
    if (cmd.rfind("manual_wheel:", 0) == 0) {
        setMode(ControlMode::MANUAL);
        int L = 0, R = 0;
        if (std::sscanf(cmd.c_str() + 13, "%d:%d", &L, &R) == 2)
            sendWheels(L, R);
        return "";
    }

    // ── Manual PTU centre ─────────────────────────────────────────────────────
    if (cmd == "manual_ptu:centre") {
        sendPTUVelocity(0, 0);                                          // stop velocity
        control_link_.sendLine("P0T0");     // hard position centre
        return "";
    }

    // ── Manual PTU velocity: manual_ptu:P:T ───────────────────────────────────
    if (cmd.rfind("manual_ptu:", 0) == 0) {
        float P = 0.0f, T = 0.0f;
        if (std::sscanf(cmd.c_str() + 11, "%f:%f", &P, &T) == 2)
            sendPTUVelocity(P, T);
        return "";
    }

    // ── Mission save (push from website, no execution): mission_save:{json} ──
    if (cmd.rfind("mission_save:", 0) == 0) {
        std::string json_str = cmd.substr(13);
        try {
            auto mj = nlohmann::json::parse(json_str);
            std::string id = mj.value("id", "unknown");
            mission_.saveMission(id, json_str, cfg_.missions_dir);
            LOGI("Mission saved (pushed, not started): " << id);
        } catch (const std::exception& e) {
            LOGE("mission_save bad JSON: " << e.what());
        }
        return "";
    }

    // ── Mission start: mission_start:{json} ───────────────────────────────────
    if (cmd.rfind("mission_start:", 0) == 0) {
        std::string json_str = cmd.substr(14);
        try {
            auto mj = nlohmann::json::parse(json_str);
            std::string id = mj.value("id", "unknown");
            mission_.saveMission(id, json_str, cfg_.missions_dir);
            if (mission_behavior_) {
                mission_behavior_->loadMission(json_str);
                setMode(ControlMode::MISSION);
            } else {
                mission_.start(json_str, this, cfg_);
            }
            LOGI("Mission started: " << id);
        } catch (const std::exception& e) {
            LOGE("mission_start bad JSON: " << e.what());
        }
        return "";
    }

    // ── Mission abort ─────────────────────────────────────────────────────────
    if (cmd == "mission_abort") {
        setMode(ControlMode::STOPPED);   // coordinator exits MISSION → onExit aborts runner
        LOGI("Mission aborted");
        return "";
    }

    // ── Load a saved mission by ID and start it ────────────────────────────────
    if (cmd.rfind("mission_load:", 0) == 0) {
        std::string id = cmd.substr(13);
        id.erase(std::remove_if(id.begin(), id.end(),
            [](char c){ return !isalnum(c) && c != '_' && c != '-'; }), id.end());
        std::string path = cfg_.missions_dir + "/" + id + ".json";
        std::ifstream f(path);
        if (f.is_open()) {
            std::string json_str((std::istreambuf_iterator<char>(f)),
                                  std::istreambuf_iterator<char>());
            handleWebCommand("mission_start:" + json_str);
        } else {
            LOGE("mission_load: not found: " << path);
        }
        return "";
    }

    // ── List saved missions ────────────────────────────────────────────────────
    if (cmd == "list_missions") {
        return MissionRunner::listMissionsJson(cfg_.missions_dir);
    }

    // ── Skip current waypoint ─────────────────────────────────────────────────
    if (cmd == "mission_skip_wp") {
        mission_.skipWaypoint();
        return "";
    }

    // ── Resume mission from last saved waypoint ────────────────────────────────
    if (cmd.rfind("mission_resume:", 0) == 0) {
        std::string id = cmd.substr(15);
        id.erase(std::remove_if(id.begin(), id.end(),
            [](char c){ return !isalnum(c) && c != '_' && c != '-'; }), id.end());
        if (mission_behavior_) {
            mission_behavior_->loadResume(id);
            setMode(ControlMode::MISSION);
        } else {
            mission_.resume(id, this, cfg_);
        }
        return "";
    }

    // ── Set follow target by gallery person ID: set_target:<id> ──────────────
    if (cmd.rfind("set_target:", 0) == 0) {
        try {
            int pid = std::stoi(cmd.substr(11));
            if (od_ptr_) od_ptr_->setTargetPerson(pid);
            LOGI("set_target: person " << pid);
        } catch (...) {}
        return "";
    }

    // ── Learn mode: collect appearance crops for AI description ───────────────
    if (cmd.rfind("learn_start:", 0) == 0) {
        try {
            int pid = std::stoi(cmd.substr(12));
            if (od_ptr_) od_ptr_->startLearn(pid);
            LOGI("learn_start: person " << pid);
        } catch (...) {}
        return "";
    }
    if (cmd == "learn_stop") {
        if (od_ptr_) od_ptr_->stopLearn();
        LOGI("learn_stop");
        return "";
    }

    // ── Stereo depth camera ───────────────────────────────────────────────────
    if (cmd == "stereo_start") { startStereoDepth(stereo_device_); return ""; }
    if (cmd == "stereo_stop")  { stopStereoDepth();  return ""; }

    // ── Stream quality: stream_quality:<1-100> ────────────────────────────────
    if (cmd.rfind("stream_quality:", 0) == 0) {
        try {
            int q = std::stoi(cmd.substr(15));
            q = std::clamp(q, 1, 100);
            sensors_.setStreamQuality(q);
            LOGI("Stream quality set to " << q);
        } catch (...) {}
        return "";
    }

    LOGW("Unknown web command: " << cmd);
    return "";
}


// ── SLAM Bridge client ────────────────────────────────────────────────────────

bool Comms::startSlamBridge(const std::string& host, int port) {
    return slam_link_.start(host, port, sensors_,
        [this](const std::string& line) { updateFromBridge(line); });
}

void Comms::stopSlamBridge() {
    slam_link_.stop();
    // Clear stale obstacle data
    sensors_.setLidar(false, -1.0f, {});
}

bool Comms::sendSlamBridge(const std::string& json_str) {
    return slam_link_.sendLine(json_str);
}

// ── Stereo depth camera ───────────────────────────────────────────────────────

void Comms::startStereoDepth(const std::string& device, bool share_left) {
    if (stereo_depth_.running()) {
        LOGI("StereoDepth already running");
        return;
    }
    stereo_device_ = device;
    LOGI("Starting stereo depth camera on " << device
         << (share_left ? " (sharing left frame for detection fallback)" : ""));
    stereo_depth_.start(device, cfg_, share_left);
}

bool Comms::getStereoLeftFrame(cv::Mat& out) {
    return stereo_depth_.getLatestLeft(out);
}

void Comms::stopStereoDepth() {
    LOGI("Stopping integrated stereo depth camera");
    stereo_depth_.stop();
}


void Comms::updateFromBridge(const std::string& line) {
    auto j = nlohmann::json::parse(line);
    const std::string type = j.at("type").get<std::string>();

    if (type == "scan_full") {
        bool obs = j.at("obs").get<bool>();
        float fwd = j.at("fwd").get<float>();

        std::vector<std::pair<float,float>> pts;
        for (auto& p : j.at("pts"))
            pts.push_back({p[0].get<float>(), p[1].get<float>()});
        sensors_.setLidar(obs, fwd, std::move(pts));
        broadcastScan();

    } else if (type == "slam_pose") {
        sensors_.setSlamPose(j.at("x").get<float>(), j.at("y").get<float>(),
                             j.at("theta").get<float>());
    } else if (type == "slam_map") {
        sendWebEvent(line);  // forward PNG to web client unchanged

    } else if (type == "planned_path") {
        if (nav2_planner_) nav2_planner_->onPlanResponse(j);
    }
}

void Comms::broadcastScan() {
    auto pts = sensors_.getLatestScan();
    if (pts.empty()) return;

    // Compact JSON: {"type":"scan","obs":bool,"fwd":float,"pts":[[angle,dist],...]}
    std::string j = "{\"type\":\"scan\",\"obs\":";
    j += sensors_.hasObstacle() ? "true" : "false";
    char fbuf[32];
    snprintf(fbuf, sizeof(fbuf), ",\"fwd\":%.2f", sensors_.getLatestLidarFwdDist());
    j += fbuf;
    j += ",\"pts\":[";
    for (size_t i = 0; i < pts.size(); ++i) {
        if (i) j += ',';
        char buf[32];
        snprintf(buf, sizeof(buf), "[%.1f,%.2f]", pts[i].first, pts[i].second);
        j += buf;
    }
    j += "]}";
    sendWebEvent(j);
}