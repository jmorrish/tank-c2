#include "comms.h"
#include "helpers.h"
#include "config.h"
#include "object_detection.h"

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

// POSIX
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>

// TCP
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>   // for TCP_NODELAY

using Clock = std::chrono::steady_clock;
using std::isfinite;

static int64_t now_ns(){
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

Comms::Comms(){}
Comms::~Comms(){
    closeControl();
    closeSensor();
    // NEW: Ensure web IPC closes
    stopWebIPC();
}

// Opens a TCP socket to ip:port with a 2-second connect timeout and TCP_NODELAY.
// Returns the socket FD, or -1 on failure. Never hangs indefinitely.
int Comms::openTcpSocket(const std::string& ip, int port) {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &sa.sin_addr) <= 0){ ::close(s); return -1; }

    // Non-blocking connect with 2-second timeout so we never hang on startup
    int flags = ::fcntl(s, F_GETFL, 0);
    ::fcntl(s, F_SETFL, flags | O_NONBLOCK);

    int rc = ::connect(s, (sockaddr*)&sa, sizeof(sa));
    if (rc < 0 && errno != EINPROGRESS){ ::close(s); return -1; }

    if (rc != 0) {   // EINPROGRESS — wait up to 2 seconds
        fd_set wset;
        FD_ZERO(&wset); FD_SET(s, &wset);
        struct timeval tv{2, 0};
        int sel = ::select(s + 1, nullptr, &wset, nullptr, &tv);
        if (sel <= 0){ ::close(s); return -1; }   // timeout or error
        int err = 0; socklen_t len = sizeof(err);
        ::getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err){ ::close(s); return -1; }
    }

    // Restore blocking mode for normal recv/send
    ::fcntl(s, F_SETFL, flags);

    struct timeval tv{0, 200000};
    setsockopt(s, SOL_SOCKET,  SO_RCVTIMEO,  &tv,  sizeof(tv));
    int flag = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    return s;
}

bool Comms::connectControl(const std::string& ip, int port){
    closeControl();
    int s = openTcpSocket(ip, port);
    if (s < 0){
        LOGE("connectControl failed to " << ip << ":" << port << " errno=" << errno);
        return false;
    }
    control_sock_ = s;
    control_ip_   = ip;
    control_port_ = port;
    LOGI("Connected to control " << ip << ":" << port << " for PTU/wheels.");
    control_rxRun_.store(true);
    control_rxThread_ = std::thread(&Comms::controlRxLoop, this);
    return true;
}

bool Comms::connectSensor(const std::string& ip, int port){
    closeSensor();
    int s = openTcpSocket(ip, port);
    if (s < 0){
        LOGE("connectSensor failed to " << ip << ":" << port << " errno=" << errno);
        return false;
    }
    sensor_sock_ = s;
    sensor_ip_   = ip;
    sensor_port_ = port;
    LOGI("Connected to sensor " << ip << ":" << port << " for IMU/GPS/encoders.");
    sensor_rxRun_.store(true);
    sensor_rxThread_ = std::thread(&Comms::sensorRxLoop, this);
    imuOn();
    imuRate(50.0f);
    return true;
}

void Comms::closeControl(){
    if (control_rxRun_.exchange(false)){
        if (control_rxThread_.joinable()) control_rxThread_.join();
    }
    if (control_sock_ >= 0){
        ::close(control_sock_);
        control_sock_ = -1;
    }
}

void Comms::closeSensor(){
    if (sensor_rxRun_.exchange(false)){
        if (sensor_rxThread_.joinable()) sensor_rxThread_.join();
    }
    if (sensor_sock_ >= 0){
        ::close(sensor_sock_);
        sensor_sock_ = -1;
    }
}

bool Comms::sendLine(int sock, std::mutex& mtx, const std::string& label, const std::string& line){
    if (sock < 0) return false;
    std::string out = line;
    if (out.empty() || out.back() != '\n') out.push_back('\n');

    LOGD(label << " >> \"" << line << "\"");

    std::lock_guard<std::mutex> lk(mtx);
    size_t to_send = out.size();
    size_t sent = 0;
    while (sent < to_send) {
        ssize_t n = ::send(sock, out.data() + sent, to_send - sent, 0);
        if (n < 0) {
            LOGE("send() failed (" << n << "): " << strerror(errno));
            return false;
        }
        sent += n;
    }
    return true;
}

bool Comms::sendPTUVelocity(float pan_sps, float tilt_sps){
    std::ostringstream oss;
    oss << "VP" << int(pan_sps) << "T" << int(tilt_sps);
    return sendLine(control_sock_, control_send_mtx_, "PTU", oss.str());
}

bool Comms::sendWheels(int left_sps, int right_sps){
    std::ostringstream oss;
    oss << "LS" << left_sps << "RS" << right_sps;
    return sendLine(control_sock_, control_send_mtx_, "WHEELS", oss.str());
}

bool Comms::imuOn(){
    return sendLine(sensor_sock_, sensor_send_mtx_, "IMU", "IMU_ON");
}

bool Comms::imuOff(){
    return sendLine(sensor_sock_, sensor_send_mtx_, "IMU", "IMU_OFF");
}

bool Comms::imuRate(float hz){
    std::ostringstream oss;
    oss << "IMU_RATE " << int(hz);
    return sendLine(sensor_sock_, sensor_send_mtx_, "IMU", oss.str());
}

bool Comms::getLatestYPR(float& yaw, float& pitch, float& roll, double& age_ms) const{
    yaw   = imu_yaw_.load();
    pitch = imu_pitch_.load();
    roll  = imu_roll_.load();
    int64_t stamp = imu_stamp_ns_.load();
    if (stamp == 0 || !isfinite(yaw) || !isfinite(pitch) || !isfinite(roll)) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

bool Comms::getLatestGPS(double& lat, double& lon, float& alt, float& speed_knots, float& course_deg, int& quality, int& sats, double& age_ms) const{
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

bool Comms::getLatestEncoders(int& left, int& right, double& age_ms) const{
    EncoderData enc = encoders_.load();
    if (!enc.valid) return false;
    left  = enc.left;
    right = enc.right;
    int64_t stamp = enc_stamp_ns_.load();
    if (stamp == 0) return false;
    age_ms = (now_ns() - stamp) / 1e6;
    return true;
}

float Comms::getLatestDistance(double* age_ms) const{
    float d = tof_dist_.load();
    if (d < 0.0f) return -1.0f;
    int64_t stamp = tof_stamp_ns_.load();
    if (stamp == 0) return -1.0f;
    if (age_ms) *age_ms = (now_ns() - stamp) / 1e6;
    return d;
}

void Comms::controlRxLoop(Comms* self){
    char buf[256];
    std::string partial;
    while (self->control_rxRun_.load()){
        // Inner recv loop — parse ENC lines from control Teensy, discard everything else
        bool connected = true;
        partial.clear();
        while (self->control_rxRun_.load() && connected){
            ssize_t n = ::recv(self->control_sock_, buf, sizeof(buf)-1, 0);
            if (n > 0){
                buf[n] = '\0';
                partial += buf;
                size_t pos;
                while ((pos = partial.find('\n')) != std::string::npos) {
                    std::string line = partial.substr(0, pos);
                    partial.erase(0, pos + 1);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    if (line.rfind("ENC", 0) == 0) {
                        int l, r;
                        if (std::sscanf(line.c_str() + 3, "%d %d", &l, &r) == 2) {
                            EncoderData enc; enc.left = l; enc.right = r; enc.valid = true;
                            self->encoders_.store(enc);
                            self->enc_stamp_ns_.store(now_ns());
                        }
                    }
                    // All other lines (ACKs, debug prints) are silently ignored
                }
            } else if (n == 0){
                LOGW("Control TCP closed by peer");
                connected = false;
            } else {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                LOGE("Control recv() failed: " << strerror(errno));
                connected = false;
            }
        }
        if (!self->control_rxRun_.load()) break;

        // Reconnect with 2 s backoff
        { std::lock_guard<std::mutex> lk(self->control_send_mtx_);
          ::close(self->control_sock_); self->control_sock_ = -1; }
        while (self->control_rxRun_.load()){
            LOGW("Control disconnected — reconnecting in 2 s...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int s = openTcpSocket(self->control_ip_, self->control_port_);
            if (s < 0) continue;
            { std::lock_guard<std::mutex> lk(self->control_send_mtx_);
              self->control_sock_ = s; }
            LOGI("Control reconnected to " << self->control_ip_ << ":" << self->control_port_);
            break;
        }
    }
}

void Comms::sensorRxLoop(Comms* self){
    char buf[4096];

    while (self->sensor_rxRun_.load()){
        // Inner recv loop
        std::string partial;
        bool connected = true;
        while (self->sensor_rxRun_.load() && connected){
        ssize_t n = ::recv(self->sensor_sock_, buf, sizeof(buf), 0);
        if (n > 0){
            partial.append(buf, n);
            size_t pos;
            while ((pos = partial.find('\n')) != std::string::npos){
                std::string line = partial.substr(0, pos);
                partial.erase(0, pos+1);
                // Parse line
                if (line.rfind("YPR", 0) == 0){
                    float y, p, r;
                    if (std::sscanf(line.c_str()+3, "%f %f %f", &y, &p, &r) == 3){
                        self->imu_yaw_.store(y);
                        self->imu_pitch_.store(p);
                        self->imu_roll_.store(r);
                        self->imu_stamp_ns_.store(now_ns());
                    }
                } else if (line.rfind("TOF", 0) == 0){
                    float d;
                    if (std::sscanf(line.c_str()+3, "%f", &d) == 1){
                        self->tof_dist_.store(d);
                        self->tof_stamp_ns_.store(now_ns());
                    }
                } else if (line.rfind("$GPGGA", 0) == 0 || line.rfind("$GNGGA", 0) == 0){
                    // Parse GGA — accept both $GPGGA (GPS-only) and $GNGGA (multi-constellation)
                    size_t star_pos = line.find('*');
                    if (star_pos == std::string::npos) continue;
                    std::string sentence = line.substr(0, star_pos);
                    std::stringstream ss(sentence);
                    std::string token;
                    std::vector<std::string> fields;
                    while (std::getline(ss, token, ',')) {
                        fields.push_back(token);
                    }
                    if (fields.size() != 15 || (fields[0] != "$GPGGA" && fields[0] != "$GNGGA")) continue;

                    std::string lat_str = fields[2];
                    std::string ns = fields[3];
                    std::string lon_str = fields[4];
                    std::string ew = fields[5];
                    int quality = 0;
                    try { quality = std::stoi(fields[6]); } catch (...) { continue; }
                    int sats = 0;
                    try { sats = std::stoi(fields[7]); } catch (...) { continue; }
                    float alt = 0.0f;
                    try { alt = std::stof(fields[9]); } catch (...) { continue; }

                    // Parse lat
                    double lat_deg = 0.0;
                    size_t dot = lat_str.find('.');
                    if (dot != std::string::npos && dot >= 2) {
                        try {
                            int deg = std::stoi(lat_str.substr(0, dot-2));
                            double min = std::stod(lat_str.substr(dot-2));
                            lat_deg = deg + min / 60.0;
                            if (ns == "S") lat_deg = -lat_deg;
                        } catch (...) { continue; }
                    } else continue;

                    // Parse lon
                    double lon_deg = 0.0;
                    dot = lon_str.find('.');
                    if (dot != std::string::npos && dot >= 3) {
                        try {
                            int deg = std::stoi(lon_str.substr(0, dot-2));
                            double min = std::stod(lon_str.substr(dot-2));
                            lon_deg = deg + min / 60.0;
                            if (ew == "W") lon_deg = -lon_deg;
                        } catch (...) { continue; }
                    } else continue;

                    {
                        std::lock_guard<std::mutex> lk(self->gps_mtx_);
                        self->gps_.lat     = lat_deg;
                        self->gps_.lon     = lon_deg;
                        self->gps_.alt     = alt;
                        self->gps_.quality = quality;
                        self->gps_.sats    = sats;
                        self->gps_.valid   = (quality > 0);
                    }
                    self->gps_stamp_ns_.store(now_ns());
                } else if (line.rfind("$GPRMC", 0) == 0 || line.rfind("$GNRMC", 0) == 0){
                    // Parse RMC — accept both $GPRMC (GPS-only) and $GNRMC (multi-constellation)
                    size_t star_pos = line.find('*');
                    if (star_pos == std::string::npos) continue;
                    std::string sentence = line.substr(0, star_pos);
                    std::stringstream ss(sentence);
                    std::string token;
                    std::vector<std::string> fields;
                    while (std::getline(ss, token, ',')) {
                        fields.push_back(token);
                    }
                    if (fields.size() != 13 || (fields[0] != "$GPRMC" && fields[0] != "$GNRMC")) continue;

                    std::string status = fields[2];
                    if (status != "A") continue;

                    float speed = 0.0f;
                    try { speed = std::stof(fields[7]); } catch (...) { continue; }
                    float course = 0.0f;
                    try { course = std::stof(fields[8]); } catch (...) { continue; }

                    {
                        std::lock_guard<std::mutex> lk(self->gps_mtx_);
                        self->gps_.speed_knots = speed;
                        self->gps_.course_deg  = course;
                    }
                    self->gps_stamp_ns_.store(now_ns());
                }
                // ignore other lines (ENC comes from control Teensy, not sensor)
            }
        } else if (n == 0){
            LOGW("Sensor TCP closed by peer");
            connected = false;
        } else {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
            LOGE("Sensor recv() failed: " << strerror(errno) << " (" << errno << ")");
            connected = false;
        }
        } // end inner recv loop

        if (!self->sensor_rxRun_.load()) break;

        // Reconnect with 2 s backoff; re-enable IMU on success
        { std::lock_guard<std::mutex> lk(self->sensor_send_mtx_);
          ::close(self->sensor_sock_); self->sensor_sock_ = -1; }
        while (self->sensor_rxRun_.load()){
            LOGW("Sensor disconnected — reconnecting in 2 s...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int s = openTcpSocket(self->sensor_ip_, self->sensor_port_);
            if (s < 0) continue;
            { std::lock_guard<std::mutex> lk(self->sensor_send_mtx_);
              self->sensor_sock_ = s; }
            LOGI("Sensor reconnected to " << self->sensor_ip_ << ":" << self->sensor_port_);
            self->imuOn();
            self->imuRate(50.0f);
            break;
        }
    } // end outer reconnect loop
}

// Web IPC implementation
bool Comms::startWebIPC(int port) {
    web_sock_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (web_sock_ < 0) { LOGE("Web IPC socket failed"); return false; }

    int opt = 1;
    setsockopt(web_sock_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    sa.sin_addr.s_addr = htonl(INADDR_ANY);  // Accept on all interfaces (Tailscale + LAN)

    if (::bind(web_sock_, (sockaddr*)&sa, sizeof(sa)) < 0) {
        LOGE("Web IPC bind failed: " << strerror(errno));
        ::close(web_sock_); web_sock_ = -1; return false;
    }
    if (::listen(web_sock_, 1) < 0) {
        LOGE("Web IPC listen failed");
        ::close(web_sock_); web_sock_ = -1; return false;
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
                if (std::chrono::duration_cast<std::chrono::milliseconds>(now - lastPush).count() >= 1000) {
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
    // Stop mission thread first
    if (mission_run_.exchange(false))
        if (mission_thread_.joinable()) mission_thread_.join();

    web_rxRun_.store(false);
    if (web_sock_ >= 0) { ::close(web_sock_); web_sock_ = -1; }
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

    constexpr double STALE_THRESHOLD_MS = 2000.0;

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
    j["distance_m"]     = getLatestDistance(&dage);
    j["distance_age_ms"] = dage;
    j["detection_fps"]    = detection_fps_.load();
    j["stream_quality"]   = stream_quality_.load();
    j["target_person_id"] = target_person_id_.load();

    // Control mode
    static const char* modeNames[] = {"follow", "manual", "mission", "stopped"};
    int m = mode_.load();
    j["mode"] = modeNames[std::clamp(m, 0, 3)];

    // Mission status
    static const char* faultNames[] = {"", "gps_lost", "stuck", "wheel_fail"};
    int fault = std::clamp(mission_fault_.load(), 0, 3);

    nlohmann::json ms;
    ms["running"]      = mission_run_.load();
    ms["waypoint_idx"] = mission_wp_idx_.load();
    ms["fault"]        = faultNames[fault];
    {
        std::lock_guard<std::mutex> lk(mission_mtx_);
        if (!mission_json_.empty()) {
            try {
                auto mj = nlohmann::json::parse(mission_json_);
                ms["id"]             = mj.value("id", "");
                ms["name"]           = mj.value("name", "");
                ms["waypoint_count"] = mj.contains("waypoints")
                                       ? (int)mj["waypoints"].size() : 0;
                int idx = mission_wp_idx_.load();
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
    if (cmd == "stop_follow" || cmd == "emergency_stop") {
        if (mission_run_.exchange(false))
            if (mission_thread_.joinable()) mission_thread_.join();
        setMode(ControlMode::STOPPED);
        sendWheels(0, 0);
        sendPTUVelocity(0, 0);
        return "";
    }

    // ── Resume autonomous follow ───────────────────────────────────────────────
    if (cmd == "resume_follow") {
        if (mission_run_.exchange(false))
            if (mission_thread_.joinable()) mission_thread_.join();
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
        sendLine(control_sock_, control_send_mtx_, "PTU", "P0T0");     // hard position centre
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
            saveMission(id, json_str);
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

            // Save to disk so Jetson can reload after restart
            saveMission(id, json_str);

            // Stop any running mission
            if (mission_run_.exchange(false))
                if (mission_thread_.joinable()) mission_thread_.join();

            {
                std::lock_guard<std::mutex> lk(mission_mtx_);
                mission_json_ = json_str;
            }
            mission_wp_idx_.store(0);
            mission_run_.store(true);
            setMode(ControlMode::MISSION);
            mission_thread_ = std::thread(&Comms::missionLoop, this);
            LOGI("Mission started: " << id);
        } catch (const std::exception& e) {
            LOGE("mission_start bad JSON: " << e.what());
        }
        return "";
    }

    // ── Mission abort ─────────────────────────────────────────────────────────
    if (cmd == "mission_abort") {
        if (mission_run_.exchange(false))
            if (mission_thread_.joinable()) mission_thread_.join();
        setMode(ControlMode::STOPPED);
        sendWheels(0, 0);
        sendPTUVelocity(0, 0);
        LOGI("Mission aborted");
        return "";
    }

    // ── Load a saved mission by ID and start it ────────────────────────────────
    if (cmd.rfind("mission_load:", 0) == 0) {
        std::string id = cmd.substr(13);
        // Sanitise id — alphanumeric, underscore, hyphen only
        id.erase(std::remove_if(id.begin(), id.end(),
            [](char c){ return !isalnum(c) && c != '_' && c != '-'; }), id.end());
        std::string path = MISSIONS_DIR + "/" + id + ".json";
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
        return listMissionsJson();
    }

    // ── Skip current waypoint ─────────────────────────────────────────────────
    if (cmd == "mission_skip_wp") {
        if (mission_run_.load()) {
            int nxt = mission_wp_idx_.fetch_add(1) + 1;
            LOGI("Mission: skipping to WP " << nxt);
        }
        return "";
    }

    // ── Resume mission from last saved waypoint ────────────────────────────────
    if (cmd.rfind("mission_resume:", 0) == 0) {
        std::string id = cmd.substr(15);
        id.erase(std::remove_if(id.begin(), id.end(),
            [](char c){ return !isalnum(c) && c != '_' && c != '-'; }), id.end());
        std::string path = MISSIONS_DIR + "/" + id + ".json";
        std::ifstream f(path);
        if (!f.is_open()) { LOGE("mission_resume: not found: " << id); return ""; }
        std::string json_str((std::istreambuf_iterator<char>(f)),
                              std::istreambuf_iterator<char>());
        int saved_idx = loadMissionState(id);
        if (mission_run_.exchange(false))
            if (mission_thread_.joinable()) mission_thread_.join();
        {
            std::lock_guard<std::mutex> lk(mission_mtx_);
            mission_json_ = json_str;
        }
        mission_wp_idx_.store(saved_idx);
        mission_fault_.store(0);
        mission_run_.store(true);
        setMode(ControlMode::MISSION);
        mission_thread_ = std::thread(&Comms::missionLoop, this);
        LOGI("Mission resumed at WP " << saved_idx << ": " << id);
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

    // ── Stream quality: stream_quality:<1-100> ────────────────────────────────
    if (cmd.rfind("stream_quality:", 0) == 0) {
        try {
            int q = std::stoi(cmd.substr(15));
            q = std::clamp(q, 1, 100);
            stream_quality_.store(q);
            LOGI("Stream quality set to " << q);
        } catch (...) {}
        return "";
    }

    LOGW("Unknown web command: " << cmd);
    return "";
}

// ── Mission persistence ────────────────────────────────────────────────────────

void Comms::saveMission(const std::string& id, const std::string& json_str) {
    ::mkdir(MISSIONS_DIR.c_str(), 0755);
    std::string path = MISSIONS_DIR + "/" + id + ".json";
    std::ofstream f(path);
    if (f.is_open()) {
        f << json_str;
        LOGI("Mission saved: " << path);
    } else {
        LOGE("Mission save failed: " << path);
    }
}

std::string Comms::listMissionsJson() const {
    nlohmann::json list = nlohmann::json::array();
    DIR* dir = opendir(MISSIONS_DIR.c_str());
    if (dir) {
        struct dirent* ent;
        while ((ent = readdir(dir)) != nullptr) {
            std::string fname = ent->d_name;
            if (fname.size() < 5 || fname.substr(fname.size()-5) != ".json") continue;
            std::string path = MISSIONS_DIR + "/" + fname;
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

// ── Mission state persistence ──────────────────────────────────────────────────

void Comms::saveMissionState(int wp_idx) {
    nlohmann::json state;
    {
        std::lock_guard<std::mutex> lk(mission_mtx_);
        state["mission_json"] = mission_json_;
    }
    state["waypoint_idx"] = wp_idx;
    std::string path = MISSIONS_DIR + "/_state.json";
    std::ofstream f(path);
    if (f.is_open()) f << state.dump();
}

int Comms::loadMissionState(const std::string& target_id) {
    std::string path = MISSIONS_DIR + "/_state.json";
    std::ifstream f(path);
    if (!f.is_open()) return 0;
    try {
        nlohmann::json state = nlohmann::json::parse(f);
        std::string json_str = state.value("mission_json", "");
        if (json_str.empty()) return 0;
        auto mj = nlohmann::json::parse(json_str);
        if (mj.value("id", "") != target_id) return 0;   // different mission
        return state.value("waypoint_idx", 0);
    } catch (...) { return 0; }
}

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

// ── Mission executor thread ────────────────────────────────────────────────────
//
// Hardened for real-world use:
//  - Stops wheels immediately if GPS is lost; aborts after 30s
//  - Detects physical "stuck" via GPS position progress every 30s
//  - Calibrates heading: uses GPS course when moving, IMU+offset when slow
//  - Checks sendWheels() return value; sets WHEEL fault if control link is down
//  - Implements loop, returnToStart, and all arrival actions
//  - Persists waypoint index to disk so mission can be resumed after Jetson reboot
//
void Comms::missionLoop(Comms* self) {
    LOGI("Mission executor started");

    // ── Parse mission JSON ────────────────────────────────────────────────────
    // Snapshot the JSON string under the lock so a concurrent mission_start /
    // mission_resume command cannot modify mission_json_ while we are parsing
    // or iterating waypoints.
    std::string json_copy;
    { std::lock_guard<std::mutex> lk(self->mission_mtx_); json_copy = self->mission_json_; }
    nlohmann::json mission;
    try { mission = nlohmann::json::parse(json_copy); }
    catch (const std::exception& e) {
        LOGE("Mission: bad JSON: " << e.what());
        self->mission_run_.store(false); self->setMode(ControlMode::STOPPED); return;
    }
    if (!mission.contains("waypoints") || !mission["waypoints"].is_array()
        || mission["waypoints"].empty()) {
        LOGE("Mission: no waypoints");
        self->mission_run_.store(false); self->setMode(ControlMode::STOPPED); return;
    }

    // ── Validate all waypoint coordinates ─────────────────────────────────────
    auto& waypoints = mission["waypoints"];
    int n = (int)waypoints.size();
    for (int i = 0; i < n; i++) {
        double lat = waypoints[i].value("lat", 0.0);
        double lon = waypoints[i].value("lng", waypoints[i].value("lon", 0.0));
        if (std::abs(lat) < 0.001 && std::abs(lon) < 0.001) {
            LOGE("Mission: WP" << i << " has invalid coordinates (0,0) — aborting");
            self->mission_run_.store(false); self->setMode(ControlMode::STOPPED); return;
        }
    }

    bool do_loop   = mission.value("loop", false);
    bool do_return = mission.value("returnToStart", false);

    // ── Heading calibration state (persists across waypoints) ─────────────────
    // We calibrate IMU yaw against GPS course when moving, then use the offset
    // when stationary (GPS course is junk below ~1 km/h).
    double heading_offset     = 0.0;   // gps_course - imu_yaw when moving
    bool   heading_calibrated = false;

    // ── Helper: navigate to a single lat/lon ──────────────────────────────────
    // Returns true=arrived, false=timeout/abort/fault.
    // Mutates heading_offset/heading_calibrated as the robot moves.
    auto navigateTo = [&](double wp_lat, double wp_lon,
                          int wp_speed_sps, float arrive_r,
                          double timeout_s) -> bool
    {
        constexpr double GPS_LOSS_ABORT_S  = 30.0;
        constexpr double STUCK_CHECK_S     = 30.0;
        constexpr double STUCK_MIN_MOVE_M  = 0.5;
        constexpr double STEER_GAIN        = 50.0;  // SPS per degree of heading error
        constexpr double MAX_STEER_FRAC    = 0.6;
        constexpr double MOVING_KNOTS      = 0.5;   // speed threshold for GPS course use

        auto t_start     = Clock::now();
        auto gps_ok_last = Clock::now();
        auto stuck_t     = Clock::now();
        double stuck_ref_lat = wp_lat, stuck_ref_lon = wp_lon;
        bool   stuck_ref_init = false;

        while (self->mission_run_.load()) {
            // ── GPS quality check ────────────────────────────────────────────
            double lat, lon; float alt, spd, crs; int qual, sats; double gps_age;
            bool have_gps = self->getLatestGPS(lat, lon, alt, spd, crs, qual, sats, gps_age);

            if (!have_gps || gps_age > 2000.0 || qual < 1) {
                double down_s = std::chrono::duration<double>(Clock::now() - gps_ok_last).count();
                self->mission_fault_.store((int)MissionFault::GPS_LOST);
                self->sendWheels(0, 0);   // stop while GPS is out — never run blind
                if (down_s >= GPS_LOSS_ABORT_S) {
                    LOGE("Mission: GPS down " << (int)down_s << "s — aborting");
                    self->mission_run_.store(false);
                    self->setMode(ControlMode::STOPPED);
                    return false;
                }
                LOGW("Mission: GPS lost (" << (int)down_s << "/" << (int)GPS_LOSS_ABORT_S << "s)");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            gps_ok_last = Clock::now();
            self->mission_fault_.store((int)MissionFault::NONE);

            if (!stuck_ref_init) {
                stuck_ref_lat = lat; stuck_ref_lon = lon;
                stuck_ref_init = true;
            }

            // ── Arrived? ────────────────────────────────────────────────────
            double dist   = haversineM(lat, lon, wp_lat, wp_lon);
            double target = bearingDeg(lat, lon, wp_lat, wp_lon);
            if (dist <= arrive_r) return true;

            // ── Nav timeout ─────────────────────────────────────────────────
            double elapsed_s = std::chrono::duration<double>(Clock::now() - t_start).count();
            if (elapsed_s > timeout_s) {
                LOGW("Mission: nav timeout (" << (int)timeout_s << "s) — skipping WP");
                return false;
            }

            // ── Stuck detection ──────────────────────────────────────────────
            double stuck_elapsed = std::chrono::duration<double>(Clock::now() - stuck_t).count();
            if (stuck_elapsed >= STUCK_CHECK_S && wp_speed_sps > 500) {
                double moved = haversineM(stuck_ref_lat, stuck_ref_lon, lat, lon);
                if (moved < STUCK_MIN_MOVE_M) {
                    LOGE("Mission: STUCK — only moved " << moved << "m in "
                         << (int)STUCK_CHECK_S << "s");
                    self->mission_fault_.store((int)MissionFault::STUCK);
                    self->sendWheels(0, 0);
                    self->mission_run_.store(false);
                    self->setMode(ControlMode::STOPPED);
                    return false;
                }
                stuck_ref_lat = lat; stuck_ref_lon = lon;
                stuck_t = Clock::now();
            }

            // ── Calibrate heading: GPS course vs IMU yaw ────────────────────
            // GPS course is the true direction of travel but meaningless below ~1 km/h.
            // We learn the IMU→GPS offset while moving and apply it when stopped.
            if (spd > MOVING_KNOTS) {
                float yaw, pitch, roll; double imu_age;
                if (self->getLatestYPR(yaw, pitch, roll, imu_age) && imu_age < 500.0) {
                    double raw_offset = std::fmod(crs - yaw + 360.0, 360.0);
                    if (!heading_calibrated) {
                        heading_offset = raw_offset;
                        heading_calibrated = true;
                    } else {
                        // Low-pass filter to dampen sensor noise
                        heading_offset = heading_offset * 0.95 + raw_offset * 0.05;
                    }
                }
            }

            double heading;
            if (spd > MOVING_KNOTS) {
                heading = std::fmod(crs + 360.0, 360.0);
            } else if (heading_calibrated) {
                float yaw, pitch, roll; double imu_age;
                if (self->getLatestYPR(yaw, pitch, roll, imu_age) && imu_age < 500.0)
                    heading = std::fmod(yaw + heading_offset + 360.0, 360.0);
                else
                    heading = std::fmod(crs + 360.0, 360.0);
            } else {
                heading = std::fmod(crs + 360.0, 360.0);
            }

            // ── Proportional differential steering ──────────────────────────
            double herr = target - heading;
            while (herr >  180.0) herr -= 360.0;
            while (herr < -180.0) herr += 360.0;
            double steer = std::clamp(herr * STEER_GAIN,
                                      -wp_speed_sps * MAX_STEER_FRAC,
                                       wp_speed_sps * MAX_STEER_FRAC);
            int L = std::clamp((int)(wp_speed_sps - steer), -WHEEL_MAX_SPS, WHEEL_MAX_SPS);
            int R = std::clamp((int)(wp_speed_sps + steer), -WHEEL_MAX_SPS, WHEEL_MAX_SPS);

            // ── Check wheel command delivery ─────────────────────────────────
            if (!self->sendWheels(L, R)) {
                self->mission_fault_.store((int)MissionFault::WHEEL);
                LOGW("Mission: wheel send failed — control link down? Waiting...");
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            self->mission_fault_.store((int)MissionFault::NONE);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;  // mission_run_ was cleared externally
    };

    // ─────────────────────────────────────────────────────────────────────────
    // Main mission loop (outer = loop support, inner = per-waypoint)
    // ─────────────────────────────────────────────────────────────────────────
    do {
        int n_wps = (int)waypoints.size();
        while (self->mission_run_.load()) {
            int idx = self->mission_wp_idx_.load();
            if (idx >= n_wps) break;  // all waypoints done

            auto& wp = waypoints[idx];
            double wp_lat   = wp.value("lat", 0.0);
            double wp_lon   = wp.value("lng", wp.value("lon", 0.0));
            float  spd_pct  = wp.value("speed", 60.0f);   // UI sends 0-100 %
            int    wp_sps   = std::clamp((int)(spd_pct / 100.0f * WHEEL_MAX_SPS),
                                         100, WHEEL_MAX_SPS);
            float  radius   = wp.value("arrivalRadius", wp.value("radius_m", 3.0f));
            std::string action = wp.value("arrivalAction", wp.value("action", "continue"));

            LOGI("Mission: WP " << idx << "/" << (n_wps-1)
                 << " action=" << action
                 << " lat=" << wp_lat << " lon=" << wp_lon);

            // ── Navigate to this waypoint ────────────────────────────────────
            bool arrived = navigateTo(wp_lat, wp_lon, wp_sps, radius, 120.0);
            self->sendWheels(0, 0);

            if (!self->mission_run_.load()) break;  // aborted during nav

            // ── Arrival action ───────────────────────────────────────────────
            if (arrived) {
                if (action == "wait" || action == "stop") {
                    float wait_s = wp.value("waitSeconds", wp.value("dwell_s", 5.0f));
                    LOGI("Mission: WP" << idx << " wait " << wait_s << "s");
                    float el = 0.0f;
                    while (self->mission_run_.load() && el < wait_s)
                        { std::this_thread::sleep_for(std::chrono::milliseconds(100)); el += 0.1f; }

                } else if (action == "scan") {
                    float from = wp.value("scanFrom", -90.0f);
                    float to   = wp.value("scanTo",    90.0f);
                    float sspd = wp.value("scanSpeed",  200.0f);
                    // Time-based PTU sweep (tune MS_PER_DEG for your PTU gear ratio)
                    constexpr float MS_PER_DEG = 10.0f;
                    float sweep_ms = std::abs(to - from) * MS_PER_DEG;
                    float dir = (to > from) ? 1.0f : -1.0f;
                    LOGI("Mission: PTU scan " << from << "° to " << to << "°");
                    self->sendPTUVelocity(dir * sspd, 0);
                    auto ts = Clock::now();
                    while (self->mission_run_.load()) {
                        if (std::chrono::duration<float,std::milli>(Clock::now()-ts).count() >= sweep_ms) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    // Return to centre
                    self->sendPTUVelocity(-dir * sspd, 0);
                    ts = Clock::now();
                    while (self->mission_run_.load()) {
                        if (std::chrono::duration<float,std::milli>(Clock::now()-ts).count() >= sweep_ms/2) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(50));
                    }
                    self->sendPTUVelocity(0, 0);

                } else if (action == "set_ptu") {
                    float pan  = wp.value("ptuPan",  0.0f);
                    float tilt = wp.value("ptuTilt", 0.0f);
                    LOGI("Mission: set PTU pan=" << pan << " tilt=" << tilt);
                    self->sendPTUVelocity(pan, tilt);

                } else if (action == "follow") {
                    float dur = wp.value("followDuration", 0.0f);
                    LOGI("Mission: follow mode for " << (dur > 0 ? std::to_string((int)dur)+"s" : "unlimited"));
                    self->setMode(ControlMode::FOLLOW);
                    if (dur > 0.0f) {
                        float el = 0.0f;
                        while (self->mission_run_.load() && el < dur)
                            { std::this_thread::sleep_for(std::chrono::milliseconds(100)); el += 0.1f; }
                        self->setMode(ControlMode::MISSION);
                    } else {
                        // Unlimited follow — mission thread exits, Movement takes over
                        self->mission_run_.store(false);
                        LOGI("Mission executor yielding to FOLLOW mode");
                        return;
                    }

                } else if (action == "custom") {
                    std::string ccmd = wp.value("customCommand", "");
                    if (!ccmd.empty()) {
                        LOGI("Mission: custom cmd: " << ccmd);
                        self->handleWebCommand(ccmd);
                    }
                }
                // "continue" — fall through, advance immediately
            }

            // ── Advance waypoint + persist state ────────────────────────────
            int next_idx = idx + 1;
            self->mission_wp_idx_.store(next_idx);
            self->saveMissionState(next_idx);
        }

        if (!self->mission_run_.load()) break;  // aborted

        // ── Return to start? ─────────────────────────────────────────────────
        if (do_return && n > 0) {
            LOGI("Mission: returning to start");
            auto& sp    = waypoints[0];
            double slat = sp.value("lat", 0.0);
            double slon = sp.value("lng", sp.value("lon", 0.0));
            float  srad = sp.value("arrivalRadius", 3.0f);
            navigateTo(slat, slon, WHEEL_MAX_SPS / 3, srad, 120.0);
            self->sendWheels(0, 0);
        }

        if (!self->mission_run_.load()) break;

        // ── Loop? ────────────────────────────────────────────────────────────
        if (do_loop) {
            LOGI("Mission: looping back to WP 0");
            self->mission_wp_idx_.store(0);
            self->saveMissionState(0);
            // continue outer do-while
        }

    } while (do_loop && self->mission_run_.load());

    // ── Complete ──────────────────────────────────────────────────────────────
    bool natural_completion = self->mission_run_.load();
    if (natural_completion) {
        LOGI("Mission: complete");
        self->sendWheels(0, 0);
        self->sendPTUVelocity(0, 0);
        self->mission_run_.store(false);
        self->setMode(ControlMode::STOPPED);
        // Clear saved state so a stale resume doesn't restart a completed mission
        std::remove((MISSIONS_DIR + "/_state.json").c_str());
    }

    // ── Fire mission event to VPS ─────────────────────────────────────────────
    {
        std::string mission_id, mission_name;
        {
            std::lock_guard<std::mutex> lk(self->mission_mtx_);
            try {
                auto m = nlohmann::json::parse(self->mission_json_);
                mission_id   = m.value("id",   "");
                mission_name = m.value("name", "");
            } catch (...) {}
        }

        nlohmann::json ev;
        ev["type"] = "event";
        ev["id"]   = mission_id;
        ev["name"] = mission_name;

        if (natural_completion) {
            ev["event"] = "mission_completed";
            LOGI("Mission event: completed — " << mission_name);
        } else {
            int fault = self->mission_fault_.load();
            if (fault != (int)MissionFault::NONE) {
                // Faulted (GPS/stuck/wheel) — send dedicated event so the UI can keep
                // the fault banner visible rather than clearing it silently
                static const char* faultNames[] = {"", "gps_lost", "stuck", "wheel_fail"};
                ev["event"] = "mission_faulted";
                ev["fault"] = faultNames[std::clamp(fault, 0, 3)];
                LOGI("Mission event: faulted (" << ev["fault"] << ") — " << mission_name);
            }
            // If fault==NONE the mission was aborted by command; server.js already
            // broadcast that event from the /api/missions/abort endpoint, so no
            // duplicate event is sent here.
        }

        if (ev.contains("event"))
            self->sendWebEvent(ev.dump());
    }

    LOGI("Mission executor finished");
}