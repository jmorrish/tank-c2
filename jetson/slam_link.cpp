#include "slam_link.h"
#include "sensor_store.h"
#include "helpers.h"

#include <nlohmann/json.hpp>

#include <cstring>
#include <cerrno>
#include <chrono>

// POSIX
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// ── Static: open a TCP socket with 2-second connect timeout ─────────────────

int SlamLink::openTcpSocket(const std::string& ip, int port) {
    int s = ::socket(AF_INET, SOCK_STREAM, 0);
    if (s < 0) return -1;

    sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (::inet_pton(AF_INET, ip.c_str(), &sa.sin_addr) <= 0) { ::close(s); return -1; }

    int flags = ::fcntl(s, F_GETFL, 0);
    ::fcntl(s, F_SETFL, flags | O_NONBLOCK);

    int rc = ::connect(s, (sockaddr*)&sa, sizeof(sa));
    if (rc < 0 && errno != EINPROGRESS) { ::close(s); return -1; }

    if (rc != 0) {
        fd_set wset;
        FD_ZERO(&wset); FD_SET(s, &wset);
        struct timeval tv{2, 0};
        int sel = ::select(s + 1, nullptr, &wset, nullptr, &tv);
        if (sel <= 0) { ::close(s); return -1; }
        int err = 0; socklen_t len = sizeof(err);
        ::getsockopt(s, SOL_SOCKET, SO_ERROR, &err, &len);
        if (err) { ::close(s); return -1; }
    }

    ::fcntl(s, F_SETFL, flags);  // restore blocking

    struct timeval tv{0, 200000};
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int flag = 1;
    setsockopt(s, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    return s;
}

// ── Public API ───────────────────────────────────────────────────────────────

bool SlamLink::start(const std::string& host, int port,
                     SensorStore& sensors, LineCb lineCb) {
    if (run_.exchange(true)) return false;  // already running

    // RX thread: connects to slam_bridge.py, reads newline-delimited JSON
    rx_thread_ = std::thread([this, host, port, lineCb]() {
        while (run_.load()) {
            int s = openTcpSocket(host, port);
            if (s < 0) {
                LOGW("SLAM bridge not reachable on " << host << ":" << port << " — retrying in 2s");
                std::this_thread::sleep_for(std::chrono::seconds(2));
                continue;
            }
            { std::lock_guard<std::mutex> lk(sock_mtx_); sock_ = s; }
            LOGI("SLAM bridge connected on " << host << ":" << port);

            std::string buf;
            char tmp[4096];
            while (run_.load()) {
                ssize_t n = ::recv(s, tmp, sizeof(tmp)-1, 0);
                if (n < 0) {
                    if (errno == EAGAIN || errno == EWOULDBLOCK) continue;
                    break;
                }
                if (n == 0) break;
                tmp[n] = '\0';
                buf += tmp;
                size_t pos;
                while ((pos = buf.find('\n')) != std::string::npos) {
                    std::string line = buf.substr(0, pos);
                    buf.erase(0, pos + 1);
                    if (!line.empty()) {
                        try { if (lineCb) lineCb(line); }
                        catch(...) { LOGW("SLAM bridge: bad JSON: " << line.substr(0,80)); }
                    }
                }
            }

            ::close(s);
            { std::lock_guard<std::mutex> lk(sock_mtx_); sock_ = -1; }
            // clear stale obstacle on disconnect
            // (caller should handle via lineCb or separate callback if needed)
            if (run_.load())
                LOGW("SLAM bridge disconnected — retrying in 2s");
            std::this_thread::sleep_for(std::chrono::seconds(2));
        }
    });

    // TX thread: sends encoder data every 100ms when fresh
    tx_thread_ = std::thread([this, &sensors]() {
        while (run_.load()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            int sock;
            { std::lock_guard<std::mutex> lk(sock_mtx_); sock = sock_; }
            if (sock < 0) continue;
            int left, right; double age_ms;
            if (!sensors.getLatestEncoders(left, right, age_ms) || age_ms > 200.0) continue;
            nlohmann::json j;
            j["type"]   = "odom_data";
            j["left"]   = left;
            j["right"]  = right;
            j["age_ms"] = age_ms;
            std::string line = j.dump() + "\n";
            ::send(sock, line.c_str(), line.size(), MSG_NOSIGNAL);
        }
    });

    return true;
}

bool SlamLink::sendLine(const std::string& json_str) {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    if (sock_ < 0) return false;
    std::string line = json_str + "\n";
    ssize_t sent = ::send(sock_, line.c_str(), line.size(), MSG_NOSIGNAL);
    return sent == (ssize_t)line.size();
}

void SlamLink::stop() {
    run_.store(false);
    { std::lock_guard<std::mutex> lk(sock_mtx_);
      if (sock_ >= 0) { ::shutdown(sock_, SHUT_RDWR); } }
    if (rx_thread_.joinable()) rx_thread_.join();
    if (tx_thread_.joinable()) tx_thread_.join();
    { std::lock_guard<std::mutex> lk(sock_mtx_);
      if (sock_ >= 0) { ::close(sock_); sock_ = -1; } }
}
