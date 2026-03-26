#include "teensy_link.h"
#include "helpers.h"

#include <cstring>
#include <cerrno>
#include <chrono>
#include <sstream>

// POSIX
#include <unistd.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>

// ── Static: open a TCP socket with 2-second connect timeout + TCP_NODELAY ────

int TeensyLink::openTcpSocket(const std::string& ip, int port) {
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

bool TeensyLink::open(const std::string& label,
                      const std::string& ip, int port,
                      LineCb lineCb,
                      std::function<void()> onConnect) {
    close();  // reap any previous connection

    label_      = label;
    ip_         = ip;
    port_       = port;
    line_cb_    = std::move(lineCb);
    on_connect_ = std::move(onConnect);

    int s = openTcpSocket(ip, port);
    if (s < 0) {
        LOGE(label_ << ": connect failed to " << ip << ":" << port << " errno=" << errno);
        return false;
    }
    sock_ = SocketFd(s);
    LOGI(label_ << ": connected to " << ip << ":" << port);

    rx_run_.store(true);
    rx_thread_ = std::thread(&TeensyLink::rxLoop, this);

    if (on_connect_) on_connect_();
    return true;
}

void TeensyLink::close() {
    if (rx_run_.exchange(false)) {
        if (rx_thread_.joinable()) rx_thread_.join();
    }
    sock_.close();
}

bool TeensyLink::sendLine(const std::string& line) {
    if (!sock_.valid()) return false;
    std::string out = line;
    if (out.empty() || out.back() != '\n') out.push_back('\n');

    LOGD(label_ << " >> \"" << line << "\"");

    std::lock_guard<std::mutex> lk(send_mtx_);
    size_t to_send = out.size();
    size_t sent = 0;
    while (sent < to_send) {
        ssize_t n = ::send(sock_, out.data() + sent, to_send - sent, 0);
        if (n < 0) {
            LOGE(label_ << ": send() failed: " << strerror(errno));
            return false;
        }
        sent += n;
    }
    return true;
}

// ── Rx thread: read lines, reconnect on failure ─────────────────────────────

void TeensyLink::rxLoop() {
    char buf[4096];

    while (rx_run_.load()) {
        std::string partial;
        bool connected = true;

        while (rx_run_.load() && connected) {
            ssize_t n = ::recv(sock_, buf, sizeof(buf) - 1, 0);
            if (n > 0) {
                buf[n] = '\0';
                partial += buf;
                size_t pos;
                while ((pos = partial.find('\n')) != std::string::npos) {
                    std::string line = partial.substr(0, pos);
                    partial.erase(0, pos + 1);
                    if (!line.empty() && line.back() == '\r') line.pop_back();
                    if (line_cb_) line_cb_(line);
                }
            } else if (n == 0) {
                LOGW(label_ << ": TCP closed by peer");
                connected = false;
            } else {
                if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) continue;
                LOGE(label_ << ": recv() failed: " << strerror(errno));
                connected = false;
            }
        }
        if (!rx_run_.load()) break;

        // Reconnect with 2s backoff
        { std::lock_guard<std::mutex> lk(send_mtx_);
          sock_.close(); }
        while (rx_run_.load()) {
            LOGW(label_ << ": disconnected — reconnecting in 2s...");
            std::this_thread::sleep_for(std::chrono::seconds(2));
            int s = openTcpSocket(ip_, port_);
            if (s < 0) continue;
            { std::lock_guard<std::mutex> lk(send_mtx_);
              sock_ = SocketFd(s); }
            LOGI(label_ << ": reconnected to " << ip_ << ":" << port_);
            if (on_connect_) on_connect_();
            break;
        }
    }
}
