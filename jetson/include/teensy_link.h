#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>
#include "helpers.h"  // SocketFd

// TCP client with auto-reconnect for Teensy communication.
// Runs an rx thread that reads newline-delimited lines and calls a
// user-supplied callback for each one.  Provides thread-safe sendLine().
class TeensyLink {
public:
    // lineCb is called for every complete line received (no trailing \n or \r).
    using LineCb = std::function<void(const std::string& line)>;

    TeensyLink() = default;
    ~TeensyLink() { close(); }

    // Connect to ip:port and start the rx thread.  label is used in log messages.
    // onConnect (optional) is called after each successful (re)connect.
    bool open(const std::string& label,
              const std::string& ip, int port,
              LineCb lineCb,
              std::function<void()> onConnect = nullptr);
    void close();

    bool connected() const { return sock_.valid(); }

    // Thread-safe send.  Appends '\n' if missing.
    bool sendLine(const std::string& line);

private:
    static int openTcpSocket(const std::string& ip, int port);
    void rxLoop();

    std::string label_;
    std::string ip_;
    int         port_ = 0;

    SocketFd    sock_;
    std::mutex  send_mtx_;

    std::thread       rx_thread_;
    std::atomic<bool> rx_run_{false};

    LineCb                  line_cb_;
    std::function<void()>   on_connect_;
};
