#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <functional>

class SensorStore;  // fwd

// TCP client for slam_bridge.py.
// RX thread reads newline-delimited JSON (scan_full, slam_pose, slam_map).
// TX thread sends encoder odometry at 10 Hz.
class SlamLink {
public:
    // Called for each received JSON line.  Caller parses and routes.
    using LineCb = std::function<void(const std::string& line)>;

    SlamLink() = default;
    ~SlamLink() { stop(); }

    // Start RX + TX threads.  sensors is used to read encoder data for TX
    // and to write lidar/pose data from RX via the lineCb.
    bool start(const std::string& host, int port,
               SensorStore& sensors, LineCb lineCb);
    void stop();

    bool running() const { return run_.load(); }

    // Send a JSON line to slam_bridge (thread-safe)
    bool sendLine(const std::string& json_str);

private:
    static int openTcpSocket(const std::string& ip, int port);

    std::atomic<bool> run_{false};

    int               sock_{-1};
    mutable std::mutex sock_mtx_;

    std::thread rx_thread_;
    std::thread tx_thread_;
};
