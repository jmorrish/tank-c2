#pragma once
#include <thread>
#include <atomic>

class SensorStore;

// Periodically reads Jetson system stats (CPU/GPU temp, CPU/GPU usage, RAM)
// and pushes them into SensorStore generic slots.
class SystemMonitor {
public:
    explicit SystemMonitor(SensorStore& sensors);
    ~SystemMonitor();
    void start();
    void stop();
private:
    void loop();
    SensorStore& sensors_;
    std::thread th_;
    std::atomic<bool> run_{false};

    // CPU usage tracking (delta between reads)
    long long prev_total_ = 0;
    long long prev_idle_  = 0;
};
