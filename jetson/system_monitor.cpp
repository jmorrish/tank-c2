#include "system_monitor.h"
#include "sensor_store.h"
#include "helpers.h"
#include <fstream>
#include <sstream>
#include <string>
#include <chrono>

// Read a single-line file and return its content (empty on failure).
static std::string readFile(const char* path) {
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string line;
    std::getline(f, line);
    return line;
}

// Read an integer from a sysfs file; returns -1 on failure.
static long long readInt(const char* path) {
    auto s = readFile(path);
    if (s.empty()) return -1;
    try { return std::stoll(s); } catch (...) { return -1; }
}

SystemMonitor::SystemMonitor(SensorStore& sensors) : sensors_(sensors) {}

SystemMonitor::~SystemMonitor() { stop(); }

void SystemMonitor::start() {
    if (run_.load()) return;
    run_.store(true);
    th_ = std::thread(&SystemMonitor::loop, this);
}

void SystemMonitor::stop() {
    run_.store(false);
    if (th_.joinable()) th_.join();
}

void SystemMonitor::loop() {
    while (run_.load()) {
        // ── CPU temperature (thermal_zone0 = cpu-thermal) ──
        long long cpu_temp_raw = readInt("/sys/devices/virtual/thermal/thermal_zone0/temp");
        if (cpu_temp_raw >= 0)
            sensors_.setGeneric("cpu_temp_c", cpu_temp_raw / 1000.0f);

        // ── GPU temperature (thermal_zone1 = gpu-thermal) ──
        long long gpu_temp_raw = readInt("/sys/devices/virtual/thermal/thermal_zone1/temp");
        if (gpu_temp_raw >= 0)
            sensors_.setGeneric("gpu_temp_c", gpu_temp_raw / 1000.0f);

        // ── GPU load (0–1000 → 0–100%) ──
        long long gpu_load = readInt("/sys/devices/platform/gpu.0/load");
        if (gpu_load >= 0)
            sensors_.setGeneric("gpu_usage_pct", gpu_load / 10.0f);

        // ── CPU usage (from /proc/stat, first line = aggregate) ──
        {
            std::ifstream f("/proc/stat");
            if (f.is_open()) {
                std::string line;
                std::getline(f, line);
                // Format: cpu  user nice system idle iowait irq softirq steal ...
                std::istringstream iss(line);
                std::string cpu_label;
                iss >> cpu_label;
                long long user, nice, system, idle, iowait, irq, softirq, steal;
                if (iss >> user >> nice >> system >> idle >> iowait >> irq >> softirq >> steal) {
                    long long total = user + nice + system + idle + iowait + irq + softirq + steal;
                    long long idle_all = idle + iowait;
                    if (prev_total_ > 0) {
                        long long d_total = total - prev_total_;
                        long long d_idle  = idle_all - prev_idle_;
                        if (d_total > 0) {
                            float usage = 100.0f * (1.0f - (float)d_idle / (float)d_total);
                            sensors_.setGeneric("cpu_usage_pct", usage);
                        }
                    }
                    prev_total_ = total;
                    prev_idle_  = idle_all;
                }
            }
        }

        // ── RAM usage (from /proc/meminfo) ──
        {
            std::ifstream f("/proc/meminfo");
            if (f.is_open()) {
                long long mem_total = 0, mem_available = 0;
                std::string line;
                while (std::getline(f, line)) {
                    if (line.rfind("MemTotal:", 0) == 0)
                        std::sscanf(line.c_str(), "MemTotal: %lld", &mem_total);
                    else if (line.rfind("MemAvailable:", 0) == 0)
                        std::sscanf(line.c_str(), "MemAvailable: %lld", &mem_available);
                }
                if (mem_total > 0) {
                    float used_mb = (mem_total - mem_available) / 1024.0f;
                    float total_mb = mem_total / 1024.0f;
                    sensors_.setGeneric("ram_used_mb", used_mb);
                    sensors_.setGeneric("ram_total_mb", total_mb);
                    sensors_.setGeneric("ram_usage_pct", 100.0f * used_mb / total_mb);
                }
            }
        }

        // Sample every 2 seconds — sufficient for monitoring, light on I/O
        for (int i = 0; i < 20 && run_.load(); ++i)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}
