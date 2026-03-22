#pragma once
#include <mutex>
#include <optional>
#include <atomic>
#include <chrono>
#include <iostream>
#include <condition_variable>
#include <queue>
#include <vector>
#include <unistd.h>
#include <Eigen/Dense>

#define LOGI(msg) std::cout << "[INFO] " << msg << std::endl
#define LOGW(msg) std::cout << "[WARN] " << msg << std::endl
#define LOGE(msg) std::cerr << "[ERROR] " << msg << std::endl
#define LOGD(msg) std::cout << "[DBG] "  << msg << std::endl

// -------------------------------
// Store the latest value (no backlog), thread-safe.
// -------------------------------
template <typename T>
class AtomicLatest {
public:
    void set(const T& v) {
        std::lock_guard<std::mutex> lk(m_);
        v_ = v;
        has_ = true;
    }
    std::optional<T> get() const {
        std::lock_guard<std::mutex> lk(m_);
        if (!has_) return std::nullopt;
        return v_;
    }
    bool has() const { std::lock_guard<std::mutex> lk(m_); return has_; }
private:
    mutable std::mutex m_;
    T v_{};
    bool has_ = false;
};

// -------------------------------
// Optional queue if you want history
// -------------------------------
template <typename T>
class ThreadSafeQueue {
public:
    void push(const T& v){
        std::lock_guard<std::mutex> lk(m_);
        q_.push(v);
        cv_.notify_one();
    }
    bool try_pop(T& out){
        std::lock_guard<std::mutex> lk(m_);
        if(q_.empty()) return false;
        out = std::move(q_.front());
        q_.pop();
        return true;
    }
    T wait_and_pop(){
        std::unique_lock<std::mutex> lk(m_);
        cv_.wait(lk, [&]{ return !q_.empty(); });
        T v = std::move(q_.front());
        q_.pop();
        return v;
    }
private:
    std::queue<T> q_;
    mutable std::mutex m_;
    std::condition_variable cv_;
};

// -------------------------------
// Simple PID controller
// -------------------------------
class PIDController {
public:
    PIDController() = default;
    PIDController(float kp, float ki, float kd, float max_i)
    : kp_(kp), ki_(ki), kd_(kd), max_i_(max_i) {}

    void configure(float kp, float ki, float kd, float max_i){
        kp_ = kp; ki_ = ki; kd_ = kd; max_i_ = max_i;
        reset();
    }

    float step(float error, float dt){
        float P = kp_ * error;
        integral_ += error * dt;
        if (integral_ >  max_i_) integral_ =  max_i_;
        if (integral_ < -max_i_) integral_ = -max_i_;
        float I = ki_ * integral_;
        float derivative = (dt > 1e-6f) ? (error - prev_error_) / dt : 0.0f;
        float D = kd_ * derivative;
        prev_error_ = error;
        return P + I + D;
    }
    void reset(){ integral_ = 0; prev_error_ = 0; }
private:
    float kp_ = 0, ki_ = 0, kd_ = 0, max_i_ = 0;
    float prev_error_ = 0.0f;
    float integral_   = 0.0f;
};

// -------------------------------
// Target message (published by detection, consumed by movement)
// -------------------------------
struct TargetMsg {
    bool  valid{false};
    float dx{0};  // pixels, +right
    float dy{0};  // pixels, +down
    int   frameW{0};
    int   frameH{0};
    std::chrono::steady_clock::time_point stamp;
};

// -------------------------------
// GPS Data
// -------------------------------
struct GPSData {
    double lat = 0.0;
    double lon = 0.0;
    float alt = 0.0f;
    float speed_knots = 0.0f;
    float course_deg = 0.0f;
    int quality = 0;
    int sats = 0;
    bool valid = false;
};

// -------------------------------
// Encoder Data
// -------------------------------
struct EncoderData {
    int left = 0;
    int right = 0;
    bool valid = false;
};

// -----------------------------------------------------------------------
// RAII wrapper for POSIX socket / file descriptors.
// Move-only — no copies of fd ownership.
// Usage:
//   SocketFd fd(::socket(...));
//   if (!fd.valid()) ...
//   fd.close();   // explicit early close
//   int raw = fd; // implicit conversion for syscalls (recv, send, etc.)
// -----------------------------------------------------------------------
struct SocketFd {
    int fd = -1;

    SocketFd() = default;
    explicit SocketFd(int f) : fd(f) {}
    ~SocketFd() { if (fd >= 0) { ::close(fd); fd = -1; } }

    SocketFd(SocketFd&& o) noexcept : fd(o.fd) { o.fd = -1; }
    SocketFd& operator=(SocketFd&& o) noexcept {
        if (this != &o) { if (fd >= 0) ::close(fd); fd = o.fd; o.fd = -1; }
        return *this;
    }
    SocketFd(const SocketFd&)            = delete;
    SocketFd& operator=(const SocketFd&) = delete;

    bool valid()   const { return fd >= 0; }
    void close()         { if (fd >= 0) { ::close(fd); fd = -1; } }
    int  release()       { int tmp = fd; fd = -1; return tmp; }
    operator int() const { return fd; }
};