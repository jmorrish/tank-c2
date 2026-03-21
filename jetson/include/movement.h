#pragma once
#include <atomic>
#include <thread>
#include "helpers.h"
#include "comms.h"
#include "config.h"
#include "runtime_config.h"
#include "logger.h"

class Movement {
public:
    Movement(Comms& comms, AtomicLatest<TargetMsg>& bus, const RuntimeConfig& cfg);
    ~Movement();

    bool start();
    void stop();
    bool isRunning() const { return run_.load(); }

private:
    void loop();

    Comms&                   comms_;
    AtomicLatest<TargetMsg>& bus_;
    RuntimeConfig            cfg_;
    std::thread              th_;
    std::atomic<bool>        run_{false};

    // PTU PID controllers (configured from RuntimeConfig in constructor)
    PIDController panPID_;
    PIDController tiltPID_;
    float lastPanVelo_  = 0.0f;
    float lastTiltVelo_ = 0.0f;

    // PTU soft-limit position tracking (accumulated steps from startup)
    float panPos_  = 0.0f;
    float tiltPos_ = 0.0f;

    // Wheel closed-loop PI controllers (encoder feedback)
    PIDController leftWheelPI_;
    PIDController rightWheelPI_;
    int lastLeft_  = 0;
    int lastRight_ = 0;

    RunLogger logger_;

    std::chrono::steady_clock::time_point tprev_ = std::chrono::steady_clock::now();
};
