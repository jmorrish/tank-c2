#pragma once
#include "behavior.h"
#include "helpers.h"
#include "runtime_config.h"
#include "logger.h"

class Comms;  // fwd

// Extracted from Movement::loop().
// Drives PTU + wheels to follow a person using PID + distance control.
class FollowBehavior : public Behavior {
public:
    FollowBehavior(Comms& comms, AtomicLatest<TargetMsg>& bus, const RuntimeConfig& cfg);

    const char* name() const override { return "follow"; }
    void onEnter() override;   // reset PIDs, zero state
    bool tick(float dt) override;

private:
    Comms&                   comms_;
    AtomicLatest<TargetMsg>& bus_;
    RuntimeConfig            cfg_;

    // PTU PID
    PIDController panPID_;
    PIDController tiltPID_;
    float lastPanVelo_  = 0.0f;
    float lastTiltVelo_ = 0.0f;
    float panPos_       = 0.0f;
    float tiltPos_      = 0.0f;

    // Wheel PI
    PIDController leftWheelPI_;
    PIDController rightWheelPI_;
    int lastLeft_  = 0;
    int lastRight_ = 0;

    RunLogger logger_;

    // Suppress repeated control-down warnings
    bool control_ok_ = true;
};
