#pragma once
#include <string>
#include "config.h"

// Runtime-tunable parameters loaded from config.json at startup.
// Missing keys fall back silently to the compiled defaults in config.h.
struct RuntimeConfig {
    // PTU PID gains
    float kp                   = Kp;
    float ki                   = Ki;
    float kd                   = Kd;
    float max_i                = MAX_I;

    // Detection confidence
    float confidence_threshold = CONFIDENCE_THRESHOLD;

    // Wheel follow behaviour
    float follow_distance_m    = FOLLOW_DISTANCE_M;
    float wheel_gain_distance  = WHEEL_GAIN_DISTANCE;
    float wheel_gain_steer     = WHEEL_GAIN_STEER;

    // PTU soft limits (accumulated steps from startup position)
    float pan_limit_steps      = 50000.0f;
    float tilt_limit_steps     = 20000.0f;

    // Wheel encoder closed-loop PI gains
    float wheel_kp             = 0.3f;
    float wheel_ki             = 0.05f;

    // IMU tilt feedforward: pixels of tilt correction per degree of body pitch
    float pixels_per_degree    = 5.0f;

    // Run log output path
    std::string log_path       = "/tmp/robot_run.csv";

    // Load from JSON; returns defaults if file is missing or unparseable
    static RuntimeConfig load(const std::string& path);
};
