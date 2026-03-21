#pragma once
#include <string>

// --------- Teensy / Network ---------
static const std::string TEENSY_IP_DEFAULT        = "192.168.0.177";
static const int         TEENSY_PORT_DEFAULT       = 23;
static const std::string SENSOR_IP_DEFAULT         = "192.168.0.178";
static const int         SENSOR_PORT_DEFAULT       = 23;

// --------- Sensor staleness ---------
// Readings older than this are treated as invalid
static constexpr double SENSOR_STALE_MS = 500.0;

// --------- TOF Serial ---------
static const char* TOF_SERIAL_PORT = "/dev/ttyUSB0";
static const int   TOF_BAUD        = 921600;

// --------- PTU PID ---------
static constexpr float Kp              = 1.5f;
static constexpr float Ki              = 0.1f;
static constexpr float Kd              = 0.1f;
static constexpr float MAX_I           = 1000.0f;
static constexpr float CENTER_THRESHOLD= 20.0f;     // pixels
static constexpr float MIN_SPEED       = 0.0f;      // sps
static constexpr float MAX_SPEED       = 2000.0f;   // sps
static constexpr float VELO_EPS        = 25.0f;     // sps
static constexpr float MAX_DELTA_VELO  = 500.0f;    // sps per step

// --------- Detection ---------
static constexpr float CONFIDENCE_THRESHOLD = 0.40f;

// --------- Wheels (follow distance) ---------
static constexpr float FOLLOW_DISTANCE_M     = 0.5f;   // target distance
static constexpr float WHEEL_GAIN_DISTANCE   = 20000.0f;
static constexpr float WHEEL_GAIN_STEER      = 0.2f;   // steer factor * panVel
static constexpr int   WHEEL_MAX_SPS         = 15000;

// --------- Cameras ---------
static const std::string CAM2_RTSP = "rtsp://192.168.0.71/z3-2.sdp";

// --------- BoTSORT paths ---------
static const std::string BOTSORT_BASE        = "/home/james/YOLOv8-TensorRT/BoTSORT-cpp";
static const std::string BOTSORT_TRACKER_CFG = BOTSORT_BASE + "/config/tracker.ini";
static const std::string BOTSORT_GMC_CFG     = BOTSORT_BASE + "/config/gmc.ini";
static const std::string BOTSORT_REID_MODEL  = BOTSORT_BASE + "/assets/mobilenetv2_x1_4_msmt17.onnx";
static const std::string BOTSORT_REID_CFG    = BOTSORT_BASE + "/config/reid.ini";

// --------- Missions ---------
static const std::string MISSIONS_DIR = "/home/james/tank_missions";

// --------- Target gallery ---------
static const std::string TARGETS_DIR  = "/home/james/tank_targets";

// --------- UI ---------
static constexpr bool DEFAULT_HEADLESS = false;