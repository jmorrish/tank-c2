#pragma once
#include <string>

// Runtime-tunable parameters loaded from config.json at startup.
// Missing keys fall back silently to the compiled defaults below.
// config.h is no longer needed — all tunables live here.
struct RuntimeConfig {

    // ── Network ─────────────────────────────────────────────────────────────
    std::string teensy_ip          = "192.168.0.177";
    int         teensy_port        = 23;
    std::string sensor_ip          = "192.168.0.178";
    int         sensor_port        = 23;
    std::string cam2_rtsp          = "rtsp://192.168.0.71/z3-2.sdp";
    int         web_ipc_port       = 9999;

    // ── Sensor staleness ────────────────────────────────────────────────────
    double sensor_stale_ms         = 500.0;

    // ── PTU PID gains ───────────────────────────────────────────────────────
    float kp                       = 1.5f;
    float ki                       = 0.1f;
    float kd                       = 0.1f;
    float max_i                    = 1000.0f;

    // ── PTU motion limits ───────────────────────────────────────────────────
    float center_threshold         = 20.0f;     // pixels — PTU stops when target within this
    float min_speed                = 0.0f;      // sps
    float max_speed                = 2000.0f;   // sps
    float velo_eps                 = 25.0f;     // sps — velocity change threshold
    float max_delta_velo           = 500.0f;    // sps per step

    // ── PTU soft limits (accumulated steps from startup) ────────────────────
    float pan_limit_steps          = 50000.0f;
    float tilt_limit_steps         = 20000.0f;

    // ── Detection ───────────────────────────────────────────────────────────
    float confidence_threshold     = 0.40f;

    // ── Person re-ID / gallery ──────────────────────────────────────────────
    float gallery_match_thresh     = 0.75f;     // cosine sim for re-ID match
    float embed_ema_alpha          = 0.1f;      // EMA weight for new embedding updates
    int   max_extra_embeddings     = 100;       // cap on extra embeddings per person

    // ── Auto-learn ──────────────────────────────────────────────────────────
    int   auto_learn_interval_ms   = 3000;
    float auto_learn_min_conf      = 0.80f;
    int   auto_learn_min_area      = 8192;      // ~64×128 px
    int   auto_learn_batch_size    = 5;

    // ── Learn mode ──────────────────────────────────────────────────────────
    int   learn_sample_ms          = 500;
    int   learn_min_crop_px        = 64;
    float learn_blur_thresh        = 80.0f;     // Laplacian variance
    float learn_dup_thresh         = 0.95f;     // cosine sim to reject duplicates

    // ── Streaming ───────────────────────────────────────────────────────────
    int   stream_max_w             = 1280;
    int   thumb_jpeg_quality       = 85;

    // ── Wheel follow behaviour ──────────────────────────────────────────────
    float follow_distance_m        = 0.5f;
    float wheel_gain_distance      = 20000.0f;
    float wheel_gain_steer         = 0.2f;
    int   wheel_max_sps            = 15000;
    float max_follow_distance_m    = 10.0f;     // stop wheels beyond this

    // ── Wheel encoder closed-loop PI ────────────────────────────────────────
    float wheel_kp                 = 0.3f;
    float wheel_ki                 = 0.05f;
    float wheel_pi_max_i           = 5000.0f;

    // ── IMU tilt feedforward ────────────────────────────────────────────────
    float pixels_per_degree        = 5.0f;

    // ── Stereo depth ────────────────────────────────────────────────────────
    float stereo_max_fps           = 10.0f;
    float stereo_left_fps          = 10.0f;
    int   stereo_num_disparities   = 128;
    int   stereo_block_size        = 9;
    int   stereo_blur_ksize        = 3;
    int   stereo_jpeg_quality      = 60;
    std::string stereo_calib_xml   = "/home/james/stereo_calib/stereo_params_cuda.xml";

    // ── Mission navigation ──────────────────────────────────────────────────
    float mission_gps_loss_abort_s = 30.0f;
    float mission_stuck_check_s    = 30.0f;
    float mission_stuck_min_move_m = 0.5f;
    float mission_steer_gain       = 50.0f;     // SPS per degree heading error
    float mission_max_steer_frac   = 0.6f;
    float mission_moving_knots     = 0.5f;      // speed threshold for GPS course
    float mission_heading_filter   = 0.05f;     // low-pass weight for heading cal
    float mission_ptu_ms_per_deg   = 10.0f;
    float mission_return_speed_frac= 0.33f;

    // ── Telemetry ───────────────────────────────────────────────────────────
    float telemetry_stale_ms       = 2000.0f;
    int   status_push_interval_ms  = 1000;

    // ── ZMQ ports ───────────────────────────────────────────────────────────
    int   zmq_detection_port       = 5555;
    int   zmq_stereo_port          = 5557;
    int   zmq_disparity_port       = 5558;

    // ── Lidar ───────────────────────────────────────────────────────────────
    std::string lidar_port         = "/dev/ttyUSB0";
    int   lidar_baud               = 115200;
    float lidar_fwd_arc_deg        = 30.0f;
    float lidar_obstacle_m         = 0.5f;
    float lidar_max_range_m        = 8.0f;
    int   lidar_scan_pts           = 180;

    // ── Paths ───────────────────────────────────────────────────────────────
    std::string botsort_base       = "/home/james/YOLOv8-TensorRT/BoTSORT-cpp";
    std::string missions_dir       = "/home/james/tank_missions";
    std::string targets_dir        = "/home/james/tank_targets";
    std::string log_path           = "/tmp/robot_run.csv";

    // Derived BoTSORT paths (computed from botsort_base)
    std::string botsortTrackerCfg() const { return botsort_base + "/config/tracker.ini"; }
    std::string botsortGmcCfg()     const { return botsort_base + "/config/gmc.ini"; }
    std::string botsortReidModel()  const { return botsort_base + "/assets/mobilenetv2_x1_4_msmt17.onnx"; }
    std::string botsortReidCfg()    const { return botsort_base + "/config/reid.ini"; }

    // Load from JSON; returns defaults if file is missing or unparseable
    static RuntimeConfig load(const std::string& path);
};
