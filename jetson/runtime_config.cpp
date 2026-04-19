#include "runtime_config.h"
#include "helpers.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <type_traits>

RuntimeConfig RuntimeConfig::load(const std::string& path) {
    RuntimeConfig cfg;
    std::ifstream f(path);
    if (!f.is_open()) {
        LOGW("Config file '" << path << "' not found — using compiled defaults");
        return cfg;
    }
    try {
        nlohmann::json j;
        f >> j;

        // Helper: only overwrite if key exists and type matches
        auto get = [&](const char* key, auto& field) {
            if (j.contains(key))
                field = j[key].get<std::decay_t<decltype(field)>>();
        };

        // Network
        get("teensy_ip",              cfg.teensy_ip);
        get("teensy_port",            cfg.teensy_port);
        get("sensor_ip",             cfg.sensor_ip);
        get("sensor_port",           cfg.sensor_port);
        get("cam2_rtsp",             cfg.cam2_rtsp);
        get("web_ipc_port",          cfg.web_ipc_port);

        // Sensor staleness
        get("sensor_stale_ms",       cfg.sensor_stale_ms);

        // PTU PID
        get("kp",                    cfg.kp);
        get("ki",                    cfg.ki);
        get("kd",                    cfg.kd);
        get("max_i",                 cfg.max_i);

        // PTU motion limits
        get("center_threshold",      cfg.center_threshold);
        get("min_speed",             cfg.min_speed);
        get("max_speed",             cfg.max_speed);
        get("velo_eps",              cfg.velo_eps);
        get("max_delta_velo",        cfg.max_delta_velo);

        // PTU soft limits
        get("pan_limit_steps",       cfg.pan_limit_steps);
        get("tilt_limit_steps",      cfg.tilt_limit_steps);

        // Detection
        get("confidence_threshold",  cfg.confidence_threshold);

        // Person re-ID / gallery
        get("gallery_match_thresh",  cfg.gallery_match_thresh);
        get("embed_ema_alpha",       cfg.embed_ema_alpha);
        get("max_extra_embeddings",  cfg.max_extra_embeddings);

        // Auto-learn
        get("auto_learn_interval_ms", cfg.auto_learn_interval_ms);
        get("auto_learn_min_conf",   cfg.auto_learn_min_conf);
        get("auto_learn_min_area",   cfg.auto_learn_min_area);
        get("auto_learn_batch_size", cfg.auto_learn_batch_size);

        // Learn mode
        get("learn_sample_ms",       cfg.learn_sample_ms);
        get("learn_min_crop_px",     cfg.learn_min_crop_px);
        get("learn_blur_thresh",     cfg.learn_blur_thresh);
        get("learn_dup_thresh",      cfg.learn_dup_thresh);

        // Streaming
        get("stream_max_w",          cfg.stream_max_w);
        get("stereo_camera_upside_down", cfg.stereo_camera_upside_down);
        get("thumb_jpeg_quality",    cfg.thumb_jpeg_quality);

        // Wheel follow
        get("follow_distance_m",     cfg.follow_distance_m);
        get("wheel_gain_distance",   cfg.wheel_gain_distance);
        get("wheel_gain_steer",      cfg.wheel_gain_steer);
        get("wheel_max_sps",         cfg.wheel_max_sps);
        get("max_follow_distance_m", cfg.max_follow_distance_m);

        // Wheel encoder PI
        get("wheel_kp",             cfg.wheel_kp);
        get("wheel_ki",             cfg.wheel_ki);
        get("wheel_pi_max_i",       cfg.wheel_pi_max_i);

        // IMU tilt
        get("pixels_per_degree",    cfg.pixels_per_degree);

        // PTU centre-to-level
        get("ptu_level_p_gain",       cfg.ptu_level_p_gain);
        get("ptu_level_max_sps",      cfg.ptu_level_max_sps);
        get("ptu_level_deadband_deg", cfg.ptu_level_deadband_deg);
        get("ptu_level_timeout_ms",   cfg.ptu_level_timeout_ms);
        get("ptu_level_tilt_sign",    cfg.ptu_level_tilt_sign);

        // Stereo depth
        get("stereo_max_fps",        cfg.stereo_max_fps);
        get("stereo_left_fps",       cfg.stereo_left_fps);
        get("stereo_num_disparities",cfg.stereo_num_disparities);
        get("stereo_block_size",     cfg.stereo_block_size);
        get("stereo_blur_ksize",     cfg.stereo_blur_ksize);
        get("stereo_jpeg_quality",   cfg.stereo_jpeg_quality);
        get("stereo_calib_xml",      cfg.stereo_calib_xml);

        // Mission navigation
        get("mission_gps_loss_abort_s",  cfg.mission_gps_loss_abort_s);
        get("mission_stuck_check_s",     cfg.mission_stuck_check_s);
        get("mission_stuck_min_move_m",  cfg.mission_stuck_min_move_m);
        get("mission_steer_gain",        cfg.mission_steer_gain);
        get("mission_max_steer_frac",    cfg.mission_max_steer_frac);
        get("mission_moving_knots",      cfg.mission_moving_knots);
        get("mission_heading_filter",    cfg.mission_heading_filter);
        get("mission_ptu_ms_per_deg",    cfg.mission_ptu_ms_per_deg);
        get("mission_return_speed_frac", cfg.mission_return_speed_frac);

        // Telemetry
        get("telemetry_stale_ms",    cfg.telemetry_stale_ms);
        get("status_push_interval_ms", cfg.status_push_interval_ms);

        // ZMQ
        get("zmq_detection_port",    cfg.zmq_detection_port);
        get("zmq_stereo_port",       cfg.zmq_stereo_port);
        get("zmq_disparity_port",    cfg.zmq_disparity_port);
        get("zmq_thermal_port",      cfg.zmq_thermal_port);

        // Lidar
        get("lidar_port",           cfg.lidar_port);
        get("lidar_baud",           cfg.lidar_baud);
        get("lidar_fwd_arc_deg",    cfg.lidar_fwd_arc_deg);
        get("lidar_obstacle_m",     cfg.lidar_obstacle_m);
        get("lidar_max_range_m",    cfg.lidar_max_range_m);
        get("lidar_scan_pts",       cfg.lidar_scan_pts);

        // Paths
        get("botsort_base",         cfg.botsort_base);
        get("missions_dir",         cfg.missions_dir);
        get("targets_dir",          cfg.targets_dir);
        get("log_path",             cfg.log_path);

        LOGI("Config loaded from '" << path << "'");
    } catch (const std::exception& e) {
        LOGE("Config parse error in '" << path << "': " << e.what() << " — using defaults");
    }
    return cfg;
}
