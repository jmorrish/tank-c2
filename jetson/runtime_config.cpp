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

        get("kp",                   cfg.kp);
        get("ki",                   cfg.ki);
        get("kd",                   cfg.kd);
        get("max_i",                cfg.max_i);
        get("confidence_threshold", cfg.confidence_threshold);
        get("follow_distance_m",    cfg.follow_distance_m);
        get("wheel_gain_distance",  cfg.wheel_gain_distance);
        get("wheel_gain_steer",     cfg.wheel_gain_steer);
        get("pan_limit_steps",      cfg.pan_limit_steps);
        get("tilt_limit_steps",     cfg.tilt_limit_steps);
        get("wheel_kp",             cfg.wheel_kp);
        get("wheel_ki",             cfg.wheel_ki);
        get("pixels_per_degree",    cfg.pixels_per_degree);
        get("log_path",             cfg.log_path);

        LOGI("Config loaded from '" << path << "'");
    } catch (const std::exception& e) {
        LOGE("Config parse error in '" << path << "': " << e.what() << " — using defaults");
    }
    return cfg;
}
