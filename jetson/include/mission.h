#pragma once
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include "control_mode.h"  // ControlMode, MissionFault

struct RuntimeConfig;  // fwd
class Comms;           // fwd — MissionRunner calls sendWheels, sendPTUVelocity, setMode, sendWebEvent
class PathPlanner;     // fwd — optional obstacle-aware path planning

// GPS waypoint mission executor.
// Owns the mission thread, JSON, waypoint state, and persistence.
class MissionRunner {
public:
    MissionRunner() = default;
    ~MissionRunner() { abort(); }

    // Start a mission from JSON string.  comms is used for actuators + events.
    void start(const std::string& json_str, Comms* comms, const RuntimeConfig& cfg);

    // Abort a running mission.  Blocks until the thread exits.
    void abort();

    // Skip to next waypoint.
    void skipWaypoint();

    // Resume a saved mission by ID (reads from missions_dir).
    void resume(const std::string& id, Comms* comms, const RuntimeConfig& cfg);

    // Accessors
    bool running()     const { return run_.load(); }
    int  waypointIdx() const { return wp_idx_.load(); }
    int  fault()       const { return fault_.load(); }

    // Current mission JSON (locked copy)
    std::string missionJson() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return json_;
    }

    // Wire a path planner for obstacle-aware navigation (optional).
    // If set, navigateTo() plans sub-waypoints via the planner.
    void setPlanner(PathPlanner* p) { planner_ = p; }

    // Persistence helpers
    void saveMission(const std::string& id, const std::string& json_str,
                     const std::string& missions_dir);
    static std::string listMissionsJson(const std::string& missions_dir);

private:
    void loop(Comms* comms, const RuntimeConfig* cfg);
    void saveMissionState(int wp_idx, const std::string& missions_dir);
    int  loadMissionState(const std::string& id, const std::string& missions_dir);

    mutable std::mutex mtx_;
    std::string        json_;
    std::atomic<int>   wp_idx_{-1};
    std::atomic<bool>  run_{false};
    std::thread        thread_;
    std::atomic<int>   fault_{0};
    PathPlanner*       planner_{nullptr};
};
