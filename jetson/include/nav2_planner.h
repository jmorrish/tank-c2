#pragma once
#include "path_planner.h"
#include "helpers.h"
#include <chrono>
#include <mutex>
#include <condition_variable>
#include <nlohmann/json.hpp>

class Comms;  // fwd

// Nav2 path planner — requests paths from slam_bridge.py via TCP.
// slam_bridge calls Nav2's ComputePathToPose action, converts the
// map-frame path back to GPS waypoints, and returns them.
// Falls back to direct GPS if Nav2 is unavailable or planning fails.
class Nav2Planner : public PathPlanner {
public:
    explicit Nav2Planner(Comms& comms);

    const char* name() const override { return "nav2"; }
    std::vector<GPSPoint> planPath(GPSPoint from, GPSPoint to) override;
    bool isPathValid() const override;

    // Called by SLAM bridge RX thread when a planned_path response arrives
    void onPlanResponse(const nlohmann::json& resp);

private:
    Comms& comms_;

    // Last planned path (for validity checking)
    mutable std::mutex path_mtx_;
    std::vector<GPSPoint> last_path_;
    std::chrono::steady_clock::time_point last_plan_time_;

    // Request/response synchronisation
    std::mutex resp_mtx_;
    std::condition_variable resp_cv_;
    nlohmann::json pending_resp_;
    bool resp_ready_ = false;
};
