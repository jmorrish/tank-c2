#include "nav2_planner.h"
#include "comms.h"
#include <chrono>

Nav2Planner::Nav2Planner(Comms& comms) : comms_(comms) {}

std::vector<GPSPoint> Nav2Planner::planPath(GPSPoint from, GPSPoint to) {
    // Build request
    nlohmann::json req;
    req["type"]     = "plan_path";
    req["from_lat"] = from.lat;
    req["from_lon"] = from.lon;
    req["to_lat"]   = to.lat;
    req["to_lon"]   = to.lon;

    // Clear previous response
    {
        std::lock_guard<std::mutex> lk(resp_mtx_);
        resp_ready_ = false;
        pending_resp_ = {};
    }

    // Send to slam_bridge via TCP
    comms_.sendSlamBridge(req.dump());

    // Wait for response (max 15s — Nav2 planning can take a few seconds)
    nlohmann::json resp;
    {
        std::unique_lock<std::mutex> lk(resp_mtx_);
        if (!resp_cv_.wait_for(lk, std::chrono::seconds(15),
                               [this]{ return resp_ready_; })) {
            LOGW("[Nav2Planner] Timeout waiting for plan — falling back to direct GPS");
            return {to};
        }
        resp = std::move(pending_resp_);
        resp_ready_ = false;
    }

    if (!resp.value("ok", false)) {
        LOGW("[Nav2Planner] Planning failed: " << resp.value("error", "unknown"));
        return {to};  // fallback: direct GPS
    }

    // Parse waypoints
    std::vector<GPSPoint> path;
    for (auto& wp : resp["waypoints"]) {
        if (wp.is_array() && wp.size() >= 2) {
            path.push_back({wp[0].get<double>(), wp[1].get<double>()});
        }
    }

    if (path.empty()) {
        LOGW("[Nav2Planner] Empty path returned — falling back to direct GPS");
        return {to};
    }

    // Store for validity checking
    {
        std::lock_guard<std::mutex> lk(path_mtx_);
        last_path_ = path;
        last_plan_time_ = std::chrono::steady_clock::now();
    }

    LOGI("[Nav2Planner] Planned path with " << path.size() << " waypoints");
    return path;
}

bool Nav2Planner::isPathValid() const {
    std::lock_guard<std::mutex> lk(path_mtx_);
    if (last_path_.empty()) return true;

    // Trigger 1: Obstacle detected in forward arc
    if (comms_.hasObstacle()) return false;

    // Trigger 2: Path is stale (>30s since last plan)
    auto age = std::chrono::steady_clock::now() - last_plan_time_;
    if (age > std::chrono::seconds(30)) return false;

    return true;
}

void Nav2Planner::onPlanResponse(const nlohmann::json& resp) {
    std::lock_guard<std::mutex> lk(resp_mtx_);
    pending_resp_ = resp;
    resp_ready_ = true;
    resp_cv_.notify_one();
}
