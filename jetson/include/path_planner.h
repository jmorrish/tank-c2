#pragma once
#include <vector>

// GPS coordinate pair.
struct GPSPoint { double lat = 0; double lon = 0; };

// Abstract path planner interface.
// Behaviors call planPath() to get a sequence of intermediate waypoints
// that avoid obstacles on the SLAM costmap (via Nav2).  Implementations
// can be swapped via config.json ("planner": "nav2" or "direct_gps").
class PathPlanner {
public:
    virtual ~PathPlanner() = default;
    virtual const char* name() const = 0;

    // Plan a route from one GPS point to another.
    // Returns intermediate GPS waypoints.  Empty = no path found.
    // Implementations may block for several seconds (Nav2 planning).
    virtual std::vector<GPSPoint> planPath(GPSPoint from, GPSPoint to) = 0;

    // Check if the last planned path is still valid.
    // Cheap to call (no RPC); based on local sensor state.
    virtual bool isPathValid() const { return true; }
};

// Direct GPS: straight line to goal — preserves pre-Nav2 behavior exactly.
struct DirectGPSPlanner : PathPlanner {
    const char* name() const override { return "direct_gps"; }
    std::vector<GPSPoint> planPath(GPSPoint /*from*/, GPSPoint to) override {
        return {to};
    }
};
