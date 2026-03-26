#pragma once
#include <vector>
#include <utility>
#include <cmath>
#include <algorithm>

// Analyse lidar scan to suggest avoidance steering direction.
// Returns: negative = turn left, positive = turn right, 0 = fully blocked.
// Used by FollowBehavior to steer around obstacles instead of hard-stopping.
inline float suggestAvoidanceSteer(
    const std::vector<std::pair<float,float>>& scan,
    float obstacle_m)
{
    float left_min  = 999.0f;   // min distance on the left (270-360°)
    float right_min = 999.0f;   // min distance on the right (0-90°)

    for (auto& [angle, dist] : scan) {
        if (dist <= 0.0f) continue;
        if (angle > 270.0f && angle <= 360.0f) left_min  = std::min(left_min, dist);
        if (angle >= 0.0f  && angle <   90.0f) right_min = std::min(right_min, dist);
    }

    // Both sides clear — shouldn't be here, but don't interfere
    if (left_min > obstacle_m && right_min > obstacle_m) return 0.0f;

    // Steer toward the side with more room
    if (left_min > right_min) return -1.0f;   // more room on left
    if (right_min > left_min) return  1.0f;   // more room on right

    return 0.0f;  // blocked everywhere — full stop
}
