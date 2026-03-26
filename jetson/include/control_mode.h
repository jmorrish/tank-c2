#pragma once

enum class ControlMode : int {
    FOLLOW  = 0,  // autonomous person-follow (default)
    MANUAL  = 1,  // web-driven manual control
    MISSION = 2,  // GPS waypoint mission running
    STOPPED = 3   // all motion halted
};

enum class MissionFault : int {
    NONE     = 0,
    GPS_LOST = 1,  // GPS signal absent for too long
    STUCK    = 2,  // physical movement not matching commanded motion
    WHEEL    = 3   // wheel command delivery failed (control socket down)
};
