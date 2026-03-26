#pragma once
#include "behavior.h"
#include "mission.h"
#include "runtime_config.h"
#include <string>

class Comms;                // fwd
class BehaviorCoordinator;  // fwd

// Thin wrapper around MissionRunner for the behavior coordinator.
// The MissionRunner manages its own thread and lifecycle — this behavior
// just provides the coordinator integration (name, onEnter/onExit).
//
// Note: MissionRunner's follow waypoint action calls setMode(FOLLOW) then
// setMode(MISSION) directly.  This means the coordinator may briefly
// transition away from MISSION during a mission.  This is intentional
// and works because MissionRunner runs its own thread independently.
class MissionBehavior : public Behavior {
public:
    MissionBehavior(MissionRunner& runner, Comms& comms,
                    const RuntimeConfig& cfg)
        : runner_(runner), comms_(comms), cfg_(cfg) {}

    const char* name() const override { return "mission"; }
    bool ownsThread() const override { return true; }

    void onEnter() override;   // starts the mission if json is loaded
    void onExit()  override;   // aborts the mission if still running

    // Call before transitioning to MISSION mode
    void loadMission(const std::string& json) { pending_json_ = json; }
    void loadResume(const std::string& id)    { pending_resume_id_ = id; }

private:
    MissionRunner&  runner_;
    Comms&          comms_;
    RuntimeConfig   cfg_;
    std::string     pending_json_;
    std::string     pending_resume_id_;
};
