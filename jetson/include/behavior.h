#pragma once

// Base class for all autonomous/semi-autonomous behaviors.
// The BehaviorCoordinator owns the dispatch loop and calls tick() on the
// active behavior.  Trivial behaviors (Stopped, Manual) are defined inline
// because they contain no logic — they just have a name.

class Behavior {
public:
    virtual ~Behavior() = default;

    // Human-readable name for logging and status JSON.
    virtual const char* name() const = 0;

    // Called once when this behavior becomes the active behavior.
    virtual void onEnter() {}

    // Called once when this behavior is being deactivated.
    virtual void onExit() {}

    // Called every coordinator tick (~2 ms) while this behavior is active.
    // Return false to request a transition to STOPPED.
    virtual bool tick(float dt) { return true; }

    // If true, behavior runs its own thread (e.g. MissionRunner) and the
    // coordinator will NOT call tick().
    virtual bool ownsThread() const { return false; }
};

// ── Trivial inline behaviors ─────────────────────────────────────────────────

struct StoppedBehavior : Behavior {
    const char* name() const override { return "stopped"; }
};

struct ManualBehavior : Behavior {
    const char* name() const override { return "manual"; }
};
