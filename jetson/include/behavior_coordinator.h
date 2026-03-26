#pragma once
#include <atomic>
#include <thread>
#include <unordered_map>
#include <chrono>
#include "behavior.h"
#include "control_mode.h"

class Comms;  // fwd

// Owns the behavior dispatch loop.  Replaces Movement.
// Transitions are thread-safe: any thread calls requestTransition(), the
// coordinator loop defers the actual switch to its own thread (no mutex
// needed around behavior state).
class BehaviorCoordinator {
public:
    explicit BehaviorCoordinator(Comms& comms);
    ~BehaviorCoordinator();

    // Register a behavior for a given control mode (call before start()).
    void addBehavior(ControlMode mode, Behavior* b);

    bool start();
    void stop();

    // Thread-safe — sets an atomic that the loop thread picks up.
    void requestTransition(ControlMode mode);

    ControlMode currentMode() const {
        return static_cast<ControlMode>(activeMode_.load());
    }

private:
    void loop();
    void doTransition(ControlMode next);
    void zeroActuators();

    Comms& comms_;
    std::unordered_map<int, Behavior*> behaviors_;

    Behavior*    active_     = nullptr;
    std::atomic<int> activeMode_{static_cast<int>(ControlMode::STOPPED)};
    std::atomic<int> pendingMode_{-1};   // -1 = no pending

    std::thread th_;
    std::atomic<bool> run_{false};
    std::chrono::steady_clock::time_point tprev_;
};
