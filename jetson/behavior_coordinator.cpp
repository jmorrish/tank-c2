#include "behavior_coordinator.h"
#include "comms.h"
#include <algorithm>

using Clock = std::chrono::steady_clock;

BehaviorCoordinator::BehaviorCoordinator(Comms& comms) : comms_(comms) {}
BehaviorCoordinator::~BehaviorCoordinator() { stop(); }

void BehaviorCoordinator::addBehavior(ControlMode mode, Behavior* b) {
    behaviors_[static_cast<int>(mode)] = b;
}

bool BehaviorCoordinator::start() {
    if (run_.exchange(true)) return false;

    // Start in STOPPED if a behavior is registered for it
    auto it = behaviors_.find(static_cast<int>(ControlMode::STOPPED));
    if (it != behaviors_.end()) {
        active_ = it->second;
        activeMode_.store(static_cast<int>(ControlMode::STOPPED));
        active_->onEnter();
        LOGI("Behavior: -> " << active_->name());
    }

    tprev_ = Clock::now();
    th_ = std::thread(&BehaviorCoordinator::loop, this);
    return true;
}

void BehaviorCoordinator::stop() {
    if (!run_.exchange(false)) return;
    if (th_.joinable()) th_.join();

    // Clean exit: stop active behavior and zero everything
    if (active_) {
        active_->onExit();
        active_ = nullptr;
    }
    zeroActuators();
}

void BehaviorCoordinator::requestTransition(ControlMode mode) {
    pendingMode_.store(static_cast<int>(mode));
}

void BehaviorCoordinator::loop() {
    while (run_.load()) {
        // Check for pending transition
        int pending = pendingMode_.exchange(-1);
        if (pending >= 0) {
            doTransition(static_cast<ControlMode>(pending));
        }

        // Compute dt
        auto tnow = Clock::now();
        float dt = std::chrono::duration<float>(tnow - tprev_).count();
        if (dt < 1e-6f) dt = 0.001f;
        tprev_ = tnow;

        // Tick active behavior (only if it doesn't own its own thread)
        if (active_ && !active_->ownsThread()) {
            if (!active_->tick(dt)) {
                // Behavior requested stop
                LOGI("Behavior " << active_->name() << " requested stop");
                doTransition(ControlMode::STOPPED);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
}

void BehaviorCoordinator::doTransition(ControlMode next) {
    int nextInt = static_cast<int>(next);
    int curInt  = activeMode_.load();
    if (nextInt == curInt) return;  // already in this mode

    auto it = behaviors_.find(nextInt);
    if (it == behaviors_.end()) {
        LOGW("No behavior registered for mode " << nextInt << " — ignoring transition");
        return;
    }

    static const char* modeNames[] = {"FOLLOW", "MANUAL", "MISSION", "STOPPED"};
    int safeIdx = std::clamp(nextInt, 0, 3);
    int safeCur = std::clamp(curInt, 0, 3);

    // Exit current
    if (active_) active_->onExit();

    // Always zero actuators between transitions — fixes multiple cleanup bugs
    zeroActuators();

    // Enter new
    active_ = it->second;
    activeMode_.store(nextInt);
    active_->onEnter();

    LOGI("Mode: " << modeNames[safeCur] << " -> " << modeNames[safeIdx]
         << " (" << active_->name() << ")");
}

void BehaviorCoordinator::zeroActuators() {
    comms_.sendPTUVelocity(0, 0);
    comms_.sendWheels(0, 0);
}
