#include "mission_behavior.h"
#include "comms.h"

void MissionBehavior::onEnter() {
    if (!pending_resume_id_.empty()) {
        runner_.resume(pending_resume_id_, &comms_, cfg_);
        pending_resume_id_.clear();
        pending_json_.clear();
    } else if (!pending_json_.empty()) {
        runner_.start(pending_json_, &comms_, cfg_);
        pending_json_.clear();
    } else {
        LOGW("[MissionBehavior] onEnter with no mission loaded");
    }
}

void MissionBehavior::onExit() {
    if (runner_.running()) {
        runner_.abort();
    }
}
