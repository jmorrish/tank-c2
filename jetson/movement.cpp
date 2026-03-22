#include "movement.h"
#include <cmath>
#include <algorithm>

using Clock = std::chrono::steady_clock;

Movement::Movement(Comms& comms, AtomicLatest<TargetMsg>& bus, const RuntimeConfig& cfg)
: comms_(comms), bus_(bus), cfg_(cfg), logger_(cfg.log_path)
{
    panPID_.configure(cfg_.kp, cfg_.ki, cfg_.kd, cfg_.max_i);
    tiltPID_.configure(cfg_.kp, cfg_.ki, cfg_.kd, cfg_.max_i);
    // PI only for wheel feedback (no derivative)
    leftWheelPI_.configure(cfg_.wheel_kp, cfg_.wheel_ki, 0.0f, 5000.0f);
    rightWheelPI_.configure(cfg_.wheel_kp, cfg_.wheel_ki, 0.0f, 5000.0f);
}

Movement::~Movement(){ stop(); }

bool Movement::start(){
    if (run_.exchange(true)) return false;
    tprev_ = Clock::now();
    th_ = std::thread(&Movement::loop, this);
    return true;
}
void Movement::stop(){
    if (!run_.exchange(false)) return;
    if (th_.joinable()) th_.join();
}

void Movement::loop(){
    while (run_.load()){
        // Only drive wheels/PTU in FOLLOW mode; other modes are handled externally
        if (comms_.getMode() != ControlMode::FOLLOW) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        }

        auto tnow = Clock::now();
        float dt = std::chrono::duration<float>(tnow - tprev_).count();
        if (dt < 1e-6f) dt = 0.001f;
        tprev_ = tnow;

        auto opt = bus_.get();
        if (!opt || !opt->valid){
            if (std::fabs(lastPanVelo_) > 1e-2f || std::fabs(lastTiltVelo_) > 1e-2f){
                LOGI("[PTU] Person lost => zero velocity");
                if (!comms_.sendPTUVelocity(0,0) && control_ok_)
                    LOGW("[Movement] sendPTUVelocity failed — control socket down?");
                lastPanVelo_ = 0.0f; lastTiltVelo_ = 0.0f;
            }
            if (lastLeft_ != 0 || lastRight_ != 0){
                if (!comms_.sendWheels(0,0) && control_ok_)
                    LOGW("[Movement] sendWheels failed — control socket down?");
                lastLeft_ = lastRight_ = 0;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
            continue;
        }
        const TargetMsg& msg = *opt;

        float dx = msg.dx;
        float dy = msg.dy;
        float dist_pix = std::sqrt(dx*dx + dy*dy);

        // --- IMU tilt feedforward ---
        // Offset tilt error by robot body pitch so the camera doesn't chase
        // the horizon when the robot is on a slope.
        float yaw = 0, pitch = 0, roll = 0; double imu_age = 1e9;
        bool haveImu = comms_.getLatestYPR(yaw, pitch, roll, imu_age);
        if (haveImu && imu_age < SENSOR_STALE_MS){
            dy -= pitch * cfg_.pixels_per_degree;
        }

        // --- PTU velocity control ---
        float panVel = 0, tiltVel = 0;
        if (dist_pix <= CENTER_THRESHOLD){
            if (std::fabs(lastPanVelo_) > 1e-2f || std::fabs(lastTiltVelo_) > 1e-2f){
                LOGI("[PTU] Centered => zero velocity");
                if (!comms_.sendPTUVelocity(0,0) && control_ok_)
                    LOGW("[Movement] sendPTUVelocity failed — control socket down?");
                lastPanVelo_ = 0.0f; lastTiltVelo_ = 0.0f;
            }
        } else {
            panVel  = panPID_.step(dx, dt);
            tiltVel = tiltPID_.step(dy, dt);

            panVel  = std::clamp(panVel,  -MAX_SPEED, MAX_SPEED);
            tiltVel = std::clamp(tiltVel, -MAX_SPEED, MAX_SPEED);

            float dPan  = std::clamp(panVel  - lastPanVelo_,  -MAX_DELTA_VELO, MAX_DELTA_VELO);
            float dTilt = std::clamp(tiltVel - lastTiltVelo_, -MAX_DELTA_VELO, MAX_DELTA_VELO);
            panVel  = lastPanVelo_  + dPan;
            tiltVel = lastTiltVelo_ + dTilt;

            // --- PTU soft position limits ---
            // Accumulate position and clamp velocity if limit reached
            panPos_  += panVel  * dt;
            tiltPos_ += tiltVel * dt;
            if ((panPos_ >  cfg_.pan_limit_steps  && panVel  > 0) ||
                (panPos_ < -cfg_.pan_limit_steps  && panVel  < 0)) panVel  = 0;
            if ((tiltPos_ >  cfg_.tilt_limit_steps && tiltVel > 0) ||
                (tiltPos_ < -cfg_.tilt_limit_steps && tiltVel < 0)) tiltVel = 0;

            float panChange  = std::fabs(panVel  - lastPanVelo_);
            float tiltChange = std::fabs(tiltVel - lastTiltVelo_);
            bool shouldSend  = (panChange > VELO_EPS || tiltChange > VELO_EPS) &&
                               (std::fabs(panVel) > MIN_SPEED || std::fabs(tiltVel) > MIN_SPEED);

            if (shouldSend){
                LOGI("[PTU] dx=" << dx << ", dy=" << dy
                     << ", panVel=" << panVel << ", tiltVel=" << tiltVel);
                bool ok = comms_.sendPTUVelocity(panVel, tiltVel);
                if (!ok && control_ok_)  LOGW("[Movement] sendPTUVelocity failed — control socket down?");
                if ( ok && !control_ok_) LOGI("[Movement] Control socket recovered");
                control_ok_ = ok;
                lastPanVelo_  = panVel;
                lastTiltVelo_ = tiltVel;
            }
        }

        // --- Wheel follow distance ---
        double dist_age = 0.0;
        float d = comms_.getLatestDistance(&dist_age);
        int wheelSpeed = 0;
        if (d > cfg_.follow_distance_m && d < 10.0f && dist_age < SENSOR_STALE_MS){
            wheelSpeed = int((d - cfg_.follow_distance_m) * cfg_.wheel_gain_distance);
        }

        int left  = wheelSpeed;
        int right = wheelSpeed;

        // Steer with PTU pan velocity
        float steer = lastPanVelo_ * cfg_.wheel_gain_steer;
        left  -= int(steer);
        right += int(steer);

        // --- Encoder closed-loop PI correction ---
        int encLeft = 0, encRight = 0; double enc_age = 1e9;
        if (comms_.getLatestEncoders(encLeft, encRight, enc_age) && enc_age < SENSOR_STALE_MS){
            left  += int(leftWheelPI_.step(float(left  - encLeft),  dt));
            right += int(rightWheelPI_.step(float(right - encRight), dt));
        }

        left  = std::clamp(left,  -WHEEL_MAX_SPS, WHEEL_MAX_SPS);
        right = std::clamp(right, -WHEEL_MAX_SPS, WHEEL_MAX_SPS);

        if (left != lastLeft_ || right != lastRight_){
            bool ok = comms_.sendWheels(left, right);
            if (!ok && control_ok_)  LOGW("[Movement] sendWheels failed — control socket down?");
            if ( ok && !control_ok_) LOGI("[Movement] Control socket recovered");
            control_ok_ = ok;
            lastLeft_ = left; lastRight_ = right;
        }

        // --- Run log ---
        logger_.log(
            /*track_id=*/0,      // ObjectDetection doesn't expose track_id here yet
            dx, dy,
            panVel, tiltVel,
            left, right,
            d,
            yaw, pitch, roll
        );

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    if (!comms_.sendPTUVelocity(0,0)) LOGW("[Movement] sendPTUVelocity failed on shutdown");
    if (!comms_.sendWheels(0,0))     LOGW("[Movement] sendWheels failed on shutdown");
}
