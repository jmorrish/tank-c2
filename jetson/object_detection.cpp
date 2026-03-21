#include "object_detection.h"
#include "comms.h"

#include "config.h"
#include "helpers.h"

#include "yolov8.hpp"
#include "BoTSORT.h"

#include <sstream>
#include <iomanip>
#include <cmath>

// Mouse callback
static void onMouse(int event, int x, int y, int, void* userdata){
    auto* self = static_cast<ObjectDetection*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN){
        // mark focus request if clicked in right half (frame2 area)
        // We'll just always set on click to keep it simple.
        LOGI("Click detected in window");
        // we don't have v4l2 autofocus here anymore; keeping the hook
    }
}

ObjectDetection::ObjectDetection(const std::string& engine_path,
                                 int cam1_index,
                                 const std::string& cam2_rtsp,
                                 bool headless,
                                 AtomicLatest<TargetMsg>& bus,
                                 Comms* comms,
                                 const RuntimeConfig& cfg)
: engine_path_(engine_path)
, cam1_index_(cam1_index)
, cam2_rtsp_(cam2_rtsp)
, headless_(headless)
, bus_(bus)
, comms_(comms)
, cfg_(cfg) {}

ObjectDetection::~ObjectDetection(){ stop(); }

bool ObjectDetection::start(){
    if (run_.exchange(true)) return false;
    rtsp_run_.store(true);
    rtsp_thread_ = std::thread(&ObjectDetection::rtspThreadFunc, this);
    main_thread_ = std::thread(&ObjectDetection::mainLoop, this);

    // NEW: Start ZMQ publisher
    try {
        zmq_pub_.bind("tcp://127.0.0.1:5555");  // Localhost ZMQ port for frames
        zmq_ready_ = true;
        LOGI("ZMQ frame publisher started on tcp://127.0.0.1:5555");
    } catch (const zmq::error_t& e) {
        LOGE("ZMQ bind failed: " << e.what());
    }

    return true;
}
void ObjectDetection::stop(){
    if (!run_.exchange(false)) return;
    rtsp_run_.store(false);
    if (rtsp_thread_.joinable()) rtsp_thread_.join();
    if (main_thread_.joinable()) main_thread_.join();

    // NEW: Stop ZMQ
    zmq_ready_ = false;
    zmq_pub_.close();
    zmq_ctx_.close();
}

// RTSP grabber — reconnects automatically on any failure
void ObjectDetection::rtspThreadFunc(){
    cv::VideoCapture cap2;
    int failCount = 0;
    constexpr int MAX_FAILS    = 10;  // consecutive read failures before reconnect
    constexpr int RETRY_SEC    = 5;   // seconds between reconnect attempts

    while (rtsp_run_.load()){
        // (Re)open if not currently open
        if (!cap2.isOpened()){
            LOGW("RTSP: opening " << cam2_rtsp_);
            cap2.open(cam2_rtsp_);
            if (!cap2.isOpened()){
                LOGW("RTSP: open failed — retrying in " << RETRY_SEC << "s");
                for (int i = 0; i < RETRY_SEC * 10 && rtsp_run_.load(); ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            cap2.set(cv::CAP_PROP_BUFFERSIZE, 1);
            failCount = 0;
            LOGI("RTSP: connected to " << cam2_rtsp_);
        }

        cv::Mat tmp;
        if (!cap2.read(tmp)){
            ++failCount;
            LOGW("RTSP: read failed (" << failCount << "/" << MAX_FAILS << ")");
            if (failCount >= MAX_FAILS){
                LOGW("RTSP: too many failures — reconnecting");
                cap2.release();
                failCount = 0;
            }
            continue;
        }

        failCount = 0;
        {
            std::lock_guard<std::mutex> lk(m2_);
            tmp.copyTo(frame2_);
        }
    }
}

// Main detection / UI loop (publishes TargetMsg)
void ObjectDetection::mainLoop(){
    // YOLO + tracker
    YOLOv8 yolov8(engine_path_);
    yolov8.make_pipe(true);
    BoTSORT tracker(BOTSORT_TRACKER_CFG, BOTSORT_GMC_CFG, BOTSORT_REID_CFG, BOTSORT_REID_MODEL);

    cv::VideoCapture cap;
    constexpr int CAM_RETRY_SEC = 3;

    // Helper: blocks until camera opens or run_ is cleared (handles hot-plug)
    auto openCam = [&]() -> bool {
        while (run_.load()){
            cap.release();
            cap.open(cam1_index_);
            if (cap.isOpened()){
                LOGI("cam1: opened index " << cam1_index_);
                return true;
            }
            LOGW("cam1: not available (index " << cam1_index_ << ") — retrying in " << CAM_RETRY_SEC << "s");
            for (int i = 0; i < CAM_RETRY_SEC * 10 && run_.load(); ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        return false;
    };

    if (!openCam()) return;   // stopped before camera ever appeared

    if (!headless_){
        cv::namedWindow(window_, cv::WINDOW_NORMAL);
        cv::setMouseCallback(window_, onMouse, this);
    }

    cv::Size input_size(640,640);
    int frame_count = 0;
    auto fps_t0 = std::chrono::steady_clock::now();
    int emptyCount = 0;
    constexpr int MAX_EMPTY = 30;  // consecutive empty frames before treating as unplugged

    while (run_.load()){
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()){
            ++emptyCount;
            if (emptyCount >= MAX_EMPTY){
                LOGW("cam1: camera lost (unplugged?) — waiting for reconnect...");
                // Clear the target so the robot stops following while camera is gone
                TargetMsg empty{}; empty.valid = false;
                bus_.set(empty);
                if (comms_) comms_->setDetectionFPS(0.0f);
                // Block here until camera comes back (hot-plug)
                if (!openCam()) return;
                emptyCount = 0;
            }
            continue;
        }
        emptyCount = 0;

        // YOLO
        std::vector<det::Object> dets;
        yolov8.copy_from_Mat(frame, input_size);
        yolov8.infer();
        yolov8.postprocess(dets);

        // Build tracker detections
        std::vector<Detection> tds;
        for (auto& o : dets){
            if (o.label == 0 && o.prob >= cfg_.confidence_threshold){
                tds.push_back(Detection{
                    cv::Rect_<float>(o.rect.x, o.rect.y, o.rect.width, o.rect.height),
                    0, o.prob
                });
            }
        }
        auto tracks = tracker.track(tds, frame);

        // Draw all tracks; prefer the previously-tracked ID to avoid snapping
        // to a new person who enters frame
        bool haveBest = false;
        cv::Rect best;
        int best_id = -1;

        // First pass: re-acquire the previously tracked ID
        for (auto& tr : tracks){
            auto tlwh = tr->get_tlwh();
            cv::Rect bb(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);
            if (!headless_){
                cv::rectangle(frame, bb, cv::Scalar(0,255,0), 2);
                cv::putText(frame, std::to_string(tr->track_id),
                            cv::Point(tlwh[0], tlwh[1]-5),
                            cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,255,255}, 1);
            }
            if (tr->track_id == tracked_id_){
                best = bb; best_id = tr->track_id; haveBest = true;
            }
        }
        // If our target was lost, pick the largest bounding box (closest person)
        // rather than an arbitrary first entry
        if (!haveBest && !tracks.empty()){
            float bestArea = 0.0f;
            for (auto& tr : tracks){
                auto tlwh = tr->get_tlwh();
                float area = tlwh[2] * tlwh[3];
                if (area > bestArea){
                    bestArea = area;
                    best    = cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);
                    best_id = tr->track_id;
                    haveBest = true;
                }
            }
        }
        tracked_id_ = haveBest ? best_id : -1;

        // Publish target
        TargetMsg msg;
        msg.valid = haveBest;
        msg.frameW = frame.cols; msg.frameH = frame.rows;
        if (haveBest){
            int cx = frame.cols/2;
            int cy = frame.rows/2;
            float boxX = best.x + best.width*0.5f;
            float boxY = best.y + best.height*0.5f;
            msg.dx = boxX - cx;
            msg.dy = boxY - cy;
        }
        msg.stamp = std::chrono::steady_clock::now();
        bus_.set(msg);

        // UI / overlays
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_t0).count();
        if (ms >= 1000){
            float fps = frame_count * 1000.0f / ms;
            frame_count = 0; fps_t0 = now;
            if (comms_) comms_->setDetectionFPS(fps);
        }

        // Grab latest RTSP frame (for side-by-side)
        cv::Mat f2;
        {
            std::lock_guard<std::mutex> lk(m2_);
            if (!frame2_.empty()) frame2_.copyTo(f2);
        }

        cv::Mat combined;
        if (!f2.empty()){
            if (frame.rows != f2.rows){
                int newH = std::min(frame.rows, f2.rows);
                cv::resize(frame, frame, cv::Size(), (float)newH/frame.rows, (float)newH/frame.rows);
                cv::resize(f2, f2, cv::Size(), (float)newH/f2.rows, (float)newH/f2.rows);
            }
            first_frame_width_ = frame.cols;
            cv::hconcat(frame, f2, combined);
        } else {
            combined = frame;
        }

        // Publish frame via ZMQ (non-blocking); rebind socket on failure
        if (zmq_ready_) {
            // Scale down to max 1280px wide before encoding to reduce bandwidth.
            // Detection runs at full resolution; only the streamed copy is resized.
            constexpr int STREAM_MAX_W = 1280;
            cv::Mat stream_frame;
            if (combined.cols > STREAM_MAX_W) {
                float scale = float(STREAM_MAX_W) / combined.cols;
                cv::resize(combined, stream_frame, cv::Size(), scale, scale, cv::INTER_LINEAR);
            } else {
                stream_frame = combined;
            }
            std::vector<uchar> buf;
            int quality = comms_ ? comms_->getStreamQuality() : 55;
            std::vector<int> enc_params = {cv::IMWRITE_JPEG_QUALITY, quality};
            cv::imencode(".jpg", stream_frame, buf, enc_params);
            zmq::message_t zmqmsg(buf.size());
            memcpy(zmqmsg.data(), buf.data(), buf.size());
            try {
                zmq_pub_.send(zmqmsg, zmq::send_flags::dontwait);
            } catch (const zmq::error_t& e) {
                LOGW("ZMQ send failed: " << e.what() << " — rebinding");
                zmq_ready_ = false;
                try {
                    zmq_pub_.close();
                    zmq_pub_ = zmq::socket_t(zmq_ctx_, zmq::socket_type::pub);
                    zmq_pub_.bind("tcp://127.0.0.1:5555");
                    zmq_ready_ = true;
                    LOGI("ZMQ rebound successfully");
                } catch (const zmq::error_t& e2) {
                    LOGE("ZMQ rebind failed: " << e2.what());
                }
            }
        }

        // Remove old waitKey since headless
    }

    if (!headless_){
        cv::destroyWindow(window_);
    }
}