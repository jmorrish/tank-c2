#include "object_detection.h"
#include "comms.h"

#include "config.h"
#include "helpers.h"

#include "yolov8.hpp"
#include "BoTSORT.h"
#include "DataType.h"

#include <sstream>
#include <iomanip>
#include <cmath>
#include <fstream>
#include <dirent.h>
#include <sys/stat.h>
#include <cctype>
#include <algorithm>

// Mouse callback
static void onMouse(int event, int x, int y, int, void* userdata){
    auto* self = static_cast<ObjectDetection*>(userdata);
    if (event == cv::EVENT_LBUTTONDOWN){
        LOGI("Click detected in window");
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

    try {
        zmq_pub_.bind("tcp://127.0.0.1:5555");
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

    zmq_ready_ = false;
    zmq_pub_.close();
    zmq_ctx_.close();
}

// ── RTSP grabber ──────────────────────────────────────────────────────────────
void ObjectDetection::rtspThreadFunc(){
    cv::VideoCapture cap2;
    int failCount = 0;
    constexpr int MAX_FAILS    = 10;
    constexpr int RETRY_SEC    = 5;

    while (rtsp_run_.load()){
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

// ── Gallery helpers ────────────────────────────────────────────────────────────

float ObjectDetection::cosineSim(const std::vector<float>& a, const std::vector<float>& b) {
    float dot = 0, na = 0, nb = 0;
    for (size_t i = 0; i < a.size(); ++i) {
        dot += a[i] * b[i];
        na  += a[i] * a[i];
        nb  += b[i] * b[i];
    }
    float n = std::sqrt(na) * std::sqrt(nb);
    return (n > 1e-8f) ? dot / n : 0.0f;
}

void ObjectDetection::loadGallery(){
    ::mkdir(TARGETS_DIR.c_str(), 0755);

    // Read next_id counter
    {
        std::ifstream cf(TARGETS_DIR + "/next_id.txt");
        if (cf.is_open()) cf >> next_id_;
        if (next_id_ < 1) next_id_ = 1;
    }

    // Load each numeric subdirectory
    DIR* dir = ::opendir(TARGETS_DIR.c_str());
    if (!dir) return;
    struct dirent* ent;
    while ((ent = ::readdir(dir)) != nullptr) {
        std::string name = ent->d_name;
        if (name.empty() || name[0] == '.') continue;
        if (!std::all_of(name.begin(), name.end(), ::isdigit)) continue;

        std::string embed_path = TARGETS_DIR + "/" + name + "/embed.bin";
        std::ifstream ef(embed_path, std::ios::binary);
        if (!ef.is_open()) continue;

        PersonRecord rec;
        rec.id = std::stoi(name);
        rec.embedding.resize(FEATURE_DIM);
        ef.read(reinterpret_cast<char*>(rec.embedding.data()),
                sizeof(float) * FEATURE_DIM);
        if ((size_t)ef.gcount() == sizeof(float) * FEATURE_DIM)
            gallery_.push_back(std::move(rec));
    }
    ::closedir(dir);

    LOGI("Gallery loaded: " << gallery_.size() << " person(s) from " << TARGETS_DIR);
}

void ObjectDetection::saveEmbedding(const PersonRecord& p) const {
    std::string dir = TARGETS_DIR + "/" + std::to_string(p.id);
    ::mkdir(dir.c_str(), 0755);
    std::ofstream f(dir + "/embed.bin", std::ios::binary);
    if (f.is_open())
        f.write(reinterpret_cast<const char*>(p.embedding.data()),
                sizeof(float) * p.embedding.size());
}

void ObjectDetection::saveThumbnail(int person_id, const cv::Mat& crop) const {
    std::string path = TARGETS_DIR + "/" + std::to_string(person_id) + "/thumb.jpg";
    std::vector<uchar> buf;
    cv::imencode(".jpg", crop, buf, {cv::IMWRITE_JPEG_QUALITY, 85});
    std::ofstream f(path, std::ios::binary);
    if (f.is_open())
        f.write(reinterpret_cast<const char*>(buf.data()), buf.size());
}

int ObjectDetection::matchOrCreate(const std::vector<float>& raw_feat) {
    // Normalise
    float n = 0; for (float x : raw_feat) n += x * x; n = std::sqrt(n);
    if (n < 1e-8f) return -1;
    std::vector<float> feat(raw_feat.size());
    for (size_t i = 0; i < raw_feat.size(); ++i) feat[i] = raw_feat[i] / n;

    // Find best gallery match
    float best_sim = -1.0f;
    int   best_idx = -1;
    for (int i = 0; i < (int)gallery_.size(); ++i) {
        float s = cosineSim(gallery_[i].embedding, feat);
        if (s > best_sim) { best_sim = s; best_idx = i; }
    }

    if (best_idx >= 0 && best_sim >= GALLERY_MATCH_THRESH) {
        // EMA update: slowly adapt to appearance changes
        auto& emb = gallery_[best_idx].embedding;
        for (size_t i = 0; i < emb.size(); ++i)
            emb[i] = emb[i] * 0.9f + feat[i] * 0.1f;
        // Re-normalise after EMA
        float en = 0; for (float x : emb) en += x * x; en = std::sqrt(en);
        if (en > 1e-8f) for (float& x : emb) x /= en;
        return gallery_[best_idx].id;
    }

    // New person
    PersonRecord rec;
    rec.id = next_id_++;
    rec.embedding = feat;
    gallery_.push_back(rec);

    // Persist counter and embedding
    {
        std::ofstream cf(TARGETS_DIR + "/next_id.txt");
        if (cf.is_open()) cf << next_id_;
    }
    saveEmbedding(rec);

    // Notify web clients
    if (comms_) {
        std::string ev = "{\"type\":\"event\",\"event\":\"new_target\",\"target_id\":"
                         + std::to_string(rec.id) + "}";
        comms_->sendWebEvent(ev);
    }

    LOGI("New person in gallery: ID " << rec.id);
    return rec.id;
}

// ── Main detection / UI loop ───────────────────────────────────────────────────
void ObjectDetection::mainLoop(){
    // Load persisted gallery before starting
    loadGallery();

    // YOLO + tracker
    YOLOv8 yolov8(engine_path_);
    yolov8.make_pipe(true);
    BoTSORT tracker(BOTSORT_TRACKER_CFG, BOTSORT_GMC_CFG, BOTSORT_REID_CFG, BOTSORT_REID_MODEL);

    cv::VideoCapture cap;
    constexpr int CAM_RETRY_SEC = 3;

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

    if (!openCam()) return;

    if (!headless_){
        cv::namedWindow(window_, cv::WINDOW_NORMAL);
        cv::setMouseCallback(window_, onMouse, this);
    }

    cv::Size input_size(640,640);
    int frame_count = 0;
    auto fps_t0 = std::chrono::steady_clock::now();
    int emptyCount = 0;
    constexpr int MAX_EMPTY = 30;

    while (run_.load()){
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()){
            ++emptyCount;
            if (emptyCount >= MAX_EMPTY){
                LOGW("cam1: camera lost (unplugged?) — waiting for reconnect...");
                TargetMsg empty{}; empty.valid = false;
                bus_.set(empty);
                if (comms_) comms_->setDetectionFPS(0.0f);
                if (!openCam()) return;
                emptyCount = 0;
            }
            continue;
        }
        emptyCount = 0;

        // ── YOLO ────────────────────────────────────────────────────────────
        std::vector<det::Object> dets;
        yolov8.copy_from_Mat(frame, input_size);
        yolov8.infer();
        yolov8.postprocess(dets);

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

        // ── Pending target switch (from web UI) ──────────────────────────────
        {
            int pending = pending_target_person_.exchange(-2);
            if (pending != -2) {
                active_target_person_id_ = pending;
                tracked_id_ = -1;   // force re-acquire
            }
        }

        // ── Gallery pass: identify every visible track ───────────────────────
        for (auto& tr : tracks) {
            if (!tr->smooth_feat) continue;
            std::vector<float> feat(tr->smooth_feat->data(),
                                    tr->smooth_feat->data() + FEATURE_DIM);
            int pid = matchOrCreate(feat);
            if (pid >= 0) track_to_person_[tr->track_id] = pid;
        }

        // ── Draw all track boxes + ID labels ────────────────────────────────
        bool haveBest = false;
        cv::Rect best;
        int best_id = -1;

        for (auto& tr : tracks){
            auto tlwh = tr->get_tlwh();
            cv::Rect bb(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);
            cv::rectangle(frame, bb, cv::Scalar(0,255,0), 2);
            cv::putText(frame, "ID:" + std::to_string(tr->track_id),
                        cv::Point(tlwh[0], tlwh[1]-5),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
            if (tr->track_id == tracked_id_){
                best = bb; best_id = tr->track_id; haveBest = true;
            }
        }

        // ── Re-acquire if tracked_id_ was lost ───────────────────────────────
        if (!haveBest && !tracks.empty()){
            // Level 1: match by gallery person_id via track_to_person_
            if (active_target_person_id_ >= 0) {
                for (auto& tr : tracks) {
                    auto it = track_to_person_.find(tr->track_id);
                    if (it != track_to_person_.end() &&
                        it->second == active_target_person_id_) {
                        auto tlwh = tr->get_tlwh();
                        best    = cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);
                        best_id = tr->track_id;
                        haveBest = true;
                        break;
                    }
                }
            }

            // Level 2: direct cosine similarity against saved embedding
            if (!haveBest && active_target_person_id_ >= 0) {
                for (auto& rec : gallery_) {
                    if (rec.id != active_target_person_id_) continue;
                    float bestSim = GALLERY_MATCH_THRESH - 0.01f;
                    for (auto& tr : tracks) {
                        if (!tr->smooth_feat) continue;
                        std::vector<float> feat(tr->smooth_feat->data(),
                                                tr->smooth_feat->data() + FEATURE_DIM);
                        float sim = cosineSim(rec.embedding, feat);
                        if (sim > bestSim) {
                            bestSim = sim;
                            auto tlwh = tr->get_tlwh();
                            best    = cv::Rect(tlwh[0], tlwh[1], tlwh[2], tlwh[3]);
                            best_id = tr->track_id;
                            haveBest = true;
                        }
                    }
                    break;
                }
            }

            // Level 3: fallback — largest bounding box (closest person)
            if (!haveBest) {
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
        }

        tracked_id_ = haveBest ? best_id : -1;

        // Update active_target_person_id_ and push to Comms for telemetry
        if (haveBest) {
            auto it = track_to_person_.find(best_id);
            if (it != track_to_person_.end())
                active_target_person_id_ = it->second;
            if (comms_) comms_->setTargetPersonId(active_target_person_id_);
        } else {
            if (comms_) comms_->setTargetPersonId(-1);
        }

        // ── Highlight active target + crosshair ─────────────────────────────
        if (haveBest){
            cv::rectangle(frame, best, cv::Scalar(255,255,0), 3);
            std::string label = "TARGET";
            auto it = track_to_person_.find(best_id);
            if (it != track_to_person_.end())
                label += " P" + std::to_string(it->second);
            cv::putText(frame, label,
                        cv::Point(best.x, best.y - 10),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, {255,255,0}, 2);
        }
        {
            int cx = frame.cols/2, cy = frame.rows/2, cs = 20;
            cv::line(frame, {cx-cs,cy}, {cx+cs,cy}, {0,200,255}, 1);
            cv::line(frame, {cx,cy-cs}, {cx,cy+cs}, {0,200,255}, 1);
        }

        // ── Save thumbnails periodically ─────────────────────────────────────
        ++thumb_frame_counter_;
        if (thumb_frame_counter_ >= THUMB_UPDATE_INTERVAL) {
            thumb_frame_counter_ = 0;
            for (auto& tr : tracks) {
                auto it = track_to_person_.find(tr->track_id);
                if (it == track_to_person_.end()) continue;
                auto tlwh = tr->get_tlwh();
                int x = std::max(0, (int)tlwh[0]);
                int y = std::max(0, (int)tlwh[1]);
                int w = std::min((int)tlwh[2], frame.cols - x);
                int h = std::min((int)tlwh[3], frame.rows - y);
                if (w > 10 && h > 10)
                    saveThumbnail(it->second, frame(cv::Rect(x, y, w, h)).clone());
            }
        }

        // ── Publish target ───────────────────────────────────────────────────
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

        // ── FPS ──────────────────────────────────────────────────────────────
        frame_count++;
        auto now = std::chrono::steady_clock::now();
        auto ms  = std::chrono::duration_cast<std::chrono::milliseconds>(now - fps_t0).count();
        if (ms >= 1000){
            float fps = frame_count * 1000.0f / ms;
            frame_count = 0; fps_t0 = now;
            if (comms_) comms_->setDetectionFPS(fps);
        }

        // ── Grab latest RTSP frame (for side-by-side) ────────────────────────
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

        // ── ZMQ publish (scaled, non-blocking) ───────────────────────────────
        if (zmq_ready_) {
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
    }

    if (!headless_){
        cv::destroyWindow(window_);
    }
}
