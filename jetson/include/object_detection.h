#pragma once
#include <atomic>
#include <map>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include "helpers.h"
#include "runtime_config.h"
#include <zmq.hpp>

class Comms; // fwd

// One entry in the persistent person gallery.
// Embedding stored as plain floats to avoid pulling Eigen into this header.
struct PersonRecord {
    int id = -1;
    std::string name;                                // display name from name.txt (empty = none set)
    std::vector<float> embedding;                    // normalised 512-dim EMA feature
    std::vector<std::vector<float>> extra_embeddings; // from learn sessions (saved to embed_gallery.bin)
    int best_thumb_area = 0;                         // pixel area of best saved thumbnail crop
};

class ObjectDetection {
public:
    ObjectDetection(const std::string& engine_path,
                    int cam1_index,
                    const std::string& cam2_rtsp,
                    bool headless,
                    AtomicLatest<TargetMsg>& bus,
                    Comms* comms,
                    const RuntimeConfig& cfg);
    ~ObjectDetection();

    bool start();
    void stop();
    bool isRunning() const { return run_.load(); }

    // Tell detection which device index is the stereo camera so it is skipped
    // during automatic camera scanning (set before start()).
    void setStereoIndex(int idx) { stereo_cam_index_ = idx; }

    // Called from Comms thread to request a target switch (thread-safe).
    void setTargetPerson(int person_id) { pending_target_person_.store(person_id); }

    // Learn mode: collect appearance crops for AI description (thread-safe).
    void startLearn(int person_id);
    void stopLearn();

private:
    void rtspThreadFunc();
    void mainLoop();

    // Gallery helpers
    void loadGallery();
    void saveEmbedding(const PersonRecord& p) const;
    void saveThumbnail(int person_id, const cv::Mat& crop) const;
    void saveExtraEmbeddings(int person_id, const std::vector<std::vector<float>>& embeddings) const;
    int  matchOrCreate(const std::vector<float>& feat);  // returns person_id
    static float cosineSim(const std::vector<float>& a, const std::vector<float>& b);

    std::string engine_path_;
    int cam1_index_;
    int stereo_cam_index_ = -1;  // device index to skip during scan (it's the stereo cam)
    std::string cam2_rtsp_;
    bool headless_;
    AtomicLatest<TargetMsg>& bus_;
    Comms* comms_;
    RuntimeConfig cfg_;

    std::thread rtsp_thread_;
    std::thread main_thread_;
    std::atomic<bool> run_{false};

    // RTSP shared frame
    std::mutex m2_;
    cv::Mat frame2_;
    std::atomic<bool> rtsp_run_{false};

    // Protects gallery_, next_id_, and best_thumb_area in PersonRecord.
    // All gallery accesses happen on the mainLoop thread today, but this
    // guards against future refactoring that moves gallery access to other
    // threads (e.g. web API reads).
    mutable std::mutex gallery_mtx_;

    // UI
    std::string window_ = "YOLOv8 + BoTSORT + PTU";

    // autofocus click
    int first_frame_width_ = 0;
    std::atomic<bool> secondCamFocusRequested_{false};

    // BoTSORT session track ID of the currently-followed person
    int tracked_id_ = -1;

    // NEW: ZMQ for publishing frames to Python
    zmq::context_t zmq_ctx_{1};
    zmq::socket_t zmq_pub_{zmq_ctx_, ZMQ_PUB};
    bool zmq_ready_ = false;

    // ── Cross-session target gallery ──────────────────────────────────────────
    static constexpr float GALLERY_MATCH_THRESH  = 0.75f;

    std::vector<PersonRecord> gallery_;
    std::map<int, int> track_to_person_;  // session track_id → gallery person_id
    int next_id_               = 1;
    int active_target_person_id_ = -1;   // gallery person we're currently following
    std::atomic<int> pending_target_person_{-2};  // -2=no change, ≥-1=requested switch

    // Describe mode: collect crops to send to Claude for appearance description
    std::atomic<bool>  learn_active_{false};
    std::atomic<bool>  learn_flush_pending_{false};
    std::atomic<int>   learn_person_id_{-1};
    std::mutex         learn_mtx_;
    std::vector<std::string>         learn_crops_;      // base64 JPEG strings
    std::vector<std::vector<float>>  learn_embeddings_; // diversity filter
    std::chrono::steady_clock::time_point learn_last_sample_{};

    // Auto-learn: passive background quality-gated embedding collection
    std::chrono::steady_clock::time_point auto_learn_last_{};
    std::vector<std::vector<float>> auto_batch_;  // pending disk write (mainLoop only)
    int auto_batch_pid_ = -1;
    static constexpr int   AUTO_INTERVAL_MS = 3000;   // sample every 3 s
    static constexpr float AUTO_MIN_CONF    = 0.80f;  // min track confidence
    static constexpr int   AUTO_MIN_AREA    = 8192;   // min box area ~64×128 px
    static constexpr int   AUTO_BATCH_SIZE  = 5;      // flush to disk every N embeds

    void flushLearnCrops();
    static std::string base64Encode(const std::vector<uchar>& data);
};
