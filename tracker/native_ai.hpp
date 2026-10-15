// In-process ReID model (OpenVINO C++ API): DINOv2-small fingerprints, run on a worker thread inside target_tracker -- no Python
// sidecar, no JPEG round trip over UDP. Same pre/post-processing as tracker/detector_sidecar.py (DinoReid), so the fingerprints
// match the sidecar's (cosine 1.0000, measured). Built only when CMake finds OpenVINO (HAVE_OPENVINO); otherwise the tracker can use
// the sidecar. (EdgeSAM / MobileSAM segmentation was removed 2026-10-15: never used -- the lock box comes from the ground-colour model
// and GrabCut.)
// NOTE: included by main.cpp AFTER fl::Box is defined (the members below use it).
#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstdio>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef HAVE_OPENVINO
#include <openvino/openvino.hpp>
#endif

// The interface the tracker uses for ReID fingerprints, whichever backend serves them (the UDP sidecar or the in-process model).
struct AiBackend {
    virtual ~AiBackend() = default;
    virtual uint32_t request_embed(const cv::Mat& frame, const fl::Box& b) = 0;     // fingerprint of the object in b (0 = not sent)
    virtual bool embed_poll(uint32_t id, std::vector<float>& out) = 0;              // reply in? (empty = failed)
    virtual bool embed_many(const cv::Mat& frame, const std::vector<fl::Box>& bs, std::vector<std::vector<float>>& out, double timeout_s) = 0;
    virtual bool has_reid() const = 0;
    virtual std::string describe() const = 0;
};

#ifdef HAVE_OPENVINO
class NativeAI : public AiBackend {
public:
    // dino_dir: directory with model.onnx (DINOv2-small)
    explicit NativeAI(const std::string& dino_dir, int threads = 0) {
        core_.set_property(ov::cache_dir(std::string(std::getenv("HOME") ? std::getenv("HOME") : ".") + "/.cache/openvino_models"));
        ov::AnyMap cfg{ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY)};
        if (threads > 0) cfg.emplace(ov::inference_num_threads.name(), threads);
        dino_ = core_.compile_model(dino_dir + "/model.onnx", "CPU", cfg);
        std::vector<float> warm; embed_now(cv::Mat(112, 112, CV_8UC3, cv::Scalar(90, 90, 90)), warm);   // first run compiles/allocates
        running_ = true;
        thread_ = std::thread([this] { loop(); });
    }
    ~NativeAI() override {
        { std::lock_guard<std::mutex> lk(mtx_); running_ = false; }      // under the lock: a worker between its check and its wait still wakes
        cv_.notify_all();
        if (thread_.joinable()) thread_.join();
    }
    bool has_reid() const override { return true; }
    std::string describe() const override { return std::string("in-process OpenVINO ") + ov::get_openvino_version().buildNumber + ", DINOv2 ReID"; }

    uint32_t request_embed(const cv::Mat& frame, const fl::Box& b) override {
        cv::Mat crop = square_crop(frame, b);
        if (crop.empty()) return 0;
        std::lock_guard<std::mutex> lk(mtx_);
        const uint32_t id = ++next_id_;
        q_.push_back({id, crop, now_s()}); res_[id] = Emb{};
        while (res_.size() > 64) res_.erase(res_.begin());
        cv_.notify_all();
        return id;
    }
    bool embed_poll(uint32_t id, std::vector<float>& out) override {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = res_.find(id); if (it == res_.end() || !it->second.done) return false;
        out = it->second.v; res_.erase(it); return true;
    }
    bool embed_many(const cv::Mat& frame, const std::vector<fl::Box>& bs, std::vector<std::vector<float>>& out, double timeout_s) override {
        std::vector<uint32_t> ids; for (const auto& b : bs) { const uint32_t id = request_embed(frame, b); if (!id) return false; ids.push_back(id); }
        out.assign(bs.size(), {}); std::vector<bool> got(bs.size(), false); size_t n = 0;
        const double t0 = now_s(), budget = timeout_s + 0.1 * (double)(ids.size() - 1);   // one worker runs them in turn (~40-130 ms each)
        while (n < ids.size() && now_s() - t0 < budget) {
            for (size_t k = 0; k < ids.size(); k++) if (!got[k] && embed_poll(ids[k], out[k])) { got[k] = true; n++; }
            if (n < ids.size()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (n < ids.size()) {                                            // late: forget them (and drop them from the queue)
            std::lock_guard<std::mutex> lk(mtx_);
            for (auto id : ids) res_.erase(id);
            q_.erase(std::remove_if(q_.begin(), q_.end(), [&](const Job& e) { return std::find(ids.begin(), ids.end(), e.id) != ids.end(); }), q_.end());
            return false;
        }
        for (const auto& v : out) if (v.empty()) return false;
        return true;
    }
    // synchronous (benchmarks): DINOv2 on a crop -> CLS + mean patch token, unit length
    bool embed_now(const cv::Mat& crop_bgr, std::vector<float>& out) {
        if (crop_bgr.empty()) return false;
        cv::Mat rgb, x; cv::cvtColor(crop_bgr, rgb, cv::COLOR_BGR2RGB);
        cv::resize(rgb, x, cv::Size(kSide, kSide), 0, 0, cv::INTER_CUBIC);
        ov::Tensor in(ov::element::f32, {1, 3, (size_t)kSide, (size_t)kSide});
        float* p = in.data<float>();
        static const float mean[3] = {0.485f, 0.456f, 0.406f}, stdv[3] = {0.229f, 0.224f, 0.225f};
        for (int c = 0; c < 3; c++) for (int y = 0; y < kSide; y++) for (int xx = 0; xx < kSide; xx++)
            p[(c * kSide + y) * kSide + xx] = (x.at<cv::Vec3b>(y, xx)[c] / 255.0f - mean[c]) / stdv[c];
        std::lock_guard<std::mutex> lk(dino_mtx_);
        if (!req_) req_ = dino_.create_infer_request();
        req_.set_input_tensor(in);
        req_.infer();
        const ov::Tensor o = req_.get_output_tensor(0);                   // [1, tokens, dim]
        const auto sh = o.get_shape(); const size_t T = sh[1], D = sh[2]; const float* h = o.data<const float>();
        out.assign(2 * D, 0.f);
        for (size_t d = 0; d < D; d++) out[d] = h[d];
        for (size_t t = 1; t < T; t++) for (size_t d = 0; d < D; d++) out[D + d] += h[t * D + d] / float(T - 1);
        double nrm = 0; for (float v : out) nrm += (double)v * v; nrm = std::sqrt(nrm) + 1e-9;
        for (float& v : out) v = float(v / nrm);
        return true;
    }
    // the same crop as the sidecar request: a square 1.4x the box's long side, around its centre
    static cv::Mat square_crop(const cv::Mat& frame, const fl::Box& b) {
        const double side = std::max(b.w, b.h) * 1.4;
        cv::Rect r((int)std::lround(b.cx() - side / 2), (int)std::lround(b.cy() - side / 2), (int)std::lround(side), (int)std::lround(side));
        r &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (r.width < 6 || r.height < 6) return cv::Mat();
        return frame(r).clone();
    }

private:
    static constexpr int kSide = 112;
    struct Job { uint32_t id; cv::Mat crop; double t; };
    struct Emb { bool done = false; std::vector<float> v; };
    static double now_s() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
    void loop() {
        while (running_) {
            Job j;
            { std::unique_lock<std::mutex> lk(mtx_); cv_.wait(lk, [&] { return !running_ || !q_.empty(); }); if (!running_) return;
              j = q_.front(); q_.pop_front(); }
            std::vector<float> v;
            // an exception must never leave this thread (std::terminate would take the whole tracker down mid-flight)
            try { if (now_s() - j.t <= 0.5) embed_now(j.crop, v); } catch (const std::exception& e) { v.clear(); std::fprintf(stderr, "[AI] DINOv2 failed: %s\n", e.what()); }
            std::lock_guard<std::mutex> lk(mtx_);
            auto it = res_.find(j.id); if (it != res_.end()) { it->second.v = std::move(v); it->second.done = true; }
        }
    }

    ov::Core core_; ov::CompiledModel dino_; ov::InferRequest req_;
    std::mutex mtx_, dino_mtx_; std::condition_variable cv_; std::atomic<bool> running_{false};
    std::thread thread_;
    std::deque<Job> q_; std::map<uint32_t, Emb> res_;
    uint32_t next_id_ = 0;
};
#endif
