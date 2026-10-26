// In-process AI models (OpenVINO C++ API), inside target_tracker: DINOv2-small (ReID fingerprints on a worker thread, click
// segmentation patch tokens) and NanoTrack (the box-size tracker). Built only when CMake finds OpenVINO (HAVE_OPENVINO).
// NOTE: included by main.cpp AFTER fl::Box is defined (the members below use it).
#pragma once
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <memory>
#include <stdexcept>
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

// The interface the tracker uses for ReID fingerprints.
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
    // synchronous: DINOv2 PATCH tokens of a square crop at `side` px (224 -> a 16x16 grid of 384-d tokens, ~180 ms on the CPU) --
    // for the colour-independent click segmentation (fl::dino_segment_from_tokens). Not normalised.
    bool patch_tokens(const cv::Mat& crop_bgr, std::vector<float>& out, int& grid, int& dim, int side = 224) {
        if (crop_bgr.empty()) return false;
        cv::Mat rgb, x; cv::cvtColor(crop_bgr, rgb, cv::COLOR_BGR2RGB);
        cv::resize(rgb, x, cv::Size(side, side), 0, 0, cv::INTER_CUBIC);
        ov::Tensor in(ov::element::f32, {1, 3, (size_t)side, (size_t)side});
        float* p = in.data<float>();
        static const float mean[3] = {0.485f, 0.456f, 0.406f}, stdv[3] = {0.229f, 0.224f, 0.225f};
        for (int c = 0; c < 3; c++) for (int y = 0; y < side; y++) for (int xx = 0; xx < side; xx++)
            p[(c * side + y) * side + xx] = (x.at<cv::Vec3b>(y, xx)[c] / 255.0f - mean[c]) / stdv[c];
        std::lock_guard<std::mutex> lk(dino_mtx_);
        if (!seg_req_) seg_req_ = dino_.create_infer_request();         // its own request: the 112 px ReID one keeps its shapes
        seg_req_.set_input_tensor(in);
        seg_req_.infer();
        const ov::Tensor o = seg_req_.get_output_tensor(0);               // [1, 1 + grid*grid, dim]
        const auto sh = o.get_shape(); const size_t T = sh[1], D = sh[2];
        const int g = (int)std::lround(std::sqrt((double)(T - 1)));
        if (T < 2 || (size_t)(g * g) != T - 1) return false;
        const float* h = o.data<const float>();
        out.assign(h + D, h + T * D); grid = g; dim = (int)D;
        return true;
    }
    // the fingerprint crop: a square 1.4x the box's long side, around its centre
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

    ov::Core core_; ov::CompiledModel dino_; ov::InferRequest req_, seg_req_;
    std::mutex mtx_, dino_mtx_; std::condition_variable cv_; std::atomic<bool> running_{false};
    std::thread thread_;
    std::deque<Job> q_; std::map<uint32_t, Emb> res_;
    uint32_t next_id_ = 0;
};

// NanoTrack (a ~2 MB Siamese tracker: backbone + neck/head ONNX, ~/models/nanotrack) through OpenVINO -- the same algorithm as OpenCV's
// cv::TrackerNano (modules/video/src/tracking/tracker_nano.cpp, OpenCV >= 4.7; this build's OpenCV 4.5.4 has neither TrackerNano nor
// the HardSwish layer its backbone needs). Unlike CSRT it predicts the box SIZE every frame: measured on sim approach clips, box IoU
// 0.82 vs CSRT 0.34 on a bus growing 40 -> 88 px, 7.5 vs 21 ms a frame. The models are compiled once (shared); each tracker has its
// own requests and template.
class NanoTrackOV {
public:
    static bool available(const std::string& dir) { return shared(dir) != nullptr; }
    explicit NanoTrackOV(const std::string& dir) : m_(shared(dir)) {
        if (!m_) throw std::runtime_error("NanoTrack models not found in " + dir);
        bb_ = m_->bb.create_infer_request(); bbz_ = m_->bbz.create_infer_request(); hd_ = m_->hd.create_infer_request();
        createHanning();
    }
    // start on box b (full-frame pixels): the template is taken from this frame
    void init(const cv::Mat& img, const cv::Rect2d& b) {
        pos_[0] = (float)(b.x + b.width * 0.5); pos_[1] = (float)(b.y + b.height * 0.5);
        sz_[0] = (float)b.width; sz_[1] = (float)b.height; imgsz_ = img.size();
        set_template(img);
    }
    // new template from the CURRENT position / size (call only on frames whose identity was confirmed)
    void refresh_template(const cv::Mat& img) { set_template(img); }
    // move the tracker to box b WITHOUT a new template (the hybrid's correction from CSRT)
    void reset_box(const cv::Rect2d& b) { pos_[0] = (float)(b.x + b.width * 0.5); pos_[1] = (float)(b.y + b.height * 0.5); sz_[0] = (float)b.width; sz_[1] = (float)b.height; }
    bool update(const cv::Mat& img, cv::Rect2d& out) {
        if (!have_t_) return false;
        imgsz_ = img.size();
        const float ssum = sz_[0] + sz_[1];
        const float wc = sz_[0] + kCtx * ssum, hc = sz_[1] + kCtx * ssum, s = std::sqrt(wc * hc), scale_z = kExemplar / s, sx = s * (float(kInstance) / kExemplar);
        float tw = sz_[0] * scale_z, th = sz_[1] * scale_z;
        cv::Mat crop; subwindow(crop, img, (int)sx, kInstance);
        bb_.set_input_tensor(blob(crop));
        bb_.infer();
        const ov::Tensor xf = bb_.get_output_tensor(0);
        hd_.set_tensor(m_->in1, zf_); hd_.set_tensor(m_->in2, ov::Tensor(xf.get_element_type(), xf.get_shape()));
        std::memcpy(hd_.get_tensor(m_->in2).data(), xf.data(), xf.get_byte_size());
        hd_.infer();
        const ov::Tensor cls = hd_.get_tensor(m_->out1), box = hd_.get_tensor(m_->out2);
        const float* c = cls.data<const float>(); const float* bp = box.data<const float>();
        const int N = kScore * kScore;
        int best = 0; float bestv = -1e9f, bx1 = 0, by1 = 0, bx2 = 0, by2 = 0, bpen = 0, bsc = 0;
        const float ratio = tw / th, tsz = sizeCal(tw, th);
        for (int i = 0; i < N; i++) {
            const int r = i / kScore, q = i % kScore;
            const float m = std::max(c[i], c[N + i]);
            const float e0 = std::exp(c[i] - m), e1 = std::exp(c[N + i] - m), score = e1 / (e0 + e1);
            const float gx = (q - kScore / 2) * kStride + kInstance / 2.0f, gy = (r - kScore / 2) * kStride + kInstance / 2.0f;
            const float x1 = gx - bp[i], y1 = gy - bp[N + i], x2 = gx + bp[2 * N + i], y2 = gy + bp[3 * N + i];
            float scr = sizeCal(x2 - x1, y2 - y1) / tsz; scr = std::max(scr, 1.0f / scr);
            float rc = ratio / ((x2 - x1) / std::max(1e-6f, (y2 - y1))); rc = std::max(rc, 1.0f / rc);
            const float pen = std::exp(-(rc * scr - 1.0f) * kPenaltyK);
            const float ps = pen * score * (1.0f - kWindow) + hann_[i] * kWindow;
            if (ps > bestv) { bestv = ps; best = i; bx1 = x1; by1 = y1; bx2 = x2; by2 = y2; bpen = pen; bsc = score; }
        }
        (void)best;
        score_ = bestv;
        const float pxs = (bx1 + bx2) / 2, pys = (by1 + by2) / 2, pw = (bx2 - bx1) / scale_z, ph = (by2 - by1) / scale_z;
        const float dx = (pxs - kInstance / 2.0f) / scale_z, dy = (pys - kInstance / 2.0f) / scale_z;
        const float lr = bpen * bsc * kLr;
        float rx = pos_[0] + dx, ry = pos_[1] + dy, rw = pw * lr + (1 - lr) * sz_[0], rh = ph * lr + (1 - lr) * sz_[1];
        rx = std::clamp(rx, 0.f, (float)imgsz_.width); ry = std::clamp(ry, 0.f, (float)imgsz_.height);
        rw = std::clamp(rw, 10.f, (float)imgsz_.width); rh = std::clamp(rh, 10.f, (float)imgsz_.height);
        pos_[0] = rx; pos_[1] = ry; sz_[0] = rw; sz_[1] = rh;
        out = cv::Rect2d(rx - rw / 2, ry - rh / 2, rw, rh);
        return true;
    }
    float score() const { return score_; }
private:
    static constexpr int kExemplar = 127, kInstance = 255, kStride = 16, kScore = (kInstance - kExemplar) / kStride + 8;   // 16
    static constexpr float kWindow = 0.455f, kLr = 0.37f, kCtx = 0.5f, kPenaltyK = 0.055f;
    struct Models { ov::Core core; ov::CompiledModel bb, bbz, hd; std::string in1, in2, out1, out2; };   // bb: 255 px search, bbz: 127 px template
    static Models* shared(const std::string& dir) {
        static std::mutex mtx; static std::map<std::string, std::unique_ptr<Models>> cache;
        std::lock_guard<std::mutex> lk(mtx);
        auto it = cache.find(dir); if (it != cache.end()) return it->second.get();
        std::unique_ptr<Models> m;
        try {
            m = std::make_unique<Models>();
            // the backbone's input is fixed at 255x255 in the file: reshaped to 127x127 for the template (OpenCV's dnn does that itself)
            auto bm = m->core.read_model(dir + "/nanotrack_backbone_sim.onnx");
            m->bb = m->core.compile_model(bm, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
            auto bz = bm->clone(); bz->reshape(ov::PartialShape{1, 3, kExemplar, kExemplar});
            m->bbz = m->core.compile_model(bz, "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
            m->hd = m->core.compile_model(dir + "/nanotrack_head_sim.onnx", "CPU", ov::hint::performance_mode(ov::hint::PerformanceMode::LATENCY));
            for (const auto& i : m->hd.inputs()) { const auto& n = i.get_any_name(); if (n == "input1") m->in1 = n; else if (n == "input2") m->in2 = n; }
            for (const auto& o : m->hd.outputs()) { const auto& n = o.get_any_name(); if (n == "output1") m->out1 = n; else if (n == "output2") m->out2 = n; }
            if (m->in1.empty() || m->in2.empty() || m->out1.empty() || m->out2.empty()) m.reset();
        } catch (const std::exception&) { m.reset(); }
        Models* p = m.get(); cache[dir] = std::move(m); return p;
    }
    static float sizeCal(float w, float h) { const float pad = (w + h) * 0.5f; return std::sqrt((w + pad) * (h + pad)); }
    void createHanning() {
        hann_.resize(kScore * kScore);
        for (int r = 0; r < kScore; r++) for (int q = 0; q < kScore; q++) {
            const double wr = 0.5 * (1 - std::cos(2 * CV_PI * r / (kScore - 1))), wq = 0.5 * (1 - std::cos(2 * CV_PI * q / (kScore - 1)));
            hann_[r * kScore + q] = (float)std::sqrt(wr * wq);    // as cv::createHanningWindow (the square root of the product)
        }
    }
    // the crop (BGR) -> NCHW float RGB, 0..255 (cv::dnn::blobFromImage(crop, 1.0, Size(), Scalar(), swapRB=true))
    static ov::Tensor blob(const cv::Mat& crop) {
        ov::Tensor t(ov::element::f32, {1, 3, (size_t)crop.rows, (size_t)crop.cols});
        float* p = t.data<float>(); const int hw = crop.rows * crop.cols;
        for (int y = 0; y < crop.rows; y++) { const cv::Vec3b* r = crop.ptr<cv::Vec3b>(y);
            for (int x = 0; x < crop.cols; x++) { const int i = y * crop.cols + x; p[i] = r[x][2]; p[hw + i] = r[x][1]; p[2 * hw + i] = r[x][0]; } }
        return t;
    }
    void subwindow(cv::Mat& dst, const cv::Mat& src, int osz, int rsz) const {
        const int c = (osz + 1) / 2;
        int x0 = (int)pos_[0] - c, x1 = x0 + osz - 1, y0 = (int)pos_[1] - c, y1 = y0 + osz - 1;
        const int lp = std::max(0, -x0), tp = std::max(0, -y0), rp = std::max(0, x1 - src.cols + 1), bp = std::max(0, y1 - src.rows + 1);
        x0 += lp; x1 += lp; y0 += tp; y1 += tp;
        cv::Mat crop;
        if (!lp && !tp && !rp && !bp) crop = src(cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1));
        else { cv::Mat tmp; cv::copyMakeBorder(src, tmp, tp, bp, lp, rp, cv::BORDER_CONSTANT, cv::mean(src)); crop = tmp(cv::Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1)); }
        cv::resize(crop, dst, cv::Size(rsz, rsz));
    }
    void set_template(const cv::Mat& img) {
        const float ssum = sz_[0] + sz_[1];
        const float we = sz_[0] + kCtx * ssum, he = sz_[1] + kCtx * ssum; const int s = (int)std::sqrt(we * he);
        cv::Mat crop; subwindow(crop, img, std::max(4, s), kExemplar);
        bbz_.set_input_tensor(blob(crop)); bbz_.infer();
        const ov::Tensor z = bbz_.get_output_tensor(0);
        zf_ = ov::Tensor(z.get_element_type(), z.get_shape()); std::memcpy(zf_.data(), z.data(), z.get_byte_size()); have_t_ = true;
    }
    Models* m_; ov::InferRequest bb_, bbz_, hd_; ov::Tensor zf_; bool have_t_ = false;
    float pos_[2] = {0, 0}, sz_[2] = {0, 0}, score_ = 0; cv::Size imgsz_; std::vector<float> hann_;
};
#endif
