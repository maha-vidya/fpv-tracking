// target_tracker -- companion program for high-speed FPV tracking and dash (see README.md and docs/FPV_DASH.md).
// One C++17 translation unit by deliberate choice, organized into commented sections: follow logic (pure, unit-tested by
// --selftest), the SkyDroid C12 TOP protocol, the MAVLink link, video capture, and the tracker application.
//
// =====================================================================================================================================
// GO ("go to the target") -- WHERE IT IS AND HOW IT WORKS.  Every part is marked with a [GO-n] banner: search for "[GO-" to jump.
// =====================================================================================================================================
// What GO does: once a target is LOCKED and followed, the operator presses GO ONCE (--go-confirm: twice, arm then confirm). The drone then
// flies FAST, straight at the target's real-world position -- re-aimed every frame, so a moving target is chased, not the spot it
// left -- and stops at the stop offset (GO SETTINGS below: + before the target, 0 at it, - past it), never lower than --alt-floor
// above its ground. What happens on arrival (landing, a release, ...) is NOT in this program: on_go_arrived() [GO-7] is the empty
// hook for it.
// GO only runs with --enable-rescue (otherwise a GO press just prints that it is disabled).
//
//   [GO-1] INPUT        operator presses -> `rescue_press` (main loop, "operator inputs"): key 'g', window GO button, stdin "go" /
//                       "go SPEED [DIST]", QGC "Rescue GO" = MAV_CMD_USER_1 (param1 = speed, param2 = stop distance), the RC rescue
//                       switch (low -> mid -> high). ABORT: 'x', stdin "abort", QGC MAV_CMD_USER_2.
//   [GO-2] PRESS        fl::RescueController: IDLE -press-> SENT (one tick) -> IDLE, if all readiness gates pass: locked >= 2 s,
//                       GUIDED, armed, link, GPS, EKF, battery, a destination within --rescue-max-range. Refusals say WHICH gate
//                       failed (fl::not_ready_reason). --go-confirm: the old two-step (press -> ARMED + a preview of distance /
//                       up-down / ETA to QGC -> press again within 3 s -> SENT).
//   [GO-3] DESTINATION  main loop "go-to-target": the target's ground position from the camera geometry (fl::view_angles ->
//                       target_ned_from_view, fused with the box-size range) smoothed by the IMM-UKF track fl::TargetEstimator,
//                       turned into the end point by fl::pursuit_point (standoff short, height floor).
//   [GO-4] GUIDANCE     fl::go_velocity (--go-guidance): how the drone flies AT a moving target -- 0 pursuit (where it is),
//                       1 lead, 2 intercept (where it will be), 3 PN proportional navigation (default). fl::pursuit_velocity brakes
//                       so the drone stops at the end point: v <= sqrt(2 a d).
//   [GO-5] START        `ro.send_goto`: this GO's speed / stop distance are fixed, the follow loop hands over (handover_session) and
//                       the go-to becomes the ONLY controller of the drone (`goto_path_active`, `goto_sent`).
//   [GO-6] DASH LOOP    every frame while `goto_path_active`: re-aim at the target (live estimate, or its track predicted up to
//                       1.5 s if hidden), fl::go_velocity -> a velocity + acceleration feed-forward command (update_goto_velocity),
//                       nose pointed at the target (so a FIXED FPV camera keeps looking at it). --goto-paced: the old paced line.
//   [GO-7] ARRIVAL      within ~1 m of the end point: "GO-TO arrived" + on_go_arrived() (the operator's own next step). The pursuit
//                       keeps holding the point relative to the target; it chases again if the target moves > 2 m away.
//   [GO-8] STOPPING     ABORT ('x' / QGC ABORT / stdin "abort"): brake, climb --abort-climb m, hold in GUIDED.  The RC rescue
//                       switch back to low during a dash: BRAKE mode.  QGC sends its own go-to: we back off (goto_overridden).
//                       Target unseen for 3 s: hold the last point (unless it was standing still).  Leaving GUIDED always wins.
//   [GO-9] MAVLINK      MavlinkLink::send_goto / update_goto_velocity / update_goto_point / cancel_goto / stop_point: the
//                       SET_POSITION_TARGET_LOCAL_NED messages, re-sent by the 20 Hz command tick until the pilot leaves GUIDED.
//   [GO-10] CAMERA      gimbal drones only (--gimbal-centre): the gimbal is aimed at the target's world position during the dash.
//                       FPV (fixed camera): nothing to aim -- the dash points the nose at the target ([GO-6] yaw_to).
//   [GO-11] TESTS       test_rescue (one-press GO, the gates, --go-confirm) and test_go_guidance (the guidance laws) in --selftest.
//
// Speed and stop point: the GO SETTINGS block right below. ArduPilot caps the speed too: WP_SPD / WP_ACC (simulation/fpv.parm).
// Safety authority stays with ArduPilot (geofence, failsafes).
// =====================================================================================================================================

// =====================================================================================================================================
// GO SETTINGS -- EDIT HERE.  The values every GO uses unless overridden at run time. Change a number, rebuild
// (cmake --build tracker/build -j), done -- nothing else in the code needs touching.
// Run-time overrides, no rebuild (the most specific wins):
//   1. ONE GO:        type "go SPEED OFFSET" in the tracker's terminal, e.g.  go 15 -10   (15 m/s, stop 10 m PAST the target)
//                     or QGC "Rescue GO" with param1 = speed (m/s), param2 = offset (m)  (0 in either = the setting below)
//   2. ONE LAUNCH:    target_tracker ... --rescue-speed 15 --go-offset -10 --goto-accel 6
//   3. these defaults
// =====================================================================================================================================
namespace go_settings {
// GO speed, m/s: how fast the dash flies. ArduPilot also caps it at WP_SPD (simulation/fpv.parm: 30 m/s) -- to go faster, raise
// that too, and kMaxSpeed below.
constexpr double kSpeed = 30.0;
// WHERE GO STOPS, metres, measured horizontally along the line GO flies:
//   +10  stop 10 m BEFORE the target (short of it)
//     0  stop AT the target
//   -10  fly over the target and stop 10 m PAST it (in the direction GO was flying when it started)
// Negative offset (-0.55 m): compensates for the drone center of mass being 0.55 m behind the nose,
// driving positive penetration through the target center.
constexpr double kStopOffset = -0.55;
// Braking (and speed-up) rate, m/s^2: the dash accelerates and decelerates at this rate so it stops ON the stop point.
// Cap dash acceleration tilt to <= 30 deg (a <= g * tan(30 deg) = 5.66 m/s^2) so target stays firmly inside camera FOV.
constexpr double kAccel = 5.5;
// FLY THROUGH the target, true / false:
//   true   accelerates all the way to the target and passes over it without braking, then brakes smoothly only after it is past.
constexpr bool kFlyThrough = false;
// Limits for run-time values (typos, a wrong QGC param): faster than kMaxSpeed or an offset beyond +-kMaxOffset is clamped.
constexpr double kMaxSpeed = 40.0;
constexpr double kMaxOffset = 100.0;
static_assert(kAccel >= 0.5 && kAccel <= 15.0, "go_settings::kAccel must be 0.5 .. 15 m/s^2");
static_assert(kSpeed > 0.0 && kSpeed <= kMaxSpeed, "go_settings::kSpeed must be > 0 and <= kMaxSpeed");
static_assert(kStopOffset >= -kMaxOffset && kStopOffset <= kMaxOffset, "go_settings::kStopOffset must be within +-kMaxOffset");
}  // namespace go_settings

#include <array>
#include <atomic>
#include <map>
#include <condition_variable>
#include <mutex>
#include <string>
// NOTE: EKF_STATUS_REPORT (message + its EKF_ATTITUDE/EKF_VELOCITY_HORIZ/
// EKF_POS_HORIZ_REL flags) is ArduPilot-specific -- it lives in the
// "ardupilotmega" MAVLink dialect, not the generic "common" one. This
// header pulls common in automatically, so nothing else needs to change.
#include <ardupilotmega/mavlink.h>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <thread>
#include <future>
#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#include <poll.h>
#include <sys/stat.h>
#include <ctime>
#include <gst/app/gstappsrc.h>
#ifdef HAVE_GST_RTSP_SERVER
#include <gst/rtsp-server/rtsp-server.h>
#endif
#include <termios.h>
#include <memory>
#include <set>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/tracking.hpp>   // contrib: TrackerCSRT (present in the Jetson 4.9 build and Ubuntu 4.5.4 packages)
#include <algorithm>
#include <cmath>
#include <fstream>
#include <numeric>
#include <cctype>
#include <cstdlib>
#include <sstream>
#include <deque>
#include <gst/gst.h>
#include <csignal>
#include <optional>
#include <limits>
#include <cstdint>

// ============================================================
// ---- follow logic: pure, deterministic, no I/O (unit-tested by --selftest)
// ============================================================
// Everything the follower decides lives here so it can be tested without a camera, a vehicle or a display.
// Documentation behind the choices (checked against primary sources):
//   * SET_POSITION_TARGET_LOCAL_NED / MAV_FRAME_BODY_NED = BODY_FRD for velocity values: forward, right, DOWN
//     (MAVLink common.xml). So vz > 0 means DESCEND.
//   * ArduPilot Copter GUIDED: velocity commands must be re-sent (about every second) or the vehicle stops after
//     GUID_TIMEOUT; "specify yaw rate of zero to stop vehicle yaw from changing" -> every zero command carries yaw_rate = 0.
//   * ArduPilot does not cap guided velocity (measured in SITL), so the speed ceilings are enforced here.
namespace fl {

constexpr double kPi = 3.14159265358979323846;
inline double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }
inline double sym_clamp(double v, double lim) { return clampd(v, -lim, lim); }
inline double deg2rad(double d) { return d * kPi / 180.0; }
inline double rad2deg(double r) { return r * 180.0 / kPi; }
inline double deadband(double v, double db) {          // shrink toward zero; |v| <= db -> 0
    if (std::fabs(v) <= db) return 0.0;
    return v > 0 ? v - db : v + db;
}

// Body-frame velocity command: vx forward, vy right, vz DOWN (m/s), yaw_rate clockwise (rad/s).
struct Cmd {
    double vx = 0, vy = 0, vz = 0, yaw_rate = 0;
    bool is_zero() const { return vx == 0 && vy == 0 && vz == 0 && yaw_rate == 0; }
};

// Hard safety ceilings (Phase 1 spec P1-06). CLI values above these are clamped at parse time.
struct Ceilings {
    double vx = 3.0, vy = 3.0, vz = 2.0, yaw = 0.5;    // m/s, m/s, m/s, rad/s
    double accel = 2.0, yaw_accel = 2.0;               // m/s^2, rad/s^2 (slew limits)
};

inline Cmd clamp_to_ceilings(Cmd c, const Ceilings& k) {
    c.vx = sym_clamp(c.vx, k.vx); c.vy = sym_clamp(c.vy, k.vy);
    c.vz = sym_clamp(c.vz, k.vz); c.yaw_rate = sym_clamp(c.yaw_rate, k.yaw);
    return c;
}

// Same ceilings, but scales vx/vy/vz by ONE factor instead of clamping each axis independently -- clamp_to_ceilings bends the
// direction of travel whenever more than one axis is saturated (e.g. a command wanting fast forward + fast climb gets vx cut
// to its ceiling and vz cut to ITS ceiling, which is not the same as the original vector scaled down; flagged in review).
// yaw_rate is unrelated to the translation vector's direction, so it is still clamped on its own.
inline Cmd clamp_keep_direction(Cmd c, const Ceilings& k) {
    double s = 1.0;
    if (std::fabs(c.vx) > k.vx) s = std::min(s, k.vx / std::fabs(c.vx));
    if (std::fabs(c.vy) > k.vy) s = std::min(s, k.vy / std::fabs(c.vy));
    if (std::fabs(c.vz) > k.vz) s = std::min(s, k.vz / std::fabs(c.vz));
    c.vx *= s; c.vy *= s; c.vz *= s;
    c.yaw_rate = sym_clamp(c.yaw_rate, k.yaw);
    return c;
}

// Per-axis acceleration limiter. A zero/emergency command bypasses it (see gate_command).
class Slew {
public:
    Cmd step(const Cmd& target, double dt, const Ceilings& k) {
        if (dt <= 0) return cur_;
        cur_.vx = toward(cur_.vx, target.vx, k.accel * dt);
        cur_.vy = toward(cur_.vy, target.vy, k.accel * dt);
        cur_.vz = toward(cur_.vz, target.vz, k.accel * dt);
        cur_.yaw_rate = toward(cur_.yaw_rate, target.yaw_rate, k.yaw_accel * dt);
        return cur_;
    }
    void reset() { cur_ = Cmd(); }
    const Cmd& last() const { return cur_; }
private:
    static double toward(double cur, double tgt, double max_step) {
        double d = tgt - cur;
        if (d > max_step) return cur + max_step;
        if (d < -max_step) return cur - max_step;
        return tgt;
    }
    Cmd cur_;
};

// ---------------------------------------------------------------------------
// Box and constant-velocity Kalman filter on (cx, cy, w, h). Used ONLY to smooth the control input and to coast
// for display; measured (Phase 0): an innovation gate cannot detect loss or drift, so it is not used for that.
// ---------------------------------------------------------------------------
struct Box {
    double x = 0, y = 0, w = 0, h = 0;                  // top-left corner and size, pixels
    double cx() const { return x + w / 2; }
    double cy() const { return y + h / 2; }
    bool valid() const { return w > 1 && h > 1; }
};

inline double iou(const Box& a, const Box& b) {
    double ix = std::max(0.0, std::min(a.x + a.w, b.x + b.w) - std::max(a.x, b.x));
    double iy = std::max(0.0, std::min(a.y + a.h, b.y + b.h) - std::max(a.y, b.y));
    double inter = ix * iy, uni = a.w * a.h + b.w * b.h - inter;
    return uni > 0 ? inter / uni : 0.0;
}

// Plausibility of a tracker box against the Kalman prediction. CSRT can jump to the wrong place and still report success (seen in the
// simulator: the box jumped 300 px to the frame corner in one frame). Reject such a box: it is not accepted, the box ages, and the
// state machine goes COASTING -> LOST. (Slow drift is NOT caught by this; that needs the appearance check of Phase 2.)
struct PlausCfg { double jump_diag = 3.0, min_jump_px = 60.0, size_lo = 0.7, size_hi = 1.4; };
// steps: frames elapsed since the last update (see BoxKalman::predict) -- the allowed jump grows with the time actually elapsed.
inline bool box_plausible(const Box& pred, const Box& meas, double frame_w, double frame_h, const PlausCfg& c = PlausCfg(), double steps = 1.0) {
    if (!meas.valid()) return false;
    if (meas.cx() < 0 || meas.cx() > frame_w || meas.cy() < 0 || meas.cy() > frame_h) return false;          // centre must be inside the frame
    double diag = std::hypot(pred.w, pred.h);
    double lim = std::max(c.min_jump_px, c.jump_diag * diag) * std::max(1.0, steps);
    if (std::hypot(meas.cx() - pred.cx(), meas.cy() - pred.cy()) > lim) return false;                        // moved too far in one frame
    double r = meas.h / std::max(1.0, pred.h);
    return r >= c.size_lo && r <= c.size_hi;                                                                 // size changed too abruptly
}

// Phase 2 appearance check (the gap box_plausible's comment names above): when a second object crosses close to the
// locked target, CSRT can drift its box onto the crosser smoothly enough -- small frame-to-frame position/size steps
// -- to pass box_plausible every frame, yet the box is now following the wrong thing. A crossing car/person is very
// rarely the same colour as the locked target, so a cheap mean-BGR-colour check against the box captured at lock time
// catches most of this: not a real appearance model, just a guard this specific failure mode (lost in a crowd of
// differently-coloured actors). Returns a sentinel (-1,-1,-1) for a box too small/out-of-frame to sample meaningfully;
// callers must treat that as "can't tell" (accept), not "mismatch" (reject), e.g. a box clipped at the frame edge.
// Samples the INNER 60% of the box (both at lock time and every frame, so the two stay comparable): a tracker box always
// includes some background around the target -- road, grass, sky -- which diluted the old whole-box mean and made a crossing
// object of a different colour harder to tell apart.
inline cv::Scalar box_mean_color(const cv::Mat& frame, const Box& b) {
    cv::Rect r((int)std::lround(b.x + 0.2 * b.w), (int)std::lround(b.y + 0.2 * b.h), (int)std::lround(0.6 * b.w), (int)std::lround(0.6 * b.h));
    r &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (r.width < 4 || r.height < 4) return cv::Scalar(-1, -1, -1);
    return cv::mean(frame(r));
}
inline bool color_plausible(const cv::Scalar& ref, const cv::Scalar& meas, double max_dist = 60.0) {
    if (ref[0] < 0 || meas[0] < 0) return true;                       // no reliable sample on one side: don't reject on this alone
    double d2 = 0; for (int i = 0; i < 3; i++) { double e = ref[i] - meas[i]; d2 += e * e; }
    return std::sqrt(d2) <= max_dist;
}

// Lock-time refinement of a ROUGH selection (a click, a QGC tap, the centre LOCK -- not a carefully drawn rectangle): OpenCV
// GrabCut separates the object from the background inside the box (colour/edge graph cut, no model, a few ms on a small ROI),
// and the box shrinks/shifts to the object's own extent. A tight initial box matters for CSRT (less background in its filter)
// and for the size reference the follow loop holds. Refuses (returns false) whenever the cut looks unreliable -- nearly
// nothing or nearly everything marked foreground, or the object not under the click -- and the caller keeps the rough box.
inline bool grabcut_refine(const cv::Mat& frame, const Box& rough, Box& out, int max_side = 160, int iters = 3) {
    if (!rough.valid() || frame.empty()) return false;
    const cv::Rect img(0, 0, frame.cols, frame.rows);
    cv::Rect roi((int)std::lround(rough.x - 0.25 * rough.w), (int)std::lround(rough.y - 0.25 * rough.h),
                 (int)std::lround(1.5 * rough.w), (int)std::lround(1.5 * rough.h));
    roi &= img;
    if (roi.width < 16 || roi.height < 16) return false;
    const double sc = std::min(1.0, (double)max_side / std::max(roi.width, roi.height));   // bound the cost: <= max_side px on a side
    cv::Mat small; cv::resize(frame(roi), small, cv::Size(), sc, sc, cv::INTER_AREA);
    cv::Rect r((int)std::lround((rough.x - roi.x) * sc), (int)std::lround((rough.y - roi.y) * sc),
               (int)std::lround(rough.w * sc), (int)std::lround(rough.h * sc));
    r &= cv::Rect(0, 0, small.cols, small.rows);
    if (r.width < 6 || r.height < 6) return false;
    cv::Mat mask(small.size(), CV_8UC1, cv::Scalar(cv::GC_BGD)), bgd, fgd;
    try { cv::grabCut(small, mask, r, bgd, fgd, iters, cv::GC_INIT_WITH_RECT); } catch (const cv::Exception&) { return false; }
    cv::Mat fg = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);
    cv::Mat lab, stats, cents; int n = cv::connectedComponentsWithStats(fg, lab, stats, cents);
    const cv::Point c((int)std::lround((rough.cx() - roi.x) * sc), (int)std::lround((rough.cy() - roi.y) * sc));
    int best = -1, best_area = 0;
    for (int i = 1; i < n; i++) {
        const cv::Rect bb(stats.at<int>(i, 0), stats.at<int>(i, 1), stats.at<int>(i, 2), stats.at<int>(i, 3));
        const int area = stats.at<int>(i, cv::CC_STAT_AREA);
        const bool under_click = bb.contains(c) || lab.at<int>(std::min(c.y, lab.rows - 1), std::min(c.x, lab.cols - 1)) == i;
        if (under_click && area > best_area) { best = i; best_area = area; }
    }
    if (best < 0) return false;
    const double frac = (double)best_area / std::max(1, r.area());
    if (frac < 0.05 || frac > 0.9) return false;                         // nothing found / no separation from the background
    const cv::Rect bb(stats.at<int>(best, 0), stats.at<int>(best, 1), stats.at<int>(best, 2), stats.at<int>(best, 3));
    out = Box{roi.x + bb.x / sc, roi.y + bb.y / sc, bb.width / sc, bb.height / sc};
    return out.w >= 8 && out.h >= 8;
}

// Lock box from a single POINT (a click, a QGC tap, the centre LOCK): GrabCut seeded at the point decides the object's extent --
// no fixed-size box is assumed anywhere. The point is marked definite foreground, the rest of a square window around it
// probable background (its outer border definite background), and the window GROWS (64 -> 128 -> 256 px) while the
// segmented object still touches its edge, so a small person and a large vehicle both come out at their own size. Refuses
// (returns false, `why` says why) when no object can be separated -- e.g. a click on open road -- instead of inventing a box.
inline bool grabcut_from_point(const cv::Mat& frame, double px, double py, Box& out, std::string& why) {
    if (frame.empty() || px < 0 || py < 0 || px >= frame.cols || py >= frame.rows) { why = "click outside the image"; return false; }
    const cv::Rect img(0, 0, frame.cols, frame.rows);
    static const int gc_min_side = std::getenv("GC_MIN_SIDE") ? std::atoi(std::getenv("GC_MIN_SIDE")) : 64;   // tuning hook
    for (int side : {64, 128, 256}) {                                   // 384 cost up to 1.7 s live and mostly found ground regions
        if (side < gc_min_side) continue;
        cv::Rect roi((int)std::lround(px - side / 2.0), (int)std::lround(py - side / 2.0), side, side);
        roi &= img;
        if (roi.width < 12 || roi.height < 12) { why = "too close to the image edge"; return false; }
        const double sc = std::min(1.0, 160.0 / std::max(roi.width, roi.height));
        cv::Mat small; cv::resize(frame(roi), small, cv::Size(), sc, sc, cv::INTER_AREA);
        // a window with (almost) no texture has no boundary to find: GrabCut's GMMs degenerate on it and it took up to 6 s on one
        // zero-variance window (review, measured) while every later click was dropped. Skip it -- a larger window may reach the
        // object's edge (a click in the middle of a big plain vehicle); all flat: refused.
        { cv::Scalar mu, sd; cv::meanStdDev(small, mu, sd);
          if (std::max({sd[0], sd[1], sd[2]}) < 2.0) { why = "nothing stands out (flat area)"; continue; } }
        const cv::Point c((int)std::lround((px - roi.x) * sc), (int)std::lround((py - roi.y) * sc));
        const int rs = std::max(2, (int)std::lround(0.04 * std::min(small.cols, small.rows)));
        cv::Mat mask(small.size(), CV_8UC1, cv::Scalar(cv::GC_PR_BGD)), bgd, fgd;
        // the window's own border is definite background -- but not where the window was cut by the image edge
        const int bd = 2;
        if (roi.x > 0) mask(cv::Rect(0, 0, bd, mask.rows)).setTo(cv::GC_BGD);
        if (roi.y > 0) mask(cv::Rect(0, 0, mask.cols, bd)).setTo(cv::GC_BGD);
        if (roi.x + roi.width < frame.cols) mask(cv::Rect(mask.cols - bd, 0, bd, mask.rows)).setTo(cv::GC_BGD);
        if (roi.y + roi.height < frame.rows) mask(cv::Rect(0, mask.rows - bd, mask.cols, bd)).setTo(cv::GC_BGD);
        cv::circle(mask, c, 2 * rs, cv::Scalar(cv::GC_PR_FGD), cv::FILLED);
        cv::circle(mask, c, rs, cv::Scalar(cv::GC_FGD), cv::FILLED);
        try { cv::grabCut(small, mask, cv::Rect(), bgd, fgd, 4, cv::GC_INIT_WITH_MASK); }
        catch (const cv::Exception&) { why = "segmentation failed"; return false; }
        cv::Mat fg = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);
        cv::Mat lab, stats, cents; int n = cv::connectedComponentsWithStats(fg, lab, stats, cents);
        const int k = (c.y < lab.rows && c.x < lab.cols) ? lab.at<int>(c.y, c.x) : 0;
        if (k <= 0 || k >= n) { why = "nothing separable under the click"; return false; }
        const cv::Rect bb(stats.at<int>(k, 0), stats.at<int>(k, 1), stats.at<int>(k, 2), stats.at<int>(k, 3));
        const int area = stats.at<int>(k, cv::CC_STAT_AREA);
        // does the object run into a window edge that is NOT the image edge? then the window was too small: grow it
        const bool touches = (bb.x <= bd && roi.x > 0) || (bb.y <= bd && roi.y > 0) ||
                             (bb.x + bb.width >= small.cols - bd && roi.x + roi.width < frame.cols) ||
                             (bb.y + bb.height >= small.rows - bd && roi.y + roi.height < frame.rows);
        // only the seed we forced as foreground came back: GrabCut found NO boundary in this window (it lies inside one uniform
        // region -- inside a large object, or on open road). Measured: that otherwise looked like a tiny 7x7 "object".
        const bool seed_only = bb.width <= 2 * rs + 2 && bb.height <= 2 * rs + 2;
        if (touches || seed_only) continue;
        if (area < 6) { why = "object too small to separate"; return false; }
        if ((double)area / small.total() > 0.7) { why = "no object boundary found (background?)"; return false; }
        out = Box{roi.x + bb.x / sc, roi.y + bb.y / sc, bb.width / sc, bb.height / sc};
        if (out.w < 4 || out.h < 4) { why = "object too small to separate"; return false; }
        // Second pass: the region under the click may be only PART of the object -- measured on live sim frames, a click near
        // the top of the target cube locked just its lit top face (14x10 of 16x22); a click on a person's head would lock the
        // head. GrabCut again on a rectangle around that part (grown 2.2x): it learns the object from the whole rectangle and
        // can join the adjacent parts of the same object, while what is clearly background stays out. Accepted only if it
        // still contains the click, grew, and did not just fill the rectangle (= found a real boundary).
        // Repeated while it keeps growing (up to 3 passes), so a part far from the object's centre still reaches the rest of it.
        const double first_area = out.w * out.h;
        static const double merge_cap = std::getenv("GC_MERGE_CAP") ? std::atof(std::getenv("GC_MERGE_CAP")) : 2.5;   // tuning hook
        static const bool gc_debug = std::getenv("GC_DEBUG") != nullptr;   // --grabcut-test tracing
        if (gc_debug) std::printf("  window %d: first region %.0f %.0f %.0fx%.0f\n", side, out.x, out.y, out.w, out.h);
        for (int pass = 0; pass < 3; pass++) {
            const Box part = out;
            const Box grown{part.cx() - 1.1 * part.w, part.cy() - 1.1 * part.h, 2.2 * part.w, 2.2 * part.h};
            Box whole;
            if (gc_debug) {
                const bool r = grabcut_refine(frame, grown, whole);
                std::printf("  pass %d: rect %.0fx%.0f -> %s %.0f %.0f %.0fx%.0f (area x%.1f of first, fill %.2f of rect)\n", pass, grown.w, grown.h,
                            r ? "got" : "refused", whole.x, whole.y, whole.w, whole.h, whole.w * whole.h / first_area, whole.w * whole.h / (grown.w * grown.h));
            }
            if (!(grabcut_refine(frame, grown, whole) && px >= whole.x && px <= whole.x + whole.w && py >= whole.y && py <= whole.y + whole.h &&
                  whole.w * whole.h > 1.15 * part.w * part.h && whole.w * whole.h < 0.85 * grown.w * grown.h &&
                  whole.w * whole.h <= merge_cap * first_area)) break;          // joins parts of ONE object; never balloons into the scene
            out = whole;
        }
        // an "object" this large is almost always ground (a road strip between markings, a field): measured live, clicks there
        // produced 150x145 and 290x138 locks
        if (out.w > 0.45 * frame.cols || out.h > 0.45 * frame.rows) { why = "selection too large -- looks like ground, not an object"; return false; }
        return true;
    }
    why = "no object boundary found around the click (larger than 256 px, or not separable from the background)";
    return false;
}

// Motion-assisted lock box. A click on a multi-part object (a car: body, windows, wheels, roof) gives GrabCut only the part under
// the click -- measured on live sim car footage with exact ground truth: mean IoU 0.23, 16% of clicks usable. But all parts of a
// moving object move TOGETHER: the region that moves like the clicked pixel is the object.
//   1. the drone's own camera motion between `prev` and `cur` is removed (similarity transform from tracked corner features,
//      RANSAC -- the ground dominates the features)
//   2. dense optical flow (Farneback) in a window around the click; the click's own residual motion v0 (median of a 5x5 patch)
//   3. the connected set of pixels moving like v0 (within 40% of its speed) and actually moving, containing the click
// Returns false when the clicked spot is not moving relative to the ground (|v0| < 1 px between the two frames) -- then colour
// GrabCut alone decides. `prev` and `cur` must be the same size; coordinates are in that image's pixels.
inline bool flow_region(const cv::Mat& prev, const cv::Mat& cur, double px, double py, Box& out, double& speed_px) {
    speed_px = 0;
    if (prev.empty() || cur.empty() || prev.size() != cur.size()) return false;
    // Only the neighbourhood of the click is used (the window + a margin for the camera-motion features): the whole-frame features,
    // warp and Farneback flow cost 43-95 ms per click even when nothing moved (review, measured) -- now a few ms. Dense flow is
    // DIS (ULTRAFAST: ~2 ms on the same window vs 38 ms Farneback).
    const int r = 128;
    cv::Rect roi((int)std::lround(px - r), (int)std::lround(py - r), 2 * r, 2 * r); roi &= cv::Rect(0, 0, cur.cols, cur.rows);
    if (roi.width < 16 || roi.height < 16) return false;
    cv::Rect big(roi.x - 96, roi.y - 96, roi.width + 192, roi.height + 192); big &= cv::Rect(0, 0, cur.cols, cur.rows);
    cv::Mat g0, g1;
    cv::cvtColor(prev(big), g0, cv::COLOR_BGR2GRAY); cv::cvtColor(cur(big), g1, cv::COLOR_BGR2GRAY);
    std::vector<cv::Point2f> p0, p1; std::vector<uchar> st; std::vector<float> err;
    cv::goodFeaturesToTrack(g0, p0, 300, 0.01, 7);
    if (p0.size() < 12) return false;
    cv::calcOpticalFlowPyrLK(g0, g1, p0, p1, st, err);
    std::vector<cv::Point2f> a, b;
    for (size_t k = 0; k < p0.size(); k++) if (st[k]) { a.push_back(p0[k]); b.push_back(p1[k]); }
    if (a.size() < 12) return false;
    cv::Mat A = cv::estimateAffinePartial2D(a, b, cv::noArray(), cv::RANSAC, 2.0);   // in `big` coordinates
    if (A.empty()) return false;
    const cv::Rect rin(roi.x - big.x, roi.y - big.y, roi.width, roi.height);           // the window inside `big`
    cv::Mat Aw = A.clone(); Aw.at<double>(0, 2) -= rin.x; Aw.at<double>(1, 2) -= rin.y;  // ... output directly in window coordinates
    cv::Mat w0; cv::warpAffine(g0, w0, Aw, rin.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
    // warpAffine maps output(x) = input(Aw^-1 x) unless WARP_INVERSE_MAP: Aw maps g0 -> window, as needed
    cv::Mat flow;
    {
        static thread_local cv::Ptr<cv::DISOpticalFlow> dis = cv::DISOpticalFlow::create(cv::DISOpticalFlow::PRESET_FAST);
        dis->calc(w0, g1(rin).clone(), flow);
    }
    const int cx = (int)std::lround(px - roi.x), cy = (int)std::lround(py - roi.y);
    if (cx < 0 || cy < 0 || cx >= flow.cols || cy >= flow.rows) return false;
    std::vector<float> vx, vy;
    for (int y = std::max(0, cy - 2); y <= std::min(flow.rows - 1, cy + 2); y++)
        for (int x = std::max(0, cx - 2); x <= std::min(flow.cols - 1, cx + 2); x++) { auto v = flow.at<cv::Point2f>(y, x); vx.push_back(v.x); vy.push_back(v.y); }
    std::nth_element(vx.begin(), vx.begin() + vx.size() / 2, vx.end()); std::nth_element(vy.begin(), vy.begin() + vy.size() / 2, vy.end());
    const double v0x = vx[vx.size() / 2], v0y = vy[vy.size() / 2], s0 = std::hypot(v0x, v0y);
    speed_px = s0;
    if (s0 < 1.0) return false;                                       // the clicked spot is not moving
    cv::Mat m(flow.size(), CV_8UC1, cv::Scalar(0));
    const double tol = std::max(1.0, 0.4 * s0);
    for (int y = 0; y < flow.rows; y++) {
        const cv::Point2f* f = flow.ptr<cv::Point2f>(y); uchar* mm = m.ptr<uchar>(y);
        for (int x = 0; x < flow.cols; x++)
            mm[x] = (std::hypot(f[x].x - v0x, f[x].y - v0y) < tol && std::hypot(f[x].x, f[x].y) > 0.5 * s0) ? 1 : 0;
    }
    cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8UC1));
    cv::Mat lab, stats, cents; cv::connectedComponentsWithStats(m, lab, stats, cents);
    const int k = lab.at<int>(cy, cx);
    if (k <= 0 || stats.at<int>(k, cv::CC_STAT_AREA) < 40) return false;
    out = Box{(double)roi.x + stats.at<int>(k, 0), (double)roi.y + stats.at<int>(k, 1), (double)stats.at<int>(k, 2), (double)stats.at<int>(k, 3)};
    return true;
}

// GrabCut initialised from a moving region `mbox`: inside it = probable object, the click = object, a ring around it = probable
// background, everything further out = background. Refines the flow region's rough extent to the object's real outline.
inline bool grabcut_from_region(const cv::Mat& frame, double px, double py, const Box& mbox, Box& out) {
    const double pad = 0.6;
    cv::Rect roi((int)std::floor(std::min(mbox.x - pad * mbox.w, px - 6)), (int)std::floor(std::min(mbox.y - pad * mbox.h, py - 6)), 0, 0);
    roi.width = (int)std::ceil(std::max(mbox.x + mbox.w * (1 + pad), px + 7)) - roi.x;
    roi.height = (int)std::ceil(std::max(mbox.y + mbox.h * (1 + pad), py + 7)) - roi.y;
    roi &= cv::Rect(0, 0, frame.cols, frame.rows);
    if (roi.width < 12 || roi.height < 12) return false;
    cv::Mat m(roi.size(), CV_8UC1, cv::Scalar(cv::GC_BGD)), bgd, fgd;
    const cv::Rect ring((int)std::lround(mbox.x - roi.x - 0.25 * mbox.w), (int)std::lround(mbox.y - roi.y - 0.25 * mbox.h), (int)std::lround(1.5 * mbox.w), (int)std::lround(1.5 * mbox.h));
    m(ring & cv::Rect(0, 0, m.cols, m.rows)).setTo(cv::GC_PR_BGD);
    const cv::Rect inner((int)std::lround(mbox.x - roi.x), (int)std::lround(mbox.y - roi.y), (int)std::lround(mbox.w), (int)std::lround(mbox.h));
    m(inner & cv::Rect(0, 0, m.cols, m.rows)).setTo(cv::GC_PR_FGD);
    const cv::Point c((int)std::lround(px - roi.x), (int)std::lround(py - roi.y));
    cv::circle(m, c, 2, cv::Scalar(cv::GC_FGD), cv::FILLED);
    try { cv::grabCut(frame(roi), m, cv::Rect(), bgd, fgd, 4, cv::GC_INIT_WITH_MASK); } catch (const cv::Exception&) { return false; }
    cv::Mat fg = (m == cv::GC_FGD) | (m == cv::GC_PR_FGD);
    cv::Mat lab, stats, cents; cv::connectedComponentsWithStats(fg, lab, stats, cents);
    if (c.x < 0 || c.y < 0 || c.x >= lab.cols || c.y >= lab.rows) return false;
    const int k = lab.at<int>(c.y, c.x);
    if (k <= 0) return false;
    out = Box{(double)roi.x + stats.at<int>(k, 0), (double)roi.y + stats.at<int>(k, 1), (double)stats.at<int>(k, 2), (double)stats.at<int>(k, 3)};
    return out.w >= 4 && out.h >= 4;
}

// The lock box for a click: colour GrabCut (P) always; if the clicked spot is moving, also the motion region refined by GrabCut (M).
// M wins when the click's colour region lies inside it and M is clearly larger (the click hit one PART of a moving object);
// otherwise P (the colour region IS the object, or the motion belongs to something else, e.g. a car passing a slow target).
// Measured on live sim footage with exact ground truth (car 245 clicks, red/orange boxes ~140 each, mean IoU):
// car 0.23 -> 0.73, red 0.73 -> 0.70, orange 0.68 -> 0.67; usable locks (IoU >= 0.5): car 16% -> 94%, red 82 -> 85%, orange 84 -> 90%.
// Lock box from a click by modelling the GROUND around it, not the object: the border ring of a square window around the click
// is clustered (k-means, 3 Lab colours -- asphalt, paint, grass); every pixel far from all of them is "not ground", and the
// connected not-ground region under the click (gaps closed, holes filled) is the object. A multi-coloured object -- a car's body,
// windows and wheels, a person's head, shirt and legs -- comes out whole, where GrabCut seeded at the click grows only the
// clicked colour (measured: a parked car locked as its 61x20 window strip, people padded from a 14x10 seed). Lightness counts
// 0.6x so a shadow (darker ground, same colour) is mostly left out. The window grows 48 -> 96 -> 192 -> 320 px; the answer is
// the smallest window whose box stays the same when the window doubles (its border was really ground, not the object).
// Measured on 50 red-cube clicks vs its true pixels: median IoU 0.95 (point GrabCut path: 0.68), bad (< 0.3) 4% (10%).
inline bool ground_side(const cv::Mat& frame, double px, double py, int side, Box& out, bool& touch) {
    touch = false;
    const cv::Rect img(0, 0, frame.cols, frame.rows);
    const cv::Rect roi = cv::Rect((int)std::lround(px - side / 2.0), (int)std::lround(py - side / 2.0), side, side) & img;
    if (roi.width < 16 || roi.height < 16) return false;
    // big windows are worked on a <= 128 px copy (measured: ~200 ms per click at full size, most of it here)
    const double sc = std::min(1.0, 128.0 / std::max(roi.width, roi.height));
    cv::Mat small; if (sc < 1.0) cv::resize(frame(roi), small, cv::Size(), sc, sc, cv::INTER_AREA); else small = frame(roi);
    cv::Mat lab; cv::cvtColor(small, lab, cv::COLOR_BGR2Lab); lab.convertTo(lab, CV_32FC3);
    const int h = lab.rows, w = lab.cols, m = std::max(3, (int)(0.08 * std::min(h, w)));
    std::vector<cv::Vec3f> ring; ring.reserve(4 * m * (w + h));
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++)
        if (y < m || y >= h - m || x < m || x >= w - m) { cv::Vec3f v = lab.at<cv::Vec3f>(y, x); v[0] *= 0.6f; ring.push_back(v); }
    if (ring.size() < 30) return false;
    std::vector<cv::Vec3f> sample;                                       // <= 1500 border pixels are plenty for 3 ground colours
    const size_t stride = std::max<size_t>(1, ring.size() / 1500);
    for (size_t i = 0; i < ring.size(); i += stride) sample.push_back(ring[i]);
    cv::Mat rp(sample.size(), 3, CV_32F, sample.data()), lbl, cen;
    // deterministic k-means, started from the ground's own brightness terciles (dark / mid / light: asphalt, shadow, paint) -- random
    // k-means++ starts gave a different lock run to run on the same click (a large truck: right box one run, the colour fallback the next)
    { std::vector<float> Ls(rp.rows); for (int q = 0; q < rp.rows; q++) Ls[q] = rp.at<float>(q, 0);
      std::vector<float> srt = Ls; std::sort(srt.begin(), srt.end()); const float t1 = srt[srt.size() / 3], t2 = srt[2 * srt.size() / 3];
      lbl.create(rp.rows, 1, CV_32S); for (int q = 0; q < rp.rows; q++) lbl.at<int>(q) = Ls[q] < t1 ? 0 : (Ls[q] < t2 ? 1 : 2); }
    cv::kmeans(rp, 3, lbl, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0), 1, cv::KMEANS_USE_INITIAL_LABELS, cen);
    auto dist = [&](cv::Vec3f v) { v[0] *= 0.6f; float b = 1e9f;
        for (int k = 0; k < 3; k++) { const float a = v[0] - cen.at<float>(k, 0), c = v[1] - cen.at<float>(k, 1), d = v[2] - cen.at<float>(k, 2); b = std::min(b, a * a + c * c + d * d); }
        return std::sqrt(b); };
    cv::Mat d(h, w, CV_32F); std::vector<float> rd; rd.reserve(ring.size());
    for (int y = 0; y < h; y++) for (int x = 0; x < w; x++) {
        const float v = dist(lab.at<cv::Vec3f>(y, x)); d.at<float>(y, x) = v;
        if (y < m || y >= h - m || x < m || x >= w - m) rd.push_back(v);
    }
    std::nth_element(rd.begin(), rd.begin() + (long)(0.95 * rd.size()), rd.end());
    const float T = std::max(8.0f, 1.6f * rd[(size_t)(0.95 * rd.size())]);
    cv::Mat fg = d > T;
    const int ks = std::max(3, (int)std::lround(side * sc / 24)) | 1;
    cv::morphologyEx(fg, fg, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(ks, ks)));
    { cv::Mat ff = fg.clone(); cv::floodFill(ff, cv::Point(0, 0), cv::Scalar(255)); fg |= ~ff; }   // fill holes (windows inside a car)
    cv::Mat labm, st, cc; const int n = cv::connectedComponentsWithStats(fg, labm, st, cc);
    const int cx = std::clamp((int)std::lround((px - roi.x) * sc), 0, w - 1), cy = std::clamp((int)std::lround((py - roi.y) * sc), 0, h - 1);
    int best = -1; double bd = 1e9;
    for (int i = 1; i < n; i++) {
        if (st.at<int>(i, cv::CC_STAT_AREA) < 12) continue;
        if (labm.at<int>(cy, cx) == i) { best = i; break; }
        const int bx = st.at<int>(i, 0), by = st.at<int>(i, 1), bw = st.at<int>(i, 2), bh = st.at<int>(i, 3);
        const double dd = std::hypot(std::max({bx - cx, 0, cx - bx - bw}), std::max({by - cy, 0, cy - by - bh}));
        if (dd < bd && dd <= std::max(0.08 * side, 8.0) * sc) { bd = dd; best = i; }   // >= 8 px: a click 1-3 px beside a small object missed it (review)
    }
    if (best < 0) return false;
    int bx = st.at<int>(best, 0), by = st.at<int>(best, 1), bw = st.at<int>(best, 2), bh = st.at<int>(best, 3);
    if (bx <= 1 || by <= 1 || bx + bw >= w - 1 || by + bh >= h - 1) { touch = true; return false; }
    // SHADOW trim: pixels of the object that have a ground colour's chroma (a, b) but are darker are its shadow (measured: the lock
    // box was 1.19x too wide, 1.12x too tall on the red box -- its shadow -- and CSRT then kept that size). Only when the click
    // stays on the trimmed object and >= 40% of it remains, so a dark, colourless object (a black car on asphalt) is never erased.
    {
        cv::Mat comp = (labm == best), keep = comp.clone();
        for (int y = by; y < by + bh; y++) for (int x = bx; x < bx + bw; x++) {
            if (!comp.at<uchar>(y, x)) continue;
            const cv::Vec3f v = lab.at<cv::Vec3f>(y, x);
            for (int k = 0; k < 3; k++) {
                const float gl = cen.at<float>(k, 0) / 0.6f, ga = cen.at<float>(k, 1), gb = cen.at<float>(k, 2);
                if (std::hypot(v[1] - ga, v[2] - gb) < 6.0f && v[0] < gl - 10.0f && v[0] > 0.25f * gl) { keep.at<uchar>(y, x) = 0; break; }
            }
        }
        cv::morphologyEx(keep, keep, cv::MORPH_OPEN, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3, 3)));
        cv::Mat l2, s2, c2; const int n2 = cv::connectedComponentsWithStats(keep, l2, s2, c2);
        const int k2 = l2.at<int>(cy, cx);
        if (n2 > 1 && k2 > 0 && s2.at<int>(k2, cv::CC_STAT_AREA) >= 0.4 * cv::countNonZero(comp)) {
            bx = s2.at<int>(k2, 0); by = s2.at<int>(k2, 1); bw = s2.at<int>(k2, 2); bh = s2.at<int>(k2, 3);
        }
    }
    if (bw * bh > 0.6 * w * h) return false;
    out = Box{roi.x + bx / sc, roi.y + by / sc, bw / sc, bh / sc};
    return true;
}
inline bool ground_model_from_point(const cv::Mat& frame, double px, double py, Box& out) {
    // up to 480 px: a car close below the drone is 250-350 px wide (the 320 px window refused it)
    Box r[5]; bool ok[5] = {false, false, false, false, false}, t[5]; const int sides[5] = {48, 96, 192, 320, 480};
    static const bool gm_dbg = std::getenv("GM_DEBUG") != nullptr;
    ok[0] = ground_side(frame, px, py, sides[0], r[0], t[0]);
    if (gm_dbg) std::printf("  gm window %d: ok %d touch %d box %.0f %.0f %.0fx%.0f\n", sides[0], ok[0], t[0], r[0].x, r[0].y, r[0].w, r[0].h);
    for (int k = 0; k < 4; k++) {                                         // stop at the first stable pair (most locks: 2 windows)
        ok[k + 1] = ground_side(frame, px, py, sides[k + 1], r[k + 1], t[k + 1]);
        if (gm_dbg) std::printf("  gm window %d: ok %d touch %d box %.0f %.0f %.0fx%.0f\n", sides[k + 1], ok[k + 1], t[k + 1], r[k + 1].x, r[k + 1].y, r[k + 1].w, r[k + 1].h);
        if (ok[k] && ok[k + 1] && iou(r[k], r[k + 1]) > 0.7) { out = r[k]; return out.w >= 6 && out.h >= 6; }
    }
    for (int k = 0; k < 5; k++) if (ok[k]) { out = r[k]; return out.w >= 6 && out.h >= 6; }
    return false;
}

// How much a box stands out from the ring around it: distance between the inner half's mean Lab colour and the ring's (a thing on
// the ground ~20-80, a patch of sky / plain road / grass < 8). The verification suite caught a click on empty sky locking a 32x20
// "object" through the point-GrabCut fallback -- that fallback now needs this contrast.
inline double box_contrast(const cv::Mat& frame, const Box& b) {
    const cv::Rect img(0, 0, frame.cols, frame.rows);
    const cv::Rect in = cv::Rect((int)std::lround(b.x + 0.25 * b.w), (int)std::lround(b.y + 0.25 * b.h), std::max(1, (int)std::lround(0.5 * b.w)), std::max(1, (int)std::lround(0.5 * b.h))) & img;
    const cv::Rect out = cv::Rect((int)std::lround(b.x - 0.5 * b.w), (int)std::lround(b.y - 0.5 * b.h), (int)std::lround(2 * b.w), (int)std::lround(2 * b.h)) & img;
    const cv::Rect bb = cv::Rect((int)std::lround(b.x), (int)std::lround(b.y), (int)std::lround(b.w), (int)std::lround(b.h)) & img;
    if (in.area() < 4 || out.area() <= bb.area()) return 0.0;
    cv::Mat lab; cv::cvtColor(frame(out), lab, cv::COLOR_BGR2Lab);
    cv::Mat mring(out.size(), CV_8UC1, cv::Scalar(255)); mring(cv::Rect(bb.x - out.x, bb.y - out.y, bb.width, bb.height)).setTo(0);
    const cv::Scalar mi = cv::mean(lab(cv::Rect(in.x - out.x, in.y - out.y, in.width, in.height))), mo = cv::mean(lab, mring);
    return std::sqrt((mi[0] - mo[0]) * (mi[0] - mo[0]) + (mi[1] - mo[1]) * (mi[1] - mo[1]) + (mi[2] - mo[2]) * (mi[2] - mo[2]));
}

inline bool lock_box_from_click(const cv::Mat& frame, const cv::Mat& prev, double prev_scale, double px, double py, Box& out, std::string& why, std::string& how,
                                bool allow_point = true) {
    static const bool timing = std::getenv("LOCK_TIMING") != nullptr;
    auto ms_since = [](std::chrono::steady_clock::time_point t) { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t).count(); };
    auto tg = std::chrono::steady_clock::now();
    Box G; bool have_g = ground_model_from_point(frame, px, py, G);
    // a "found object" that looks like its own surroundings is ground (a shadow edge, a paint boundary): measured, clicks on empty
    // runway locked 140x190 boxes of plain ground
    if (have_g && box_contrast(frame, G) < 10.0) have_g = false;
    if (timing) std::fprintf(stderr, "[LOCKT] ground model %.0f ms\n", ms_since(tg));
    tg = std::chrono::steady_clock::now();
    // point GrabCut only as the fallback (measured 30-545 ms per click, and unused whenever the ground model found the object)
    // (allow_point = false: a SNAPPED lock, which may not use this answer anyway -- it took 0.9-3.7 s there, measured in review)
    Box P; const bool have_p = !have_g && allow_point && grabcut_from_point(frame, px, py, P, why);
    if (timing) std::fprintf(stderr, "[LOCKT] point grabcut %.0f ms\n", ms_since(tg));
    tg = std::chrono::steady_clock::now();
    Box mreg; double spd = 0;
    if (!prev.empty() && prev_scale > 0) {
        cv::Mat cur_s; if (prev_scale < 0.999) cv::resize(frame, cur_s, prev.size(), 0, 0, cv::INTER_AREA); else cur_s = frame;
        if (flow_region(prev, cur_s, px * prev_scale, py * prev_scale, mreg, spd)) {
            Box mb{mreg.x / prev_scale, mreg.y / prev_scale, mreg.w / prev_scale, mreg.h / prev_scale}, M;
            // a speck is not an object (seen live: a 5x5 "moving object" that then failed the lock) -- fall back to colour
            if (grabcut_from_region(frame, px, py, mb, M) && M.w >= 8 && M.h >= 8 && M.w * M.h >= 100 &&
                M.w <= 0.45 * frame.cols && M.h <= 0.45 * frame.rows) {
                const bool take_m = !have_p ||
                    ([&] { const double ix = std::max(0.0, std::min(P.x + P.w, M.x + M.w) - std::max(P.x, M.x)),
                                        iy = std::max(0.0, std::min(P.y + P.h, M.y + M.h) - std::max(P.y, M.y));
                           return ix * iy >= 0.8 * P.w * P.h && M.w * M.h >= 1.3 * P.w * P.h; })();
                // the ground model agrees with the moving region: take its tighter box (the motion region carries the shadow and
                // whatever moved next to it -- measured on the red cube: 45x38 vs the true 32x38); disagreeing, trust the motion
                if (have_g && iou(G, M) >= 0.5) { out = G; how = "moving object (ground model)"; return true; }
                // disagreeing on a SMALL object (motion region < ~65x65 px): the motion region is the one that is off -- measured on
                // 11 of 12 bad cube locks it carried the shadow or a neighbouring car (e.g. 63x62 for a 32x22 cube); on a big object
                // (a car) the ground model can come out partial, so the motion region wins there
                if (have_g && M.w * M.h < 4000) { out = G; how = "moving object (ground model)"; return true; }
                if (take_m) { out = M; how = "moving object"; return true; }
            }
        }
    }
    if (timing) std::fprintf(stderr, "[LOCKT] motion region %.0f ms\n", ms_since(tg));
    if (have_g) { out = G; how = "ground model"; return true; }
    if (have_p && box_contrast(frame, P) >= 10.0) { out = P; how = spd >= 1.0 ? "colour (motion region not used)" : "colour (not moving)"; return true; }
    if (have_p) why = "nothing stands out from its surroundings at the click (plain sky / road?)";
    return false;
}

// Candidate objects NEAR a click that missed (the operator clicked beside a small walking person -- seen live: clicks 30-100 px
// off, because the person had walked on or the window showed an older frame). Centres of (1) things MOVING relative to the ground
// (camera motion removed, frame difference), then (2) small things standing out from their surroundings (local contrast), within
// `radius` of the click, nearest first. Images: prev/cur at the same (tracking) scale; returned points in full-frame pixels.
inline std::vector<cv::Point2d> objects_near(const cv::Mat& prev, const cv::Mat& cur, double scale, double px, double py, double radius) {
    std::vector<std::pair<double, cv::Point2d>> found;
    const double spx = px * scale, spy = py * scale, sr = radius * scale;
    cv::Rect roi((int)std::lround(spx - sr), (int)std::lround(spy - sr), (int)std::lround(2 * sr), (int)std::lround(2 * sr));
    roi &= cv::Rect(0, 0, cur.cols, cur.rows);
    if (roi.width < 16 || roi.height < 16) return {};
    cv::Mat g1; cv::cvtColor(cur, g1, cv::COLOR_BGR2GRAY);
    auto collect = [&](const cv::Mat& mask, int min_area, int max_area) {
        cv::Mat lab, st, cen; int n = cv::connectedComponentsWithStats(mask, lab, st, cen);
        for (int k = 1; k < n; k++) {
            const int a = st.at<int>(k, cv::CC_STAT_AREA);
            if (a < min_area || a > max_area) continue;
            const double cx = roi.x + cen.at<double>(k, 0), cy = roi.y + cen.at<double>(k, 1);
            const double d = std::hypot(cx - spx, cy - spy);
            if (d <= sr) found.push_back({d, cv::Point2d(cx / scale, cy / scale)});
        }
    };
    if (!prev.empty() && prev.size() == cur.size()) {                    // (1) moving
        cv::Mat g0; cv::cvtColor(prev, g0, cv::COLOR_BGR2GRAY);
        std::vector<cv::Point2f> p0, p1; std::vector<uchar> stt; std::vector<float> err;
        cv::goodFeaturesToTrack(g0, p0, 300, 0.01, 7);
        if (p0.size() >= 12) {
            cv::calcOpticalFlowPyrLK(g0, g1, p0, p1, stt, err);
            std::vector<cv::Point2f> a, b; for (size_t k = 0; k < p0.size(); k++) if (stt[k]) { a.push_back(p0[k]); b.push_back(p1[k]); }
            cv::Mat A = a.size() >= 12 ? cv::estimateAffinePartial2D(a, b, cv::noArray(), cv::RANSAC, 2.0) : cv::Mat();
            if (!A.empty()) {
                cv::Mat w0, d; cv::warpAffine(g0, w0, A, g1.size(), cv::INTER_LINEAR, cv::BORDER_REPLICATE);
                cv::absdiff(g1(roi), w0(roi), d);
                cv::Scalar med = cv::mean(d);
                cv::Mat m = d > std::max(18.0, 4.0 * med[0]);
                cv::morphologyEx(m, m, cv::MORPH_CLOSE, cv::Mat::ones(5, 5, CV_8UC1));
                collect(m, 6, (int)(0.15 * roi.area()));
            }
        }
    }
    if (found.empty()) {                                                  // (2) standing out from the surroundings
        cv::Mat bg, d; cv::blur(g1(roi), bg, cv::Size(31, 31)); cv::absdiff(g1(roi), bg, d);
        cv::Mat m = d > 25; cv::morphologyEx(m, m, cv::MORPH_OPEN, cv::Mat::ones(2, 2, CV_8UC1));
        collect(m, 6, (int)(0.05 * roi.area()));
    }
    std::sort(found.begin(), found.end(), [](const auto& x, const auto& y) { return x.first < y.first; });
    std::vector<cv::Point2d> out;
    for (auto& f : found) {                                               // fragments of one object (two faces of a box) count once
        bool dup = false;
        for (auto& o : out) if (std::hypot(o.x - f.second.x, o.y - f.second.y) < 15.0 / scale) { dup = true; break; }
        if (!dup) out.push_back(f.second);
    }
    return out;
}

// The whole click -> lock box decision (run in the background, see the lock job in the main loop): the object under the click; if
// there is none, the nearest moving / distinct object within `snap_radius` (the operator meant it); the box padded to the minimum the
// tracker can start on at its working scale (CSRT needs >= 8 px there: 7x9 px people were refused at half scale).
// COMPLETE OBJECT (GrabCut): the lock box from the ground model / motion often covers only one PART of an object with parts of
// different colours -- a truck's cab vs its cargo box, a van's side vs its roof (operator report, live: 5 of 6 vehicle locks partial,
// one on the wrong part). GrabCut on a window around the box (grown 2.5x): the box is probable object, the click definite object,
// the window border definite background (except where the image edge cut it). It learns object vs background colours from the
// whole window, so it joins the adjacent parts of the same object and keeps the road out. The window grows (up to 2x more) while
// the object still runs into its border. Accepted only if it contains the click, is at least the box, stands out from its own
// surroundings, and did not just fill the window (= no real boundary: open ground). Else the box is kept.
inline bool complete_object(const cv::Mat& frame, const Box& b, double px, double py, Box& out) {
    if (!b.valid()) return false;
    const cv::Rect img(0, 0, frame.cols, frame.rows);
    // windows: 2.5x the box (>= 64 px), then doubling while the object runs into the border -- up to the whole view (a van filling
    // the view from close up: the click's region was 9x11 px of it)
    double side = std::max(64.0, 2.5 * std::max(b.w, b.h));
    const int last = 4;
    for (int it = 0; it <= last; it++, side *= 2.0) {
        if (it > 0 && side / 2.0 >= std::max(frame.cols, frame.rows)) return false;
        cv::Rect roi((int)std::lround(b.cx() - side / 2), (int)std::lround(b.cy() - side / 2), (int)std::lround(side), (int)std::lround(side));
        roi &= img;
        if (roi.width < 24 || roi.height < 24) return false;
        const double sc = std::min(1.0, 160.0 / std::max(roi.width, roi.height));   // (128 px was faster but measurably less accurate)
        cv::Mat small; cv::resize(frame(roi), small, cv::Size(), sc, sc, cv::INTER_AREA);
        cv::Mat mask(small.size(), CV_8UC1, cv::Scalar(cv::GC_PR_BGD)), bgd, fgd;
        const int bd = 2;
        const bool cl = roi.x > 0, ct = roi.y > 0, cr = roi.x + roi.width < frame.cols, cb = roi.y + roi.height < frame.rows;
        if (cl) mask(cv::Rect(0, 0, bd, mask.rows)).setTo(cv::GC_BGD);
        if (ct) mask(cv::Rect(0, 0, mask.cols, bd)).setTo(cv::GC_BGD);
        if (cr) mask(cv::Rect(mask.cols - bd, 0, bd, mask.rows)).setTo(cv::GC_BGD);
        if (cb) mask(cv::Rect(0, mask.rows - bd, mask.cols, bd)).setTo(cv::GC_BGD);
        const cv::Rect ib((int)std::lround((b.x - roi.x) * sc), (int)std::lround((b.y - roi.y) * sc), std::max(1, (int)std::lround(b.w * sc)), std::max(1, (int)std::lround(b.h * sc)));
        mask(ib & cv::Rect(0, 0, mask.cols, mask.rows)).setTo(cv::GC_PR_FGD);
        const cv::Point c((int)std::lround((px - roi.x) * sc), (int)std::lround((py - roi.y) * sc));
        if (c.x < 0 || c.y < 0 || c.x >= mask.cols || c.y >= mask.rows) return false;
        cv::circle(mask, c, std::max(2, (int)std::lround(0.02 * mask.cols)), cv::Scalar(cv::GC_FGD), cv::FILLED);
        try { cv::grabCut(small, mask, cv::Rect(), bgd, fgd, 3, cv::GC_INIT_WITH_MASK); } catch (const cv::Exception&) { return false; }
        cv::Mat fg = (mask == cv::GC_FGD) | (mask == cv::GC_PR_FGD);
        // SHADOW trim (as in the ground model): object pixels with a ground colour's chroma but darker are the object's shadow --
        // GrabCut keeps a cube's shadow with the cube (measured: 52x43 boxes for a 32x30 cube). Ground colours: 3 clusters of the
        // window's border. Kept only if >= 40% of the object remains (a black car on asphalt is never erased).
        {
            cv::Mat lab; cv::cvtColor(small, lab, cv::COLOR_BGR2Lab); lab.convertTo(lab, CV_32FC3);
            const int m = std::max(2, (int)(0.06 * std::min(lab.rows, lab.cols)));
            std::vector<cv::Vec3f> ring;
            for (int y = 0; y < lab.rows; y++) for (int x = 0; x < lab.cols; x++)
                if ((y < m || y >= lab.rows - m || x < m || x >= lab.cols - m) && !fg.at<uchar>(y, x)) ring.push_back(lab.at<cv::Vec3f>(y, x));
            if (ring.size() >= 30) {
                std::vector<cv::Vec3f> smp; const size_t stp = std::max<size_t>(1, ring.size() / 800);
                for (size_t i = 0; i < ring.size(); i += stp) smp.push_back(ring[i]);
                cv::Mat rp((int)smp.size(), 3, CV_32F, smp.data()), lbl, cen;
                // deterministic k-means, started from the ground's own brightness terciles (dark / mid / light: asphalt, shadow, paint) -- random
                // k-means++ starts gave a different lock run to run on the same click (a large truck: right box one run, the colour fallback the next)
                { std::vector<float> Ls(rp.rows); for (int q = 0; q < rp.rows; q++) Ls[q] = rp.at<float>(q, 0);
                  std::vector<float> srt = Ls; std::sort(srt.begin(), srt.end()); const float t1 = srt[srt.size() / 3], t2 = srt[2 * srt.size() / 3];
                  lbl.create(rp.rows, 1, CV_32S); for (int q = 0; q < rp.rows; q++) lbl.at<int>(q) = Ls[q] < t1 ? 0 : (Ls[q] < t2 ? 1 : 2); }
                cv::kmeans(rp, 3, lbl, cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 10, 1.0), 1, cv::KMEANS_USE_INITIAL_LABELS, cen);
                cv::Mat keep = fg.clone();
                for (int y = 0; y < lab.rows; y++) for (int x = 0; x < lab.cols; x++) {
                    if (!fg.at<uchar>(y, x)) continue;
                    const cv::Vec3f v = lab.at<cv::Vec3f>(y, x);
                    for (int k = 0; k < 3; k++) {
                        const float gl = cen.at<float>(k, 0), ga = cen.at<float>(k, 1), gb = cen.at<float>(k, 2);
                        if (std::hypot(v[1] - ga, v[2] - gb) < 6.0f && v[0] < gl - 10.0f && v[0] > 0.25f * gl) { keep.at<uchar>(y, x) = 0; break; }
                    }
                }
                if (cv::countNonZero(keep) >= 0.4 * cv::countNonZero(fg)) fg = keep;
            }
        }
        cv::morphologyEx(fg, fg, cv::MORPH_OPEN, cv::Mat::ones(3, 3, CV_8UC1));
        cv::circle(fg, c, 2, cv::Scalar(255), cv::FILLED);                // the click itself is object (the opening can erase a 1 px seed)
        cv::Mat lab, st, cc; cv::connectedComponentsWithStats(fg, lab, st, cc);
        const int k = lab.at<int>(c.y, c.x);
        if (k <= 0) return false;
        const cv::Rect r(st.at<int>(k, 0), st.at<int>(k, 1), st.at<int>(k, 2), st.at<int>(k, 3));
        static const bool dbg = std::getenv("GC_DEBUG") != nullptr;
        if (dbg) std::printf("  complete: window %dx%d at %d,%d -> region %d,%d %dx%d area %.2f\n", roi.width, roi.height, roi.x, roi.y,
                             (int)(roi.x + r.x / sc), (int)(roi.y + r.y / sc), (int)(r.width / sc), (int)(r.height / sc), st.at<int>(k, cv::CC_STAT_AREA) / (double)mask.total());
        if ((double)st.at<int>(k, cv::CC_STAT_AREA) > 0.55 * mask.total()) return false;   // filled the window: no boundary (ground)
        const bool touches = (r.x <= bd && cl) || (r.y <= bd && ct) || (r.x + r.width >= mask.cols - bd && cr) || (r.y + r.height >= mask.rows - bd && cb);
        // only the seed came back: the window lies INSIDE one uniform object (its own colour was taken as background) -- a larger one
        const bool seed_only = r.area() <= std::max(49.0, 1.3 * ib.area());   // nothing beyond the seed / the start box
        if ((touches || seed_only) && it < last) continue;                // the object runs on: a larger window
        if (seed_only) return false;
        if (touches) return false;
        Box g{roi.x + r.x / sc, roi.y + r.y / sc, r.width / sc, r.height / sc};
        g = Box{std::min(g.x, b.x), std::min(g.y, b.y), 0, 0} ;
        g.w = std::max(roi.x + (r.x + r.width) / sc, b.x + b.w) - g.x; g.h = std::max(roi.y + (r.y + r.height) / sc, b.y + b.h) - g.y;   // never smaller than the box
        if (g.w * g.h < 1.15 * b.w * b.h) return false;                   // no real growth: keep the box (no needless change)
        // How far it may grow. GrabCut cannot tell "the next PART of this object, in another colour" (a truck's cab next to its cargo
        // box: ~1.3x) from "ANOTHER object touching it" (the red cube parked against a car: 3-6x -- measured, median IoU on the cube
        // 0.92 -> 0.66 when growth was free). A start box that is already a real object (>= 24 px) may grow to 2.2x its area; a tiny
        // start (a seed inside a big object -- a van filling the view, the click's region 9x11 px) may grow freely.
        const double max_growth = std::max(b.w, b.h) >= 24 ? 2.2 : 1e9;
        if (g.w * g.h > max_growth * b.w * b.h) return false;
        if (std::max(b.w, b.h) < 24) {
            // a tiny start may grow a lot -- but only into ONE object of its own colour, and not over 5% of the view (measured live: an
            // 8x10 start grew to a 314x236 region of the scene)
            if (g.w * g.h > 0.05 * frame.cols * frame.rows) return false;
            cv::Mat labc; cv::cvtColor(small, labc, cv::COLOR_BGR2Lab);
            cv::Mat seedm(small.size(), CV_8UC1, cv::Scalar(0)); seedm(ib & cv::Rect(0, 0, seedm.cols, seedm.rows)).setTo(255);
            const cv::Mat regm = (lab == k);
            const cv::Scalar ms = cv::mean(labc, seedm), mr = cv::mean(labc, regm);
            if (std::hypot(std::hypot(ms[0] - mr[0], ms[1] - mr[1]), ms[2] - mr[2]) > 25.0) return false;
        }
        // never a box over 12% of the view (a click on the road beside a truck grew to 549x472 = 28%: measured). A close bus is ~4-5%.
        if (g.w * g.h > 0.12 * frame.cols * frame.rows) return false;
        if (box_contrast(frame, g) < 10.0) return false;                  // the grown box must still stand out from its surroundings
        out = g; return true;
    }
    return false;
}

struct LockJob { bool ok = false; Box box; std::string why, how; double px = 0, py = 0, used_px = 0, used_py = 0, ms = 0; cv::Mat frame; double t = 0; };
inline LockJob run_lock_job(cv::Mat frame, cv::Mat prev, double scale, double px, double py, double snap_radius, double t) {
    LockJob j; j.frame = frame; j.t = t; j.px = j.used_px = px; j.py = j.used_py = py;
    const auto t0 = std::chrono::steady_clock::now();
    j.ok = lock_box_from_click(frame, prev, scale, px, py, j.box, j.why, j.how);
    if (!j.ok && snap_radius > 0) {
        cv::Mat cur_s; if (scale < 0.999) cv::resize(frame, cur_s, prev.empty() ? cv::Size() : prev.size(), scale, scale, cv::INTER_AREA); else cur_s = frame;
        const auto cands = objects_near(prev, cur_s, scale, px, py, snap_radius);
        // only snap when ONE object is clearly the nearest: measured live, a click between a person and the target box (51 vs ~70 px)
        // snapped to the box -- the wrong object. Two similarly near candidates = ambiguous: refuse and ask for a direct click.
        if (cands.size() >= 2) {
            const double d0 = std::hypot(cands[0].x - px, cands[0].y - py), d1 = std::hypot(cands[1].x - px, cands[1].y - py);
            if (d0 > 15.0 && d1 < 1.25 * d0) { j.why = "two objects near the click -- click directly on the one you want"; return j; }
        }
        for (size_t k = 0; k < cands.size() && k < 1; k++) {
            std::string why2, how2; Box b2;
            // a SNAPPED lock must be a real, separate object (ground model or motion), not the colour fallback -- the verification
            // suite caught a click on empty sky snapping 88 px to the horizon line and locking a 94x29 strip of it
            if (lock_box_from_click(frame, prev, scale, cands[k].x, cands[k].y, b2, why2, how2, false) &&
                (how2.rfind("ground model", 0) == 0 || how2.rfind("moving object", 0) == 0)) {
                j.ok = true; j.box = b2; j.used_px = cands[k].x; j.used_py = cands[k].y;
                j.how = how2 + ", snapped " + std::to_string((int)std::lround(std::hypot(cands[k].x - px, cands[k].y - py))) + " px from the click";
                break;
            }
        }
    }
    if (j.ok) {
        // (the GrabCut completion runs AFTER the lock, in the background -- see "COMPLETE" in the main loop: inside the lock job it
        // made every lock ~300 ms slower, measured)
        static const bool sync_complete = std::getenv("LOCK_COMPLETE_SYNC") != nullptr;   // --lock-job-test / benchmarks: in line
        Box whole;
        if (sync_complete && complete_object(frame, j.box, j.used_px, j.used_py, whole)) { j.box = whole; j.how += ", completed (GrabCut)"; }
        const double minpx = std::max(6.0, 4.0 / std::max(0.1, scale));      // the object's own size: the tracker grows its box itself
        if (j.box.w < minpx || j.box.h < minpx) {
            const double nw = std::max(j.box.w, minpx), nh = std::max(j.box.h, minpx);
            j.box = Box{j.box.cx() - nw / 2, j.box.cy() - nh / 2, nw, nh};
        }
        j.box.x = std::max(0.0, std::min(j.box.x, frame.cols - j.box.w)); j.box.y = std::max(0.0, std::min(j.box.y, frame.rows - j.box.h));
    }
    j.ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    return j;
}

// Slow adaptation of the lock-time colour reference to gradual lighting change (a cloud, a shadow, the sun angle while circling):
// only from a sample that is clearly the same object (within half the rejection distance), and slowly enough (alpha ~1%/frame)
// that a crossing object, which CSRT hands over within a second or so, cannot drag the reference with it.
inline cv::Scalar adapt_color(const cv::Scalar& ref, const cv::Scalar& meas, double max_dist = 60.0, double alpha = 0.01) {
    if (ref[0] < 0 || meas[0] < 0) return ref;
    double d2 = 0; for (int i = 0; i < 3; i++) { double e = ref[i] - meas[i]; d2 += e * e; }
    if (std::sqrt(d2) > max_dist / 2) return ref;
    cv::Scalar out = ref; for (int i = 0; i < 3; i++) out[i] = (1 - alpha) * ref[i] + alpha * meas[i];
    return out;
}

// Appearance model of the operator-locked object -- its OWN look, taken at lock time (no detector, no re-ID model): a hue-
// saturation histogram of the inner box (colour distribution: robust to small shifts, pose and blur) plus a small colour
// template (structure), the template refreshed slowly from confident frames only. Measured on the sim footage: grey-level
// template matching alone was the wrong tool -- on a small plain-coloured target it scored the TRUE target at 0.41 and patches
// of background at 0.86-0.92; the colour histogram separates them.
//   score(): 0..1, how much a box looks like the locked object.
//   search(): while the target is hidden, where in a window around the prediction are its colours (histogram back-projection),
//             best of the 4 strongest spots by score(), with a mild preference for spots near the prediction.
class Appearance {
public:
    bool init(const cv::Mat& f, const Box& b) {
        H0_ = hist(f, b); T0_ = tmpl(f, b); ok_ = !H0_.empty() && !T0_.empty();
        if (!ok_) return false;
        // NOT the lock frame's own score (a perfect 1.0 against itself): measured live, the true target then scored below 70% of
        // that on 22% of frames, and a box 3 px off (normal CSRT jitter on a 15 px target) on 84% -- every lock "lost" its target
        // within frames. Start from a realistic typical level and learn it from the accepted boxes.
        T1_ = T0_.clone(); level_ = 0.8; calib_.clear(); return true;
    }
    bool ok() const { return ok_; }
    double level() const { return level_; }                     // running typical score of the accepted boxes
    // floors between "covered by an occluder" (0.35 in the self-test, <= 0.23 live) and a small person tracked correctly (0.44-0.55
    // live): 0.42 while the level is being calibrated, 0.38 after
    bool accepts(double s) const { return s >= (calib_.size() < 5 ? 0.42 : std::max(0.38, 0.55 * level_)); }   // a box still on the object
    bool reacquires(double s) const { return s >= std::max(0.45, 0.70 * level_); }     // a search hit good enough to relock on
    // Measured on 30 s of live sim footage (red target, orange lookalike, road, target covered): with the histogram of the INNER
    // half of the box and a shift-tolerant template match, the true target scores p5 0.91 on the histogram, 0.84 with 3 px of
    // jitter, while lookalike / road / covered target score ~0 -- the old whole-box version gave 0.63 / 0.53 vs 0.26.
    double score(const cv::Mat& f, const Box& b) const {
        if (!ok_) return 1.0;
        cv::Mat h = hist(f, b);
        const double hs = h.empty() ? 0.0 : 1.0 - cv::compareHist(H0_, h, cv::HISTCMP_BHATTACHARYYA);
        const double ts = std::max(local_match(f, b, T0_), local_match(f, b, T1_));
        return 0.65 * hs + 0.35 * std::max(0.0, ts);
    }
    void learn(const cv::Mat& f, const Box& b, double s) {
        if (!ok_) return;
        // The typical score depends on the target: measured live, a 12 px person (mostly padding + road in its box) scores
        // 0.44-0.55 even when tracked correctly, a car or a coloured box 0.7-0.9. So the level is CALIBRATED on the first five
        // accepted frames (their median) instead of assuming 0.8, then follows slowly.
        if (calib_.size() < 5) {
            calib_.push_back(s);
            if (calib_.size() == 5) { std::vector<double> c = calib_; std::nth_element(c.begin(), c.begin() + 2, c.end()); level_ = std::clamp(c[2], 0.45, 0.95); }
        } else level_ = std::clamp(0.97 * level_ + 0.03 * s, 0.45, 0.95);
        if (s > 0.9 * level_) { cv::Mat t = tmpl(f, b); if (!t.empty()) cv::addWeighted(T1_, 0.9, t, 0.1, 0.0, T1_); }
    }
    bool search(const cv::Mat& f, const Box& pred, int coast, Box& out, double& sc) const {
        const double r = std::max(std::max(6.0, pred.w), std::max(6.0, pred.h)) * (1.5 + 0.1 * coast) + 6;
        std::vector<Cand> cs; if (!candidates(f, pred, r, 4, cs)) return false;
        bool found = false; sc = -1e9;
        for (const Cand& c : cs) {
            const double s = c.score - 0.15 * std::hypot(c.box.x - pred.x, c.box.y - pred.y) / std::max(1.0, r);
            if (s > sc) { sc = s; out = c.box; found = true; }
        }
        return found;
    }
    // Every separate colour-matching blob of the target's size within r px of `around` (its strongest `n`), with its appearance
    // score -- the raw material for the occlusion search AND for noticing a lookalike next to the target.
    struct Cand { Box box; double score; };
    // skip: candidates on this box (the target itself) are returned unscored (score -1) -- their scoring was 70% of the calls (review)
    bool candidates(const cv::Mat& f, const Box& around, double r, int n, std::vector<Cand>& out, const Box* skip = nullptr) const {
        out.clear();
        if (!ok_) return false;
        const Box& pred = around;
        const double w = std::max(6.0, pred.w), h = std::max(6.0, pred.h);
        cv::Rect win((int)std::floor(pred.x - r), (int)std::floor(pred.y - r), (int)std::ceil(w + 2 * r), (int)std::ceil(h + 2 * r));
        win &= cv::Rect(0, 0, f.cols, f.rows);
        if (win.width <= w + 2 || win.height <= h + 2) return false;
        cv::Mat hsv, bp; cv::cvtColor(f(win), hsv, cv::COLOR_BGR2HSV);
        double hmax = 0; cv::minMaxLoc(H0_, nullptr, &hmax);
        const int ch[] = {0, 1}; const float hr[] = {0, 180}, sr[] = {0, 256}; const float* rg[] = {hr, sr};
        cv::calcBackProject(&hsv, 1, ch, H0_, bp, rg, hmax > 0 ? 255.0 / hmax : 1.0);
        cv::Mat mass; bp.convertTo(mass, CV_32F); cv::boxFilter(mass, mass, -1, cv::Size((int)std::lround(w), (int)std::lround(h)));
        for (int k = 0; k < n; k++) {                                     // the n strongest separate peaks
            double mv = 0; cv::Point ml; cv::minMaxLoc(mass, nullptr, &mv, nullptr, &ml);
            if (mv <= 1e-3) break;
            const Box c{win.x + ml.x - w / 2, win.y + ml.y - h / 2, w, h};
            const bool on_skip = skip && std::fabs(c.cx() - skip->cx()) < 0.85 * skip->w && std::fabs(c.cy() - skip->cy()) < 0.85 * skip->h;
            out.push_back(Cand{c, on_skip ? -1.0 : score(f, c)});
            // suppress a 2w x 2h area: a w x h one left half-box-shifted copies of the SAME peak as the next "candidates" (review: most
            // of the slots were the target itself, a real lookalike could be pushed out, and a shifted copy could fake a tie)
            cv::rectangle(mass, cv::Rect((int)std::lround(ml.x - w), (int)std::lround(ml.y - h), (int)std::lround(2 * w), (int)std::lround(2 * h)), cv::Scalar(0), cv::FILLED);
        }
        return !out.empty();
    }
private:
    static cv::Mat crop(const cv::Mat& f, const Box& b) {
        cv::Rect r((int)std::lround(b.x), (int)std::lround(b.y), (int)std::lround(b.w), (int)std::lround(b.h));
        r &= cv::Rect(0, 0, f.cols, f.rows);
        return (r.width < 3 || r.height < 3) ? cv::Mat() : f(r);
    }
    // best colour-template match of `t` (scaled to the box) anywhere within +-25% of the box: tolerant of a few px of jitter
    static double local_match(const cv::Mat& f, const Box& b, const cv::Mat& t) {
        // compared at <= 48 px box size: the cost grew with the box (a 300x150 car: ~500 M multiply-adds per call, several calls a
        // frame -- large targets slowed the whole tracker); the template is only 24x32 anyway
        const double k = std::min(1.0, 48.0 / std::max(b.w, b.h));
        const int w = std::max(4, (int)std::lround(b.w * k)), h = std::max(4, (int)std::lround(b.h * k));
        cv::Mat area0 = crop(f, Box{b.x - 0.25 * b.w, b.y - 0.25 * b.h, 1.5 * b.w, 1.5 * b.h});
        if (area0.empty() || t.empty()) return 0.0;
        cv::Mat area; if (k < 1.0) cv::resize(area0, area, cv::Size(), k, k, cv::INTER_AREA); else area = area0;
        if (area.cols <= w || area.rows <= h) return 0.0;
        cv::Mat ts, af, res; cv::resize(t, ts, cv::Size(w, h)); area.convertTo(af, CV_32FC3);
        cv::matchTemplate(af, ts, res, cv::TM_CCOEFF_NORMED);
        double mx = 0; cv::minMaxLoc(res, nullptr, &mx);
        return std::isfinite(mx) ? mx : 0.0;
    }
    static cv::Mat hist(const cv::Mat& f, const Box& b) {
        cv::Mat c = crop(f, Box{b.x + 0.25 * b.w, b.y + 0.25 * b.h, 0.5 * b.w, 0.5 * b.h});   // inner half: little background
        if (c.empty()) return cv::Mat();
        cv::Mat hsv, h; cv::cvtColor(c, hsv, cv::COLOR_BGR2HSV);
        const int ch[] = {0, 1}, bins[] = {18, 8}; const float hr[] = {0, 180}, sr[] = {0, 256}; const float* rg[] = {hr, sr};
        cv::calcHist(&hsv, 1, ch, cv::Mat(), h, 2, bins, rg);
        cv::normalize(h, h, 1, 0, cv::NORM_L1);
        return h;
    }
    static cv::Mat tmpl(const cv::Mat& f, const Box& b) {
        cv::Mat c = crop(f, b);
        if (c.empty()) return cv::Mat();
        cv::Mat t; cv::resize(c, t, cv::Size(24, 32), 0, 0, cv::INTER_AREA); t.convertTo(t, CV_32FC3);
        return t;
    }
    static double ncc3(const cv::Mat& a, const cv::Mat& b) {
        cv::Scalar ma = cv::mean(a), mb = cv::mean(b);
        cv::Mat da = a - ma, db = b - mb;
        const double num = da.dot(db), den = std::sqrt(da.dot(da) * db.dot(db));
        return den > 1e-6 ? num / den : 0.0;
    }
    cv::Mat H0_, T0_, T1_; double level_ = 0.9; bool ok_ = false; std::vector<double> calib_;
};

class BoxKalman {
public:
    explicit BoxKalman(const Box& b0 = Box(), double r_frac = 0.08, double q_pos = 1.0, double q_vel = 0.5)
        : r_frac_(r_frac), q_pos_(q_pos), q_vel_(q_vel) { reset(b0); }

    void reset(const Box& b0) {
        x_ = {b0.cx(), b0.cy(), b0.w, b0.h, 0.0, 0.0};
        for (auto& v : P_) v = 0.0;
        const double p0[6] = {25, 25, 25, 25, 100, 100};
        for (int i = 0; i < 6; i++) P_[i * 6 + i] = p0[i];
    }

    // steps: elapsed time in NOMINAL frames (1 at the steady frame rate; 2 after one dropped frame). The velocity state is in
    // px per nominal frame, so a late frame now predicts as far as the target actually moved, instead of always one frame's worth
    // (which made every dropped/late frame look like the target jumped, and the plausibility gate reject a correct box).
    void predict(double steps = 1.0) {
        // x' = F x with F = I + dt*[vx,vy] coupling (dt = steps); P' = F P F^T + Q*dt
        const double dt = std::max(0.0, steps);
        x_[0] += dt * x_[4]; x_[1] += dt * x_[5];
        x_[2] = std::max(x_[2], 4.0); x_[3] = std::max(x_[3], 4.0);
        double FP[36];
        for (int c = 0; c < 6; c++) {                    // FP = F * P
            for (int r = 0; r < 6; r++) {
                double v = P_[r * 6 + c];
                if (r == 0) v += dt * P_[4 * 6 + c];
                if (r == 1) v += dt * P_[5 * 6 + c];
                FP[r * 6 + c] = v;
            }
        }
        for (int r = 0; r < 6; r++) {                    // P = FP * F^T
            for (int c = 0; c < 6; c++) {
                double v = FP[r * 6 + c];
                if (c == 0) v += dt * FP[r * 6 + 4];
                if (c == 1) v += dt * FP[r * 6 + 5];
                P_[r * 6 + c] = v;
            }
        }
        for (int i = 0; i < 4; i++) P_[i * 6 + i] += q_pos_ * q_pos_ * dt;
        for (int i = 4; i < 6; i++) P_[i * 6 + i] += q_vel_ * q_vel_ * dt;
    }

    void update(const Box& m) {
        const double z[4] = {m.cx(), m.cy(), m.w, m.h};
        const double s = std::max({x_[2], x_[3], 8.0}) * r_frac_, R = s * s;
        double S[16], Sinv[16];
        for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) S[r * 4 + c] = P_[r * 6 + c] + (r == c ? R : 0.0);
        if (!invert4(S, Sinv)) return;
        // MANOEUVRE detection (adaptive process noise): a constant-velocity filter trusts its old velocity, so after the target
        // turns, speeds up or reverses, the prediction stays off for many frames (measured on a fast, turning clip: p90 error one
        // whole box size) -- and the prediction is what the tracker falls back on when a frame is doubtful or the target hidden,
        // i.e. exactly when it then got lost. When the position innovation is improbable for the filter's own uncertainty (NIS >
        // 9, i.e. beyond ~3 sigma), the velocity uncertainty is re-opened in proportion, so this and the next measurements pull
        // the velocity to the new motion at once; a target moving steadily never triggers it (small and slow ones stay smooth).
        if (adaptive) {
            const double ex = z[0] - x_[0], ey = z[1] - x_[1];
            const double nis = ex * (Sinv[0] * ex + Sinv[1] * ey) + ey * (Sinv[4] * ex + Sinv[5] * ey);
            if (nis > 9.0) {
                const double f = std::min(nis / 9.0, 30.0);
                for (int i = 0; i < 6; i++) { P_[4 * 6 + i] *= std::sqrt(f); P_[i * 6 + 4] *= std::sqrt(f); P_[5 * 6 + i] *= std::sqrt(f); P_[i * 6 + 5] *= std::sqrt(f); }
                for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) S[r * 4 + c] = P_[r * 6 + c] + (r == c ? R : 0.0);
                if (!invert4(S, Sinv)) return;
                manoeuvres_++;
            }
        }
        double K[24];                                    // K = P[:, :4] * Sinv  (6x4)
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 4; c++) {
                double v = 0;
                for (int k = 0; k < 4; k++) v += P_[r * 6 + k] * Sinv[k * 4 + c];
                K[r * 4 + c] = v;
            }
        }
        double innov[4];
        for (int i = 0; i < 4; i++) innov[i] = z[i] - x_[i];
        {   // likelihood of this measurement under the filter (for the IMM's model weights): N(innov; 0, S)
            double q = 0; for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) q += innov[r] * Sinv[r * 4 + c] * innov[c];
            const double det = std::max(det4(S), 1e-12);
            last_lik_ = std::exp(-0.5 * std::min(q, 200.0)) / std::sqrt(std::pow(2 * M_PI, 4) * det);
        }
        for (int r = 0; r < 6; r++) {
            double d = 0;
            for (int c = 0; c < 4; c++) d += K[r * 4 + c] * innov[c];
            x_[r] += d;
        }
        double KP[36];                                   // P = P - K * P[:4, :]
        for (int r = 0; r < 6; r++) {
            for (int c = 0; c < 6; c++) {
                double v = 0;
                for (int k = 0; k < 4; k++) v += K[r * 4 + k] * P_[k * 6 + c];
                KP[r * 6 + c] = v;
            }
        }
        for (int i = 0; i < 36; i++) P_[i] -= KP[i];
    }

    void damp_velocity(double f) { x_[4] *= f; x_[5] *= f; }
    long manoeuvres() const { return manoeuvres_; }
    bool adaptive = true;                                // manoeuvre detection by NIS (off inside the IMM: the IMM does it properly)
    double last_lik() const { return last_lik_; }
    const std::array<double, 6>& state() const { return x_; }
    const double* cov() const { return P_; }
    void set(const std::array<double, 6>& x, const double* P) { x_ = x; for (int i = 0; i < 36; i++) P_[i] = P[i]; }
    void scale_size(double f) { x_[2] *= f; x_[3] *= f; }             // the target grew / shrank by f (e.g. the range changed)
    // camera-motion compensation: the whole image moved by the similarity transform A (2x3) since the last frame -- move the box
    // (centre and size) and its velocity with it, so the prediction stays on the target while the drone/gimbal moves
    void warp(const cv::Mat& A) {
        if (A.empty()) return;
        const double a = A.at<double>(0, 0), b = A.at<double>(0, 1), tx = A.at<double>(0, 2), c = A.at<double>(1, 0), d = A.at<double>(1, 1), ty = A.at<double>(1, 2);
        const double cx = x_[0], cy = x_[1], sc = std::sqrt(std::fabs(a * d - b * c));
        x_[0] = a * cx + b * cy + tx; x_[1] = c * cx + d * cy + ty;
        x_[2] *= sc; x_[3] *= sc;
        const double vx = x_[4], vy = x_[5]; x_[4] = a * vx + b * vy; x_[5] = c * vx + d * vy;
    }
    Box box() const { return Box{x_[0] - x_[2] / 2, x_[1] - x_[3] / 2, x_[2], x_[3]}; }
    double vx() const { return x_[4]; }
    double vy() const { return x_[5]; }
    double p(int r, int c) const { return P_[r * 6 + c]; }

private:
    long manoeuvres_ = 0; double last_lik_ = 1.0;
    static double det4(const double* a) {                 // Gaussian elimination with partial pivoting
        double m[4][4]; for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) m[r][c] = a[r * 4 + c];
        double det = 1;
        for (int col = 0; col < 4; col++) {
            int piv = col; for (int r = col + 1; r < 4; r++) if (std::fabs(m[r][col]) > std::fabs(m[piv][col])) piv = r;
            if (std::fabs(m[piv][col]) < 1e-300) return 0;
            if (piv != col) { for (int c = 0; c < 4; c++) std::swap(m[piv][c], m[col][c]); det = -det; }
            det *= m[col][col];
            for (int r = col + 1; r < 4; r++) { const double f = m[r][col] / m[col][col]; for (int c = col; c < 4; c++) m[r][c] -= f * m[col][c]; }
        }
        return det;
    }
    static bool invert4(const double* a, double* inv) {  // Gauss-Jordan, 4x4
        double m[4][8];
        for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) { m[r][c] = a[r * 4 + c]; m[r][4 + c] = (r == c); }
        for (int col = 0; col < 4; col++) {
            int piv = col;
            for (int r = col + 1; r < 4; r++) if (std::fabs(m[r][col]) > std::fabs(m[piv][col])) piv = r;
            if (std::fabs(m[piv][col]) < 1e-12) return false;
            if (piv != col) for (int c = 0; c < 8; c++) std::swap(m[piv][c], m[col][c]);
            double d = m[col][col];
            for (int c = 0; c < 8; c++) m[col][c] /= d;
            for (int r = 0; r < 4; r++) {
                if (r == col) continue;
                double f = m[r][col];
                for (int c = 0; c < 8; c++) m[r][c] -= f * m[col][c];
            }
        }
        for (int r = 0; r < 4; r++) for (int c = 0; c < 4; c++) inv[r * 4 + c] = m[r][4 + c];
        return true;
    }
    std::array<double, 6> x_{};
    double P_[36];
    double r_frac_, q_pos_, q_vel_;
};

// The target's image-motion filter: an INTERACTING MULTIPLE MODEL (IMM) of two constant-velocity Kalman filters -- CALM (little
// process noise: a smooth, precise track of a target moving steadily) and MANOEUVRE (a lot: a target turning, braking, reversing or
// accelerating hard). Every frame the two estimates are mixed by the model-switching probabilities, both predict, and each new box
// re-weights them by how well each one had predicted it -- so the output is the calm filter's smoothness while the target behaves,
// and follows a manoeuvre within a frame or two when it does not (the single filter either lagged turns or was noisy on straight
// runs). A UKF would not help here: the box motion and the measurement are both linear in the image, where a UKF equals a KF.
// Same interface as BoxKalman.
class TargetFilter {
public:
    // r_frac 0.05 (was 0.08): measured on the fast-turning / tiny clips, p90 prediction error 0.46 -> 0.35 / 0.31 -> 0.26 box sizes,
    // no change on the steady clip
    explicit TargetFilter(const Box& b0 = Box(), double r_frac = 0.05, double q_pos = 1.0, double q_vel = 0.5) {
        static const double qc = std::getenv("IMM_QC") ? std::atof(std::getenv("IMM_QC")) : 0.3;   // calm velocity noise (px/frame)
        static const double qm = std::getenv("IMM_QM") ? std::atof(std::getenv("IMM_QM")) : 3.0;   // manoeuvre velocity noise
        (void)q_vel;                                     // the calm model's velocity noise is qc
        m_[0] = BoxKalman(b0, r_frac, q_pos, qc); m_[1] = BoxKalman(b0, r_frac, q_pos, qm);
        m_[0].adaptive = false; m_[1].adaptive = false;
        reset(b0);
    }
    void reset(const Box& b0) { m_[0].reset(b0); m_[1].reset(b0); mu_ = {0.9, 0.1}; c_ = mu_; }
    void predict(double steps = 1.0) {
        // switching probabilities over `steps` frames (calm -> manoeuvre 5%/frame, back 15%/frame)
        const double p01 = std::min(0.5, 0.05 * steps), p10 = std::min(0.5, 0.15 * steps);
        const double PI[2][2] = {{1 - p01, p01}, {p10, 1 - p10}};
        for (int j = 0; j < 2; j++) c_[j] = PI[0][j] * mu_[0] + PI[1][j] * mu_[1];
        std::array<double, 6> x0[2]; double P0[2][36];
        for (int j = 0; j < 2; j++) {
            const double w0 = PI[0][j] * mu_[0] / std::max(c_[j], 1e-12), w1 = PI[1][j] * mu_[1] / std::max(c_[j], 1e-12);
            const auto& xa = m_[0].state(); const auto& xb = m_[1].state();
            for (int i = 0; i < 6; i++) x0[j][i] = w0 * xa[i] + w1 * xb[i];
            const double* Pa = m_[0].cov(); const double* Pb = m_[1].cov();
            for (int r = 0; r < 6; r++) for (int c = 0; c < 6; c++)
                P0[j][r * 6 + c] = w0 * (Pa[r * 6 + c] + (xa[r] - x0[j][r]) * (xa[c] - x0[j][c])) + w1 * (Pb[r * 6 + c] + (xb[r] - x0[j][r]) * (xb[c] - x0[j][c]));
        }
        for (int j = 0; j < 2; j++) { m_[j].set(x0[j], P0[j]); m_[j].predict(steps); }
        mu_ = c_;                                    // the predicted mode probabilities (until a measurement re-weights them; review)
    }
    void update(const Box& m) {
        double L[2];
        for (int j = 0; j < 2; j++) { m_[j].update(m); L[j] = m_[j].last_lik(); }
        const double s0 = L[0] * c_[0], s1 = L[1] * c_[1], sum = s0 + s1;
        if (sum > 1e-300 && std::isfinite(sum)) { mu_[0] = std::clamp(s0 / sum, 1e-4, 1 - 1e-4); mu_[1] = 1 - mu_[0]; }
        else mu_ = c_;
    }
    void warp(const cv::Mat& A) { m_[0].warp(A); m_[1].warp(A); }
    void damp_velocity(double f) { m_[0].damp_velocity(f); m_[1].damp_velocity(f); }
    void scale_size(double f) { m_[0].scale_size(f); m_[1].scale_size(f); }
    Box box() const { const auto x = mixed(); return Box{x[0] - x[2] / 2, x[1] - x[3] / 2, x[2], x[3]}; }
    double vx() const { return mixed()[4]; }
    double vy() const { return mixed()[5]; }
    double p(int r, int c) const {
        const auto x = mixed(); double v = 0;
        for (int j = 0; j < 2; j++) { const auto& xj = m_[j].state(); v += mu_[j] * (m_[j].p(r, c) + (xj[r] - x[r]) * (xj[c] - x[c])); }
        return v;
    }
    double manoeuvre_prob() const { return mu_[1]; }
private:
    std::array<double, 6> mixed() const {
        std::array<double, 6> x{}; for (int j = 0; j < 2; j++) for (int i = 0; i < 6; i++) x[i] += mu_[j] * m_[j].state()[i];
        return x;
    }
    BoxKalman m_[2]; std::array<double, 2> mu_{0.9, 0.1}, c_{0.9, 0.1};
};

// The filter's own GATE for a new box: the jump from the prediction must lie within ~4 sigma of the filter's position uncertainty
// (plus the measurement noise), never tighter than 60% of the box nor wider than the fixed plausibility limit. Steady target: a
// tight gate -- CSRT hopping onto a neighbour 1-2 box widths away is refused (live: the box jumped 54-158 px onto other objects);
// manoeuvring target: the IMM's manoeuvre model widens it, so the true motion still passes.
// The floor also scales with the target's own image speed (2.5x per frame) and the box: a gate that rejects the very measurement
// showing a manoeuvre never lets the filter see it (measured: a 14 px target reversing at 14 px/frame -- tracked 100% -> 17%
// with a 0.6-box floor). A slow, steady target keeps a tight gate.
inline double gate_radius(const TargetFilter& kf, const Box& pred, double steps = 1.0) {
    const double sz = std::max(pred.w, pred.h), r = 0.05 * sz;
    const double sigma = std::sqrt(std::max(0.0, 0.5 * (kf.p(0, 0) + kf.p(1, 1))) + r * r);
    const double spd = std::hypot(kf.vx(), kf.vy()) * std::max(1.0, steps);
    return std::max({0.8 * sz + 2.5 * spd, 4.0 * sigma, 15.0});
}
inline bool filter_gate(const TargetFilter& kf, const Box& pred, const Box& b, double steps = 1.0, double widen = 1.0) {
    if (!b.valid() || !pred.valid()) return true;
    const double lim = widen * gate_radius(kf, pred, steps);
    return std::hypot(b.cx() - pred.cx(), b.cy() - pred.cy()) <= lim;
}

// ---------------------------------------------------------------------------
// Control laws (angle based, so they work for any lens: pinhole fx/fy come from the camera's field of view).
// ---------------------------------------------------------------------------
struct Intrinsics {
    double fx = 205.5, fy = 205.5, cx = 320.0, cy = 240.0;   // Gazebo gimbal camera: 640x480, hfov 2.0 rad
    static Intrinsics from_hfov(double width, double height, double hfov_rad) {
        Intrinsics k; k.fx = k.fy = (width / 2.0) / std::tan(hfov_rad / 2.0); k.cx = width / 2.0; k.cy = height / 2.0; return k;
    }
};

struct Gains {
    double kyaw = 1.0, kz = 2.0, kx = 3.0;                    // rad/s per rad, m/s per rad, m/s per unit size error (UNTUNED)
    double klat = 4.0;                                        // m/s per rad of bearing, --lateral-mode strafe (UNTUNED)
    double db_yaw = 0.02, db_z = 0.03, db_x = 0.05;           // dead-bands: rad, rad, fraction of reference height
    double min_depression_deg = 10.0;                         // gimbal mode: below this downward angle the ground-distance estimate is ill-conditioned -> no forward/back command
};

// Opt-in (--best-view): the plain follow loop above only PRESERVES whatever framing the officer happened
// to lock at -- it never judges whether that's actually a good view. This adds a vz contribution (on top
// of the existing centring vz, not instead of it) that climbs when the target's apparent size grows too
// large (too close: less context, more collision risk) and descends/approaches when it shrinks too small
// (too far to make out). A genuine autonomous-altitude decision -- a deliberate departure from this
// project's "altitude is the pilot's job" default -- so it is opt-in, and the result still passes through
// the existing --alt-floor/--alt-ceiling and velocity-ceiling clamps downstream, same as any other command.
inline double best_view_vz(double box_h_frac, double min_frac, double max_frac, double gain) {
    if (box_h_frac > max_frac) return -gain * (box_h_frac - max_frac);   // too close/large: climb (vz<0 = up)
    if (box_h_frac < min_frac) return gain * (min_frac - box_h_frac);    // too far/small: descend/approach
    return 0.0;
}

// Image-based servoing: yaw toward the target, climb/descend to centre it, close/open by apparent size relative
// to the size at lock time (no assumed object size).
// yaw_offset_rad: the camera's own yaw relative to the nose (a panned gimbal), so the vehicle turns toward the TARGET, not toward
// the image centre. centre_vz = false: no altitude command from the vertical image error ("--vz-mode hold"): with a camera tilted
// down, a target below the image centre means "farther away", not "lower", and chasing it with altitude drives the drone toward
// the ground (down to --alt-floor) -- altitude then stays the pilot's.
// strafe = true (--lateral-mode strafe, from the previous iteration's follow law): centre the target by sliding sideways (vy)
// instead of turning, so the heading stays where the pilot put it -- useful with a forward camera on a fixed-heading pass, or
// when turning would swing a body-fixed camera off a second object of interest.
// vert_err_rad: the vertical error to centre on instead of the raw image row (a camera fixed to the airframe, see
// body_cam_vertical_error); NaN = the image row.
inline Cmd image_control(const Box& b, double ref_h, const Intrinsics& k, const Gains& g, const Ceilings& c,
                         double yaw_offset_rad = 0.0, bool centre_vz = true, bool strafe = false,
                         double vert_err_rad = std::numeric_limits<double>::quiet_NaN()) {
    Cmd out;
    if (!b.valid() || ref_h <= 1.0) return out;
    const double bearing = deadband(yaw_offset_rad + std::atan((b.cx() - k.cx) / k.fx), g.db_yaw);
    if (strafe) out.vy = g.klat * bearing;                                            // target right -> slide right
    else out.yaw_rate = g.kyaw * bearing;
    const double verr = std::isfinite(vert_err_rad) ? vert_err_rad : std::atan((b.cy() - k.cy) / k.fy);
    if (centre_vz) out.vz = g.kz * deadband(verr, g.db_z);                            // target below centre -> descend
    out.vx = g.kx * deadband((ref_h - b.h) / ref_h, g.db_x);                          // smaller than at lock -> go forward
    return clamp_keep_direction(out, c);
}

// Integral term for the bearing loop (--ki-yaw): a P-only loop following a target that keeps moving sideways settles with a
// standing offset (it needs an error to produce any turn rate at all); a small integral removes it. Anti-windup: the integral
// contribution is clamped to +/- `limit`, it does not grow while the output is saturated, and it is reset whenever steering stops
// (the previous iteration's PID had neither, so it wound up during every coast/lost period).
class BearingIntegrator {
public:
    double step(double err_rad, double dt, double ki, double limit, bool saturated) {
        if (ki <= 0 || dt <= 0 || dt > 0.5) return out();
        if (!saturated || (acc_ > 0) != (err_rad > 0)) acc_ += err_rad * dt;   // only unwind while saturated
        acc_ = sym_clamp(acc_, limit / ki); ki_ = ki;
        return out();
    }
    void reset() { acc_ = 0; }
    double out() const { return ki_ * acc_; }
private:
    double acc_ = 0, ki_ = 0;
};

// Gimbal-angle servoing (the C12 tracks the target and steers the gimbal; we follow the gimbal).
// gimbal_yaw_deg: + right of the airframe nose; gimbal_pitch_deg: + up, negative looks down.
// Ground distance assumes flat ground (or water) under the target: d = height / tan(depression).
inline double ground_distance(double height_m, double gimbal_pitch_deg) {
    double dep = -gimbal_pitch_deg;
    if (dep < 1.0) return std::numeric_limits<double>::infinity();                   // looking at/above the horizon
    return height_m / std::tan(deg2rad(dep));
}

inline Cmd gimbal_control(double gimbal_yaw_deg, double gimbal_pitch_deg, double height_m, double ref_dist_m,
                          const Gains& g, const Ceilings& c) {
    Cmd out;
    out.yaw_rate = g.kyaw * deadband(deg2rad(gimbal_yaw_deg), g.db_yaw);
    // d = h / tan(depression) changes by ~40% for a 1 deg wobble at 7 deg: only trust it when the gimbal looks steeply enough down.
    if (-gimbal_pitch_deg >= g.min_depression_deg) {
        double d = ground_distance(height_m, gimbal_pitch_deg);
        if (std::isfinite(d) && ref_dist_m > 0.5)
            out.vx = g.kx * deadband((d - ref_dist_m) / ref_dist_m, g.db_x);               // farther than at lock -> forward
    }
    out.vz = 0.0;                                                                       // altitude is the pilot's
    return clamp_keep_direction(out, c);
}

// Altitude guard: never command a descent at/below the floor, never a climb at/above the ceiling; if the altitude is
// unknown, block descent. ArduPilot's geofence is the real protection; this stops the follower from fighting it.
inline Cmd altitude_guard(Cmd c, bool alt_valid, double alt_m, double floor_m, double ceil_m) {
    if (!alt_valid) { if (floor_m > 0 && c.vz > 0) c.vz = 0; return c; }
    if (floor_m > 0 && alt_m <= floor_m && c.vz > 0) c.vz = 0;
    if (ceil_m > 0 && alt_m >= ceil_m && c.vz < 0) c.vz = 0;
    return c;
}

// ---------------------------------------------------------------------------
// Follow state machine. Velocity is allowed ONLY in LOCKED (with GUIDED confirmed and the link up).
// ---------------------------------------------------------------------------
enum class FState { IDLE, LOCKED, COASTING, LOST, LINK_LOST };

inline const char* to_str(FState s) {
    switch (s) {
        case FState::IDLE: return "IDLE";
        case FState::LOCKED: return "LOCKED";
        case FState::COASTING: return "COASTING";
        case FState::LOST: return "LOST";
        case FState::LINK_LOST: return "LINK_LOST";
    }
    return "?";
}

struct FsmTimes { double coast_after = 0.3, lost_after = 1.0, unlock_after = 5.0, link_timeout = 2.0, link_resume = 5.0; };

struct FsmIn {
    double now = 0;                  // seconds, monotonic
    double box_age = 0;              // seconds since the last ACCEPTED tracker box
    bool lock_request = false;       // operator selected a target this tick
    bool unlock_request = false;     // operator released
    bool link_ok = true;             // vehicle heartbeat fresh
    bool guided = true;              // GUIDED confirmed
    bool require_guided = true;      // false (--lock-without-guided): a lock may exist outside GUIDED (track only); steering still
                                     // needs GUIDED -- the MAVLink tick sends nothing unless the vehicle is in GUIDED
};

class FollowFsm {
public:
    explicit FollowFsm(FsmTimes t = FsmTimes()) : t_(t) {}
    FState state() const { return st_; }
    const char* reason() const { return reason_; }           // why the last transition happened
    double locked_for(double now) const { return st_ == FState::LOCKED ? now - locked_at_ : 0.0; }
    bool steering_allowed() const { return st_ == FState::LOCKED; }
    // true if the operator's select/lock request was accepted this tick
    bool update(const FsmIn& in) {
        bool accepted = false;
        if (in.unlock_request) { go(FState::IDLE, "operator unlock", in.now); return false; }
        if (in.require_guided && st_ != FState::IDLE && st_ != FState::LINK_LOST && !in.guided) { go(FState::IDLE, "left GUIDED: lock dropped", in.now); }
        if (!in.link_ok) {
            if (st_ != FState::IDLE && st_ != FState::LINK_LOST) { link_lost_at_ = in.now; link_prev_ = st_; }
            if (st_ != FState::IDLE) go(FState::LINK_LOST, "no vehicle heartbeat", in.now);   // LINK_LOST only exists with a lock
        } else if (st_ == FState::LINK_LOST) {
            // A SHORT gap (<= link_resume s; measured in SITL under load: 22 gaps of 0.1-0.8 s in one session, each dropping the lock
            // for good) with the vehicle still in GUIDED: the same lock carries on -- the camera never stopped tracking it, and the
            // pilot has not taken over (a mode change still drops it). A longer gap: the operator must lock again.
            if (in.now - link_lost_at_ <= t_.link_resume && (!in.require_guided || in.guided)) {
                const FState back = in.box_age > t_.lost_after ? FState::LOST : in.box_age > t_.coast_after ? FState::COASTING : FState::LOCKED;
                const double keep = locked_at_; st_ = back; last_ = back; reason_ = "link back: lock resumed"; locked_at_ = keep;
                if (back == FState::LOST) lost_at_ = in.now;
                (void)link_prev_;
            } else go(FState::IDLE, "link back: operator must lock again", in.now);         // never an automatic relock after a long gap
        }
        if (in.lock_request) {
            if (!in.link_ok) { reason_ = "lock refused: no vehicle link"; }
            else if (in.require_guided && !in.guided) { reason_ = "lock refused: not in GUIDED"; }
            else {
                // A NEW selection is always a fresh lock, even if the old one was still LOCKED/COASTING/LOST: go() is a no-op
                // for an unchanged state, which used to leave locked_at_ at the OLD lock's time -- so the rescue's "locked >= 2 s"
                // gate was satisfied instantly for a target picked a moment ago.
                go(FState::LOCKED, "operator lock", in.now);
                locked_at_ = in.now; last_ = FState::LOCKED;
                accepted = true;
            }
            return accepted;
        }
        switch (st_) {
            case FState::LOCKED:
            case FState::COASTING:                                                            // judged on the full age at once (a 4 s blackout is LOST, not "coasting")
                if (in.box_age > t_.lost_after) go(FState::LOST, "lost target", in.now);
                else if (in.box_age > t_.coast_after) go(FState::COASTING, "box stale", in.now);
                else if (st_ == FState::COASTING) go(FState::LOCKED, "box recovered", in.now);
                break;
            case FState::LOST:                                                                // latched until unlock/relock/timeout
                if (in.box_age > t_.unlock_after || in.now - lost_at_ >= t_.unlock_after - t_.lost_after) go(FState::IDLE, "lock dropped after timeout", in.now);
                break;
            default: break;
        }
        return accepted;
    }
private:
    void go(FState s, const char* why, double now) {
        if (s == st_) return;
        st_ = s; reason_ = why;
        if (s == FState::LOCKED && last_ != FState::COASTING) locked_at_ = now;
        if (s == FState::LOST) lost_at_ = now;
        last_ = s;
    }
    FsmTimes t_;
    FState st_ = FState::IDLE, last_ = FState::IDLE;
    const char* reason_ = "start";
    double locked_at_ = 0, lost_at_ = 0, link_lost_at_ = 0; FState link_prev_ = FState::IDLE;
};

// ---------------------------------------------------------------------------
// Command gate: the ONLY path to the vehicle. Zero (with yaw_rate = 0) unless steering is allowed and the command
// is fresh. Zero/emergency commands are NOT slewed (they take effect immediately).
// ---------------------------------------------------------------------------
inline Cmd gate_command(const Cmd& desired, bool steering_allowed, bool guided, bool link_ok, double cmd_age_s,
                        double stale_s) {
    if (!steering_allowed || !guided || !link_ok || cmd_age_s > stale_s) return Cmd();
    return desired;
}

// ---------------------------------------------------------------------------
// Go-to-target: the officer confirms once, the drone computes the target's location and sends ONE go-to command to the flight
// controller (SET_POSITION_TARGET_LOCAL_NED position target + MAV_CMD_DO_CHANGE_SPEED for a fast transit). That is the entire
// behaviour: no further re-targeting, no arrival handling and no release/trigger logic here -- the flight controller's own GUIDED
// position control flies there and, once nothing further is commanded, naturally stays near that point (standard GUIDED
// behaviour, not something built on top of it). Any extinguisher/payload hardware is the officer's own switch wired to the flight
// controller, entirely outside this software. Obstacle avoidance does not exist: the transit altitude must clear obstacles.
// ---------------------------------------------------------------------------
struct Ned { double n = 0, e = 0, d = 0; };                       // metres from the EKF origin; d is DOWN

// Ground position of the tracked point, assuming flat ground under it. bearing_right: + clockwise from the airframe nose;
// depression: angle below the horizontal of the line of sight; height: vehicle height above the tracked point.
inline bool target_ned_from_view(const Ned& veh, double veh_yaw_rad, double height_m, double bearing_right_rad, double depression_rad,
                                 double min_depression_rad, Ned& out, double min_height_m = 0.5) {
    if (depression_rad < min_depression_rad || height_m < min_height_m) return false;                 // too shallow or below floor
    double dist = (depression_rad > 1e-4) ? (height_m / std::tan(depression_rad)) : 0.0;
    double hdg = veh_yaw_rad + bearing_right_rad;
    out.n = veh.n + dist * std::cos(hdg); out.e = veh.e + dist * std::sin(hdg); out.d = veh.d + height_m;
    return true;
}

// 3D position along the camera line-of-sight ray, valid in full 3D space (airborne, rooftop, ground, elevated)
inline Ned target_ned_from_ray(const Ned& veh, double veh_yaw_rad, double slant_m, double bearing_right_rad, double depression_rad) {
    const double hdg = veh_yaw_rad + bearing_right_rad;
    const double ch = std::cos(depression_rad);
    return Ned{veh.n + slant_m * ch * std::cos(hdg),
               veh.e + slant_m * ch * std::sin(hdg),
               veh.d + slant_m * std::sin(depression_rad)};
}

inline double bearing_to(const Ned& from, const Ned& to) { return std::atan2(to.e - from.e, to.n - from.n); }   // rad from north, clockwise

// RANGE FROM THE BOX SIZE: D = f * H / h_px. The flat-ground range (height / tan(depression)) is accurate looking well down but
// degrades fast at shallow angles (its error grows as 1/sin^2(depression), and below --min-depression there is no fix at all) --
// exactly when a target is far off or the drone is low. H is not assumed (no class, no "a person is 1.7 m"): it is LEARNED for
// this target from the reliable fixes (looking >= 25 deg down: H = slant range * h_px / f, median of the last 40), and then gives
// a range of its own wherever the box is seen. The caller fuses the two by their uncertainty (or uses this alone when too shallow).
class SizeRange {
public:
    void reset() { hs_.clear(); }
    void learn(double slant_m, double h_px, double f_px, double depression_rad = 0) {
        if (h_px < 8 || slant_m <= 0 || f_px <= 0) return;
        hs_.push_back(slant_m * h_px / f_px); if (hs_.size() > 40) hs_.pop_front();
        dep_ = dep_ <= 0 ? depression_rad : 0.9 * dep_ + 0.1 * depression_rad;
    }
    // the apparent size depends on the view angle (an object's ground length shows more from above): far from the angle it was learned
    // at, the size range is trusted much less (review: ~2x off for a car seen shallow after learning steep)
    double angle_factor(double depression_rad) const { return std::fabs(depression_rad - dep_) < 0.26 ? 1.0 : 2.5; }
    bool ready() const { return hs_.size() >= 8; }
    double H() const { std::vector<double> v(hs_.begin(), hs_.end()); std::nth_element(v.begin(), v.begin() + v.size() / 2, v.end()); return v[v.size() / 2]; }
    double slant(double h_px, double f_px) const { return H() * f_px / std::max(1.0, h_px); }
    // its 1-sigma: ~15% (box-height jitter, the view angle changing the apparent size) + 0.5 m
    static double sigma(double slant_m) { return 0.5 + 0.15 * slant_m; }
private:
    std::deque<double> hs_; double dep_ = 0;
};

// Line of sight of an image point, as (bearing right of the airframe nose, depression below the horizontal), for a camera whose
// boresight is at cam_yaw_deg (+ right of the nose) / cam_pitch_deg (+ up, negative looks down). The pixel ray (forward, right =
// px_right/fx, down = px_down/fy) is rotated by the camera's pitch, then its yaw -- a real 3D rotation, not a sum of angles, which
// diverges badly once the camera looks steeply down (a sideways pixel offset is mostly a bearing change at level, but not at -60).
//   stabilized = true : a stabilized gimbal (the C12, or any earth-levelled mount) -- cam_pitch_deg is already relative to the
//                       horizon, so the vehicle's own pitch must NOT be added (it was, previously: the dash's nose-down pitch then
//                       inflated the depression and pulled the refined endpoint short).
//   stabilized = false: a camera fixed to the airframe -- cam_pitch_deg is relative to the body, and the vehicle's pitch is applied.
// For a body-fixed camera the vehicle's ROLL is applied too (before its pitch: body -> level = Ry(pitch) Rx(roll), the ZYX
// convention of MAVLink ATTITUDE): negligible for a gentle multirotor follow, but an FPV airframe banks 30-60 deg in a turn,
// where ignoring it puts the line of sight -- and every ground estimate built on it -- far off. A stabilized gimbal removes roll.
struct ViewAngles { double bearing_right = 0, depression = 0; };
inline ViewAngles view_angles(double px_right, double px_down, double fx, double fy, double cam_yaw_deg, double cam_pitch_deg,
                              bool stabilized, double veh_pitch_rad, double veh_roll_rad = 0.0) {
    double x = 1.0, y = px_right / fx, z = px_down / fy;                       // camera frame: forward, right, down
    auto pitch_by = [&](double th) { double c = std::cos(th), s = std::sin(th), x2 = x * c + z * s, z2 = -x * s + z * c; x = x2; z = z2; };
    pitch_by(deg2rad(cam_pitch_deg));                                           // camera pitch (+ up) about its right axis
    { double ps = deg2rad(cam_yaw_deg), c = std::cos(ps), s = std::sin(ps), x2 = x * c - y * s, y2 = x * s + y * c; x = x2; y = y2; }   // camera yaw (+ right)
    if (!stabilized) {                                                          // body-fixed: the airframe's roll, then pitch, tilt the camera
        const double c = std::cos(veh_roll_rad), sn = std::sin(veh_roll_rad), y2 = y * c - z * sn, z2 = y * sn + z * c; y = y2; z = z2;
        pitch_by(veh_pitch_rad);
    }
    ViewAngles v; v.bearing_right = std::atan2(y, x); v.depression = std::atan2(z, std::hypot(x, y));
    return v;
}

// Vertical centring error for a camera FIXED to the airframe (an FPV drone: no gimbal, so the drone itself climbs / descends to
// keep the target in view). The raw image row is the wrong error there: pitching nose-down to accelerate tilts the camera down with
// the airframe, the target jumps up in the image, and a loop centring the image row climbs for its own pitch (and descends when it
// brakes) -- altitude coupled to every speed change. Instead the target's depression below the HORIZON (attitude removed, as for the
// ground estimate) is held at a reference: ref_dep_rad, the depression at lock (the framing the pilot chose), or -- NaN -- the
// camera's level-flight boresight (in a hover exactly the image centre). A pitch change no longer moves the error. Only when the
// airframe is pitched so far that the target would leave the view is the reference pulled towards the actual boresight (kept within
// `keep_frac` of the half vertical field of view, half_vfov_rad) -- at FPV lean angles (40-60 deg) this is what keeps it in frame.
// + = the target is below the reference -> descend (image_control's convention).
inline double body_cam_depression(double px_right, double px_down, double fx, double fy, double cam_yaw_deg, double cam_pitch_deg,
                                  double veh_pitch_rad, double veh_roll_rad) {
    return view_angles(px_right, px_down, fx, fy, cam_yaw_deg, cam_pitch_deg, false, veh_pitch_rad, veh_roll_rad).depression;
}
inline double body_cam_vertical_error(double px_right, double px_down, double fx, double fy, double cam_yaw_deg, double cam_pitch_deg,
                                      double veh_pitch_rad, double veh_roll_rad, double half_vfov_rad,
                                      double ref_dep_rad = std::numeric_limits<double>::quiet_NaN(), double keep_frac = 0.7) {
    const double dep = body_cam_depression(px_right, px_down, fx, fy, cam_yaw_deg, cam_pitch_deg, veh_pitch_rad, veh_roll_rad);
    const double ref = std::isfinite(ref_dep_rad) ? ref_dep_rad : body_cam_depression(0, 0, 1, 1, cam_yaw_deg, cam_pitch_deg, 0.0, 0.0);
    const double now_b = body_cam_depression(0, 0, 1, 1, cam_yaw_deg, cam_pitch_deg, veh_pitch_rad, veh_roll_rad);   // actual boresight
    const double m = keep_frac * half_vfov_rad;
    return dep - std::clamp(ref, now_b - m, now_b + m);
}

// =====================================================================================================================================
// [GO-2] ARM / CONFIRM -- the GO state machine (pure logic, tested by test_rescue). See the GO map at the top of the file.
//   ONE press = GO (default): IDLE -press-> SENT. --go-confirm: IDLE -press-> ARMED -press within 3 s-> SENT.
//   RState: DISABLED (no --enable-rescue) | IDLE (waiting for a press) | ARMED (--go-confirm only: confirm window open) |
//           SENT (send the go-to now; lasts ONE tick, then back to IDLE so the officer can GO again / redirect)
//   REnd:   why the last attempt ended (shown to the officer): aborted, mode changed, link lost, not ready, timeout, too far
//   RescueIn: everything the gates need this frame (lock, mode, link, GPS, EKF, battery, the destination); RescueOut.send_goto is
//   true on the single tick the main loop must start the dash ([GO-5]).
// =====================================================================================================================================
// FIXED CAMERA (FPV) -- keeping the locked target ON THE + vertically. The camera cannot tilt, so the target moves up/down in the
// picture only when the DRONE moves: going BACK or DOWN raises it in the picture, going FORWARD or UP lowers it. Both are used
// together, as ONE loop on the same error, so nothing fights:
//   vz (image_control, kz):           target below the + -> descend, above -> climb (never below --alt-floor)
//   vx (this function, k):            target below the + -> back away, above -> come closer
// Earlier, "move closer to see it bigger" (view control) drove vx while "back away at the height floor" switched on and off
// against it: the drone nodded forward/back +-20-28 deg every 1-2 s and the target kept leaving the frame (measured in SITL).
// Here vx comes ONLY from the centring error, smoothly (deadband() shrinks, it does not switch), capped at vmax.
// vert_err: body_cam_vertical_error, + = below the +; db: dead-band (rad); k: m/s per rad. Returns the forward speed (m/s).
inline double fixed_cam_centre_vx(double vert_err, double db, double k, double vmax) {
    if (!std::isfinite(vert_err)) return 0.0;
    return std::clamp(-k * deadband(vert_err, db), -vmax, vmax);
}

enum class RState { DISABLED, IDLE, ARMED, SENT };
enum class REnd { NONE, ABORTED, MODE_CHANGED, LINK_LOST, NOT_READY, CONFIRM_TIMEOUT, OUT_OF_RANGE };
inline const char* to_str(RState s) {
    switch (s) { case RState::DISABLED: return "DISABLED"; case RState::IDLE: return "IDLE"; case RState::ARMED: return "ARMED";
                 case RState::SENT: return "SENT"; }
    return "?";
}
inline const char* to_str(REnd e) {
    switch (e) { case REnd::NONE: return "none"; case REnd::ABORTED: return "aborted by the officer"; case REnd::MODE_CHANGED: return "mode changed";
                 case REnd::LINK_LOST: return "link lost"; case REnd::NOT_READY: return "not ready"; case REnd::CONFIRM_TIMEOUT: return "confirm window expired";
                 case REnd::OUT_OF_RANGE: return "destination out of range"; }
    return "?";
}

struct RescueCfg {
    bool enabled = false;
    double speed_ms = 5.0;            // ground speed for the transit (DO_CHANGE_SPEED); the vehicle's WP_SPD must allow it
    bool confirm = false;             // false (default): ONE press goes. true (--go-confirm): press to arm, press again to confirm
    double confirm_window_s = 3.0;    // --go-confirm: the second press (confirm) must come within this
    double min_locked_s = 2.0;        // locked at least this long before arming
    double max_range_m = 150.0;       // refuse destinations farther than this from the vehicle
    double min_battery_pct = 0;       // refuse to arm below this battery percentage (0 = off; unknown battery passes)
};

struct RescueIn {
    double now = 0;
    bool press = false, abort = false;
    FState follow = FState::IDLE; double locked_for = 0;
    bool guided = true, vehicle_armed = true, link_ok = true, gps_ok = true, pos_valid = true, ekf_ok = true;
    int battery_pct = -1;             // -1 = unknown
    bool dest_ok = false;             // a destination could be computed from the current view
    Ned dest;                         // target location for the CURRENT view
    Ned veh;                          // vehicle position now
};

struct RescueOut {
    RState state = RState::DISABLED; REnd end = REnd::NONE;
    bool send_goto = false;           // true on the one tick the go-to command should be sent (dest + speed)
    Ned dest;
    bool transitioned = false;
    std::string detail;               // which specific gate is blocking a NOT_READY, e.g. "not locked" / "no destination estimate yet"
};

// First readiness gate that's actually failing, in the same order `ready` below checks them -- "not ready" alone left the
// officer guessing which of eight conditions it was; found live that this mattered in practice (crossing objects making the
// box age into COASTING right as the officer reaches for the rescue key is a completely different problem from "no GPS").
inline std::string not_ready_reason(const RescueIn& in, const RescueCfg& cfg) {
    if (in.follow != FState::LOCKED) return "target not locked";
    if (in.locked_for < cfg.min_locked_s) { char b[64]; std::snprintf(b, sizeof b, "lock held %.1fs, need %.1fs", in.locked_for, cfg.min_locked_s); return b; }
    if (!in.guided) return "vehicle not in GUIDED";
    if (!in.vehicle_armed) return "vehicle not armed";
    if (!in.link_ok) return "no vehicle link";
    if (!in.gps_ok) return "no GPS fix";
    if (!in.pos_valid) return "vehicle position not valid";
    if (!in.ekf_ok) return "EKF not healthy";
    if (!in.dest_ok) return "no 3D target destination fix yet";
    if (cfg.min_battery_pct > 0 && in.battery_pct >= 0 && in.battery_pct < cfg.min_battery_pct) return "battery " + std::to_string(in.battery_pct) + "% below minimum";
    return "";
}

// ONE press (default): if every readiness gate passes, the go-to is sent at once (SENT for one tick, then IDLE: another press
// re-sends GO at the target's current position, e.g. to redirect it). --go-confirm restores the two-step: press once -> ARMED
// (a confirm window opens), press again within the window -> sent.
class RescueController {
public:
    explicit RescueController(RescueCfg c = RescueCfg()) : cfg_(c) { st_ = cfg_.enabled ? RState::IDLE : RState::DISABLED; }
    RState state() const { return st_; }
    const RescueCfg& cfg() const { return cfg_; }
    void reset() {
        if (st_ != RState::DISABLED) st_ = RState::IDLE;
        end_ = REnd::NONE;
        armed_at_ = 0;
    }

    RescueOut step(const RescueIn& in) {
        RescueOut o; o.state = st_; o.end = end_;
        if (st_ == RState::DISABLED) return o;
        const bool ready = in.follow == FState::LOCKED && in.locked_for >= cfg_.min_locked_s && in.guided && in.vehicle_armed && in.link_ok &&
                           in.gps_ok && in.pos_valid && in.ekf_ok && in.dest_ok &&
                           !(cfg_.min_battery_pct > 0 && in.battery_pct >= 0 && in.battery_pct < cfg_.min_battery_pct);
        if (st_ == RState::SENT) { st_ = RState::IDLE; o.transitioned = true; }               // one tick in SENT, then ready for another confirm
        switch (st_) {
            case RState::IDLE:
                if (in.press) {
                    if (ready && !cfg_.confirm) {                                 // ONE-PRESS GO: the same gates, then go at once
                        if (std::hypot(in.dest.n - in.veh.n, in.dest.e - in.veh.e) > cfg_.max_range_m) { end_ = REnd::OUT_OF_RANGE; o.end = end_; o.transitioned = true; }
                        else { st_ = RState::SENT; o.dest = in.dest; o.send_goto = true; o.transitioned = true; end_ = REnd::NONE; }
                    }
                    else if (ready) { st_ = RState::ARMED; armed_at_ = in.now; o.transitioned = true; }   // --go-confirm: arm, wait for the 2nd press
                    else { end_ = REnd::NOT_READY; o.end = end_; o.detail = not_ready_reason(in, cfg_); o.transitioned = true; }
                }
                break;
            case RState::ARMED:
                if (in.abort) { st_ = RState::IDLE; end_ = REnd::ABORTED; o.end = end_; o.transitioned = true; }
                else if (!ready) { st_ = RState::IDLE; end_ = !in.link_ok ? REnd::LINK_LOST : (!in.guided ? REnd::MODE_CHANGED : REnd::NOT_READY); o.end = end_; o.detail = not_ready_reason(in, cfg_); o.transitioned = true; }
                else if (in.now - armed_at_ > cfg_.confirm_window_s) { st_ = RState::IDLE; end_ = REnd::CONFIRM_TIMEOUT; o.end = end_; o.transitioned = true; }
                else if (in.press && in.now - armed_at_ > 0.3) {   // a deliberate second press, not a double-send
                    double range = std::hypot(in.dest.n - in.veh.n, in.dest.e - in.veh.e);
                    if (range > cfg_.max_range_m) { st_ = RState::IDLE; end_ = REnd::OUT_OF_RANGE; o.end = end_; o.transitioned = true; }
                    else { st_ = RState::SENT; o.dest = in.dest; o.send_goto = true; o.transitioned = true; end_ = REnd::NONE; }
                }
                break;
            default: break;
        }
        o.state = st_; o.end = end_;
        return o;
    }
private:
    RescueCfg cfg_; RState st_ = RState::DISABLED; REnd end_ = REnd::NONE; double armed_at_ = 0;
};

// EKF_STATUS_REPORT health (ardupilotmega dialect): ALL of attitude, horizontal velocity and absolute horizontal position must be
// good, and neither "never been healthy" nor "GPS glitching" may be set. (A previous version accepted ANY one of the three good
// flags, so an EKF with attitude only -- no position at all -- passed the rescue's "EKF healthy" gate.)
inline bool ekf_flags_healthy(uint16_t flags) {
    const uint16_t need = 1 /*ATTITUDE*/ | 2 /*VELOCITY_HORIZ*/ | 16 /*POS_HORIZ_ABS*/;
    const uint16_t bad = 128 /*CONST_POS_MODE*/ | 1024 /*UNINITIALIZED*/ | 32768 /*GPS_GLITCHING*/;
    return (flags & need) == need && (flags & bad) == 0;
}

// Height of the vehicle above the ground the target stands on, for the flat-ground range estimate. "home": altitude above the
// home/takeoff point -- silently wrong whenever the target's ground is higher or lower than home (a hill, a rooftop, a riverbank).
// "rangefinder": a fresh downward rangefinder reading -- the ground right under the vehicle, still assumed flat out to the target,
// but no longer tied to where it took off. "auto": the rangefinder when fresh, else home. Returns false if the required source is
// unavailable; `used` names the source actually used (shown to the officer).
inline bool pick_height(const std::string& mode, bool alt_valid, double alt_home_m, double rng_m, double rng_age_s,
                        double& out_m, std::string& used) {
    const bool rng_fresh = rng_age_s < 0.5 && rng_m > 0.2;
    if ((mode == "rangefinder" || mode == "auto") && rng_fresh) { out_m = rng_m; used = "rangefinder"; return true; }
    if (mode == "rangefinder") { used = "rangefinder (no data)"; return false; }
    if (!alt_valid) { used = "home (no altitude)"; return false; }
    out_m = alt_home_m; used = "home"; return true;
}

// World-frame target estimator: a constant-velocity Kalman filter on the target's ground position (north, east), fed by the
// per-frame flat-ground estimate (target_ned_from_view). It turns a noisy per-frame position into a smoothed position AND a
// velocity -- which the image-space Kalman (BoxKalman) cannot give: a box moving in the image mixes the target's motion with the
// drone's own. Used for follow feed-forward (match the target's speed instead of always lagging behind it), the dash's lead point
// (fly to where the target WILL be) and staying over a moving target after arrival. Time is in seconds (not frames).
class Cv1d {                                                     // one axis: state [p, v]
public:
    void reset(double p) { p_ = p; v_ = 0; P_[0] = 4; P_[1] = P_[2] = 0; P_[3] = 25; }
    void predict(double dt, double q_acc) {
        p_ += v_ * dt;
        const double a = P_[0] + dt * (P_[1] + P_[2]) + dt * dt * P_[3], b = P_[1] + dt * P_[3], c = P_[2] + dt * P_[3];
        const double q = q_acc * q_acc;
        P_[0] = a + q * dt * dt * dt * dt / 4; P_[1] = b + q * dt * dt * dt / 2; P_[2] = c + q * dt * dt * dt / 2; P_[3] += q * dt * dt;
    }
    void update(double z, double r) {
        const double S = P_[0] + r, k0 = P_[0] / S, k1 = P_[2] / S, y = z - p_;
        p_ += k0 * y; v_ += k1 * y;
        const double p0 = P_[0], p1 = P_[1];
        P_[0] -= k0 * p0; P_[1] -= k0 * p1; P_[2] -= k1 * p0; P_[3] -= k1 * p1;
    }
    double p() const { return p_; }
    double v() const { return v_; }
    double var_v() const { return P_[3]; }
private:
    double p_ = 0, v_ = 0, P_[4] = {4, 0, 0, 25};
};

// The target's GROUND track (NED, metres): position + velocity + turn rate, from the per-frame ground fixes. An INTERACTING MULTIPLE
// MODEL of two filters on the same 5-state x = [n, e, vn, ve, w]:
//   STRAIGHT: constant velocity (w held at 0) -- linear;
//   TURNING:  coordinated turn (the velocity vector rotates at w, w a random walk) -- NONLINEAR in w, so its prediction is an
//             UNSCENTED transform (sigma points through the exact turn equations): an IMM-UKF.
// Each update re-weights the two by how well each predicted the fix; the output and the look-ahead (predict) blend them. A vehicle
// turning a corner is now predicted ALONG ITS ARC (the GO aim point, the speed match), where the old straight-line filter put it
// off the outside of the turn and lagged its new direction. The fix itself is linear (position), so the update is a plain Kalman
// update on the predicted moments. imm = 0: the straight-line filter (the self-test baseline).
class TargetEstimator {
public:
    // imm: 1 = IMM-UKF (always, live), 0 = the straight-line filter (the self-test baseline only)
    explicit TargetEstimator(double q_acc = 2.0, int imm = 1) : q_acc_(q_acc), imm_(imm != 0) {}
    void reset() { n_ = 0; }
    // meas: the per-frame ground position; sigma_m: its 1-sigma error (grows with range -- see geometry_sigma)
    void update(double now, const Ned& meas, double sigma_m) {
        if (!imm_) {
            if (n_ == 0 || now - t_ > 1.0) { kn_.reset(meas.n); ke_.reset(meas.e); n_ = 1; t_ = now; d_ = meas.d; return; }   // first fix, or a gap
            const double dt = std::max(1e-3, now - t_);
            kn_.predict(dt, q_acc_); ke_.predict(dt, q_acc_);
            const double r = sigma_m * sigma_m;
            kn_.update(meas.n, r); ke_.update(meas.e, r);
            d_ = 0.9 * d_ + 0.1 * meas.d; t_ = now; n_++;
            return;
        }
        const double r = sigma_m * sigma_m;
        if (n_ == 0 || now - t_ > 1.0) {                                  // first fix, or a gap: start over at rest
            for (int j = 0; j < 2; j++) {
                m_[j].x = {meas.n, meas.e, 0, 0, 0}; m_[j].P.fill(0);
                m_[j].P[0] = 4; m_[j].P[6] = 4; m_[j].P[12] = 25; m_[j].P[18] = 25; m_[j].P[24] = j == 1 ? 0.25 : 1e-4;
            }
            mu_ = {0.8, 0.2}; n_ = 1; t_ = now; d_ = meas.d; return;
        }
        const double dt = std::clamp(now - t_, 1e-3, 1.0);
        // 1. mixing (model switching per second: straight -> turning 10%/s, back 30%/s)
        static const double r01 = std::getenv("EST_P01") ? std::atof(std::getenv("EST_P01")) : 0.1, r10 = std::getenv("EST_P10") ? std::atof(std::getenv("EST_P10")) : 0.3;   // tuned: see test_estimator
        const double p01 = std::min(0.3, r01 * dt), p10 = std::min(0.3, r10 * dt);
        const double PI[2][2] = {{1 - p01, p01}, {p10, 1 - p10}};
        double c[2]; for (int j = 0; j < 2; j++) c[j] = PI[0][j] * mu_[0] + PI[1][j] * mu_[1];
        Model mixd[2];
        for (int j = 0; j < 2; j++) {
            const double w0 = PI[0][j] * mu_[0] / std::max(c[j], 1e-12), w1 = PI[1][j] * mu_[1] / std::max(c[j], 1e-12);
            for (int i = 0; i < 5; i++) mixd[j].x[i] = w0 * m_[0].x[i] + w1 * m_[1].x[i];
            for (int a = 0; a < 5; a++) for (int b = 0; b < 5; b++) {
                const double d0a = m_[0].x[a] - mixd[j].x[a], d0b = m_[0].x[b] - mixd[j].x[b], d1a = m_[1].x[a] - mixd[j].x[a], d1b = m_[1].x[b] - mixd[j].x[b];
                mixd[j].P[a * 5 + b] = w0 * (m_[0].P[a * 5 + b] + d0a * d0b) + w1 * (m_[1].P[a * 5 + b] + d1a * d1b);
            }
        }
        // 2. predict (unscented) + 3. update each model, with its likelihood
        double L[2];
        for (int j = 0; j < 2; j++) {
            m_[j] = mixd[j];
            ukf_predict(m_[j], dt, j == 1);
            L[j] = kf_update(m_[j], meas.n, meas.e, r);
        }
        const double s0 = L[0] * c[0], s1 = L[1] * c[1], sum = s0 + s1;
        if (sum > 1e-300 && std::isfinite(sum)) { mu_[0] = std::clamp(s0 / sum, 1e-4, 1 - 1e-4); mu_[1] = 1 - mu_[0]; } else { mu_[0] = c[0]; mu_[1] = c[1]; }
        d_ = 0.9 * d_ + 0.1 * meas.d; t_ = now; n_++;
    }
    // usable: enough fixes, recent, and the velocity has actually converged (not just the initial guess)
    bool valid(double now, double max_age = 0.5) const {
        if (!imm_) return n_ >= 10 && now - t_ < max_age && kn_.var_v() < 1.0 && ke_.var_v() < 1.0;
        return n_ >= 10 && now - t_ < max_age && mixed_P(2, 2) < 1.0 && mixed_P(3, 3) < 1.0;
    }
    double last_fix() const { return t_; }
    Ned pos() const { if (!imm_) return Ned{kn_.p(), ke_.p(), d_}; const auto x = mixed(); return Ned{x[0], x[1], d_}; }
    double vn() const { return imm_ ? mixed()[2] : kn_.v(); }
    double ve() const { return imm_ ? mixed()[3] : ke_.v(); }
    double turn_rate() const { return imm_ ? mixed()[4] : 0.0; }        // rad/s, + = turning right (clockwise seen from above)
    double turn_prob() const { return imm_ ? mu_[1] : 0.0; }
    // where it will be t_ahead s after the last fix: each model along its OWN motion (the turning one along the arc), blended
    Ned predict(double t_ahead) const {
        if (!imm_) return Ned{kn_.p() + kn_.v() * t_ahead, ke_.p() + ke_.v() * t_ahead, d_};
        double n = 0, e = 0;
        for (int j = 0; j < 2; j++) { auto x = m_[j].x; move(x, t_ahead, j == 1); n += mu_[j] * x[0]; e += mu_[j] * x[1]; }
        return Ned{n, e, d_};
    }
private:
    struct Model { std::array<double, 5> x{}; std::array<double, 25> P{}; };
    // the motion over dt: straight (w ignored and decays) or the exact coordinated turn
    static void move(std::array<double, 5>& x, double dt, bool turning) {
        const double w = turning ? x[4] : 0.0, vn = x[2], ve = x[3];
        if (std::fabs(w) < 1e-4) { x[0] += vn * dt; x[1] += ve * dt; }
        else {
            const double sn = std::sin(w * dt), cs = std::cos(w * dt);
            x[0] += (vn * sn - ve * (1 - cs)) / w; x[1] += (vn * (1 - cs) + ve * sn) / w;
            x[2] = vn * cs - ve * sn; x[3] = vn * sn + ve * cs;
        }
        if (!turning) x[4] = 0.0;
    }
    void ukf_predict(Model& m, double dt, bool turning) const {
        // sigma points: x +- columns of sqrt(n P) (lambda = 0: equal weights 1/2n, a symmetric and always-positive set)
        const int n = 5; double Lc[25] = {0};
        std::array<double, 25> A = m.P; for (auto& v : A) v *= n;
        for (int i = 0; i < n; i++) A[i * 5 + i] += 1e-9;
        for (int i = 0; i < n; i++) for (int j = 0; j <= i; j++) {           // Cholesky A = Lc Lc^T
            double sum = A[i * 5 + j]; for (int k = 0; k < j; k++) sum -= Lc[i * 5 + k] * Lc[j * 5 + k];
            if (i == j) Lc[i * 5 + i] = std::sqrt(std::max(sum, 1e-12)); else Lc[i * 5 + j] = sum / Lc[j * 5 + j];
        }
        std::array<double, 5> X[10];
        for (int k = 0; k < n; k++) for (int sgn = 0; sgn < 2; sgn++) {
            auto& xs = X[2 * k + sgn]; xs = m.x;
            for (int i = 0; i < n; i++) xs[i] += (sgn ? -1 : 1) * Lc[i * 5 + k];
            move(xs, dt, turning);
        }
        std::array<double, 5> mean{}; for (auto& xs : X) for (int i = 0; i < n; i++) mean[i] += xs[i] / 10.0;
        std::array<double, 25> P{};
        for (auto& xs : X) for (int a = 0; a < n; a++) for (int b = 0; b < n; b++) P[a * 5 + b] += (xs[a] - mean[a]) * (xs[b] - mean[b]) / 10.0;
        // process noise: white acceleration on each axis (q_acc), the turn rate a random walk (turning) or pinned near 0 (straight)
        const double q = q_acc_ * q_acc_;
        for (int ax = 0; ax < 2; ax++) {                                   // discrete white acceleration (as the straight-line filter had)
            P[ax * 5 + ax] += q * dt * dt * dt * dt / 4; P[(ax + 2) * 5 + (ax + 2)] += q * dt * dt;
            P[ax * 5 + ax + 2] += q * dt * dt * dt / 2; P[(ax + 2) * 5 + ax] += q * dt * dt * dt / 2;
        }
        static const double wq = std::getenv("EST_WQ") ? std::atof(std::getenv("EST_WQ")) : 0.05;
        P[24] += turning ? wq * dt : 1e-6;
        if (!turning) { for (int i = 0; i < 5; i++) { P[4 * 5 + i] = P[i * 5 + 4] = 0; } P[24] = 1e-4; }
        m.x = mean; m.P = P;
    }
    // position fix (linear): Kalman update; returns the fix's likelihood under this model
    static double kf_update(Model& m, double zn, double ze, double r) {
        const double S00 = m.P[0] + r, S01 = m.P[1], S10 = m.P[5], S11 = m.P[6] + r, det = S00 * S11 - S01 * S10;
        if (std::fabs(det) < 1e-12) return 1e-300;
        const double i00 = S11 / det, i01 = -S01 / det, i10 = -S10 / det, i11 = S00 / det;
        const double y0 = zn - m.x[0], y1 = ze - m.x[1];
        double K[10];
        for (int a = 0; a < 5; a++) { K[a * 2] = m.P[a * 5] * i00 + m.P[a * 5 + 1] * i10; K[a * 2 + 1] = m.P[a * 5] * i01 + m.P[a * 5 + 1] * i11; }
        for (int a = 0; a < 5; a++) m.x[a] += K[a * 2] * y0 + K[a * 2 + 1] * y1;
        std::array<double, 25> P = m.P;
        for (int a = 0; a < 5; a++) for (int b = 0; b < 5; b++) P[a * 5 + b] -= K[a * 2] * m.P[b] + K[a * 2 + 1] * m.P[5 + b];
        for (int a = 0; a < 5; a++) for (int b = a + 1; b < 5; b++) P[a * 5 + b] = P[b * 5 + a] = 0.5 * (P[a * 5 + b] + P[b * 5 + a]);   // symmetric
        m.P = P;
        const double q = y0 * (i00 * y0 + i01 * y1) + y1 * (i10 * y0 + i11 * y1);
        return std::exp(-0.5 * std::min(q, 200.0)) / (2 * M_PI * std::sqrt(det));
    }
    std::array<double, 5> mixed() const { std::array<double, 5> x{}; for (int j = 0; j < 2; j++) for (int i = 0; i < 5; i++) x[i] += mu_[j] * m_[j].x[i]; return x; }
    double mixed_P(int a, int b) const {
        const auto x = mixed(); double v = 0;
        for (int j = 0; j < 2; j++) v += mu_[j] * (m_[j].P[a * 5 + b] + (m_[j].x[a] - x[a]) * (m_[j].x[b] - x[b]));
        return v;
    }
    double q_acc_; Cv1d kn_, ke_; int n_ = 0; double t_ = 0, d_ = 0;
    bool imm_ = true; Model m_[2]; std::array<double, 2> mu_{0.8, 0.2};
};

// 1-sigma ground-position error of the flat-ground estimate: d = h / tan(dep), so a small angle error moves it by
// h / sin^2(dep) per radian. ang_err ~1 deg (box edge + attitude + timing); floor 0.5 m.
inline double geometry_sigma(double height_m, double depression_rad, double ang_err_rad = 0.0175) {
    const double s = std::sin(std::max(depression_rad, 0.05));
    return 0.5 + height_m / (s * s) * ang_err_rad;
}

// Follow feed-forward: the target's ground velocity rotated into the body frame (forward, right), scaled by `gain` (0 = off).
// Added to the image-servo command, it lets the drone match a moving target's speed instead of only reacting to the error it has
// already built up (a pure P loop always trails a moving target), and adds the sideways (vy) component the image servo never had.
inline Cmd add_feedforward(Cmd c, double vn, double ve, double yaw_rad, double gain) {
    if (gain <= 0) return c;
    c.vx += gain * (vn * std::cos(yaw_rad) + ve * std::sin(yaw_rad));
    c.vy += gain * (-vn * std::sin(yaw_rad) + ve * std::cos(yaw_rad));
    return c;
}

// Vehicle pose history for latency compensation: a video frame shows the world as it was `latency` seconds ago (measured in the
// sim: ~0.2 s behind the telemetry), so its geometry must use the vehicle's position, yaw and gimbal angle FROM THEN. With the
// current pose instead, a turning or accelerating drone sees the target move with it -- measured: the ground-position error rose
// from ~1.2 m to 5 m (p90 20 m) while chasing, the velocity estimate followed the drone, and the chase fed on itself.
struct PoseSample { double t = 0, n = 0, e = 0, d = 0, yaw = 0, pitch = 0, roll = 0, alt = 0, mpitch = 0, myaw = 0; };
class PoseHistory {
public:
    void push(const PoseSample& p) { if (!h_.empty() && p.t <= h_.back().t) return; h_.push_back(p); while (h_.size() > 200) h_.pop_front(); }
    void clear() { h_.clear(); }
    // linear interpolation at time t (angles along the shorter way round); false when t is not covered (then use the latest)
    bool at(double t, PoseSample& out) const {
        if (h_.size() < 2 || t < h_.front().t || t > h_.back().t) return false;
        size_t i = 1; while (i < h_.size() && h_[i].t < t) i++;
        const PoseSample &a = h_[i - 1], &b = h_[i];
        const double u = (b.t - a.t) > 1e-9 ? (t - a.t) / (b.t - a.t) : 0.0;
        auto lin = [u](double x, double y) { return x + u * (y - x); };
        auto ang = [u](double x, double y) { double dd = std::remainder(y - x, 2 * M_PI); return std::remainder(x + u * dd, 2 * M_PI); };
        auto angd = [u](double x, double y) { double dd = std::remainder(y - x, 360.0); return std::remainder(x + u * dd, 360.0); };
        out = PoseSample{t, lin(a.n, b.n), lin(a.e, b.e), lin(a.d, b.d), ang(a.yaw, b.yaw), lin(a.pitch, b.pitch), lin(a.roll, b.roll), lin(a.alt, b.alt),
                         lin(a.mpitch, b.mpitch), angd(a.myaw, b.myaw)};
        return true;
    }
private:
    std::deque<PoseSample> h_;
};

// FPV chase (--fpv): fly WITH the target instead of turning on the spot. Horizontal velocity (NED) = the target's own estimated
// ground velocity (feed-forward: no trailing) + a pull along the line of sight that holds `standoff` metres (dead-band 1 m),
// capped at `vmax`; returned in the body frame (forward, right). Measured in SITL before this: chasing a 4 m/s target the drone
// only yawed (median ground speed 0.1 m/s) because the size servo was capped at 1-1.5 m/s and had no target velocity.
// radial_only (default): only the component along the line of sight -- distance is held by flying, sideways target motion is
// followed by YAW. Measured: the full 2D feed-forward turned velocity-estimate noise (p90 ~4 m/s) into sideways lurches (roll std
// 7 deg) that shook the target out of the view.
// pull_max: the approach / back-off part alone (measured: from 28 m the uncapped pull charged at the full 10 m/s, the hard pitch
// threw the target out of the bottom of the view); the feed-forward still adds the target's own speed on top
struct ChaseCfg { double standoff = 10.0, k = 0.5, ff = 0.9, vmax = 12.0, db = 1.0, pull_max = 1e9; bool radial_only = false; };
inline Cmd chase_cmd(const ChaseCfg& c, double veh_n, double veh_e, double yaw_rad, const Ned& tgt, double tvn, double tve) {
    const double dn = tgt.n - veh_n, de = tgt.e - veh_e, d = std::hypot(dn, de);
    double vn = c.ff * tvn, ve = c.ff * tve;
    if (c.radial_only && d > 0.5) { const double r = (vn * dn + ve * de) / d; vn = r * dn / d; ve = r * de / d; }
    if (d > 0.5) {
        // inside the standoff the feed-forward may not close the distance any further (measured: an over-estimated target speed at
        // close range brought the drone to 6 m of a 12 m standoff and the target dropped out of the bottom of the view)
        const double toward = (vn * dn + ve * de) / d;
        if (d < c.standoff && toward > 0) { vn -= toward * dn / d; ve -= toward * de / d; }
        const double pull = sym_clamp(c.k * deadband(d - c.standoff, c.db), c.pull_max); vn += pull * dn / d; ve += pull * de / d;
    }
    const double sp = std::hypot(vn, ve);
    if (sp > c.vmax) { vn *= c.vmax / sp; ve *= c.vmax / sp; }
    Cmd out; out.vx = vn * std::cos(yaw_rad) + ve * std::sin(yaw_rad); out.vy = -vn * std::sin(yaw_rad) + ve * std::cos(yaw_rad);
    return out;
}

// How fast the bearing from the drone to the target turns (rad/s, + clockwise seen from above): the yaw rate that keeps a moving
// target centred with no error at all. A yaw loop driven by the image error alone trails a target that keeps circling -- measured:
// 93 px steady offset at 0.45 rad/s (= rate / kyaw).
inline double bearing_rate(double dn, double de, double rvn, double rve) {
    const double d2 = dn * dn + de * de;
    return d2 > 1e-6 ? (dn * rve - de * rvn) / d2 : 0.0;
}

// =====================================================================================================================================
// [GO-4] GUIDANCE -- how the drone flies at the target during the dash (pure functions, tested by test_go_guidance).
//   pursuit_point    the end point: `standoff` short of the target, never below the height floor
//   pursuit_velocity straight at a point, braking so it stops there (v <= sqrt(2 a d), allowing for the autopilot's lag)
//   intercept_point  where a moving target will be when the drone reaches it (iterated time-to-go)
//   go_velocity      the command for each --go-guidance mode (0 pursuit, 1 lead, 2 intercept, 3 PN), capped at the GO speed
//   pace_along       the old --goto-paced mode only: a point paced along a fixed line with a braking taper
// Used every frame by the dash loop [GO-6]. See the GO map at the top of the file.
// =====================================================================================================================================
// GO STOP POINT: where the dash ends RIGHT NOW, from the target's CURRENT position and the signed offset `standoff` (metres,
// horizontal -- go_settings::kStopOffset / --go-offset / "go SPEED OFFSET"):
//   standoff > 0  BEFORE the target: that far short of it on the line from the vehicle to it; closer than that = hold where it is
//   standoff = 0  AT the target
//   standoff < 0  PAST the target: |standoff| beyond it, along the APPROACH direction from `from` (where GO started) to the target.
//                 Not from the vehicle: its own direction to the target flips as it flies over, which would turn the stop point
//                 round behind it. Without `from`, the vehicle (fine for the one-off estimate at the press, where they coincide).
// Never lower than `floor_d` (NED down: the stop point is at least that high). Re-evaluated every frame while the dash runs, so a
// moving target is chased, not a point it has left.
// FLY-THROUGH GO: how far the vehicle is PAST the target along the approach direction (from `from`, where GO started, to the target),
// metres, horizontal: < 0 still before it, > 0 beyond it. If GO started right above it (no direction): along the heading yaw_rad.
inline double past_target_m(const Ned& veh, const Ned& tgt, const Ned& from, double yaw_rad) {
    double un = tgt.n - from.n, ue = tgt.e - from.e; const double h = std::hypot(un, ue);
    if (h > 1e-3) { un /= h; ue /= h; } else { un = std::cos(yaw_rad); ue = std::sin(yaw_rad); }
    return (veh.n - tgt.n) * un + (veh.e - tgt.e) * ue;
}
inline Ned pursuit_point(const Ned& veh, const Ned& tgt, double standoff, double floor_d, const Ned* from = nullptr) {
    const double dn = tgt.n - veh.n, de = tgt.e - veh.e, dh = std::hypot(dn, de);
    Ned e{tgt.n, tgt.e, tgt.d}; // Sets destination altitude directly to the target's depth (tgt.d)
    
    if (std::abs(standoff) < 1e-4) {
        e.n = tgt.n; e.e = tgt.e;
    } else if (standoff > 0) {
        if (dh > standoff) { const double k = (dh - standoff) / dh; e.n = veh.n + k * dn; e.e = veh.e + k * de; }
        else { e.n = veh.n; e.e = veh.e; }
    } else {
        const Ned& o = from ? *from : veh;
        const double an = tgt.n - o.n, ae = tgt.e - o.e, ah = std::hypot(an, ae);
        if (ah > 1e-3) { e.n = tgt.n - standoff * an / ah; e.e = tgt.e - standoff * ae / ah; }
        else { e.n = tgt.n; e.e = tgt.e; }
    }
    
    if (e.d > floor_d) e.d = floor_d;                                   // too low: hold the floor height (NED d grows downward)
    return e;
}

// GO pursuit velocity: straight at the end point at the GO speed, braking from the vehicle's ACTUAL distance so that it stops
// there: v <= sqrt(2 a (d - v_now * lag)) -- the autopilot needs ~`lag` s to respond. When lag == 0 (direct contact/zero standoff),
// maintains positive closing velocity all the way down to d < 0.05m to prevent premature stopping short of the target.
inline Ned pursuit_velocity(const Ned& veh, const Ned& end, double v_now, double speed, double acc, double lag) {
    const double dn = end.n - veh.n, de = end.e - veh.e, dd = end.d - veh.d, d = std::sqrt(dn * dn + de * de + dd * dd);
    if (d < 0.05) return Ned{0, 0, 0};
    const double d_eff = lag > 0.0 ? std::max(0.0, d - v_now * lag) : d;
    double s = std::min(speed, std::sqrt(2.0 * acc * d_eff));
    if (lag <= 0.0 && d > 0.05) {
        s = std::min(speed, s + 0.3); // positive closing velocity all the way down to target
    }
    return Ned{dn / d * s, de / d * s, dd / d * s};
}

// GO guidance (--go-guidance). PURSUIT aims at where the target IS: against a moving target that is a tail chase -- the drone always
// heads for a point the target has already left, curves in behind it and arrives late. INTERCEPT aims at where it WILL BE when the
// drone gets there: the target's own track (the IMM-UKF, which follows turns along the arc) predicted by the time to go, iterated
// (the time to go depends on the aim point). Near the end the time to go shrinks to 0, so the end point is the target itself.
// predict(t): the target's position t s from now.
template <class Predict>
inline Ned intercept_point(const Ned& veh, double speed, const Predict& predict, double max_t = 8.0) {
    Ned T = predict(0.0);
    for (int i = 0; i < 4; i++) {
        const double d = std::sqrt((T.n - veh.n) * (T.n - veh.n) + (T.e - veh.e) * (T.e - veh.e) + (T.d - veh.d) * (T.d - veh.d));
        T = predict(std::min(max_t, d / std::max(0.5, speed)));
    }
    return T;
}
// GO velocity against a MOVING target (modes 1-3 of --go-guidance; 0 = pursuit_velocity alone, the previous law):
// The previous law braked to a stop AT the target's current point as if it stood still -- against a target crossing at 5 m/s the
// drone slowed to below the target's speed short of it and never reached it (simulated). Here the approach is RELATIVE: the
// target's own velocity (v_tgt, from the IMM-UKF) plus a closing velocity braked on the relative distance, so the drone matches
// the target's motion as it arrives (then the end point, `standoff` short of it, is held relative to it).
//   1 LEAD (parallel navigation): closing velocity straight at the target -- relative motion along the line of sight, so the line
//     of sight does not rotate: the collision course proportional navigation converges to, without its delay.
//   2 INTERCEPT: aimed at where the target will be when the drone gets there (intercept_point: the IMM's turn-aware prediction).
//   3 PN (proportional navigation, N): LEAD plus N x the measured line-of-sight rate (from the drone's ACTUAL velocity) as a
//     perpendicular correction -- removes what the lead leaves (the target accelerating/turning, the drone's response lag).
// The total is capped at the GO speed (direction kept). Vertical: as the pursuit's.
template <class Predict>
inline Ned go_velocity(int mode, const Ned& veh, const Ned& v_veh, const Ned& tgt, const Ned& v_tgt, const Predict& predict,
                       double standoff, double floor_d, double speed, double acc, double lag, double N, const Ned* from = nullptr) {
    // standoff: the signed stop offset (+ before / - past the target), `from`: where GO started -- see pursuit_point
    const double vnow = std::sqrt(v_veh.n * v_veh.n + v_veh.e * v_veh.e + v_veh.d * v_veh.d);
    const double eff_lag = (standoff <= 0.0) ? 0.0 : lag;
    if (mode == 0) return pursuit_velocity(veh, pursuit_point(veh, tgt, standoff, floor_d, from), vnow, speed, acc, eff_lag);
    const Ned aim = mode == 2 ? intercept_point(veh, speed, predict) : tgt;
    const Ned end = pursuit_point(veh, aim, standoff, floor_d, from);
    const Ned vrel{v_veh.n - v_tgt.n, v_veh.e - v_tgt.e, v_veh.d - v_tgt.d};
    const double vrel_m = std::sqrt(vrel.n * vrel.n + vrel.e * vrel.e + vrel.d * vrel.d);
    Ned v = pursuit_velocity(veh, end, vrel_m, speed, acc, eff_lag);                 // closing velocity, braked on the relative motion
    // near the aim point the target's motion takes over; far away (intercept) the aim already leads it
    const double dn = end.n - veh.n, de = end.e - veh.e, dd = end.d - veh.d, d = std::sqrt(dn * dn + de * de + dd * dd);
    const double ff = mode == 2 ? std::clamp(1.0 - d / std::max(1.0, 1.5 * speed), 0.0, 1.0) : 1.0;
    v.n += ff * v_tgt.n; v.e += ff * v_tgt.e; v.d += ff * v_tgt.d;
    if (mode == 3) {
        // True 3D Proportional Navigation: nullifies line-of-sight rotation rate vector in 3D space
        const double rn = tgt.n - veh.n, re = tgt.e - veh.e, rd = tgt.d - veh.d;
        const double r2 = rn * rn + re * re + rd * rd, r = std::sqrt(r2);
        if (r > 0.05) {
            const double un = rn / r, ue = re / r, ud = rd / r;
            // Target relative velocity to vehicle in 3D NED coordinates
            const double vtn = v_tgt.n - v_veh.n, vte = v_tgt.e - v_veh.e, vtd = v_tgt.d - v_veh.d;
            // Velocity component along LOS
            const double v_para = vtn * un + vte * ue + vtd * ud;
            // Cross-LOS relative velocity vector in 3D space
            const double v_perp_n = vtn - v_para * un;
            const double v_perp_e = vte - v_para * ue;
            const double v_perp_d = vtd - v_para * ud;
            // 3D Proportional Navigation steering correction
            const double gain = std::max(1.0, N - 1.0);
            v.n += gain * v_perp_n;
            v.e += gain * v_perp_e;
            v.d += gain * v_perp_d;
        }
    }
    const double vm = std::sqrt(v.n * v.n + v.e * v.e + v.d * v.d);
    if (vm > speed) { v.n *= speed / vm; v.e *= speed / vm; v.d *= speed / vm; }
    return v;
}

// Dash pacing with a braking taper: the paced point advances at `speed` but slows as it nears the end, so the commanded point
// never outruns what the vehicle can stop from (v <= sqrt(2 a d_remaining)) -- previously the point moved at full speed right up
// to the end and stopped dead, leaving ArduPilot to brake hard / overshoot at the target. Returns the new distance along the line.
inline double pace_along(double along, double total, double speed, double decel, double dt) {
    if (total <= 0) return 0;
    const double rem = std::max(0.0, total - along);
    const double v = decel > 0 ? std::min(speed, std::sqrt(2.0 * decel * rem) + 0.3) : speed;   // +0.3 m/s: always finishes
    return std::min(total, along + v * std::max(0.0, dt));
}

// Geodetic position of a point `dn`/`de` metres north/east of a known lat/lon (flat-earth: exact enough over the few hundred
// metres a rescue covers -- the error is ~1 cm at 500 m).
inline void offset_latlon(double lat_deg, double lon_deg, double dn, double de, double& out_lat, double& out_lon) {
    const double R = 6378137.0;
    out_lat = lat_deg + rad2deg(dn / R);
    out_lon = lon_deg + rad2deg(de / (R * std::cos(deg2rad(lat_deg))));
}

// ---- view control: get a big enough view of the locked target, by ZOOMING NOW and MOVING CLOSER together --------------------
// Why: a person seen from 12-17 m is ~10 px tall; on so few pixels the tracker and its appearance check have almost nothing to
// work with, and people were being lost. Measured live: "move closer first, zoom only if you can't" was too slow -- the walker
// was lost ~2 s after the lock, long before the drone got near. So both act together:
//   * ZOOM (instant): the zoom that makes the target min_px tall, from its UNZOOMED size h1 = h / zoom -- limited to zoom_max, by
//     a x1.25 step per update, and never further IN while the target crosses the view fast (a narrow view would lose it)
//   * MOVE CLOSER (takes time): towards the distance at which h1 itself would be min_px, never inside min_dist; as the drone
//     closes in, h1 grows and the zoom comes back out -> the wide view returns once it is no longer needed
//   * too big unzoomed (h1 > max_px): back off; zoom never goes below 1
struct ViewCfg { double min_px = 48, max_px = 120, min_dist = 8, zoom_max = 3, zoom_step = 1.25, fast_rad_s = 0.3;
                 bool was_approaching = false; };   // hysteresis: keep approaching up to min_px, start only below 0.8 x min_px
struct ViewDecision {
    double ref_h = 0;          // the apparent height (at the CURRENT zoom) the follow loop should hold; 0 = no change
    double goal_dist = 0;      // when approaching / backing off with a distance estimate: the distance to fly to (m)
    double zoom = 1;           // the zoom to use from now on
    bool approaching = false, backing_off = false;
    int climb = 0;             // +1: climb for a wider view (large / fast target), -1: come back down (target small at full zoom), 0: hold
    std::string why;
};
inline ViewDecision decide_view(const ViewCfg& c, double h_px, double zoom, bool dist_ok, double dist_m, double target_rate_rad_s) {
    ViewDecision d; d.zoom = zoom; d.ref_h = h_px;
    const double h1 = h_px / std::max(1.0, zoom);                         // size without zoom
    const bool fast = std::fabs(target_rate_rad_s) > c.fast_rad_s;
    // zoom: towards min_px at the current distance (gradual, bounded)
    double want_zoom = std::clamp(c.min_px / std::max(1.0, h1), 1.0, c.zoom_max);
    // a fast target: a WIDER view, one step out at a time (it stays in the frame however it turns)
    if (fast) want_zoom = std::max(1.0, zoom / c.zoom_step);
    d.zoom = std::clamp(want_zoom, zoom / c.zoom_step, zoom * c.zoom_step);
    // height: a large target (bigger than max_px unzoomed) or a fast one already at the widest zoom -> climb for a wider view; a small
    // one at full zoom (and not fast) -> come back down towards the lock height (the caller bounds both)
    if (h1 > c.max_px || (fast && zoom <= 1.0 + 1e-6)) d.climb = +1;
    else if (!fast && h1 < 0.9 * c.min_px && zoom >= c.zoom_max - 1e-6) d.climb = -1;
    // distance: closer while the unzoomed target is too small, back off if too big
    char b[200];
    // hysteresis (measured live: the box size swinging 44-58 px around the old single 0.9 x min_px threshold flipped approach / hold
    // every ~0.5 s -- the forward command went 3.9 -> 0 -> 3.9 m/s and the drone pitched +-38 deg, throwing the target out of view)
    const double start_below = c.was_approaching ? c.min_px : 0.8 * c.min_px;
    if (h1 < start_below && dist_ok && dist_m > 1.05 * c.min_dist) {
        const double d_star = std::max(c.min_dist, dist_m * h1 / c.min_px);
        d.approaching = true; d.ref_h = h_px * dist_m / d_star; d.goal_dist = d_star;
        std::snprintf(b, sizeof b, "zoom %.1fx + move closer %.0f -> %.0f m (target %.0f px unzoomed)", d.zoom, dist_m, d_star, h1);
    } else if (h1 > c.max_px) {
        d.backing_off = true; d.ref_h = zoom * 0.9 * c.max_px;
        if (dist_ok) d.goal_dist = dist_m * h1 / (0.9 * c.max_px);       // apparent size ~ 1/distance
        std::snprintf(b, sizeof b, "back off (target %.0f px unzoomed)", h1);
    } else if (std::fabs(d.zoom - zoom) > 1e-6) {
        std::snprintf(b, sizeof b, "zoom %.1fx -> %.1fx%s", zoom, d.zoom, fast ? " (fast target: not zooming in)" : "");
    } else {
        std::snprintf(b, sizeof b, "%s", h1 < 0.9 * c.min_px ? (fast ? "hold (fast target: wide view kept)" : "hold (closest allowed, max zoom)") : "good view");
    }
    d.why = b;
    return d;
}
// A box in the image before a zoom change by factor r (new/old), as it appears after it (zoom is about the image centre).
inline Box zoom_box(const Box& b, double r, double cx, double cy) { return Box{(b.x - cx) * r + cx, (b.y - cy) * r + cy, b.w * r, b.h * r}; }
// Digital zoom: the central 1/z of the image, scaled back to full size. On a camera streamed above the tracking resolution (the
// C12's 1280x720 processed at half size) this adds real detail; on a native-resolution source it still enlarges the target for
// the tracker's template.
inline void digital_zoom(const cv::Mat& in, double z, cv::Mat& out) {
    if (z <= 1.0 + 1e-6) { out = in; return; }
    const int w = (int)std::lround(in.cols / z), h = (int)std::lround(in.rows / z);
    cv::resize(in(cv::Rect((in.cols - w) / 2, (in.rows - h) / 2, w, h)), out, in.size(), 0, 0, cv::INTER_LINEAR);
}

// Global (camera) motion between two consecutive frames: a similarity transform from tracked corner features with RANSAC (the
// ground/background dominates, a small target is an outlier). Computed on a <=320 px copy for speed (~3-6 ms) and returned in
// full-frame pixels. Empty if it cannot be estimated (too few features: then nothing is compensated).
class CameraMotion {
public:
    // exclude: the tracked object's box in the PREVIOUS frame (full-frame pixels) -- its features are left out. A large object (a car
    // filling a third of the view) otherwise drags the "camera motion" with its own motion, the prediction goes wrong with it and
    // good boxes get rejected.
    cv::Mat step(const cv::Mat& frame, const Box* exclude = nullptr) {
        const double sc = std::min(1.0, 320.0 / frame.cols);
        cv::Mat small, g; cv::resize(frame, small, cv::Size(), sc, sc, cv::INTER_AREA); cv::cvtColor(small, g, cv::COLOR_BGR2GRAY);
        cv::Mat A;
        if (!prev_.empty() && prev_.size() == g.size()) {
            std::vector<cv::Point2f> p0, p1; std::vector<uchar> st; std::vector<float> err;
            cv::Mat mask;
            if (exclude && exclude->valid()) {
                mask = cv::Mat(prev_.size(), CV_8UC1, cv::Scalar(255));
                const cv::Rect r = cv::Rect((int)std::floor((exclude->cx() - 0.65 * exclude->w) * sc), (int)std::floor((exclude->cy() - 0.65 * exclude->h) * sc),
                                            (int)std::ceil(1.3 * exclude->w * sc), (int)std::ceil(1.3 * exclude->h * sc)) & cv::Rect(0, 0, prev_.cols, prev_.rows);
                if (r.area() < 0.7 * prev_.total()) mask(r).setTo(0); else mask.release();   // never mask (nearly) everything
            }
            cv::goodFeaturesToTrack(prev_, p0, 150, 0.01, 8, mask);
            if (p0.size() >= 12) {
                cv::calcOpticalFlowPyrLK(prev_, g, p0, p1, st, err);
                std::vector<cv::Point2f> a, b;
                for (size_t k = 0; k < p0.size(); k++) if (st[k]) { a.push_back(p0[k] / sc); b.push_back(p1[k] / sc); }
                if (a.size() >= 12) A = cv::estimateAffinePartial2D(a, b, cv::noArray(), cv::RANSAC, 2.0 / sc);
            }
        }
        prev_ = g;
        return A;
    }
    void reset() { prev_.release(); }
private:
    cv::Mat prev_;
};
inline Box warp_box(const Box& bx, const cv::Mat& A) {
    if (A.empty()) return bx;
    const double a = A.at<double>(0, 0), b = A.at<double>(0, 1), tx = A.at<double>(0, 2), c = A.at<double>(1, 0), d = A.at<double>(1, 1), ty = A.at<double>(1, 2);
    const double cx = bx.cx(), cy = bx.cy(), sc = std::sqrt(std::fabs(a * d - b * c));
    const double ncx = a * cx + b * cy + tx, ncy = c * cx + d * cy + ty;
    return Box{ncx - bx.w * sc / 2, ncy - bx.h * sc / 2, bx.w * sc, bx.h * sc};
}

// Operator messages over the telemetry radio (STATUSTEXT: 50 chars, low bandwidth). fit_statustext keeps the text within the
// field and marks a cut with "~" instead of silently chopping it; MsgThrottle drops an identical message repeated within
// `repeat_s` and caps the overall rate, so a flickering state (LOCKED<->COASTING) cannot flood a long-range link.
inline std::string fit_statustext(const std::string& s, size_t max_len = 50) {
    if (s.size() <= max_len) return s;
    return s.substr(0, max_len - 1) + "~";
}
class MsgThrottle {
public:
    MsgThrottle(double repeat_s = 3.0, double min_gap_s = 0.25) : repeat_s_(repeat_s), min_gap_s_(min_gap_s) {}
    // urgent messages (warnings/errors) bypass the overall rate cap but are still de-duplicated
    bool allow(const std::string& text, double now, bool urgent = false) {
        auto it = last_.find(text);
        if (it != last_.end() && now - it->second < repeat_s_) return false;
        if (!urgent && now - last_any_ < min_gap_s_) return false;
        last_[text] = now; last_any_ = now;
        if (last_.size() > 64) last_.clear();                                  // bounded: old entries only matter for repeat_s
        return true;
    }
private:
    double repeat_s_, min_gap_s_, last_any_ = -1e9; std::map<std::string, double> last_;
};

// ---------------------------------------------------------------------------
// Frame gap: a dead/stalled video source is detected by ARRIVAL TIME, not by comparing pixels (a static noise-free scene and
// compressed video with skipped blocks legitimately repeat identical frames; Gazebo does). A stalled read also stops the
// heartbeats and ages the box, so the vehicle gets zero velocity either way.
// ---------------------------------------------------------------------------
class FrameGap {
public:
    // returns the seconds since the previous frame arrived (0 for the first frame)
    double observe(double now) { double g = first_ ? 0.0 : now - last_; first_ = false; last_ = now; return g; }
    static bool stalled(double gap_s, double limit_s = 0.3) { return gap_s > limit_s; }
private:
    double last_ = 0; bool first_ = true;
};

}  // namespace fl


// ============================================================
// ---- SkyDroid C12 gimbal camera: "TOP" protocol over UDP port 5000 (Skydroid "TOP Protocol EN" v1.1.6)
// ============================================================
// Frame (ASCII): "#TP" + src + dst + len(1 hex char) + ctrl(r/w) + ident(3) + data(len) + crc(2 hex chars)
//   crc = sum of all preceding bytes mod 256 (verified against 8 of the document's own example frames).
//   G-class (gimbal) commands use "#TP" with src 'U' (UART/UDP host) and dst 'G' (gimbal).
// Used here: GAA (attitude push, 1..100 Hz), GOT (target point in the 1280x720 frame), SUM (01 confirm / 00 stop tracking),
//            PTZ (centre / stop). The camera reports NO tracking status: lost-target detection is our own CSRT's job.
// Open points to confirm on the real camera: angle unit of the push (0.01 deg per the doc's example), the reference frame of
// the gimbal angles (body or earth), and what the camera does when it loses the target.
namespace top {

inline std::string crc(const std::string& body) {
    unsigned sum = 0;
    for (unsigned char c : body) sum += c;
    char b[3]; std::snprintf(b, sizeof b, "%02X", sum & 0xFF);
    return b;
}
inline std::string build(char src, char dst, char ctrl, const std::string& ident, const std::string& data = "") {
    char len = "0123456789ABCDEF"[std::min<size_t>(data.size(), 15)];
    std::string body = std::string("#TP") + src + dst + len + ctrl + ident + data;
    return body + crc(body);
}
struct Frame { char src = 0, dst = 0, ctrl = 0; std::string ident, data; };
inline bool parse(const std::string& s, Frame& f) {
    if (s.size() < 12 || (s.compare(0, 3, "#TP") != 0 && s.compare(0, 3, "#tp") != 0)) return false;
    std::string body = s.substr(0, s.size() - 2);
    std::string c = s.substr(s.size() - 2);
    for (auto& ch : c) ch = (char)std::toupper((unsigned char)ch);
    if (crc(body) != c) return false;
    int n = std::isdigit((unsigned char)s[5]) ? s[5] - '0' : std::toupper((unsigned char)s[5]) - 'A' + 10;
    if ((int)(s.size() - 12) != n) return false;
    f.src = s[3]; f.dst = s[4]; f.ctrl = s[6]; f.ident = s.substr(7, 3); f.data = s.substr(10, s.size() - 12);
    return true;
}
inline std::string hex4(int v) { char b[8]; std::snprintf(b, sizeof b, "%04X", v & 0xFFFF); return b; }
inline std::string enable_push(int hz) { char b[4]; std::snprintf(b, sizeof b, "%02X", std::max(0, std::min(100, hz))); return build('U', 'G', 'w', "GAA", b); }
inline std::string track_point(int x, int y) { return build('U', 'G', 'w', "GOT", hex4(x) + hex4(y)); }       // origin top-left of the 1280x720 frame
inline std::string track_confirm() { return build('U', 'G', 'w', "SUM", "01"); }
inline std::string track_stop() { return build('U', 'G', 'w', "SUM", "00"); }
inline std::string gimbal_center() { return build('U', 'G', 'w', "PTZ", "05"); }
inline int s16(const std::string& h) { int v = (int)std::strtol(h.c_str(), nullptr, 16); return v >= 0x8000 ? v - 0x10000 : v; }
// push frame "#TPUGCrGAC" + Y(4) P(4) R(4) (0.01 deg units) -> degrees
inline bool parse_attitude(const std::string& s, double& yaw, double& pitch, double& roll) {
    Frame f;
    if (!parse(s, f) || f.ident != "GAC" || f.data.size() != 12) return false;
    yaw = s16(f.data.substr(0, 4)) / 100.0; pitch = s16(f.data.substr(4, 4)) / 100.0; roll = s16(f.data.substr(8, 4)) / 100.0;
    return true;
}
}  // namespace top

class C12Link {
public:
    using Clock = std::chrono::steady_clock;
    C12Link(const std::string& host, int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("C12: socket() failed");
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port);
        if (::inet_pton(AF_INET, host.c_str(), &a.sin_addr) != 1) throw std::runtime_error("C12: bad IPv4 address '" + host + "'");
        if (::connect(fd_, (sockaddr*)&a, sizeof a) < 0) throw std::runtime_error(std::string("C12: connect failed: ") + std::strerror(errno));
        running_ = true;
        thread_ = std::thread([this] { run(); });
        std::printf("[C12] udp://%s:%d\n", host.c_str(), port);
    }
    ~C12Link() { stop(); }
    C12Link(const C12Link&) = delete;
    C12Link& operator=(const C12Link&) = delete;

    void stop() { if (running_.exchange(false)) { if (thread_.joinable()) thread_.join(); ::close(fd_); fd_ = -1; } }
    void send(const std::string& frame) { std::lock_guard<std::mutex> lk(tx_); if (fd_ >= 0) { ssize_t r = ::send(fd_, frame.data(), frame.size(), 0); (void)r; } }
    void start_tracking(int x, int y) { send(top::track_point(x, y)); send(top::track_confirm()); tracking_ = true; }
    void stop_tracking() { send(top::track_stop()); tracking_ = false; }
    bool tracking_requested() const { return tracking_; }
    void want_push(int hz) { push_hz_ = hz; send(top::enable_push(hz)); }
    // latest gimbal attitude; returns false if none received yet. age_s = seconds since the last push
    bool gimbal(double& yaw_deg, double& pitch_deg, double& age_s) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!have_) return false;
        yaw_deg = yaw_; pitch_deg = pitch_; age_s = std::chrono::duration<double>(Clock::now() - at_).count();
        return true;
    }
private:
    void run() {
        char buf[256]; auto last_enable = Clock::now();
        while (running_) {
            pollfd p{fd_, POLLIN, 0};
            int r = ::poll(&p, 1, 100);
            if (r > 0 && (p.revents & POLLIN)) {
                ssize_t n = ::recv(fd_, buf, sizeof buf - 1, 0);
                if (n > 0) {
                    std::string s(buf, (size_t)n); double y, pi, ro;
                    if (top::parse_attitude(s, y, pi, ro)) { std::lock_guard<std::mutex> lk(mtx_); yaw_ = y; pitch_ = pi; at_ = Clock::now(); have_ = true; }
                }
            }
            auto now = Clock::now();                                      // the push enable can be lost (UDP): repeat it until data flows
            bool flowing; { std::lock_guard<std::mutex> lk(mtx_); flowing = have_ && std::chrono::duration<double>(now - at_).count() < 1.0; }
            if (push_hz_ > 0 && !flowing && std::chrono::duration<double>(now - last_enable).count() > 2.0) { send(top::enable_push(push_hz_)); last_enable = now; }
        }
    }
    int fd_ = -1; std::atomic<bool> running_{false}; std::thread thread_;
    std::mutex tx_; mutable std::mutex mtx_;
    double yaw_ = 0, pitch_ = 0; Clock::time_point at_{}; bool have_ = false;
    std::atomic<bool> tracking_{false}; std::atomic<int> push_hz_{0};
};


// ============================================================
// ---- optional ReID sidecar (tracker/detector_sidecar.py): DINOv2 fingerprints over localhost UDP -- the fallback when this
// build has no OpenVINO (the in-process model in native_ai.hpp is preferred). Class-free: the project tracks any clicked object.
//   tracker -> sidecar (port P):    "TTE1 <id>\n" + JPEG crop
//   sidecar -> tracker (port P+1):  "TTV1 <id> <n> v1 ... vn"
// ============================================================
#include "native_ai.hpp"                                       // the in-process ReID model (and the AiBackend interface)

class DetectorLink : public AiBackend {
public:
    using Clock = std::chrono::steady_clock;
    bool has_reid() const override { return true; }
    std::string describe() const override { return "Python ReID sidecar over UDP"; }
    explicit DetectorLink(int port) {
        fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (fd_ < 0) throw std::runtime_error("reid sidecar: socket() failed");
        sockaddr_in me{}; me.sin_family = AF_INET; me.sin_port = htons((uint16_t)(port + 1)); ::inet_pton(AF_INET, "127.0.0.1", &me.sin_addr);
        if (::bind(fd_, (sockaddr*)&me, sizeof me) < 0) throw std::runtime_error("reid sidecar: bind 127.0.0.1:" + std::to_string(port + 1) + " failed: " + std::strerror(errno));
        to_.sin_family = AF_INET; to_.sin_port = htons((uint16_t)port); ::inet_pton(AF_INET, "127.0.0.1", &to_.sin_addr);
        int buf = 1 << 20; ::setsockopt(fd_, SOL_SOCKET, SO_SNDBUF, &buf, sizeof buf);
        running_ = true; thread_ = std::thread([this] { run(); });
        std::printf("[REID] sidecar link: crops -> 127.0.0.1:%d, fingerprints <- :%d\n", port, port + 1);
    }
    ~DetectorLink() { running_ = false; if (thread_.joinable()) thread_.join(); if (fd_ >= 0) ::close(fd_); }
    DetectorLink(const DetectorLink&) = delete;
    DetectorLink& operator=(const DetectorLink&) = delete;

    // ReID fingerprint request of the object in box b: a square crop 1.4x its long side; returns the id (0 = not sent)
    uint32_t request_embed(const cv::Mat& frame, const fl::Box& b) override {
        const double side = std::max(b.w, b.h) * 1.4;
        cv::Rect r((int)std::lround(b.cx() - side / 2), (int)std::lround(b.cy() - side / 2), (int)std::lround(side), (int)std::lround(side));
        r &= cv::Rect(0, 0, frame.cols, frame.rows);
        if (r.width < 6 || r.height < 6) return 0;
        cv::Mat c; cv::resize(frame(r), c, cv::Size(160, 160), 0, 0, cv::INTER_AREA);
        std::vector<uchar> jpg; cv::imencode(".jpg", c, jpg, {cv::IMWRITE_JPEG_QUALITY, 90});
        const uint32_t id = ++next_id_;
        char hdr[32]; int n = std::snprintf(hdr, sizeof hdr, "TTE1 %u\n", id);
        std::vector<uchar> pkt(hdr, hdr + n); pkt.insert(pkt.end(), jpg.begin(), jpg.end());
        { std::lock_guard<std::mutex> lk(mtx_); embs_[id] = {}; emb_done_[id] = false;
          while (emb_done_.size() > 32) { embs_.erase(emb_done_.begin()->first); emb_done_.erase(emb_done_.begin()); } }
        ::sendto(fd_, pkt.data(), pkt.size(), 0, (sockaddr*)&to_, sizeof to_);
        return id;
    }
    // true once the reply is in (out empty = the sidecar could not embed it); false = still waiting / unknown id
    bool embed_poll(uint32_t id, std::vector<float>& out) override {
        std::lock_guard<std::mutex> lk(mtx_);
        auto it = emb_done_.find(id); if (it == emb_done_.end() || !it->second) return false;
        out = embs_[id]; embs_.erase(id); emb_done_.erase(it); return true;
    }
    // several boxes at once: all requests out first, then one shared wait
    bool embed_many(const cv::Mat& frame, const std::vector<fl::Box>& bs, std::vector<std::vector<float>>& out, double timeout_s) override {
        std::vector<uint32_t> ids; for (const auto& b : bs) { const uint32_t id = request_embed(frame, b); if (!id) return false; ids.push_back(id); }
        out.assign(bs.size(), {}); std::vector<bool> got(bs.size(), false); size_t n = 0;
        const auto t0 = Clock::now();
        while (n < ids.size() && std::chrono::duration<double>(Clock::now() - t0).count() < timeout_s) {
            for (size_t k = 0; k < ids.size(); k++) if (!got[k] && embed_poll(ids[k], out[k])) { got[k] = true; n++; }
            if (n < ids.size()) std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if (n < ids.size()) { std::lock_guard<std::mutex> lk(mtx_); for (auto id : ids) { embs_.erase(id); emb_done_.erase(id); } return false; }
        for (const auto& v : out) if (v.empty()) return false;
        return true;
    }
private:
    std::map<uint32_t, std::vector<float>> embs_; std::map<uint32_t, bool> emb_done_;
    void run() {
        std::vector<char> buf(65536);
        while (running_) {
            pollfd p{fd_, POLLIN, 0};
            if (::poll(&p, 1, 100) <= 0 || !(p.revents & POLLIN)) continue;
            ssize_t n = ::recv(fd_, buf.data(), buf.size() - 1, 0);
            if (n <= 4 || std::strncmp(buf.data(), "TTV1", 4) != 0) continue;
            buf[(size_t)n] = 0;
            std::istringstream in(std::string(buf.data(), (size_t)n));
            std::string magic; uint32_t id = 0; int cnt = 0;
            if (in >> magic >> id >> cnt && cnt >= 0 && cnt <= 4096) {
                std::vector<float> v((size_t)cnt); for (int k = 0; k < cnt; k++) in >> v[(size_t)k];
                std::lock_guard<std::mutex> lk(mtx_);
                if (emb_done_.count(id)) { embs_[id] = in ? v : std::vector<float>(); emb_done_[id] = true; }
            }
        }
    }
    int fd_ = -1; sockaddr_in to_{}; std::atomic<bool> running_{false}; std::thread thread_;
    mutable std::mutex mtx_; uint32_t next_id_ = 0;
};


// ============================================================
// ---- --selftest: unit checks for the follow logic (no camera, vehicle, display or network needed)
// ============================================================
namespace selftest {
int checks = 0, failed = 0;
#define CHECK(cond) do { ++selftest::checks; if (!(cond)) { ++selftest::failed; std::printf("  FAIL line %d: %s\n", __LINE__, #cond); } } while (0)
#define NEAR(a, b, tol) CHECK(std::fabs((a) - (b)) <= (tol))

void test_basics() {
    using namespace fl;
    NEAR(clampd(5, 0, 3), 3, 1e-12); NEAR(clampd(-5, 0, 3), 0, 1e-12); NEAR(sym_clamp(-9, 2), -2, 1e-12);
    NEAR(deadband(0.01, 0.02), 0, 1e-12); NEAR(deadband(0.05, 0.02), 0.03, 1e-12); NEAR(deadband(-0.05, 0.02), -0.03, 1e-12);
    Ceilings k; Cmd c; c.vx = 9; c.vy = -9; c.vz = 9; c.yaw_rate = -9;
    Cmd o = clamp_to_ceilings(c, k);
    NEAR(o.vx, 3.0, 1e-12); NEAR(o.vy, -3.0, 1e-12); NEAR(o.vz, 2.0, 1e-12); NEAR(o.yaw_rate, -0.5, 1e-12);
    // clamp_keep_direction: vx=6,vz=3 (2:1 ratio) saturates vx first (ceiling 3.0), so BOTH scale by 0.5 -> vz=1.5, keeping the ratio;
    // clamp_to_ceilings would have clamped vx to 3 and left vz at 2 (ratio 1.5:1 -- a bent path), which this must NOT reproduce.
    Cmd d; d.vx = 6.0; d.vz = 3.0;
    Cmd kd = clamp_keep_direction(d, k);
    NEAR(kd.vx, 3.0, 1e-12); NEAR(kd.vz, 1.5, 1e-12); NEAR(kd.vx / kd.vz, d.vx / d.vz, 1e-9);
    Cmd d2; d2.vx = 1.0; d2.vz = 0.5; Cmd kd2 = clamp_keep_direction(d2, k);                 // within ceilings: unchanged
    NEAR(kd2.vx, 1.0, 1e-12); NEAR(kd2.vz, 0.5, 1e-12);
    CHECK(Cmd().is_zero()); CHECK(Cmd().yaw_rate == 0.0);
    NEAR(Intrinsics::from_hfov(640, 480, 2.0).fx, 205.5, 0.1);
}

void test_slew() {
    using namespace fl;
    Ceilings k; Slew s; Cmd tgt; tgt.vx = 3.0; tgt.yaw_rate = 0.5;
    Cmd a = s.step(tgt, 0.05, k);
    NEAR(a.vx, 0.1, 1e-9);                       // 2 m/s^2 * 0.05 s
    NEAR(a.yaw_rate, 0.1, 1e-9);                 // 2 rad/s^2 * 0.05 s
    for (int i = 0; i < 100; i++) a = s.step(tgt, 0.05, k);
    NEAR(a.vx, 3.0, 1e-9);                       // reaches the target, never overshoots
    CHECK(s.step(tgt, 0.0, k).vx == 3.0);        // dt <= 0: unchanged
    Cmd z; a = s.step(z, 0.05, k); NEAR(a.vx, 2.9, 1e-9);
    s.reset(); CHECK(s.last().is_zero());
}

void test_gate_and_guards() {
    using namespace fl;
    Cmd d; d.vx = 1; d.vy = 1; d.vz = 1; d.yaw_rate = 0.3;
    CHECK(gate_command(d, true, true, true, 0.1, 0.3).vx == 1.0);
    CHECK(gate_command(d, false, true, true, 0.1, 0.3).is_zero());       // not LOCKED
    CHECK(gate_command(d, true, false, true, 0.1, 0.3).is_zero());       // not GUIDED
    CHECK(gate_command(d, true, true, false, 0.1, 0.3).is_zero());       // link down
    CHECK(gate_command(d, true, true, true, 0.31, 0.3).is_zero());       // stale
    CHECK(gate_command(d, false, true, true, 0.1, 0.3).yaw_rate == 0.0); // zero carries yaw_rate = 0 (ArduPilot doc)
    Cmd c; c.vz = 1.0;
    CHECK(altitude_guard(c, true, 1.0, 2.0, 0).vz == 0.0);               // at/below the floor: no descent
    CHECK(altitude_guard(c, true, 5.0, 2.0, 0).vz == 1.0);
    CHECK(altitude_guard(c, false, 0, 2.0, 0).vz == 0.0);                // altitude unknown: block descent
    Cmd up; up.vz = -1.0;
    CHECK(altitude_guard(up, true, 50, 2.0, 40).vz == 0.0);              // at/above the ceiling: no climb
    CHECK(altitude_guard(up, true, 50, 2.0, 0).vz == -1.0);              // ceiling disabled
}

void test_kalman() {
    using namespace fl;
    BoxKalman kf(Box{100, 100, 50, 80});
    Box b0 = kf.box(); NEAR(b0.x, 100, 1e-9); NEAR(b0.h, 80, 1e-9);
    kf.predict(); NEAR(kf.p(0, 0), 25 + 100 + 1 + 0.0, 1.0);             // position variance grows by velocity variance + Q
    NEAR(kf.p(4, 4), 100.25, 1e-9);
    double rms_start = kf.p(0, 0);
    for (int i = 0; i < 40; i++) { kf.predict(); kf.update(Box{100 + 5.0 * (i + 1), 100, 50, 80}); }   // 5 px/frame
    NEAR(kf.vx(), 5.0, 0.3);                                             // converges on the true velocity
    CHECK(kf.p(0, 0) < rms_start);                                       // measurements reduce the uncertainty
    { BoxKalman a(Box{100, 100, 50, 80}), b(Box{100, 100, 50, 80});                     // a 2-frame gap predicts twice as far
      for (int i = 0; i < 40; i++) { a.predict(); a.update(Box{100 + 5.0 * (i + 1), 100, 50, 80}); b.predict(); b.update(Box{100 + 5.0 * (i + 1), 100, 50, 80}); }
      double x0 = a.box().cx(); a.predict(1.0); b.predict(2.0); NEAR(a.box().cx() - x0, 5.0, 0.4); NEAR(b.box().cx() - x0, 10.0, 0.8); }
    kf.damp_velocity(0.0); NEAR(kf.vx(), 0.0, 1e-12);
    BoxKalman small(Box{0, 0, 2, 2});
    for (int i = 0; i < 5; i++) small.predict();
    CHECK(small.box().w >= 4.0 && small.box().h >= 4.0);                 // w, h floor
    NEAR(iou(Box{0, 0, 10, 10}, Box{0, 0, 10, 10}), 1.0, 1e-12); NEAR(iou(Box{0, 0, 10, 10}, Box{20, 20, 5, 5}), 0.0, 1e-12);
}

void test_control() {
    using namespace fl;
    Intrinsics k; Gains g; Ceilings c;
    Box centred{k.cx - 20, k.cy - 30, 40, 60};
    Cmd o = image_control(centred, 60, k, g, c); CHECK(o.is_zero());      // centred, same size: nothing to do
    Box right{k.cx + 100 - 20, k.cy - 30, 40, 60};
    o = image_control(right, 60, k, g, c);
    NEAR(o.yaw_rate, g.kyaw * (std::atan(100 / k.fx) - g.db_yaw), 1e-9); CHECK(o.yaw_rate > 0);   // target right -> turn right
    Box below{k.cx - 20, k.cy + 80 - 30, 40, 60};
    o = image_control(below, 60, k, g, c); CHECK(o.vz > 0);               // target below -> descend (z down positive)
    Box farther{k.cx - 15, k.cy - 22, 30, 45};                            // smaller than at lock -> go forward
    o = image_control(farther, 60, k, g, c); CHECK(o.vx > 0); CHECK(o.vy == 0.0);
    Box nearer{k.cx - 30, k.cy - 45, 60, 90};
    o = image_control(nearer, 60, k, g, c); CHECK(o.vx < 0);
    Box huge{0, 0, 640, 100};
    o = image_control(huge, 60, k, g, c); CHECK(o.vx >= -c.vx);           // ceilings hold
    CHECK(image_control(Box(), 60, k, g, c).is_zero()); CHECK(image_control(centred, 0, k, g, c).is_zero());
    // best_view_vz: climbs (negative) when too large/close, descends (positive) when too small/far, zero in the healthy band
    CHECK(best_view_vz(0.5, 0.12, 0.35, 2.0) < 0.0);
    CHECK(best_view_vz(0.05, 0.12, 0.35, 2.0) > 0.0);
    CHECK(best_view_vz(0.20, 0.12, 0.35, 2.0) == 0.0);
    NEAR(best_view_vz(0.45, 0.12, 0.35, 2.0), -2.0 * (0.45 - 0.35), 1e-12);
    NEAR(best_view_vz(0.05, 0.12, 0.35, 2.0), 2.0 * (0.12 - 0.05), 1e-12);
    // gimbal law
    NEAR(ground_distance(10, -45), 10.0, 1e-9); NEAR(ground_distance(10, -30), 10 / std::tan(deg2rad(30)), 1e-9);
    CHECK(!std::isfinite(ground_distance(10, 0)));
    o = gimbal_control(20, -30, 10, 12.0, g, c); CHECK(o.yaw_rate > 0);   // gimbal right -> turn right
    NEAR(o.vx, g.kx * ((10 / std::tan(deg2rad(30)) - 12.0) / 12.0 - g.db_x), 1e-9);   // farther than at lock -> forward
    o = gimbal_control(0, -30, 10, 17.0, g, c); CHECK(o.vx == 0.0);       // within the dead-band: hold
    o = gimbal_control(0, -60, 10, 17.0, g, c); CHECK(o.vx < 0);          // closer than at lock -> back off
    o = gimbal_control(0, 0, 10, 17.0, g, c); CHECK(o.vx == 0.0 && o.vz == 0.0);   // looking at the horizon: no range
    o = gimbal_control(0, -6.9, 0.7, 2.0, g, c); CHECK(o.vx == 0.0);                       // shallow (6.9 deg < 10): distance not trusted
    o = gimbal_control(0, -30, 10, 12.0, g, c); CHECK(o.vx > 0);                           // steep enough: used
}

void test_fsm() {
    using namespace fl;
    FollowFsm f; FsmIn in; in.now = 0;
    CHECK(f.state() == FState::IDLE);
    in.lock_request = true; CHECK(f.update(in)); CHECK(f.state() == FState::LOCKED); in.lock_request = false;
    in.now = 1; in.box_age = 0.1; f.update(in); CHECK(f.state() == FState::LOCKED); CHECK(f.steering_allowed());
    NEAR(f.locked_for(3.0), 3.0, 1e-9);
    in.now = 2; in.box_age = 0.31; f.update(in); CHECK(f.state() == FState::COASTING); CHECK(!f.steering_allowed());
    in.box_age = 0.05; f.update(in); CHECK(f.state() == FState::LOCKED);          // recovered in time
    in.box_age = 0.5; f.update(in); CHECK(f.state() == FState::COASTING);
    in.box_age = 1.01; f.update(in); CHECK(f.state() == FState::LOST);            // escalates on the full age
    FollowFsm blackout; FsmIn bi; bi.lock_request = true; blackout.update(bi); bi.lock_request = false; bi.now = 1; bi.box_age = 4.0;
    blackout.update(bi); CHECK(blackout.state() == FState::LOST);                 // a long video stall goes straight to LOST
    in.box_age = 0.05; f.update(in); CHECK(f.state() == FState::LOST);            // latched: a late box does not re-engage
    in.box_age = 5.01; f.update(in); CHECK(f.state() == FState::IDLE);            // unlock after the timeout
    { FollowFsm l; FsmIn li; li.lock_request = true; l.update(li); li.lock_request = false; li.now = 1.0; li.box_age = 1.5; l.update(li); l.update(li);
      CHECK(l.state() == FState::LOST); li.box_age = 0.0; li.now = 3.0; l.update(li); CHECK(l.state() == FState::LOST);    // fresh boxes do not revive it
      li.now = 5.1; l.update(li); CHECK(l.state() == FState::IDLE); }                                                      // but the clock drops the lock
    FollowFsm g; FsmIn gi;
    gi.link_ok = false; gi.lock_request = true; CHECK(!g.update(gi)); CHECK(g.state() == FState::IDLE);   // lock refused: no link
    gi.link_ok = true; gi.guided = false; CHECK(!g.update(gi)); CHECK(g.state() == FState::IDLE);        // refused: not GUIDED
    gi.guided = true; CHECK(g.update(gi)); CHECK(g.state() == FState::LOCKED); gi.lock_request = false;
    gi.link_ok = false; g.update(gi); CHECK(g.state() == FState::LINK_LOST);
    gi.link_ok = true; gi.now = 1.0; g.update(gi); CHECK(g.state() == FState::LOCKED);             // a short gap, still GUIDED: the lock carries on
    gi.link_ok = false; gi.now = 2.0; g.update(gi); CHECK(g.state() == FState::LINK_LOST);
    gi.link_ok = true; gi.now = 8.0; g.update(gi); CHECK(g.state() == FState::IDLE);              // a long gap: no automatic relock
    { FollowFsm q; FsmIn qi; qi.lock_request = true; q.update(qi); qi.lock_request = false;
      qi.link_ok = false; qi.now = 0.5; q.update(qi); qi.link_ok = true; qi.guided = false; qi.now = 1.0; q.update(qi);
      CHECK(q.state() == FState::IDLE); }                                                         // back, but the pilot left GUIDED: dropped
    FollowFsm h; FsmIn hi; hi.lock_request = true; h.update(hi); hi.lock_request = false;
    hi.guided = false; h.update(hi); CHECK(h.state() == FState::IDLE);                            // leaving GUIDED drops the lock
    hi.guided = true; h.update(hi); CHECK(h.state() == FState::IDLE);                             // and nothing resumes by itself
    { FollowFsm r; FsmIn ri; ri.lock_request = true; r.update(ri); ri.lock_request = false;          // relock while LOCKED restarts the clock
      ri.now = 10; ri.box_age = 0.05; r.update(ri); NEAR(r.locked_for(10), 10.0, 1e-9);
      ri.lock_request = true; CHECK(r.update(ri)); NEAR(r.locked_for(10), 0.0, 1e-9); CHECK(r.state() == FState::LOCKED);
      ri.lock_request = false; ri.box_age = 2.0; r.update(ri); CHECK(r.state() == FState::LOST);       // relock from LOST also restarts it
      ri.now = 11; ri.lock_request = true; CHECK(r.update(ri)); NEAR(r.locked_for(11.5), 0.5, 1e-9); }
    { FollowFsm t; FsmIn ti; ti.require_guided = false; ti.guided = false; ti.lock_request = true;      // track-only: lock without GUIDED
      CHECK(t.update(ti)); CHECK(t.state() == FState::LOCKED); ti.lock_request = false; ti.now = 1; ti.box_age = 0.05;
      t.update(ti); CHECK(t.state() == FState::LOCKED);                                                  // not dropped for being outside GUIDED
      ti.guided = true; t.update(ti); CHECK(t.state() == FState::LOCKED);                                // pilot engages GUIDED: still locked
      ti.guided = false; t.update(ti); CHECK(t.state() == FState::LOCKED); }                             // pilot leaves GUIDED: lock kept, steering stops in the link
    FollowFsm u; FsmIn ui; ui.lock_request = true; u.update(ui); ui.lock_request = false; ui.unlock_request = true;
    u.update(ui); CHECK(u.state() == FState::IDLE);
    FollowFsm n; FsmIn ni; ni.link_ok = false; n.update(ni); CHECK(n.state() == FState::IDLE);   // LINK_LOST only exists with a lock
}

void test_top() {
    CHECK(top::track_point(640, 360) == "#TPUG8wGOT0280016895");            // document example (checksum included)
    CHECK(top::track_confirm() == "#TPUG2wSUM0162");
    CHECK(top::track_stop() == "#TPUG2wSUM0061");
    CHECK(top::enable_push(1) == "#TPUG2wGAA0136");
    CHECK(top::build('U', 'D', 'w', "REC", "0A") == "#TPUD2wREC0A54");
    CHECK(top::crc("#TPUG2wPTZ00") == "6A");
    top::Frame f; CHECK(top::parse("#TPUG2wSUM0162", f)); CHECK(f.ident == "SUM" && f.data == "01" && f.ctrl == 'w' && f.dst == 'G');
    CHECK(!top::parse("#TPUG2wSUM0163", f));                                   // bad checksum
    CHECK(!top::parse("#TPUG3wSUM0162", f));                                   // length mismatch
    CHECK(!top::parse("hello world!", f));
    double y, p, r;                                                            // attitude push: yaw -50.00, pitch -30.00, roll 0
    std::string push = top::build('U', 'G', 'r', "GAC", "EC78" "F448" "0000");
    CHECK(top::parse_attitude(push, y, p, r)); NEAR(y, -50.0, 1e-9); NEAR(p, -30.0, 1e-9); NEAR(r, 0.0, 1e-9);
    CHECK(!top::parse_attitude(top::build('U', 'G', 'r', "GAC", "EC78"), y, p, r));
    CHECK(top::s16("FFFF") == -1); CHECK(top::s16("0280") == 640);
}

void test_plausible() {
    using namespace fl;
    Box pred{300, 235, 35, 60};
    CHECK(box_plausible(pred, Box{305, 238, 36, 61}, 640, 480));                 // normal frame-to-frame change
    CHECK(!box_plausible(pred, Box{-2, 17, 34, 58}, 640, 480));                  // the real failure: jump to the corner
    CHECK(!box_plausible(pred, Box{700, 235, 35, 60}, 640, 480));                // centre outside the frame
    CHECK(!box_plausible(pred, Box{300, 235, 35, 140}, 640, 480));               // height more than doubled
    CHECK(!box_plausible(pred, Box{300, 235, 35, 30}, 640, 480));                // height halved
    CHECK(box_plausible(pred, Box{330, 235, 35, 60}, 640, 480));                 // a 30 px move is fine (limit = max(60, 3 x diag))
    CHECK(!box_plausible(pred, Box(), 640, 480));
    CHECK(!box_plausible(pred, Box{520, 235, 35, 60}, 640, 480));                                  // 220 px in one frame: rejected
    CHECK(box_plausible(pred, Box{520, 235, 35, 60}, 640, 480, PlausCfg(), 2.0));                  // ...but fine after a 2-frame gap
}

void test_color_plausible() {
    using namespace fl;
    cv::Scalar red(20, 20, 200), similar_red(35, 30, 190), blue(200, 30, 20);
    CHECK(color_plausible(red, similar_red));                 // small lighting/compression variation: accepted
    CHECK(!color_plausible(red, blue));                        // a differently-coloured crossing object: rejected
    CHECK(color_plausible(cv::Scalar(-1, -1, -1), blue));      // no reliable sample (box clipped at the edge): don't reject on this alone
    CHECK(color_plausible(red, cv::Scalar(-1, -1, -1)));
    cv::Scalar r2 = red; for (int i = 0; i < 300; i++) r2 = adapt_color(r2, similar_red);     // gradual light change: follows it
    NEAR(r2[0], similar_red[0], 1.0);
    cv::Scalar r3 = red; for (int i = 0; i < 300; i++) r3 = adapt_color(r3, blue);             // a different object: never adopted
    CHECK(r3 == red);
}

// [GO-11] TESTS: one-press GO, the readiness gates and the --go-confirm two-step of [GO-2].
void test_rescue() {
    using namespace fl;
    Ned veh{0, 0, -10}, t;                                                          // 10 m up, facing north
    CHECK(target_ned_from_view(veh, 0.0, 10.0, 0.0, deg2rad(45), deg2rad(10), t)); NEAR(t.n, 10.0, 1e-9); NEAR(t.e, 0.0, 1e-9); NEAR(t.d, 0.0, 1e-9);
    CHECK(target_ned_from_view(veh, deg2rad(90), 10.0, 0.0, deg2rad(45), deg2rad(10), t)); NEAR(t.n, 0.0, 1e-9); NEAR(t.e, 10.0, 1e-9);          // facing east
    CHECK(target_ned_from_view(veh, 0.0, 10.0, deg2rad(90), deg2rad(45), deg2rad(10), t)); NEAR(t.e, 10.0, 1e-9);                                // target to the right
    CHECK(!target_ned_from_view(veh, 0.0, 10.0, 0.0, deg2rad(5), deg2rad(10), t));                                                               // shallow: refused
    CHECK(!target_ned_from_view(veh, 0.0, 0.2, 0.0, deg2rad(45), deg2rad(10), t));                                                               // too low
    NEAR(bearing_to(Ned{0, 0, 0}, Ned{0, 10, 0}), kPi / 2, 1e-9);

    // ONE-PRESS GO (default): a single press with every gate passing sends the go-to at once
    {
        RescueCfg oc; oc.enabled = true; RescueController c(oc);
        RescueIn x; x.now = 1.0; x.follow = FState::LOCKED; x.locked_for = 5; x.dest_ok = true; x.dest = Ned{24, 0, -8}; x.veh = Ned{0, 0, -8}; x.press = true;
        RescueOut q = c.step(x); CHECK(q.send_goto); NEAR(q.dest.n, 24.0, 1e-9); CHECK(c.state() == RState::SENT);
        x.press = false; x.now = 1.1; c.step(x); CHECK(c.state() == RState::IDLE);                       // one tick, then ready again
        x.press = true; x.now = 5.0; CHECK(c.step(x).send_goto);                                         // another press: GO again
        x.now = 6.0; x.locked_for = 0.5; q = c.step(x); CHECK(!q.send_goto); CHECK(q.end == REnd::NOT_READY);   // the gates still apply
        x.locked_for = 5; x.dest = Ned{400, 0, -8}; q = c.step(x); CHECK(!q.send_goto); CHECK(q.end == REnd::OUT_OF_RANGE);
    }
    // --go-confirm: the two-step (everything below)
    RescueCfg cfg; cfg.enabled = true; cfg.confirm = true;
    auto ready = [&](double now) { RescueIn i; i.now = now; i.follow = FState::LOCKED; i.locked_for = 5; i.dest_ok = true; i.dest = Ned{24, 0, -8}; i.veh = Ned{0, 0, -8}; return i; };
    RescueController off; CHECK(off.state() == RState::DISABLED); RescueIn p = ready(0); p.press = true; CHECK(!off.step(p).send_goto);
    RescueController r(cfg); RescueIn e0 = ready(0); e0.press = true; e0.locked_for = 0.5; RescueOut o = r.step(e0); CHECK(o.end == REnd::NOT_READY); CHECK(r.state() == RState::IDLE);
    CHECK(o.detail.find("lock held") != std::string::npos);                          // "not ready" alone doesn't say WHICH gate failed
    { RescueIn x = ready(0); x.press = true; x.follow = FState::COASTING; CHECK(r.step(x).detail == "target not locked"); }
    { RescueIn x = ready(0); x.press = true; x.dest_ok = false; CHECK(r.step(x).detail.find("destination") != std::string::npos); }
    { RescueIn x = ready(0); x.press = true; x.guided = false; CHECK(r.step(x).detail == "vehicle not in GUIDED"); }
    RescueIn a = ready(1.0); a.press = true; o = r.step(a); CHECK(r.state() == RState::ARMED); CHECK(!o.send_goto);
    RescueIn go = ready(1.5); go.press = true; o = r.step(go); CHECK(o.send_goto); NEAR(o.dest.n, 24.0, 1e-9); CHECK(o.end == REnd::NONE);
    o = r.step(ready(1.6)); CHECK(r.state() == RState::IDLE);                                            // one tick in SENT, then back to IDLE: ready for another confirm
    auto arm_and_confirm = [&](RescueController& c, double t0) { RescueIn x = ready(t0); x.press = true; c.step(x); x.now = t0 + 0.5; return c.step(x); };
    { RescueController c(cfg); o = arm_and_confirm(c, 1.0); CHECK(o.send_goto); NEAR(o.dest.n, 24.0, 1e-9); }
    { RescueController c(cfg); RescueIn x = ready(2.0); x.abort = true; o = c.step(x); CHECK(!o.send_goto); }   // abort with no press: no-op, stays IDLE
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; c.step(x); x.press = false; x.now = 1.5; x.abort = true; o = c.step(x); CHECK(o.end == REnd::ABORTED); CHECK(!o.send_goto); }   // abort while ARMED
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; c.step(x); x.now = 2.0; x.guided = false; x.press = false; o = c.step(x); CHECK(o.end == REnd::MODE_CHANGED); CHECK(!o.send_goto); }   // pilot took over while armed
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; c.step(x); x.press = false; x.now = 4.5; o = c.step(x); CHECK(o.end == REnd::CONFIRM_TIMEOUT); }
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; x.dest = Ned{400, 0, -8}; c.step(x); x.now = 1.5; o = c.step(x); CHECK(o.end == REnd::OUT_OF_RANGE); CHECK(!o.send_goto); }   // beyond max range
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; c.step(x); x.gps_ok = false; x.press = false; x.now = 1.2; o = c.step(x); CHECK(o.end == REnd::NOT_READY); }
    { RescueController c(cfg); RescueIn x = ready(1.0); x.press = true; c.step(x); x.ekf_ok = false; x.press = false; x.now = 1.2; o = c.step(x); CHECK(o.end == REnd::NOT_READY); }   // EKF unhealthy: refuse, same as a bad GPS fix
    { RescueController c(cfg); RescueIn x = ready(0); x.ekf_ok = false; x.press = true; o = c.step(x); CHECK(o.end == REnd::NOT_READY); CHECK(c.state() == RState::IDLE); }             // can't even arm with EKF unhealthy
    { RescueCfg bc = cfg; bc.min_battery_pct = 30; RescueController c(bc); RescueIn x = ready(0); x.press = true; x.battery_pct = 25; o = c.step(x);
      CHECK(o.end == REnd::NOT_READY); CHECK(o.detail.find("battery") != std::string::npos); CHECK(c.state() == RState::IDLE);
      x.battery_pct = -1; c.step(x); CHECK(c.state() == RState::ARMED); }                                                   // unknown battery: not blocked
    { RescueCfg bc = cfg; bc.min_battery_pct = 30; RescueController c(bc); RescueIn x = ready(0); x.press = true; x.battery_pct = 80; c.step(x); CHECK(c.state() == RState::ARMED); }
}

void test_strafe_integral() {
    using namespace fl;
    Intrinsics k; Gains g; Ceilings c; Box right{k.cx + 100 - 20, k.cy - 30, 40, 60};
    Cmd o = image_control(right, 60, k, g, c, 0, true, true); CHECK(o.vy > 0); CHECK(o.yaw_rate == 0.0);     // strafe: slide, don't turn
    o = image_control(right, 60, k, g, c); CHECK(o.vy == 0.0); CHECK(o.yaw_rate > 0);                         // default: turn
    BearingIntegrator bi; double v = 0;
    for (int i = 0; i < 100; i++) v = bi.step(0.1, 0.05, 0.2, 0.3, false);
    NEAR(v, 0.2 * 0.1 * 5.0, 1e-9);                                                                            // integrates a standing error
    for (int i = 0; i < 1000; i++) v = bi.step(0.1, 0.05, 0.2, 0.3, false);
    NEAR(v, 0.3, 1e-9);                                                                                        // clamped (anti-windup)
    double before = bi.out(); bi.step(0.1, 0.05, 0.2, 0.3, true); NEAR(bi.out(), before, 1e-12);              // saturated: does not grow
    bi.reset(); NEAR(bi.out(), 0.0, 1e-12);
}

void test_view_angles() {
    using namespace fl;
    const double f = 500;
    ViewAngles v = view_angles(0, 0, f, f, 0, 0, false, 0); NEAR(v.bearing_right, 0, 1e-12); NEAR(v.depression, 0, 1e-12);
    v = view_angles(0, 100, f, f, 0, 0, false, 0); NEAR(v.depression, std::atan(0.2), 1e-12);                   // pixel below centre
    v = view_angles(100, 0, f, f, 0, 0, false, 0); NEAR(v.bearing_right, std::atan(0.2), 1e-12);
    v = view_angles(0, 0, f, f, 0, 0, false, deg2rad(-10)); NEAR(v.depression, deg2rad(10), 1e-9);              // body-fixed: nose-down pitch looks down
    v = view_angles(0, 0, f, f, 0, 0, true, deg2rad(-10)); NEAR(v.depression, 0, 1e-9);                          // stabilized: vehicle pitch ignored
    v = view_angles(0, 0, f, f, 0, -45, true, deg2rad(-15)); NEAR(v.depression, deg2rad(45), 1e-9);              // the gimbal's own tilt counts
    v = view_angles(0, 0, f, f, 30, -45, true, 0); NEAR(v.bearing_right, deg2rad(30), 1e-9); NEAR(v.depression, deg2rad(45), 1e-9);   // panned gimbal
    v = view_angles(0, 0, f, f, 0, -30, false, deg2rad(-10)); NEAR(v.depression, deg2rad(40), 1e-9);             // body mount: tilt + pitch add
    v = view_angles(0, 0, f, f, 0, -90, true, 0); NEAR(v.depression, kPi / 2, 1e-9);                              // straight down
    // FPV bank: body-fixed camera, rolled 45 deg right; a point 45 deg right of the image centre is now looking 30 deg down
    v = view_angles(f, 0, f, f, 0, 0, false, 0, deg2rad(45)); NEAR(v.depression, deg2rad(30), 1e-9); CHECK(v.bearing_right > 0);
    v = view_angles(f, 0, f, f, 0, 0, true, 0, deg2rad(45)); NEAR(v.depression, 0.0, 1e-12);                     // stabilized: roll ignored
    v = view_angles(0, 0, f, f, 0, 0, false, 0, deg2rad(45)); NEAR(v.depression, 0.0, 1e-12);                    // boresight unaffected by roll
    // looking steeply down, a sideways pixel is NOT a plain bearing offset of atan(px/f): the 3D rotation matters
    v = view_angles(100, 0, f, f, 0, -60, true, 0); CHECK(v.bearing_right > std::atan(0.2) * 1.5);
    Intrinsics k; Gains g; Ceilings c; Box centred{k.cx - 20, k.cy - 30, 40, 60};
    CHECK(image_control(centred, 60, k, g, c, deg2rad(30)).yaw_rate > 0);                     // gimbal panned right: turn right
    Box below{k.cx - 20, k.cy + 80 - 30, 40, 60};
    CHECK(image_control(below, 60, k, g, c, 0, false).vz == 0.0);                             // vz hold: no altitude chasing
    // FPV (camera fixed to the airframe, tilted 30 deg down): the vertical error is the depression below the horizon vs the
    // level-flight boresight, so the drone's own pitch does not move it
    const double hv = deg2rad(30);
    NEAR(body_cam_vertical_error(0, 0, f, f, 0, -30, 0, 0, hv), 0.0, 1e-12);                                  // hover, centred
    // nose down 10 deg to accelerate: the same target now shows 10 deg ABOVE the image centre -- still no error (the image row
    // alone would have read -10 deg and climbed)
    NEAR(body_cam_vertical_error(0, -f * std::tan(deg2rad(10)), f, f, 0, -30, deg2rad(-10), 0, hv), 0.0, 1e-9);
    NEAR(body_cam_vertical_error(0, f * std::tan(deg2rad(5)), f, f, 0, -30, 0, 0, hv), deg2rad(5), 1e-9);    // below: + (descend)
    // banked 40 deg: a tilted camera's centre really does look less steeply down (asin(sin 30 cos 40) = 22.5 deg) -- the error is
    // that true difference, -7.5 deg, not the image row's 0
    NEAR(body_cam_vertical_error(0, 0, f, f, 0, -30, 0, deg2rad(40), hv), std::asin(0.5 * std::cos(deg2rad(40))) - deg2rad(30), 1e-9);
    // pitched 45 deg nose-down (a hard dash): a target held at the level boresight (30 deg down) would sit 45 deg above the centre,
    // outside a 30 deg half view -- the reference is pulled to within 0.7 x 30 deg of the actual boresight (75 deg): error -24 deg (climb)
    NEAR(body_cam_vertical_error(0, -f * std::tan(deg2rad(45)), f, f, 0, -30, deg2rad(-45), 0, hv), deg2rad(30 - 54), 1e-9);
    // FPV speed, camera -15 deg, locked with the target 40 deg down (ref): leaning 45 deg nose-down the boresight looks 60 deg down,
    // the target shows 20 deg above the centre -- inside the 0.7 x 30 deg band, so the lock-time framing is held: no error
    NEAR(body_cam_vertical_error(0, -f * std::tan(deg2rad(20)), f, f, 0, -15, deg2rad(-45), 0, hv, deg2rad(40)), 0.0, 1e-9);
    NEAR(body_cam_vertical_error(0, 0, f, f, 0, -15, 0, 0, hv, deg2rad(40)), deg2rad(15 - 36), 1e-9);   // hover: ref clamped to 15+21
    // fixed-camera centring, forward/back part: below the + -> back away, above -> come closer, centred -> nothing; capped; smooth
    CHECK(fixed_cam_centre_vx(deg2rad(10), 0.03, 4.0, 3.0) < -0.3);
    CHECK(fixed_cam_centre_vx(deg2rad(-10), 0.03, 4.0, 3.0) > 0.3);
    CHECK(fixed_cam_centre_vx(0.02, 0.03, 4.0, 3.0) == 0.0);
    NEAR(fixed_cam_centre_vx(deg2rad(80), 0.03, 4.0, 3.0), -3.0, 1e-12);
    CHECK(std::fabs(fixed_cam_centre_vx(0.031, 0.03, 4.0, 3.0)) < 0.01);                   // just past the dead-band: tiny, no jump
    CHECK(image_control(centred, 60, k, g, c, 0, true, false, deg2rad(5)).vz > 0);           // the override drives vz: descend
    CHECK(image_control(below, 60, k, g, c, 0, true, false, 0.0).vz == 0.0);                 // ... and replaces the image row
}

void test_height() {
    using namespace fl;
    double h = 0; std::string u;
    CHECK(pick_height("auto", true, 20, 12, 0.1, h, u)); NEAR(h, 12, 1e-12); CHECK(u == "rangefinder");   // terrain-relative wins
    CHECK(pick_height("auto", true, 20, 12, 2.0, h, u)); NEAR(h, 20, 1e-12); CHECK(u == "home");          // stale: fall back
    CHECK(pick_height("home", true, 20, 12, 0.1, h, u)); NEAR(h, 20, 1e-12);
    CHECK(!pick_height("rangefinder", true, 20, 12, 2.0, h, u));                                          // required but stale: refuse
    CHECK(!pick_height("auto", false, 0, 0, 1e9, h, u));
}

void test_statustext() {
    using namespace fl;
    CHECK(fit_statustext("short") == "short");
    std::string longs(80, 'a'); std::string f = fit_statustext(longs); CHECK(f.size() == 50); CHECK(f.back() == '~');
    MsgThrottle t(3.0, 0.25);
    CHECK(t.allow("A", 0.0)); CHECK(!t.allow("A", 1.0));            // same text within 3 s: dropped
    CHECK(t.allow("B", 1.1));                                        // different text after the gap: sent
    CHECK(!t.allow("C", 1.2));                                       // within the 0.25 s overall gap: dropped
    CHECK(t.allow("D", 1.2, true));                                  // urgent bypasses the rate cap
    CHECK(t.allow("A", 3.5));                                        // repeat allowed after 3 s
}

void test_estimator() {
    using namespace fl;
    TargetEstimator e; double t = 0;
    for (int i = 0; i < 60; i++, t += 0.1) e.update(t, Ned{10 + 3.0 * t, 5 - 1.0 * t, 0}, 0.5);   // 3 m/s north, 1 m/s west, 10 Hz
    CHECK(e.valid(t - 0.1)); NEAR(e.vn(), 3.0, 0.2); NEAR(e.ve(), -1.0, 0.2);
    Ned ahead = e.predict(2.0); NEAR(ahead.n, 10 + 3.0 * (t - 0.1) + 6.0, 0.6);
    CHECK(!e.valid(t + 1.0));                                                      // stale
    e.update(t + 2.0, Ned{0, 0, 0}, 0.5); CHECK(!e.valid(t + 2.0));               // after a gap: restarts, not valid yet
    TargetEstimator s; t = 0; for (int i = 0; i < 30; i++, t += 0.1) s.update(t, Ned{7, 7, 0}, 0.5);
    NEAR(s.vn(), 0.0, 0.2); NEAR(s.ve(), 0.0, 0.2);                                // stationary target: ~zero velocity
    CHECK(geometry_sigma(10, deg2rad(15)) > geometry_sigma(10, deg2rad(45)));     // shallow views are less certain
    {   // range from the box size: H learned from steep fixes (a 1.6 m tall object, f = 537 px), then the range far off, shallow
        SizeRange sr; CHECK(!sr.ready());
        for (int i = 0; i < 10; i++) { const double slant = 15.0 + i; sr.learn(slant, 1.6 * 537 / slant, 537); }
        CHECK(sr.ready()); NEAR(sr.H(), 1.6, 1e-6);
        NEAR(sr.slant(1.6 * 537 / 60.0, 537), 60.0, 1e-6);                         // seen 60 m away: 60 m
        CHECK(SizeRange::sigma(60) < geometry_sigma(5, std::atan2(5, 60)));         // low and far (5 deg): it beats the flat-ground range
    }
    Cmd c = add_feedforward(Cmd(), 3.0, 0.0, 0.0, 1.0); NEAR(c.vx, 3.0, 1e-9); NEAR(c.vy, 0.0, 1e-9);           // facing north, target north
    c = add_feedforward(Cmd(), 3.0, 0.0, kPi / 2, 1.0); NEAR(c.vx, 0.0, 1e-9); NEAR(c.vy, -3.0, 1e-9);         // facing east: target moves to our left
    CHECK(add_feedforward(Cmd(), 3.0, 1.0, 0.0, 0.0).is_zero());                                                // gain 0 = off
    double a = 0; int steps = 0; while (a < 20.0 && steps < 1000) { a = pace_along(a, 20.0, 5.0, 2.0, 0.05); steps++; }
    NEAR(a, 20.0, 1e-9); CHECK(steps > 80);                                       // tapers: slower than 20 m / 5 m/s = 80 steps
    NEAR(pace_along(0, 100, 5, 2, 0.1), 0.5, 1e-9);                               // far from the end: full speed
    CHECK(pace_along(99.9, 100, 5, 2, 0.1) - 99.9 < 0.1);                         // near the end: slow
    // IMM-UKF vs the straight-line filter on a vehicle TURNING a corner (10 m/s, 0.4 rad/s: a 25 m radius) from noisy fixes (0.7 m,
    // 10 Hz): where will it be 1 s later? And on a straight run the IMM must be no worse.
    {
        cv::RNG rng(11);
        auto trial = [&](int imm, bool turning, double& err) {
            TargetEstimator est(2.0, imm); double tt = 0, n = 0, ee = 0, hd = 0; err = 0; int k = 0;
            for (int i = 0; i < 120; i++, tt += 0.1) {
                const double w = (turning && i >= 30) ? 0.4 : 0.0;           // straight 3 s, then the turn
                hd += w * 0.1; n += 10 * std::cos(hd) * 0.1; ee += 10 * std::sin(hd) * 0.1;
                est.update(tt, Ned{n + rng.gaussian(0.7), ee + rng.gaussian(0.7), 0}, 0.7);
                if (i >= 50 && i < 110) {                                      // in the turn: 1 s ahead vs the truth 1 s ahead
                    double h2 = hd, n2 = n, e2 = ee; for (int q = 0; q < 10; q++) { h2 += w * 0.1; n2 += 10 * std::cos(h2) * 0.1; e2 += 10 * std::sin(h2) * 0.1; }
                    const Ned p = est.predict(1.0); err += std::hypot(p.n - n2, p.e - e2); k++;
                }
            }
            err /= k;
        };
        double imm_turn, old_turn, imm_str, old_str;
        trial(1, true, imm_turn); trial(0, true, old_turn); trial(1, false, imm_str); trial(0, false, old_str);
        if (std::getenv("TRK_DEBUG")) std::printf("estimator 1 s ahead: turning IMM %.2f m vs straight-line %.2f m | straight IMM %.2f vs %.2f m\n", imm_turn, old_turn, imm_str, old_str);
        CHECK(imm_turn < 0.6 * old_turn);                                      // follows the arc
        CHECK(imm_str < 1.3 * old_str);                                        // and costs little on a straight run
    }
}

void test_latlon() {
    using namespace fl;
    double la, lo; offset_latlon(-35.0, 149.0, 111.32, 0, la, lo); NEAR(la, -35.0 + 0.001, 2e-6); NEAR(lo, 149.0, 1e-12);   // ~111 m per 0.001 deg lat
    offset_latlon(-35.0, 149.0, 0, 100, la, lo); NEAR(lo - 149.0, rad2deg(100 / (6378137.0 * std::cos(deg2rad(-35.0)))), 1e-12); CHECK(lo > 149.0);
}

void test_grabcut_point() {
    using namespace fl;
    cv::Mat img(480, 640, CV_8UC3); cv::randn(img, cv::Scalar(90, 90, 90), cv::Scalar(10, 10, 10));     // textured grey ground
    cv::rectangle(img, cv::Rect(300, 200, 16, 22), cv::Scalar(30, 30, 200), cv::FILLED);                 // small red target
    cv::rectangle(img, cv::Rect(80, 280, 150, 70), cv::Scalar(170, 120, 40), cv::FILLED);                // a large blue "vehicle"
    cv::rectangle(img, cv::Rect(620, 100, 15, 30), cv::Scalar(40, 200, 40), cv::FILLED);                 // a green object at the image edge
    Box b; std::string why;
    CHECK(grabcut_from_point(img, 308, 211, b, why));                                                    // click in the middle of it
    NEAR(b.x, 300, 3); NEAR(b.y, 200, 3); NEAR(b.w, 16, 4); NEAR(b.h, 22, 4);                           // its own size, no fixed box
    CHECK(grabcut_from_point(img, 303, 218, b, why)); NEAR(b.w, 16, 4); NEAR(b.h, 22, 4);               // a sloppy click near its corner
    CHECK(grabcut_from_point(img, 150, 315, b, why));                                                    // the large object: the window grows
    NEAR(b.x, 80, 4); NEAR(b.y, 280, 4); NEAR(b.w, 150, 6); NEAR(b.h, 70, 6);
    CHECK(grabcut_from_point(img, 628, 115, b, why)); NEAR(b.w, 15, 4); NEAR(b.h, 30, 4);              // at the image edge
    cv::Mat road(480, 640, CV_8UC3); cv::randn(road, cv::Scalar(90, 90, 90), cv::Scalar(10, 10, 10));
    CHECK(!grabcut_from_point(road, 320, 240, b, why) || b.w * b.h < 30);                                // open road: refused (or a speck)
    CHECK(!grabcut_from_point(img, 700, 100, b, why));                                                   // outside the image
}

void test_camera_motion_large_object() {
    using namespace fl;
    // the background moves 6 px right; a LARGE object (40% of the view) moves 20 px left. With the object excluded, the estimate
    // is the background's motion; without, the object's features pull it.
    cv::RNG rng(3);
    cv::Mat bg(480, 960, CV_8UC3); rng.fill(bg, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255));
    cv::GaussianBlur(bg, bg, cv::Size(5, 5), 1.5);
    cv::Mat obj(200, 380, CV_8UC3); rng.fill(obj, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255)); cv::GaussianBlur(obj, obj, cv::Size(5, 5), 1.5);
    auto frame = [&](int bx, int ox) { cv::Mat f = bg(cv::Rect(300 - bx, 0, 640, 480)).clone(); obj.copyTo(f(cv::Rect(150 + ox, 140, 380, 200))); return f; };
    const Box ob{150, 140, 380, 200};
    CameraMotion with, without;
    with.step(frame(0, 0)); without.step(frame(0, 0));
    cv::Mat Aw = with.step(frame(6, -20), &ob), Ao = without.step(frame(6, -20));
    CHECK(!Aw.empty()); if (!Aw.empty()) NEAR(Aw.at<double>(0, 2), 6.0, 1.0);
    // (without the exclusion the result is either pulled away from +6 or RANSAC happens to pick the background: not asserted)
    (void)Ao;
}

void test_chase() {
    using namespace fl;
    // GO pursuit end point: 1 m short of the target horizontally, never below the floor, hold when already that close
    Ned e = pursuit_point(Ned{0, 0, -10}, Ned{10, 0, -10}, 1.0, 0.0); NEAR(e.n, 9.0, 1e-9); NEAR(e.d, -10.0, 1e-9);
    e = pursuit_point(Ned{0, 0, -12}, Ned{16, 0, 0}, 1.0, -1.5);                        // descending straight at a target on the ground
    CHECK(e.d <= -1.5 + 1e-9);                                                          // ... but never below 1.5 m
    NEAR(e.n, 15.0, 1e-9);                                                              // 1 m short HORIZONTALLY (the stop distance)
    {   // pursuit velocity: full speed far away, braking near the end (allowing for the response lag), zero at it
        Ned v = pursuit_velocity(Ned{0, 0, -10}, Ned{40, 0, -10}, 0.0, 8.0, 4.0, 0.5); NEAR(v.n, 8.0, 1e-9);
        v = pursuit_velocity(Ned{0, 0, -10}, Ned{2, 0, -10}, 4.0, 8.0, 4.0, 0.5); CHECK(v.n == 0.0);   // 2 m to go at 4 m/s: brake now
        v = pursuit_velocity(Ned{0, 0, -10}, Ned{3, 0, -6}, 0.0, 8.0, 4.0, 0.5); CHECK(v.d > 0); NEAR(v.n / v.d, 3.0 / 4.0, 1e-9);   // straight line (descending: NED d grows downward)
        v = pursuit_velocity(Ned{1, 1, -2}, Ned{1, 1, -2}, 0.0, 8.0, 4.0, 0.5); NEAR(std::hypot(v.n, v.e), 0.0, 1e-12);
    }
    e = pursuit_point(Ned{5, 5, -2}, Ned{5.5, 5, -2}, 1.0, 0.0); NEAR(e.n, 5.0, 1e-9); NEAR(e.e, 5.0, 1e-9);   // within 1 m: hold
    // pose history: interpolation, yaw across +-pi, gimbal yaw across +-180, outside the covered time -> false
    PoseHistory h;
    h.push(PoseSample{0.0, 0, 0, -10, 3.0, 0, 0, 10, -30, 170});
    h.push(PoseSample{1.0, 10, 0, -10, -3.0, 0, 0, 10, -40, -170});
    PoseSample p; CHECK(h.at(0.5, p)); NEAR(p.n, 5.0, 1e-9); NEAR(p.mpitch, -35.0, 1e-9);
    NEAR(std::fabs(p.yaw), M_PI, 0.01);                                  // through +-pi, not through 0
    NEAR(std::fabs(p.myaw), 180.0, 1e-6);
    CHECK(!h.at(1.5, p)); CHECK(!h.at(-0.1, p));
    // chase: at the standoff, a target moving north at 4 m/s -> fly north at ff * 4; facing east that is -vy (left)
    ChaseCfg c; c.standoff = 10; c.ff = 0.9; c.k = 0.5; c.vmax = 12;
    Cmd k = chase_cmd(c, 0, 0, 0.0, Ned{10, 0, 0}, 4, 0); NEAR(k.vx, 3.6, 1e-9); NEAR(k.vy, 0.0, 1e-9);
    k = chase_cmd(c, 0, 0, M_PI / 2, Ned{10, 0, 0}, 4, 0); NEAR(k.vx, 0.0, 1e-9); NEAR(k.vy, -3.6, 1e-9);
    // too far: pulled in along the line of sight; too close: backs off; never faster than vmax
    k = chase_cmd(c, 0, 0, 0.0, Ned{30, 0, 0}, 0, 0); NEAR(k.vx, 0.5 * 19.0, 1e-9);
    k = chase_cmd(c, 0, 0, 0.0, Ned{5, 0, 0}, 0, 0); CHECK(k.vx < 0);
    k = chase_cmd(c, 0, 0, 0.0, Ned{60, 0, 0}, 10, 0); NEAR(std::hypot(k.vx, k.vy), 12.0, 1e-9);
    k = chase_cmd(c, 0, 0, 0.0, Ned{10.5, 0, 0}, 0, 0); NEAR(k.vx, 0.0, 1e-9);   // within the 1 m dead-band: no hunting
    k = chase_cmd(c, 0, 0, 0.0, Ned{6, 0, 0}, -5, 0); CHECK(k.vx < 0);              // too close, target coming at us: back off
    k = chase_cmd(c, 0, 0, 0.0, Ned{6, 0, 0}, 5, 2); CHECK(k.vx < 0.0); NEAR(k.vy, 1.8, 1e-9);   // too close: FF never closes in
    ChaseCfg cp = c; cp.pull_max = 4.0; k = chase_cmd(cp, 0, 0, 0.0, Ned{60, 0, 0}, 3, 0); NEAR(k.vx, 0.9 * 3 + 4.0, 1e-9);   // capped approach + FF
    ChaseCfg cr = c; cr.radial_only = true;
    k = chase_cmd(cr, 0, 0, 0.0, Ned{10, 0, 0}, 4, 3); NEAR(k.vx, 3.6, 1e-9); NEAR(k.vy, 0.0, 1e-9);   // sideways part dropped
    NEAR(bearing_rate(10, 0, 0, 2), 0.2, 1e-12);                                    // target north, moving east: bearing turns right
    NEAR(bearing_rate(0, 10, 2, 0), -0.2, 1e-12);
}

void test_appearance() {
    using namespace fl;
    cv::Mat img(240, 320, CV_8UC3); cv::randn(img, cv::Scalar(90, 90, 90), cv::Scalar(12, 12, 12));   // textured grey "road"
    cv::rectangle(img, cv::Rect(100, 100, 16, 22), cv::Scalar(30, 30, 200), cv::FILLED);               // small red target
    cv::Mat car(40, 60, CV_8UC3); cv::randn(car, cv::Scalar(130, 130, 135), cv::Scalar(40, 40, 40));   // textured grey occluder
    Appearance ap; CHECK(ap.init(img, Box{100, 100, 16, 22}));
    CHECK(ap.accepts(ap.score(img, Box{101, 100, 16, 22})));                                             // the target (1 px off): matches
    CHECK(ap.accepts(ap.score(img, Box{103, 102, 16, 22})));                                             // 3 px of jitter: still the target
    cv::Mat occ = img.clone(); car.copyTo(occ(cv::Rect(85, 90, 60, 40)));                                // hidden behind the "car"
    CHECK(!ap.accepts(ap.score(occ, Box{100, 100, 16, 22})));                                            // the occluder does NOT look like it
    cv::Mat moved(240, 320, CV_8UC3); cv::randn(moved, cv::Scalar(90, 90, 90), cv::Scalar(12, 12, 12));
    cv::rectangle(moved, cv::Rect(130, 104, 16, 22), cv::Scalar(30, 30, 200), cv::FILLED);              // reappears 30 px further on
    Box found; double sc = 0;
    CHECK(ap.search(moved, Box{118, 102, 16, 22}, 8, found, sc)); CHECK(iou(found, Box{130, 104, 16, 22}) > 0.5);
    CHECK(ap.reacquires(sc));
}

void test_refine() {
    using namespace fl;
    cv::Mat img(240, 320, CV_8UC3, cv::Scalar(90, 90, 90));                    // grey "road"
    cv::rectangle(img, cv::Rect(150, 90, 24, 50), cv::Scalar(30, 30, 200), cv::FILLED);   // a red "person", 24x50
    cv::randn(img, cv::Scalar(90, 90, 90), cv::Scalar(6, 6, 6)); cv::rectangle(img, cv::Rect(150, 90, 24, 50), cv::Scalar(30, 30, 200), cv::FILLED);
    Box out; CHECK(grabcut_refine(img, Box{127, 80, 70, 70}, out));             // a rough 70x70 click box around it
    NEAR(out.x, 150, 4); NEAR(out.y, 90, 4); NEAR(out.w, 24, 6); NEAR(out.h, 50, 6);
    cv::Mat flat(240, 320, CV_8UC3, cv::Scalar(90, 90, 90));
    CHECK(!grabcut_refine(flat, Box{127, 80, 70, 70}, out) || out.w * out.h < 70 * 70);   // nothing there: refuse (or at least not grow)
}

// IMM vs a single filter: a target running straight (with measurement noise), then REVERSING. The IMM must predict the reversal
// much better than the calm filter alone, and the straight run no worse than the noisy (manoeuvre) filter alone.
void test_imm() {
    using namespace fl;
    cv::RNG rng(3);
    auto run = [&](int kind, double& err_turn, double& err_straight) {   // 0: IMM, 1: calm only, 2: manoeuvre only
        TargetFilter imm(Box{100, 200, 30, 30}); BoxKalman calm(Box{100, 200, 30, 30}, 0.08, 1.0, 0.3), man(Box{100, 200, 30, 30}, 0.08, 1.0, 3.0);
        calm.adaptive = false; man.adaptive = false;
        double x = 115, v = 8; err_turn = err_straight = 0; int nt = 0, ns = 0;
        for (int i = 1; i < 80; i++) {
            if (i == 50) v = -8;                                            // hard reversal
            x += v;
            const Box m{x - 15 + rng.gaussian(1.5), 200 + rng.gaussian(1.5), 30, 30};
            Box pred;
            if (kind == 0) { imm.predict(); pred = imm.box(); imm.update(m); }
            else if (kind == 1) { calm.predict(); pred = calm.box(); calm.update(m); }
            else { man.predict(); pred = man.box(); man.update(m); }
            const double e = std::fabs(pred.cx() - x);
            if (i >= 51 && i < 61) { err_turn += e; nt++; } else if (i >= 25 && i < 50) { err_straight += e; ns++; }
        }
        err_turn /= nt; err_straight /= ns;
    };
    double ti, si, tc, sc, tm, sm; run(0, ti, si); run(1, tc, sc); run(2, tm, sm);
    if (std::getenv("TRK_DEBUG")) std::printf("imm test: after reversal IMM %.1f calm %.1f manoeuvre %.1f px | straight IMM %.1f calm %.1f manoeuvre %.1f px\n", ti, tc, tm, si, sc, sm);
    CHECK(ti < 0.6 * tc);                                                   // follows the reversal far better than the calm filter
    CHECK(si <= sm * 1.05);                                                 // and is at least as smooth as the manoeuvre filter when straight
}

// GO guidance on a simulated drone (velocity response 0.4 s, <= 6 m/s^2) at 10 m/s against a target CROSSING at 5 m/s 40 m ahead,
// and one TURNING (5 m/s, 0.25 rad/s): time to reach it (< 1.5 m). Intercept and PN must beat the tail-chasing pursuit.
// [GO-11] TESTS: the guidance laws of [GO-4] against moving / turning / crossing targets.
void test_go_guidance() {
    using namespace fl;
    // the SIGNED stop offset (go_settings::kStopOffset): + before the target, 0 at it, - past it along the approach direction
    {
        const Ned start{0, 0, -10}, tgt{20, 0, 0}; const double floor_d = -3;       // target 20 m north, stop >= 3 m above its ground
        Ned e = pursuit_point(start, tgt, 10, floor_d, &start); NEAR(e.n, 10, 1e-9); NEAR(e.d, -3, 1e-9);   // +10: 10 m BEFORE
        e = pursuit_point(start, tgt, 0, floor_d, &start);      NEAR(e.n, 20, 1e-9);                         // 0: AT it
        e = pursuit_point(start, tgt, -10, floor_d, &start);    NEAR(e.n, 30, 1e-9); NEAR(e.e, 0, 1e-9); NEAR(e.d, -3, 1e-9);   // -10: PAST
        // flown OVER the target (vehicle at 25 m): the past-stop stays 10 m beyond it in the approach direction, not behind the drone
        e = pursuit_point(Ned{25, 0, -5}, tgt, -10, floor_d, &start); NEAR(e.n, 30, 1e-9);
        auto none = [](double) { return Ned{20, 0, 0}; };
        // right over the target with a -10 offset the dash still flies ON (northwards), towards the point beyond
        const Ned v = go_velocity(0, Ned{20, 0, -3}, Ned{10, 0, 0}, tgt, Ned{}, none, -10, floor_d, 25, 8, 0.5, 3, &start);
        CHECK(v.n > 1.0);
        // FLY-THROUGH: how far past the target, along the approach (start -> target, 20 m north): before -, past +, sideways ignored
        NEAR(past_target_m(Ned{5, 0, -8}, tgt, start, 0), -15, 1e-9);
        NEAR(past_target_m(Ned{32, 4, -3}, tgt, start, 0), 12, 1e-9);
        NEAR(past_target_m(Ned{20, 0, -3}, tgt, tgt, kPi / 2), 0, 1e-9);                  // started over it: along the heading
        NEAR(past_target_m(Ned{20, 5, -3}, tgt, tgt, kPi / 2), 5, 1e-9);
        // ... and its approach velocity, with the stop point put 1 km past: full GO speed right at the target, no braking before it
        { const Ned vv = go_velocity(0, Ned{18, 0, -3}, Ned{25, 0, 0}, tgt, Ned{}, none, -1000, floor_d, 25, 3, 0.5, 3, &start);
          NEAR(std::hypot(vv.n, vv.e), 25.0, 0.5); }
        // ... while a +10 offset at the same place means it has gone too far: it is not sent onward
        CHECK(go_velocity(0, Ned{20, 0, -3}, Ned{0, 0, 0}, tgt, Ned{}, none, 10, floor_d, 25, 8, 0.5, 3, &start).n <= 1e-9);
    }
    auto run = [](int mode, bool turning) {                                // 0 pursuit, 1 lead, 2 intercept, 3 PN
        Ned p{0, 0, -10}, v{0, 0, 0}, tp{40, -15, -10}; double th = kPi / 2, w = turning ? 0.25 : 0.0;
        Ned vprev{}; bool have = false; const double dt = 0.05;
        for (int k = 0; k < 600; k++) {
            const Ned tv{5 * std::cos(th), 5 * std::sin(th), 0};
            auto pred = [&](double t) {                                    // the target's own motion model (as the IMM would know it)
                if (std::fabs(w) < 1e-6) return Ned{tp.n + tv.n * t, tp.e + tv.e * t, tp.d};
                const double s2 = std::sin(w * t), c2 = std::cos(w * t);
                return Ned{tp.n + (tv.n * s2 - tv.e * (1 - c2)) / w, tp.e + (tv.n * (1 - c2) + tv.e * s2) / w, tp.d};
            };
            const double vnow = std::sqrt(v.n * v.n + v.e * v.e + v.d * v.d);
            (void)vnow; (void)vprev; (void)have;
            const Ned cmd = go_velocity(mode, p, v, tp, tv, pred, 0.0, 0.0, 10, 4, 0.5, 3.0);
            const double a = std::min(1.0, dt / 0.4);                     // the vehicle's velocity response, accel-limited
            Ned dv{a * (cmd.n - v.n), a * (cmd.e - v.e), a * (cmd.d - v.d)};
            const double dvm = std::sqrt(dv.n * dv.n + dv.e * dv.e + dv.d * dv.d), lim = 6.0 * dt;
            if (dvm > lim) { dv.n *= lim / dvm; dv.e *= lim / dvm; dv.d *= lim / dvm; }
            v = Ned{v.n + dv.n, v.e + dv.e, v.d + dv.d}; p = Ned{p.n + v.n * dt, p.e + v.e * dt, p.d + v.d * dt};
            tp = Ned{tp.n + tv.n * dt, tp.e + tv.e * dt, tp.d}; th += w * dt;
            if (std::hypot(tp.n - p.n, tp.e - p.e) < 1.5) return (k + 1) * dt;
        }
        return 99.0;
    };
    double t[4][2]; for (int m = 0; m < 4; m++) for (int tr = 0; tr < 2; tr++) t[m][tr] = run(m, tr == 1);
    if (std::getenv("TRK_DEBUG")) std::printf("GO guidance, time to reach: crossing pursuit %.1f lead %.1f intercept %.1f PN %.1f s | turning pursuit %.1f lead %.1f intercept %.1f PN %.1f s\n",
                                              t[0][0], t[1][0], t[2][0], t[3][0], t[0][1], t[1][1], t[2][1], t[3][1]);
    for (int m = 1; m < 4; m++) for (int tr = 0; tr < 2; tr++) CHECK(t[m][tr] < 99 && t[m][tr] < t[0][tr]);   // all beat the tail chase
}

void test_view() {
    using namespace fl;
    ViewCfg c;
    ViewDecision d = decide_view(c, 12, 1.0, true, 30, 0.05);                     // a person 12 px at 30 m: zoom NOW and move closer
    CHECK(d.approaching); NEAR(d.zoom, 1.25, 1e-9); NEAR(d.ref_h, 12 * 30 / 8.0, 1e-9);   // needs 4x: zoom step 1.25, head for 8 m
    d = decide_view(c, 15, 1.25, true, 30, 0.05); NEAR(d.zoom, 1.25 * 1.25, 1e-9);           // keeps zooming in, step by step
    d = decide_view(c, 30, 1.0, true, 16, 0.05); CHECK(d.approaching); NEAR(d.zoom, 1.25, 1e-9); NEAR(d.ref_h, 48, 1e-9);
    d = decide_view(c, 60, 2.0, true, 10, 0.05); NEAR(d.zoom, 1.6, 1e-9);                     // got closer: 30 px unzoomed -> zoom comes back out
    d = decide_view(c, 30, 1.0, true, 8.2, 0.05); CHECK(!d.approaching); CHECK(d.zoom > 1.0); // at min distance: zoom only
    d = decide_view(c, 30, 1.0, false, 0, 0.05); CHECK(!d.approaching); CHECK(d.zoom > 1.0);  // no distance estimate: zoom only
    d = decide_view(c, 30, 1.0, true, 16, 0.6); CHECK(d.approaching); NEAR(d.zoom, 1.0, 1e-9); // fast crossing: move, but no zoom-in
    CHECK(d.climb == +1);                                                                     //   ... and climb for a wider view
    d = decide_view(c, 40, 2.0, true, 16, 0.6); NEAR(d.zoom, 2.0 / 1.25, 1e-9); CHECK(d.climb == 0);   //   zoomed in: zoom out a step first
    d = decide_view(c, 200, 1.0, true, 10, 0.0); CHECK(d.climb == +1);                        // a large target: climb
    d = decide_view(c, 30, 3.0, true, 8.2, 0.05); CHECK(d.climb == -1);                       // small at full zoom: come back down
    d = decide_view(c, 150, 1.0, true, 10, 0.0); CHECK(d.backing_off); CHECK(d.ref_h < 120);   // too big: back off
    d = decide_view(c, 70, 1.0, true, 10, 0.0); CHECK(!d.approaching && !d.backing_off); NEAR(d.zoom, 1.0, 1e-9); NEAR(d.ref_h, 70, 1e-9);
    d = decide_view(c, 30, 3.0, true, 8.2, 0.05); NEAR(d.zoom, 3.0, 1e-9);                    // 10 px unzoomed wants 4.8x: stays at the 3x maximum
    Box z = zoom_box(Box{330, 250, 10, 20}, 2.0, 320, 240); NEAR(z.x, 340, 1e-9); NEAR(z.y, 260, 1e-9); NEAR(z.w, 20, 1e-9);
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(0, 0, 0)); cv::rectangle(img, cv::Rect(310, 230, 20, 20), cv::Scalar(255, 255, 255), cv::FILLED);
    cv::Mat out; digital_zoom(img, 2.0, out); CHECK(out.size() == img.size());
    cv::Mat wm; cv::inRange(out, cv::Scalar(200, 200, 200), cv::Scalar(255, 255, 255), wm); cv::Rect wb = cv::boundingRect(wm);
    NEAR(wb.width, 40, 2); NEAR(wb.x, 300, 2);                                    // the 20 px square became 40 px, still centred
}

void test_camera_motion() {
    using namespace fl;
    cv::Mat a(240, 320, CV_8UC3); cv::randn(a, cv::Scalar(100, 100, 100), cv::Scalar(40, 40, 40)); cv::GaussianBlur(a, a, cv::Size(5, 5), 0);
    cv::Mat M = (cv::Mat_<double>(2, 3) << 1, 0, 7, 0, 1, -4), b; cv::warpAffine(a, b, M, a.size(), cv::INTER_LINEAR, cv::BORDER_REFLECT);
    CameraMotion cm; cm.step(a); cv::Mat A = cm.step(b);
    CHECK(!A.empty()); if (!A.empty()) { NEAR(A.at<double>(0, 2), 7, 0.5); NEAR(A.at<double>(1, 2), -4, 0.5); }   // the image shifted (7, -4)
    BoxKalman kf(Box{100, 100, 20, 30}); kf.warp(M); NEAR(kf.box().x, 107, 1e-9); NEAR(kf.box().y, 96, 1e-9);     // the prediction moves along
    Box w = warp_box(Box{100, 100, 20, 30}, M); NEAR(w.x, 107, 1e-9); NEAR(w.w, 20, 1e-9);
}

void test_ekf() {
    using namespace fl;
    CHECK(ekf_flags_healthy(1 | 2 | 16));
    CHECK(ekf_flags_healthy(1 | 2 | 4 | 8 | 16 | 32));                   // extra good flags are fine
    CHECK(!ekf_flags_healthy(1));                                          // attitude only: NOT healthy (the old check passed this)
    CHECK(!ekf_flags_healthy(1 | 2));                                      // no absolute position
    CHECK(!ekf_flags_healthy(1 | 2 | 16 | 1024));                          // uninitialized
    CHECK(!ekf_flags_healthy(1 | 2 | 16 | 32768));                         // GPS glitching
    CHECK(!ekf_flags_healthy(1 | 2 | 16 | 128));                           // constant-position mode
    CHECK(!ekf_flags_healthy(0));
}

void test_framegap() {
    using namespace fl;
    FrameGap g; NEAR(g.observe(10.0), 0.0, 1e-12); NEAR(g.observe(10.1), 0.1, 1e-9); CHECK(!FrameGap::stalled(0.1));
    NEAR(g.observe(11.0), 0.9, 1e-9); CHECK(FrameGap::stalled(0.9)); CHECK(!FrameGap::stalled(0.3));
}

}  // namespace selftest

void occlusion_selftest();   // defined after LockTracker (needs the real CSRT engine)
void csrt_scale_selftest();

int run_selftest() {
    std::printf("selftest: follow logic\n");
    selftest::test_basics(); selftest::test_slew(); selftest::test_gate_and_guards(); selftest::test_kalman();
    selftest::test_control(); selftest::test_fsm(); selftest::test_framegap(); selftest::test_top(); selftest::test_plausible(); selftest::test_color_plausible(); selftest::test_rescue(); selftest::test_ekf(); selftest::test_view_angles(); selftest::test_strafe_integral(); selftest::test_view(); selftest::test_imm(); selftest::test_go_guidance(); selftest::test_camera_motion(); selftest::test_height(); selftest::test_statustext(); selftest::test_estimator(); selftest::test_latlon(); selftest::test_refine(); selftest::test_appearance(); selftest::test_camera_motion_large_object(); selftest::test_chase(); selftest::test_grabcut_point(); occlusion_selftest(); csrt_scale_selftest();
    std::printf("selftest: %d checks, %d failed\n", selftest::checks, selftest::failed);
    return selftest::failed == 0 ? 0 : 1;
}


// ============================================================
// ---- MAVLink link: companion identity, transports (udp / tcp / serial), 20 Hz command tick, camera component
// ============================================================
// Documentation behind the design (primary sources: MAVLink common.xml + guides, ArduPilot Copter docs/parameters):
//   * Companion identity: sysid = the vehicle's own system id (default 1), compid 191 = MAV_COMP_ID_ONBOARD_COMPUTER; HEARTBEAT type
//     MAV_TYPE_ONBOARD_CONTROLLER (18), autopilot MAV_AUTOPILOT_INVALID (8). Never a GCS id: ArduPilot treats MAV_GCS_SYSID
//     (default 255) as the ground station for failsafe/RC-override, so a companion with 255 would mask a lost GCS.
//   * HEARTBEAT guide: broadcast even when idle, but NOT from a thread unaware of the component's health -> heartbeats are sent
//     only while the main loop's liveness stamp is fresh (< 1 s).
//   * Mode change: MAV_CMD_DO_SET_MODE (176) via COMMAND_LONG (SET_MODE is superseded since 2015-12): param1 =
//     MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, param2 = custom mode (Copter GUIDED = 4); confirm with COMMAND_ACK and the next heartbeat;
//     resend with an incremented confirmation field (command protocol).
//   * MAV_CMD_SET_MESSAGE_INTERVAL (511): param1 message id, param2 interval in microseconds, param7 = 0 (flight-stack default target).
//   * SET_POSITION_TARGET_LOCAL_NED, MAV_FRAME_BODY_NED (= BODY_FRD for velocity): type_mask bits X1 Y2 Z4 VX8 VY16 VZ32 AX64 AY128
//     AZ256 FORCE512 YAW1024 YAW_RATE2048. Default 1479 (position, acceleration and yaw ignored; velocity + yaw rate used).
//   * Copter GUIDED: velocity commands must be re-sent (about every second; the vehicle stops after GUID_TIMEOUT, default 3 s) -> 20 Hz tick.
//   * Camera protocol v2 (common.xml): CAMERA_INFORMATION flags HAS_TRACKING_POINT 512 / HAS_TRACKING_RECTANGLE 1024;
//     MAV_CMD_CAMERA_TRACK_POINT 2004 / _RECTANGLE 2005 / STOP_TRACKING 2010 (normalised 0..1 image coordinates);
//     CAMERA_TRACKING_IMAGE_STATUS (275) with status flags IDLE 0 / ACTIVE 1 / ERROR 2 / MTI 4 / COASTING 8.

struct LinkConfig {
    std::string connection = "udpin:0.0.0.0:14560";   // udpin:HOST:PORT | udpout:HOST:PORT | tcp:HOST:PORT | serial:/dev/serial0:921600
    uint8_t sysid = 1;                                 // the vehicle's own system id (companion = a component of the vehicle)
    uint8_t compid = 191;                              // MAV_COMP_ID_ONBOARD_COMPUTER
    int type_mask = 1479;                              // see above; 1991 (adds FORCE_SET) also works in SITL
    double stale_s = 0.3;                              // a command older than this is replaced by zero
    double hb_timeout_s = 2.0;                         // vehicle heartbeat older than this = link lost
    int tick_hz = 20;
    // Only this system id may command the companion (tracking + rescue MAV_CMD_USER_*): QGC's default GCS id is 255 (ArduPilot's
    // MAV_GCS_SYSID). Previously ANY sender on the telemetry network -- including broadcast commands -- could arm and confirm a
    // dash. 0 = accept any sender (bench/sim only). This is a filter, not authentication: enable MAVLink2 signing on the
    // vehicle/GCS link for that.
    uint8_t gcs_sysid = 255;
    fl::Ceilings ceilings;
    // MAVLink camera component (QGroundControl tap/rectangle tracking). Disabled unless cam_enable.
    bool cam_enable = false;
    uint8_t cam_compid = 100;                          // MAV_COMP_ID_CAMERA
    std::string cam_video_uri = "udp://0.0.0.0:5600";  // what QGC should play (VIDEO_STREAM_INFORMATION)
    bool cam_video_rtsp = false;
    int cam_w = 1280, cam_h = 720;
    double cam_fps = 30.0;
};

struct VehicleState {
    bool known = false;                  // a vehicle heartbeat was seen at least once
    bool link_ok = false;                // heartbeat fresh
    double hb_age = 1e9;                 // seconds since the last vehicle heartbeat
    uint8_t sysid = 0, compid = 0;       // the vehicle
    uint32_t custom_mode = 0;
    bool guided = false;                 // Copter GUIDED = custom mode 4
    bool armed = false;
    bool alt_valid = false;
    double alt_m = 0;                    // metres above the home/origin altitude (GLOBAL_POSITION_INT.relative_alt, else -LOCAL z)
    bool pos_valid = false;
    double pos_n = 0, pos_e = 0, pos_d = 0;   // LOCAL_POSITION_NED, metres from the EKF origin
    float roll = 0, pitch = 0, yaw = 0;  // rad
    float yawspeed = 0;                  // rad/s (ATTITUDE), for the yaw feed-forward
    bool mount_known = false; double mount_pitch_deg = 0, mount_yaw_deg = 0, mount_age = 1e9;   // GIMBAL_DEVICE_ATTITUDE_STATUS (autopilot mount)
    int rc[19] = {0};                    // 1-indexed PWM, 0 = never received
    double rc_age = 1e9;                 // seconds since the last RC_CHANNELS message (stale RC must not be read as a live switch position)
    int gps_fix = 0, gps_sats = 0;
    bool ekf_ok = false;                 // false until a fresh EKF_STATUS_REPORT says otherwise (never assume healthy)
    bool ekf_known = false;
    std::array<float, 3> vel{0.f, 0.f, 0.f};  // NED m/s
    bool geo_valid = false; double lat_deg = 0, lon_deg = 0;   // GLOBAL_POSITION_INT, for the QGC map markers
    double rng_m = 0;                    // downward rangefinder (DISTANCE_SENSOR, orientation PITCH_270), metres, attitude-corrected
    double rng_age = 1e9;                // seconds since the last valid downward reading
    int battery_pct = -1;                // SYS_STATUS.battery_remaining, -1 = unknown
};

struct TrackRequest {                    // from QGC, normalised 0..1
    enum Kind { POINT, RECT, STOP } kind = STOP;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0, radius = 0;
};

// rescue/track control from QGC (MAV_CMD_USER_1..5): same meaning as g / x / u, a direct brake, and LOCK (relock the detector's
// candidate if there is one, else lock a box at the image centre -- point the camera at the target first)
enum class OpCmd { PRESS, ABORT, UNLOCK, BRAKE, LOCK };

class MavlinkLink {
public:
    using Clock = std::chrono::steady_clock;
    explicit MavlinkLink(const std::string& connection) : MavlinkLink(make_cfg(connection)) {}
    explicit MavlinkLink(const LinkConfig& cfg) : cfg_(cfg) {
        if (cfg_.sysid == 255) throw std::runtime_error("--sysid 255 is the ground-station identity (MAV_GCS_SYSID); use the vehicle's own id (default 1)");
        open_transport();
        t_start_ = Clock::now(); liveness_ = t_start_;
        running_ = true;
        thread_ = std::thread([this] { run(); });
    }
    ~MavlinkLink() { shutdown(); }
    MavlinkLink(const MavlinkLink&) = delete;
    MavlinkLink& operator=(const MavlinkLink&) = delete;

    // ---- called by the frame loop -------------------------------------------------------------------
    void liveness() { std::lock_guard<std::mutex> lk(mtx_); liveness_ = Clock::now(); }   // proves the main loop is alive
    VehicleState state() const {
        std::lock_guard<std::mutex> lk(mtx_);
        VehicleState s = vs_;
        s.hb_age = vs_.known ? sec(Clock::now() - last_hb_) : 1e9;
        s.link_ok = link_alive_locked(Clock::now());
        s.rc_age = rc_known_ ? sec(Clock::now() - last_rc_) : 1e9;
        if (s.ekf_known && sec(Clock::now() - last_ekf_) > 3.0) s.ekf_ok = false;
        s.rng_age = rng_known_ ? sec(Clock::now() - last_rng_) : 1e9;
        s.mount_age = s.mount_known ? sec(Clock::now() - last_mount_) : 1e9;   // requested at 2 Hz: a silent EKF report is not a healthy one
        return s;
    }
    // Desired velocity (body frame) for the NEXT ticks; `allowed` = the follow state machine permits steering.
    void command(const fl::Cmd& c, bool allowed) {
        std::lock_guard<std::mutex> lk(mtx_);
        want_ = c; allowed_ = allowed; want_at_ = Clock::now();
    }
    void begin_session() { std::lock_guard<std::mutex> lk(mtx_); session_ = true; zero_until_ = Clock::time_point(); slew_.reset(); }
    // GO takes over: silence NOW -- the usual 1 s of zero-velocity commands (end_session) fought the GO's own velocity commands
    // at 20 Hz and held the drone back for the first second of every dash (measured)
    void handover_session() { std::lock_guard<std::mutex> lk(mtx_); session_ = false; zero_until_ = Clock::time_point(); want_ = fl::Cmd(); allowed_ = false; }
    void end_session() {                                   // after the lock ends: 1 s of zeros, then silence (never fight the pilot)
        std::lock_guard<std::mutex> lk(mtx_);
        if (session_) { session_ = false; zero_until_ = Clock::now() + std::chrono::seconds(1); }
        want_ = fl::Cmd(); allowed_ = false;
    }
    void request_guided() {                                // non-blocking: DO_SET_MODE with ACK + heartbeat confirmation
        std::lock_guard<std::mutex> lk(mtx_);
        if (vs_.guided) return;
        pend_mode_ = true; pend_sent_ = 0; pend_start_ = Clock::now(); pend_last_ = Clock::time_point();
    }
    bool mode_pending() const { std::lock_guard<std::mutex> lk(mtx_); return pend_mode_; }

// [GO-9] MAVLINK: how a GO reaches the flight controller. send_goto() starts it (position target + speed), the dash then
// streams update_goto_velocity() (velocity + acceleration feed-forward, nose toward the target) or update_goto_point() (a
// plain hold), cancel_goto() stops the resends, stop_point() is where the drone can brake to. Re-sent at the 20 Hz tick
// until the pilot leaves GUIDED. See the GO map at the top of the file.
    // ---- go-to-target: a destination, kept alive by periodic resends until the pilot leaves GUIDED. No arrival/hold/release
    // logic here (see the fl:: go-to-target comment above RescueController).
    void send_goto(const fl::Ned& dest, double speed_ms, double yaw_rad) {
        std::lock_guard<std::mutex> lk(mtx_);
        goto_active_ = true; goto_dest_ = dest; goto_has_vel_ = false; goto_speed_ = speed_ms; goto_yaw_ = yaw_rad;
        goto_speed_last_ = Clock::time_point(); goto_pos_last_ = Clock::time_point();   // force an immediate send on the next tick
    }
    // Moves the destination (e.g. the next point along a paced diagonal line) without re-triggering the DO_CHANGE_SPEED resend
    // timer; the position target still goes out on its own next tick (<= 1 s, usually sooner since this is called every frame).
    // velocity (NED) + yaw: the GO pursuit's command. Only while it keeps coming (refreshed < 0.3 s ago); after that the resend
    // becomes a plain hold where the vehicle is
    // acc: the commanded acceleration (feed-forward, NED m/s^2): ArduPilot tilts for it at once instead of waiting for a velocity error
    void update_goto_velocity(const fl::Ned& vel, double yaw_rad, const fl::Ned& acc = fl::Ned{}, const fl::Ned* dest = nullptr) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!goto_active_) return;
        goto_vel_ = vel; goto_acc_ = acc; goto_has_vel_ = true; goto_yaw_ = yaw_rad; goto_vel_at_ = Clock::now();
        if (dest) goto_dest_ = *dest;
        goto_pos_last_ = Clock::time_point();
    }
    void update_goto_point(const fl::Ned& dest, double yaw_rad) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!goto_active_) return;
        goto_dest_ = dest; goto_has_vel_ = false; goto_yaw_ = yaw_rad;
        goto_pos_last_ = Clock::time_point();          // position resends promptly (as fast as the caller updates it); speed timer untouched
    }
    // Stops the go-to resend loop (does NOT by itself stop the vehicle -- GUIDED holds the last position target it received;
    // pair this with send_mode_change(BRAKE) to actually arrest the motion). Safe to call even if no go-to is active.
    // ABORT: zero velocity NOW (sent on the next 50 ms tick, before anything else), the go-to and the follow loop cancelled -- the
    // caller then commands BRAKE. A position hold here instead would first pull a fast-moving drone back toward the hold point.
    void stop_now() {
        std::lock_guard<std::mutex> lk(mtx_);
        goto_active_ = false; goto_has_vel_ = false; hold_pending_ = false; stop_pending_ = true; stop_yaw_ = vs_.yaw;
        session_ = false; want_ = fl::Cmd();
    }
    // A position+velocity target keeps moving on the autopilot by itself (ArduPilot advances it until GUID_TIMEOUT, 3 s, with no
    // update): unless someone else has taken over (hold_here = false, e.g. a QGC go-to), the last thing sent is a plain hold here.
    // Where the vehicle comes to rest from its current velocity at a firm braking deceleration (m/s^2): a hold placed HERE instead
    // of at the current position. Holding at the current position while moving fast made the autopilot overshoot it by
    // v^2/2a and fly BACK (inferred in review: ~13 m at 8 m/s), toward or past the target. Called with mtx_ held.
    fl::Ned stop_point_locked(double decel = 2.5) const {
        const double vh = std::hypot(vs_.vel[0], vs_.vel[1]), sh = vh * vh / (2 * decel);
        fl::Ned p{vs_.pos_n, vs_.pos_e, vs_.pos_d};
        if (vh > 0.3) { p.n += sh * vs_.vel[0] / vh; p.e += sh * vs_.vel[1] / vh; }
        // vertically only UPWARD (climbing): a descending GO's stop point projected down had no height floor here (review)
        if (vs_.vel[2] < -0.3) p.d += vs_.vel[2] * std::fabs(vs_.vel[2]) / (2 * decel);
        return p;
    }
    fl::Ned stop_point(double decel = 2.5) const { std::lock_guard<std::mutex> lk(mtx_); return stop_point_locked(decel); }
    void cancel_goto(bool hold_here = true) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (goto_active_ && goto_has_vel_ && hold_here && vs_.pos_valid) {
            hold_pending_ = true; hold_at_ = stop_point_locked(); hold_yaw_ = vs_.yaw;
        } else {
            hold_pending_ = false;
        }
        goto_active_ = false; goto_has_vel_ = false;
    }
    void clear_hold() {
        std::lock_guard<std::mutex> lk(mtx_);
        hold_pending_ = false;
        stop_pending_ = false;
        goto_active_ = false;
        goto_has_vel_ = false;
    }
    // One-shot safety mode change (BRAKE on abort, LOITER on lock loss): fires DO_SET_MODE 3x with the command protocol's
    // incrementing confirm field for UDP reliability, then returns -- unlike request_guided() this does not wait for or track
    // an ACK/heartbeat confirmation, because it is used for urgent one-off events, not the normal arm-to-fly path.
    void send_mode_change(int custom_mode) {
        // queued for the I/O thread's tick (3 sends, one per 50 ms tick) instead of sleeping here: this is called from the frame
        // loop, which previously stalled ~60 ms on every BRAKE/LOITER -- at exactly the moment the video and lock matter most
        std::lock_guard<std::mutex> lk(mtx_);
        urgent_mode_ = custom_mode; urgent_left_ = 3;
    }
    // STATUSTEXT to the pilot's GCS (QGC shows these): previously a lost lock only printed to the companion's own console
    // (nothing the officer in the field would ever see); text is truncated to the message's 50-char field.
    void send_statustext(uint8_t severity, const std::string& text) {
        char buf[51] = {0}; std::strncpy(buf, text.c_str(), 50);
        mavlink_message_t m; mavlink_msg_statustext_pack(cfg_.sysid, cfg_.compid, &m, severity, buf, 0, 0);
        send_msg(m);
    }

    // ---- QGC map marker -----------------------------------------------------------------------------------------------
    // The estimated target (and the dash destination) shown live on QGC's Fly View map, as an ADSB_VEHICLE "traffic" icon
    // with a callsign -- QGC draws any ADSB_VEHICLE arriving on the vehicle link (ADSBVehicleManager; an icon, no altitude
    // needed). It must never look like real traffic to the FLIGHT CONTROLLER, whose ADS-B avoidance could react to an
    // "aircraft" sitting on the target: ArduPilot's AP_ADSB drops any report whose ICAO address is above 24 bits or that
    // lacks the altitude flag, so both are used here (kMarkerIcaoBase > 0xFFFFFF, and no ADSB_FLAGS_VALID_ALTITUDE).
    // QGC removes a marker 120 s after its last update; there is no explicit "delete".
    static constexpr uint32_t kMarkerIcaoBase = 0x7E000000u;
    // Point the autopilot's gimbal (MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW as COMMAND_INT: param1 pitch deg, + up; param2 yaw deg relative
    // to the nose; x = flags 0 = yaw follows the vehicle; z = gimbal id 0 = primary). Used by --gimbal-centre to keep the locked
    // target vertically centred, the way the C12 does it with its own gimbal.
    void send_gimbal_pitchyaw(double pitch_deg, double yaw_deg) {
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid ? vs_.sysid : cfg_.sysid; tc = vs_.compid ? vs_.compid : 1; }
        mavlink_message_t m;
        mavlink_msg_command_int_pack(cfg_.sysid, cfg_.compid, &m, ts, tc, MAV_FRAME_LOCAL_NED, MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW, 0, 0,
                                     (float)pitch_deg, (float)yaw_deg, NAN, NAN, 0 /*flags*/, 0, 0 /*gimbal id*/);
        send_msg(m);
    }

    void send_map_marker(uint32_t id, double lat_deg, double lon_deg, double heading_deg, double speed_ms, const std::string& label) {
        char cs[9] = {0}; std::strncpy(cs, label.c_str(), 8);
        uint16_t flags = ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_CALLSIGN;
        uint16_t hdg = 0, spd = 0;
        if (std::isfinite(heading_deg) && speed_ms > 0.3) {
            flags |= ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_VELOCITY;
            hdg = (uint16_t)std::lround(std::fmod(std::fmod(heading_deg, 360.0) + 360.0, 360.0) * 100.0);
            spd = (uint16_t)std::lround(std::min(speed_ms, 600.0) * 100.0);
        }
        mavlink_message_t m;
        mavlink_msg_adsb_vehicle_pack(cfg_.sysid, cfg_.compid, &m, kMarkerIcaoBase + id, (int32_t)std::lround(lat_deg * 1e7),
                                      (int32_t)std::lround(lon_deg * 1e7), ADSB_ALTITUDE_TYPE_GEOMETRIC, 0, hdg, spd, 0, cs,
                                      ADSB_EMITTER_TYPE_NO_INFO, 0 /*tslc*/, flags, 0);
        send_msg(m);
    }

    // ---- camera component (QGC tracking) ------------------------------------------------------------
    bool poll_track_request(TrackRequest& out) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (track_q_.empty()) return false;
        out = track_q_.front(); track_q_.pop_front(); return true;
    }
    // p1/p2: the command's param1/param2 (QGC actions can carry them: GO = speed m/s, stop distance m; <= 0 = configured value)
    bool poll_op_request(OpCmd& out, double* p1 = nullptr, double* p2 = nullptr) {
        std::lock_guard<std::mutex> lk(mtx_);
        if (op_q_.empty()) return false;
        out = op_q_.front(); op_q_.pop_front();
        const std::array<double, 2> p = op_p_.empty() ? std::array<double, 2>{0, 0} : op_p_.front(); if (!op_p_.empty()) op_p_.pop_front();
        if (p1) *p1 = p[0];
        if (p2) *p2 = p[1];
        return true;
    }
    // True if the flight controller's own reported position target (POSITION_TARGET_LOCAL_NED) has drifted more than
    // margin_m from the LAST point we actually sent -- i.e. something else (QGC's "Go to location"/"Orbit"/"ROI", which stay
    // in GUIDED and so are not caught by the mode check) is now also writing position targets and will fight our resends.
    // Compared against goto_dest_ (what we last SENT), not a freshly recomputed waypoint, so the FC's normal reporting lag
    // behind our own continuously-paced target is not mistaken for an override.
    bool goto_overridden(double margin_m) const {
        std::lock_guard<std::mutex> lk(mtx_);
        if (!goto_active_ || !pos_target_known_) return false;
        if (sec(Clock::now() - last_pos_target_) > 1.0) return false;        // stale report: nothing to compare against
        double dn = pos_target_n_ - goto_dest_.n, de = pos_target_e_ - goto_dest_.e, dd = pos_target_d_ - goto_dest_.d;
        return std::sqrt(dn * dn + de * de + dd * dd) > margin_m;
    }
    // status: fl-independent; box normalised (top-left, bottom-right), NaN when unused
    void set_tracking_status(uint8_t flags, double x0, double y0, double x1, double y1) {
        std::lock_guard<std::mutex> lk(mtx_);
        trk_flags_ = flags; trk_box_[0] = x0; trk_box_[1] = y0; trk_box_[2] = x1; trk_box_[3] = y1;
    }

    // ---- exit: 3 x zero velocity, stop the thread ---------------------------------------------------
    void shutdown() {
        bool was = running_.exchange(false);
        if (was) {
            if (thread_.joinable()) thread_.join();
            for (int i = 0; i < 3; i++) { send_velocity(fl::Cmd()); std::this_thread::sleep_for(std::chrono::milliseconds(20)); }
            close_transport();
        }
    }

private:
    static LinkConfig make_cfg(const std::string& conn) { LinkConfig c; c.connection = conn; return c; }
    static double sec(Clock::duration d) { return std::chrono::duration<double>(d).count(); }
    static bool is_multirotor(uint8_t t) { return t == MAV_TYPE_QUADROTOR || t == MAV_TYPE_HEXAROTOR || t == MAV_TYPE_OCTOROTOR || t == MAV_TYPE_TRICOPTER || t == MAV_TYPE_DECAROTOR; }

    // ---- transport ----------------------------------------------------------------------------------
    enum class Kind { UDP_IN, UDP_OUT, TCP, SERIAL };
    static speed_t baud_const(int b) {
        switch (b) { case 9600: return B9600; case 19200: return B19200; case 38400: return B38400; case 57600: return B57600;
                     case 115200: return B115200; case 230400: return B230400; case 460800: return B460800; case 921600: return B921600;
                     case 1500000: return B1500000; default: throw std::runtime_error("unsupported serial baud " + std::to_string(b)); }
    }
    void parse_connection() {
        std::string s = cfg_.connection;
        if (s.rfind("udp:", 0) == 0) s = "udpin:" + s.substr(4);          // old convention
        auto split_hp = [](const std::string& hp, std::string& host, int& port) {
            auto c = hp.rfind(':');
            if (c == std::string::npos) throw std::runtime_error("expected HOST:PORT in '" + hp + "'");
            host = hp.substr(0, c); port = std::stoi(hp.substr(c + 1));
        };
        if (s.rfind("udpin:", 0) == 0) { kind_ = Kind::UDP_IN; split_hp(s.substr(6), host_, port_); }
        else if (s.rfind("udpout:", 0) == 0) { kind_ = Kind::UDP_OUT; split_hp(s.substr(7), host_, port_); }
        else if (s.rfind("tcp:", 0) == 0) { kind_ = Kind::TCP; split_hp(s.substr(4), host_, port_); }
        else {
            kind_ = Kind::SERIAL;
            std::string rest = s.rfind("serial:", 0) == 0 ? s.substr(7) : s;
            auto c = rest.rfind(':');
            if (c == std::string::npos) { dev_ = rest; baud_ = 115200; }
            else { dev_ = rest.substr(0, c); baud_ = std::stoi(rest.substr(c + 1)); }
            if (dev_.empty() || dev_[0] != '/') throw std::runtime_error("unrecognised connection string '" + cfg_.connection +
                "' (use udpin:HOST:PORT, udpout:HOST:PORT, tcp:HOST:PORT or serial:/dev/serial0:921600)");
        }
    }
    void open_transport() {
        parse_connection();
        if (kind_ == Kind::UDP_IN || kind_ == Kind::UDP_OUT) {
            fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
            if (fd_ < 0) throw std::runtime_error("socket() failed");
            sockaddr_in a{}; a.sin_family = AF_INET;
            if (::inet_pton(AF_INET, host_.c_str(), &a.sin_addr) != 1) throw std::runtime_error("bad IPv4 address '" + host_ + "'");
            a.sin_port = htons((uint16_t)port_);
            if (kind_ == Kind::UDP_IN) {
                int one = 1; ::setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &one, sizeof one);
                if (::bind(fd_, (sockaddr*)&a, sizeof a) < 0) throw std::runtime_error("bind " + host_ + ":" + std::to_string(port_) + " failed: " + std::strerror(errno));
            } else { peer_ = a; have_peer_ = true; sockaddr_in any{}; any.sin_family = AF_INET; ::bind(fd_, (sockaddr*)&any, sizeof any); }
        } else if (kind_ == Kind::TCP) {
            connect_tcp();
        } else {
            fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
            if (fd_ < 0) throw std::runtime_error("cannot open serial port " + dev_ + ": " + std::strerror(errno));
            termios tio{};
            if (::tcgetattr(fd_, &tio) != 0) throw std::runtime_error("tcgetattr failed on " + dev_);
            ::cfmakeraw(&tio);
            speed_t sp = baud_const(baud_);
            ::cfsetispeed(&tio, sp); ::cfsetospeed(&tio, sp);
            tio.c_cflag |= (CLOCAL | CREAD); tio.c_cflag &= ~(CSTOPB | CRTSCTS);       // 8N1, no flow control
            tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 0;
            if (::tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("tcsetattr failed on " + dev_);
        }
        std::printf("[MAV] link %s (sysid %d, compid %d)\n", cfg_.connection.c_str(), cfg_.sysid, cfg_.compid);
    }
    void connect_tcp() {
        fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) throw std::runtime_error("socket() failed");
        sockaddr_in a{}; a.sin_family = AF_INET; a.sin_port = htons((uint16_t)port_);
        if (::inet_pton(AF_INET, host_.c_str(), &a.sin_addr) != 1) throw std::runtime_error("bad IPv4 address '" + host_ + "'");
        if (::connect(fd_, (sockaddr*)&a, sizeof a) < 0) { ::close(fd_); fd_ = -1; throw std::runtime_error("tcp connect failed: " + std::string(std::strerror(errno))); }
        ::fcntl(fd_, F_SETFL, ::fcntl(fd_, F_GETFL) | O_NONBLOCK);
    }
    void close_transport() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
    void write_raw(const uint8_t* buf, size_t n) {
        if (fd_ < 0) return;
        std::lock_guard<std::mutex> lk(tx_mtx_);
        if (kind_ == Kind::UDP_IN || kind_ == Kind::UDP_OUT) {
            if (!have_peer_) return;                                           // nobody to answer yet (peer learned from the first packet)
            ::sendto(fd_, buf, n, 0, (sockaddr*)&peer_, sizeof peer_);
        } else {
            size_t off = 0;
            while (off < n) {
                ssize_t w = ::write(fd_, buf + off, n - off);
                if (w < 0) { if (errno == EAGAIN || errno == EINTR) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; } return; }
                off += (size_t)w;
            }
        }
    }
    void send_msg(const mavlink_message_t& m) { uint8_t buf[MAVLINK_MAX_PACKET_LEN]; uint16_t n = mavlink_msg_to_send_buffer(buf, &m); write_raw(buf, n); }

    // ---- outgoing messages --------------------------------------------------------------------------
    uint32_t boot_ms() const { return (uint32_t)(sec(Clock::now() - t_start_) * 1000.0); }
    void send_heartbeat(uint8_t comp, uint8_t type) {
        mavlink_message_t m; mavlink_msg_heartbeat_pack(cfg_.sysid, comp, &m, type, MAV_AUTOPILOT_INVALID, 0, 0, MAV_STATE_ACTIVE); send_msg(m);
    }
    void send_command_long(uint8_t tsys, uint8_t tcomp, uint16_t cmd, uint8_t confirm, float p1 = 0, float p2 = 0, float p3 = 0, float p4 = 0, float p5 = 0, float p6 = 0, float p7 = 0) {
        mavlink_message_t m; mavlink_msg_command_long_pack(cfg_.sysid, cfg_.compid, &m, tsys, tcomp, cmd, confirm, p1, p2, p3, p4, p5, p6, p7); send_msg(m);
    }
    void send_velocity(const fl::Cmd& c) {
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid ? vs_.sysid : cfg_.sysid; tc = vs_.compid ? vs_.compid : 1; }
        mavlink_message_t m;
        mavlink_msg_set_position_target_local_ned_pack(cfg_.sysid, cfg_.compid, &m, boot_ms(), ts, tc, MAV_FRAME_BODY_NED, (uint16_t)cfg_.type_mask,
            0, 0, 0, (float)c.vx, (float)c.vy, (float)c.vz, 0, 0, 0, 0, (float)c.yaw_rate);
        send_msg(m);
    }
    // Position go-to: SET_POSITION_TARGET_LOCAL_NED, frame LOCAL_NED (1), mask 2552 = ignore vx,vy,vz,ax,ay,az,yaw_rate (position
    // and yaw are used); MAV_CMD_DO_CHANGE_SPEED (178), param1 SPEED_TYPE_GROUNDSPEED (1), param2 = speed m/s, param3 -1 (no throttle change).
    void send_goto_position(const fl::Ned& d, double yaw_rad, bool vel_only = false, const fl::Ned& v = fl::Ned{}, const fl::Ned& a = fl::Ned{}) {
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid ? vs_.sysid : cfg_.sysid; tc = vs_.compid ? vs_.compid : 1; }
        mavlink_message_t m;
        // 2552: position + yaw; 2503: velocity + yaw; 2055: velocity + ACCELERATION + yaw (the GO with feed-forward)
        const bool use_acc = vel_only && (a.n != 0 || a.e != 0 || a.d != 0);
        mavlink_msg_set_position_target_local_ned_pack(cfg_.sysid, cfg_.compid, &m, boot_ms(), ts, tc, MAV_FRAME_LOCAL_NED, vel_only ? (use_acc ? 2055 : 2503) : 2552,
            (float)d.n, (float)d.e, (float)d.d, vel_only ? (float)v.n : 0.f, vel_only ? (float)v.e : 0.f, vel_only ? (float)v.d : 0.f,
            use_acc ? (float)a.n : 0.f, use_acc ? (float)a.e : 0.f, use_acc ? (float)a.d : 0.f, (float)yaw_rad, 0);
        send_msg(m);
    }
    void send_goto_speed(double speed_ms) {
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid; tc = vs_.compid; }
        send_command_long(ts, tc, MAV_CMD_DO_CHANGE_SPEED, 0, 1.0f /*SPEED_TYPE_GROUNDSPEED*/, (float)speed_ms, -1.0f);
    }
    void send_param_set(const char* param_id, float value) {
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid ? vs_.sysid : cfg_.sysid; tc = vs_.compid ? vs_.compid : 1; }
        char id_buf[17] = {0};
        std::strncpy(id_buf, param_id, 16);
        mavlink_message_t m;
        mavlink_msg_param_set_pack(cfg_.sysid, cfg_.compid, &m, ts, tc, id_buf, value, MAV_PARAM_TYPE_REAL32);
        send_msg(m);
    }
    void request_streams() {                               // MAV_CMD_SET_MESSAGE_INTERVAL: id, interval us, ..., param7 = 0
        struct S { int id; int us; };
        const S want[] = {{MAVLINK_MSG_ID_RC_CHANNELS, 100000}, {MAVLINK_MSG_ID_ATTITUDE, 20000}, {MAVLINK_MSG_ID_LOCAL_POSITION_NED, 50000},   // 50 / 20 Hz: the centring uses the yaw at each frame's time (10 Hz left it up to 100 ms stale)
                          {MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000}, {MAVLINK_MSG_ID_GPS_RAW_INT, 500000}, {MAVLINK_MSG_ID_EKF_STATUS_REPORT, 500000},
                          {MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED, 200000},
                          {MAVLINK_MSG_ID_DISTANCE_SENSOR, 100000},      // downward rangefinder, if fitted (--height-source)
                          {MAVLINK_MSG_ID_SYS_STATUS, 1000000},
                          {MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS, 100000}};   // the autopilot's mount (--gimbal-centre)         // battery, for the rescue's minimum-battery gate   // to detect a QGC go-to/orbit/ROI fighting our own go-to (see goto_overridden)
        uint8_t ts, tc; { std::lock_guard<std::mutex> lk(mtx_); ts = vs_.sysid; tc = vs_.compid; }
        for (const S& s : want) send_command_long(ts, tc, MAV_CMD_SET_MESSAGE_INTERVAL, 0, (float)s.id, (float)s.us);
    }

    // ---- incoming messages --------------------------------------------------------------------------
    // The link is alive while ANY message from the autopilot keeps coming (ATTITUDE alone is 50 Hz) -- a heartbeat-only check
    // (1 Hz) read every late heartbeat as a dead link. Heartbeats must still come within 2.5x the timeout (the mode they carry
    // must not be stale). Called with mtx_ held.
    bool link_alive_locked(Clock::time_point now) const {
        return vs_.known && sec(now - last_any_) <= cfg_.hb_timeout_s && sec(now - last_hb_) <= std::max(2.5 * cfg_.hb_timeout_s, 5.0);
    }
    void handle(const mavlink_message_t& m) {
        { std::lock_guard<std::mutex> lk(mtx_); if (vs_.known && m.sysid == vs_.sysid && m.compid == vs_.compid) last_any_ = Clock::now(); }
        if (m.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
            mavlink_heartbeat_t h; mavlink_msg_heartbeat_decode(&m, &h);
            if (h.autopilot == MAV_AUTOPILOT_INVALID || !is_multirotor(h.type)) return;     // only the flight controller counts
            bool regained = false;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                if (vs_.known && m.sysid != vs_.sysid) return;
                regained = !vs_.known || sec(Clock::now() - last_hb_) > cfg_.hb_timeout_s;
                vs_.known = true; vs_.sysid = m.sysid; vs_.compid = m.compid; last_hb_ = last_any_ = Clock::now();
                vs_.custom_mode = h.custom_mode; vs_.guided = (h.custom_mode == 4); vs_.armed = (h.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
                if (vs_.guided && pend_mode_) { pend_mode_ = false; log("mode GUIDED confirmed"); }
            }
            if (regained) {
                log("vehicle heartbeat " + std::to_string(m.sysid) + ":" + std::to_string(m.compid) + " (link up)");
                request_streams();
                send_param_set("FS_CRASH_CHECK", 0.0f);
                send_param_set("ATC_ANGLE_MAX", 30.0f);
            }
            return;
        }
        std::lock_guard<std::mutex> lk(mtx_);
        switch (m.msgid) {
            case MAVLINK_MSG_ID_ATTITUDE: { mavlink_attitude_t a; mavlink_msg_attitude_decode(&m, &a); vs_.roll = a.roll; vs_.pitch = a.pitch; vs_.yaw = a.yaw; vs_.yawspeed = a.yawspeed; break; }
            case MAVLINK_MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS: {
                // the mount's attitude (earth-frame quaternion, unless YAW_IN_VEHICLE_FRAME): pitch + up, yaw relative to the nose
                mavlink_gimbal_device_attitude_status_t g; mavlink_msg_gimbal_device_attitude_status_decode(&m, &g);
                const double w = g.q[0], x = g.q[1], y = g.q[2], z = g.q[3];
                const double pitch = std::asin(std::clamp(2.0 * (w * y - z * x), -1.0, 1.0));
                double yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
                if (!(g.flags & GIMBAL_DEVICE_FLAGS_YAW_IN_VEHICLE_FRAME)) yaw -= vs_.yaw;      // earth-frame yaw -> relative to the nose
                vs_.mount_pitch_deg = fl::rad2deg(pitch); vs_.mount_yaw_deg = fl::rad2deg(std::atan2(std::sin(yaw), std::cos(yaw)));
                vs_.mount_known = true; last_mount_ = Clock::now(); break; }
            case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: { mavlink_global_position_int_t g; mavlink_msg_global_position_int_decode(&m, &g);
                vs_.alt_m = g.relative_alt / 1000.0; vs_.alt_valid = true;
                if (g.lat != 0 || g.lon != 0) { vs_.lat_deg = g.lat / 1e7; vs_.lon_deg = g.lon / 1e7; vs_.geo_valid = true; } break; }
            case MAVLINK_MSG_ID_LOCAL_POSITION_NED: { mavlink_local_position_ned_t l; mavlink_msg_local_position_ned_decode(&m, &l);
                vs_.vel = {l.vx, l.vy, l.vz}; vs_.pos_n = l.x; vs_.pos_e = l.y; vs_.pos_d = l.z; vs_.pos_valid = true;
                if (!have_gpi_) { vs_.alt_m = -l.z; vs_.alt_valid = true; } break; }
            case MAVLINK_MSG_ID_RC_CHANNELS: { mavlink_rc_channels_t r; mavlink_msg_rc_channels_decode(&m, &r);
                const uint16_t* c = &r.chan1_raw; for (int i = 0; i < 18; i++) vs_.rc[i + 1] = (c[i] == UINT16_MAX) ? 0 : c[i];
                rc_known_ = true; last_rc_ = Clock::now(); break; }
            case MAVLINK_MSG_ID_GPS_RAW_INT: { mavlink_gps_raw_int_t g; mavlink_msg_gps_raw_int_decode(&m, &g); vs_.gps_fix = g.fix_type; vs_.gps_sats = g.satellites_visible; break; }
            case MAVLINK_MSG_ID_EKF_STATUS_REPORT: { mavlink_ekf_status_report_t e; mavlink_msg_ekf_status_report_decode(&m, &e);
                vs_.ekf_ok = fl::ekf_flags_healthy(e.flags); vs_.ekf_known = true; last_ekf_ = Clock::now(); break; }
            case MAVLINK_MSG_ID_COMMAND_ACK: { mavlink_command_ack_t a; mavlink_msg_command_ack_decode(&m, &a);
                if (a.command == MAV_CMD_DO_SET_MODE && a.result != MAV_RESULT_ACCEPTED && a.result != MAV_RESULT_IN_PROGRESS) {
                    // a refusal is final for this request: stop re-sending it, so the caller's next request_guided() starts clean
                    pend_mode_ = false; log("DO_SET_MODE refused, result " + std::to_string(a.result)); } break; }
            case MAVLINK_MSG_ID_COMMAND_LONG: {
                mavlink_command_long_t c; mavlink_msg_command_long_decode(&m, &c);
                if (cfg_.cam_enable) handle_camera_command(m, c);
                handle_op_command(m, c);           // rescue arm/confirm/abort/unlock/brake: addressed to this compid, independent of the camera component
                break;
            }
            case MAVLINK_MSG_ID_DISTANCE_SENSOR: { mavlink_distance_sensor_t d; mavlink_msg_distance_sensor_decode(&m, &d);
                // only a DOWNWARD sensor with an in-range reading; corrected for tilt so it is a vertical height, not a slant range
                if (d.orientation == MAV_SENSOR_ROTATION_PITCH_270 && d.current_distance > d.min_distance && d.current_distance < d.max_distance) {
                    vs_.rng_m = d.current_distance / 100.0 * std::cos(vs_.roll) * std::cos(vs_.pitch); rng_known_ = true; last_rng_ = Clock::now();
                } break; }
            case MAVLINK_MSG_ID_SYS_STATUS: { mavlink_sys_status_t ss; mavlink_msg_sys_status_decode(&m, &ss); vs_.battery_pct = ss.battery_remaining; break; }
            case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED: { mavlink_position_target_local_ned_t p; mavlink_msg_position_target_local_ned_decode(&m, &p);
                if ((p.type_mask & 0x7) == 0) { pos_target_n_ = p.x; pos_target_e_ = p.y; pos_target_d_ = p.z; pos_target_known_ = true; last_pos_target_ = Clock::now(); } break; }
            default: break;
        }
        if (m.msgid == MAVLINK_MSG_ID_GLOBAL_POSITION_INT) have_gpi_ = true;
    }

    // Camera component: the QGC tracking path. Called with mtx_ held.
    void handle_camera_command(const mavlink_message_t& m, const mavlink_command_long_t& c) {
        if (c.target_component != cfg_.cam_compid && c.target_component != 0) return;
        if (cfg_.gcs_sysid != 0 && m.sysid != cfg_.gcs_sysid) return;          // not from our ground station
        if (c.target_system != cfg_.sysid && c.target_system != 0) return;
        auto ack = [&](uint8_t result) { mavlink_message_t a; mavlink_msg_command_ack_pack(cfg_.sysid, cfg_.cam_compid, &a, c.command, result, 0, 0, m.sysid, m.compid); send_msg_locked(a); };
        switch (c.command) {
            case MAV_CMD_REQUEST_MESSAGE:
                ack(MAV_RESULT_ACCEPTED);
                if ((int)c.param1 == MAVLINK_MSG_ID_CAMERA_INFORMATION) send_camera_information();
                else if ((int)c.param1 == MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION) send_video_stream_information();
                else if ((int)c.param1 == MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS) send_tracking_status();
                break;
            case MAV_CMD_REQUEST_CAMERA_INFORMATION: ack(MAV_RESULT_ACCEPTED); send_camera_information(); break;
            case MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION: ack(MAV_RESULT_ACCEPTED); send_video_stream_information(); break;
            case MAV_CMD_SET_MESSAGE_INTERVAL:
                if ((int)c.param1 == MAVLINK_MSG_ID_CAMERA_TRACKING_IMAGE_STATUS) { trk_interval_us_ = std::max(0, (int)c.param2); ack(MAV_RESULT_ACCEPTED); }
                else ack(MAV_RESULT_UNSUPPORTED);
                break;
            case MAV_CMD_CAMERA_TRACK_POINT: { TrackRequest t; t.kind = TrackRequest::POINT; t.x0 = c.param1; t.y0 = c.param2; t.radius = c.param3; track_q_.push_back(t); ack(MAV_RESULT_ACCEPTED);
                log("QGC TRACK_POINT x=" + std::to_string(c.param1) + " y=" + std::to_string(c.param2)); break; }
            case MAV_CMD_CAMERA_TRACK_RECTANGLE: { TrackRequest t; t.kind = TrackRequest::RECT; t.x0 = c.param1; t.y0 = c.param2; t.x1 = c.param3; t.y1 = c.param4; track_q_.push_back(t); ack(MAV_RESULT_ACCEPTED);
                log("QGC TRACK_RECTANGLE (" + std::to_string(c.param1) + "," + std::to_string(c.param2) + ")-(" + std::to_string(c.param3) + "," + std::to_string(c.param4) + ")"); break; }
            case MAV_CMD_CAMERA_STOP_TRACKING: { TrackRequest t; t.kind = TrackRequest::STOP; track_q_.push_back(t); ack(MAV_RESULT_ACCEPTED); log("QGC STOP_TRACKING"); break; }
            default: break;                                                    // other commands: not ours
        }
    }
    // Rescue control from QGC (a Custom Action or joystick button mapped to one of these, QGC-version-specific -- see
    // docs/FPV_DASH.md): MAV_CMD_USER_1..5 (31010-31014, confirmed against common.xml) addressed to THIS compid (191), not the
    // camera component -- these are vehicle-level commands, independent of --cam-component. Mapped to the same press/abort/
    // unlock inputs as the keyboard/stdin/RC paths (RescueController treats a "press" the same regardless of source: arms
    // when IDLE, confirms when ARMED), plus a direct BRAKE that works even when the rescue isn't armed at all -- a QGC panic
    // button. Called with mtx_ held.
    void handle_op_command(const mavlink_message_t& m, const mavlink_command_long_t& c) {
        // must be addressed to THIS component exactly (no broadcast) and come from the configured ground station
        if (c.target_component != cfg_.compid || c.target_system != cfg_.sysid) return;
        if (cfg_.gcs_sysid != 0 && m.sysid != cfg_.gcs_sysid) {
            if (c.command >= MAV_CMD_USER_1 && c.command <= MAV_CMD_USER_5) log("ignored MAV_CMD_USER_* from sysid " + std::to_string(m.sysid) + " (not --gcs-sysid)");
            return;
        }
        auto ack = [&](uint8_t result) { mavlink_message_t a; mavlink_msg_command_ack_pack(cfg_.sysid, cfg_.compid, &a, c.command, result, 0, 0, m.sysid, m.compid); send_msg_locked(a); };
        OpCmd op;
        switch (c.command) {
            case MAV_CMD_USER_1: op = OpCmd::PRESS; break;
            case MAV_CMD_USER_2: op = OpCmd::ABORT; break;
            case MAV_CMD_USER_3: op = OpCmd::UNLOCK; break;
            case MAV_CMD_USER_4: op = OpCmd::BRAKE; break;
            case MAV_CMD_USER_5: op = OpCmd::LOCK; break;
            default: return;                                                   // anything else: not ours
        }
        // a RETRY of a command already taken (QGC resends with confirmation > 0 when an ACK is lost): ACK again, do not act twice --
        // two GO presses arm AND confirm the dash
        const auto tnow = Clock::now();
        if (c.confirmation > 0 && c.command == last_op_cmd_ && sec(tnow - last_op_at_) < 3.0) { ack(MAV_RESULT_ACCEPTED); return; }
        last_op_cmd_ = c.command; last_op_at_ = tnow;
        op_q_.push_back(op); op_p_.push_back({(double)c.param1, (double)c.param2}); ack(MAV_RESULT_ACCEPTED);
    }
    void send_msg_locked(const mavlink_message_t& m) { send_msg(m); }          // tx has its own mutex; mtx_ may be held by the caller
    static void pad32(char* dst, const char* s) { std::memset(dst, 0, 32); std::strncpy(dst, s, 32); }
    void send_camera_information() {
        mavlink_message_t m; char vendor[32], model[32]; pad32(vendor, "Companion"); pad32(model, "FollowCam");
        uint32_t flags = CAMERA_CAP_FLAGS_CAPTURE_VIDEO | CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM | CAMERA_CAP_FLAGS_HAS_TRACKING_POINT | CAMERA_CAP_FLAGS_HAS_TRACKING_RECTANGLE;
        uint8_t vend[32], mod[32]; std::memcpy(vend, vendor, 32); std::memcpy(mod, model, 32);
        char uri[140] = {0};
        mavlink_msg_camera_information_pack(cfg_.sysid, cfg_.cam_compid, &m, boot_ms(), vend, mod, 0x01000000, NAN, NAN, NAN, (uint16_t)cfg_.cam_w, (uint16_t)cfg_.cam_h, 0, flags, 0, uri, 0, 0);
        send_msg(m);
    }
    void send_video_stream_information() {
        mavlink_message_t m; char name[32]; pad32(name, "Companion video");
        char uri[160] = {0}; std::strncpy(uri, cfg_.cam_video_uri.c_str(), 159);
        mavlink_msg_video_stream_information_pack(cfg_.sysid, cfg_.cam_compid, &m, 1, 1, cfg_.cam_video_rtsp ? VIDEO_STREAM_TYPE_RTSP : VIDEO_STREAM_TYPE_RTPUDP,
            VIDEO_STREAM_STATUS_FLAGS_RUNNING, (float)cfg_.cam_fps, (uint16_t)cfg_.cam_w, (uint16_t)cfg_.cam_h, 4000, 0, 90, name, uri, 0, 0);
        send_msg(m);
    }
    void send_tracking_status() {                                              // called with mtx_ held
        mavlink_message_t m;
        const double nan = (double)NAN;
        double x0 = trk_box_[0], y0 = trk_box_[1], x1 = trk_box_[2], y1 = trk_box_[3];
        bool idle = (trk_flags_ == CAMERA_TRACKING_STATUS_FLAGS_IDLE) || !std::isfinite(x0);
        mavlink_msg_camera_tracking_image_status_pack(cfg_.sysid, cfg_.cam_compid, &m, idle ? 0 : trk_flags_, idle ? 0 : CAMERA_TRACKING_MODE_RECTANGLE,
            CAMERA_TRACKING_TARGET_DATA_IN_STATUS, (float)nan, (float)nan, (float)nan, idle ? (float)nan : (float)x0, idle ? (float)nan : (float)y0,
            idle ? (float)nan : (float)x1, idle ? (float)nan : (float)y1, 0);
        send_msg(m);
    }

    void log(const std::string& s) const { std::printf("[MAV] %s\n", s.c_str()); std::fflush(stdout); }

    // ---- the I/O thread: rx + heartbeats + 20 Hz tick, sleeping in poll() until data or the next deadline -----
    void run() {
        auto next_tick = Clock::now(), next_hb = Clock::now(), next_trk = Clock::now();
        const auto tick_period = std::chrono::microseconds(1000000 / std::max(1, cfg_.tick_hz));
        mavlink_status_t status{}; mavlink_message_t msg;
        uint8_t buf[1024];
        while (running_) {
            auto now = Clock::now();
            auto deadline = std::min(next_tick, next_hb);
            int timeout_ms = (int)std::max<long>(0, std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
            if (fd_ >= 0) {
                pollfd p{fd_, POLLIN, 0};
                int r = ::poll(&p, 1, std::min(timeout_ms, 50));
                if (r > 0 && (p.revents & POLLIN)) {
                    ssize_t n;
                    if (kind_ == Kind::UDP_IN || kind_ == Kind::UDP_OUT) {
                        sockaddr_in from{}; socklen_t fl_ = sizeof from;
                        n = ::recvfrom(fd_, buf, sizeof buf, 0, (sockaddr*)&from, &fl_);
                        if (n > 0 && kind_ == Kind::UDP_IN) { std::lock_guard<std::mutex> lk(tx_mtx_); peer_ = from; have_peer_ = true; }   // re-learn on every packet
                    } else n = ::read(fd_, buf, sizeof buf);
                    if (n > 0) {
                        for (ssize_t i = 0; i < n; i++) {
                            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) handle(msg);
                        }
                    } else if (n == 0 && kind_ == Kind::TCP) {
                        reconnect();
                    }
                } else if (r > 0 && (p.revents & (POLLHUP | POLLERR))) { if (kind_ == Kind::TCP) reconnect(); else std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
            } else { std::this_thread::sleep_for(std::chrono::milliseconds(50)); if (kind_ == Kind::TCP) reconnect(); }
            now = Clock::now();
            if (now >= next_hb) {                                              // heartbeats: only while the main loop is alive
                next_hb = now + std::chrono::seconds(1);
                bool alive; { std::lock_guard<std::mutex> lk(mtx_); alive = sec(now - liveness_) < 1.0; }
                if (alive) { send_heartbeat(cfg_.compid, MAV_TYPE_ONBOARD_CONTROLLER); if (cfg_.cam_enable) send_heartbeat(cfg_.cam_compid, MAV_TYPE_CAMERA); }
                else if (!stall_logged_) { log("main loop stalled: heartbeat stopped"); stall_logged_ = true; }
                if (alive) stall_logged_ = false;
            }
            if (cfg_.cam_enable) {
                std::lock_guard<std::mutex> lk(mtx_);
                if (trk_interval_us_ > 0 && now >= next_trk) { next_trk = now + std::chrono::microseconds(trk_interval_us_); send_tracking_status(); }
            }
            if (now >= next_tick) { next_tick += tick_period; if (next_tick < now) next_tick = now + tick_period; tick(now, std::chrono::duration<double>(tick_period).count()); }
        }
    }
    void reconnect() {
        close_transport();
        try {
            connect_tcp(); log("tcp reconnected");
            // several reconnects within 10 s: another companion program is connected to the same link and the two keep taking it
            // from each other -- measured: a leftover second tracker made a 5 m/s GO crawl at 1-2 m/s (half its commands lost)
            const auto t = Clock::now();
            reconnects_.push_back(t); while (!reconnects_.empty() && sec(t - reconnects_.front()) > 10.0) reconnects_.pop_front();
            if (reconnects_.size() >= 3 && sec(t - last_dup_warn_) > 10.0) {
                last_dup_warn_ = t;
                log("WARNING: the link keeps reconnecting -- is ANOTHER target_tracker running? Close it: two companions fight over the vehicle");
            }
        } catch (const std::exception&) { fd_ = -1; std::this_thread::sleep_for(std::chrono::milliseconds(500)); }
    }
    std::deque<Clock::time_point> reconnects_; Clock::time_point last_dup_warn_{};

    // 20 Hz: pending mode change, then the (gated, slewed) velocity command.
    void tick(Clock::time_point now, double dt) {
        fl::Cmd out; bool send = false;
        uint8_t ts = 0, tc = 0; bool do_mode = false; int confirm = 0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            bool link_ok = link_alive_locked(now);
            if (pend_mode_) {
                if (sec(now - pend_start_) > 2.0) { pend_mode_ = false; log("GUIDED not confirmed within 2 s"); }
                else if (pend_sent_ < 3 && vs_.known && sec(now - pend_last_) >= 0.5) { do_mode = true; confirm = pend_sent_++; pend_last_ = now; ts = vs_.sysid; tc = vs_.compid; }
            }
            if (session_ || now < zero_until_) {
                if (vs_.guided && link_ok) {
                    if (session_) {
                        double age = sec(now - want_at_);
                        fl::Cmd gated = fl::gate_command(want_, allowed_, vs_.guided, link_ok, age, cfg_.stale_s);
                        if (gated.is_zero()) { slew_.reset(); out = fl::Cmd(); }       // zero/emergency: immediate, no ramp
                        else out = slew_.step(fl::clamp_keep_direction(gated, cfg_.ceilings), dt, cfg_.ceilings);
                    } else { slew_.reset(); out = fl::Cmd(); }
                    send = true;
                } else { slew_.reset(); }   // link down or outside GUIDED (track-only lock, or the pilot took over): re-engaging starts from zero, not from a stale ramp
            }
        }
        int urgent = -1, urgent_confirm = 0;
        { std::lock_guard<std::mutex> lk(mtx_);
          if (urgent_left_ > 0) { urgent = urgent_mode_; urgent_confirm = 3 - urgent_left_; urgent_left_--; ts = vs_.sysid; tc = vs_.compid; pend_mode_ = false; do_mode = false; } }
        if (urgent >= 0) send_command_long(ts, tc, MAV_CMD_DO_SET_MODE, (uint8_t)urgent_confirm, (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, (float)urgent);
        if (do_mode) send_command_long(ts, tc, MAV_CMD_DO_SET_MODE, (uint8_t)confirm, (float)MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, 4.0f);
        if (send) send_velocity(out);

        // go-to-target: periodic resend (packets can drop; GUID_TIMEOUT's exact scope for a position-only target is undocumented,
        // so resending is the safe choice) until the pilot leaves GUIDED -- nothing else ever cancels or replaces it automatically.
        bool want_speed = false, want_pos = false, gvel = false; fl::Ned gd{}, gv{}, ga{}; double gy = 0, gs = 0;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            if (goto_active_) {
                if (!vs_.guided) { goto_active_ = false; log("left GUIDED: go-to resend stopped"); }
                else {
                    gd = goto_dest_; gy = goto_yaw_; gs = goto_speed_; gv = goto_vel_; ga = goto_acc_;
                    // the velocity only while the pursuit keeps refreshing it (a stale one would keep moving the setpoint): else a hold
                    gvel = goto_has_vel_ && sec(now - goto_vel_at_) < 1.0;
                    if (goto_has_vel_ && !gvel) {                         // the pursuit stopped commanding: hold at target
                        goto_has_vel_ = false; want_pos = true;
                        if (goto_dest_.n == 0 && goto_dest_.e == 0 && goto_dest_.d == 0) goto_dest_ = stop_point_locked();
                        gd = goto_dest_; gy = vs_.yaw;
                    }
                    if (sec(now - goto_speed_last_) >= 2.0) { want_speed = true; goto_speed_last_ = now; }
                    if (sec(now - goto_pos_last_) >= 1.0) { want_pos = true; goto_pos_last_ = now; }
                }
            }
        }
        if (want_speed) send_goto_speed(gs);
        if (want_pos) send_goto_position(gd, gy, gvel, gv, gvel ? ga : fl::Ned{});
        bool stop = false; double sy = 0;
        { std::lock_guard<std::mutex> lk(mtx_); if (stop_pending_ && vs_.link_ok) { stop = true; sy = stop_yaw_; } stop_pending_ = false; }
        if (stop) for (int i = 0; i < 2; i++) send_goto_position(fl::Ned{}, sy, true, fl::Ned{});   // velocity 0 (twice: UDP)
        bool hold = false; fl::Ned ha{}; double hy = 0;
        { std::lock_guard<std::mutex> lk(mtx_); if (hold_pending_ && vs_.guided) { hold = true; ha = hold_at_; hy = hold_yaw_; } hold_pending_ = false; }
        if (hold) send_goto_position(ha, hy);
    }

    LinkConfig cfg_;
    Kind kind_ = Kind::UDP_IN; std::string host_, dev_; int port_ = 0, baud_ = 115200;
    int fd_ = -1; sockaddr_in peer_{}; bool have_peer_ = false;
    std::mutex tx_mtx_;
    bool goto_active_ = false, goto_has_vel_ = false; fl::Ned goto_dest_, goto_vel_, goto_acc_; double goto_yaw_ = 0, goto_speed_ = 0;
    Clock::time_point goto_vel_at_{}; bool hold_pending_ = false; fl::Ned hold_at_; double hold_yaw_ = 0;
    bool stop_pending_ = false; double stop_yaw_ = 0;
    uint16_t last_op_cmd_ = 0; Clock::time_point last_op_at_{};
    Clock::time_point goto_speed_last_{}, goto_pos_last_{};
    mutable std::mutex mtx_;
    VehicleState vs_; Clock::time_point last_hb_{}, last_any_{}, last_rc_{}, last_ekf_{}, last_rng_{}, last_mount_{}, t_start_{}, liveness_{}; bool rc_known_ = false, rng_known_ = false;
    bool have_gpi_ = false, stall_logged_ = false;
    fl::Cmd want_; bool allowed_ = false; Clock::time_point want_at_{}; bool session_ = false; Clock::time_point zero_until_{}; fl::Slew slew_;
    int urgent_mode_ = 0, urgent_left_ = 0;   // send_mode_change(): BRAKE/LOITER, sent from the tick, overrides a pending GUIDED request
    bool pend_mode_ = false; int pend_sent_ = 0; Clock::time_point pend_start_{}, pend_last_{};
    std::deque<TrackRequest> track_q_; uint8_t trk_flags_ = 0; double trk_box_[4] = {NAN, NAN, NAN, NAN}; int trk_interval_us_ = 0;
    std::deque<OpCmd> op_q_; std::deque<std::array<double, 2>> op_p_;
    double pos_target_n_ = 0, pos_target_e_ = 0, pos_target_d_ = 0; bool pos_target_known_ = false; Clock::time_point last_pos_target_{};
    std::atomic<bool> running_{false};
    std::thread thread_;
};



// ============================================================
// ---- video capture: interface
// ============================================================
// Common interface so main.cpp doesn't care which backend is in use.
class ICameraCapture {
public:
    virtual ~ICameraCapture() = default;
    virtual bool isOpened() const = 0;
    virtual bool read(cv::Mat& out) = 0;
    virtual void release() = 0;
};

// Thin wrapper around cv::VideoCapture -- covers usb/opencv/csi/rtsp-in,
// since in native C++ (unlike the Python port) OpenCV's own GStreamer
// backend handles all of these through the same VideoCapture(pipeline,
// CAP_GSTREAMER) call, no PyGObject fallback juggling required.
class OpenCvCapture : public ICameraCapture {
public:
    explicit OpenCvCapture(cv::VideoCapture cap) : cap_(std::move(cap)) {}
    bool isOpened() const override { return cap_.isOpened(); }
    bool read(cv::Mat& out) override {
        bool ok = cap_.read(out);
        return ok && !out.empty();
    }
    void release() override { cap_.release(); }

private:
    cv::VideoCapture cap_;
};

// Wraps any ICameraCapture so a stall anywhere in the underlying backend (a GStreamer/decoder hiccup --
// confirmed live via the JSON log's per-stage timing: "cap":6156ms against a normal ~100ms, with
// trk/ctl/draw all ~0, meaning the stall was entirely inside cap->read()) cannot block the main
// tracker/control loop. The main loop calls link->liveness() once per iteration; the MAVLink heartbeat
// watchdog is an INTENTIONAL, SEPARATE safety check on that loop (it must stop heartbeating if the real
// control loop ever freezes) -- but a slow camera is not a frozen control loop, and must not be confused
// with one.
//
// IMPORTANT: the capture is opened by a FACTORY run on the background thread, not already-constructed and
// handed in. A live repro found the main thread stuck inside cv::VideoCapture's own constructor (confirmed
// via a line-buffered run: it printed "Opening GStreamer UDP stream..." and nothing else for 20+ seconds,
// outliving SIGTERM) -- OpenCV's GStreamer backend blocks there, unbounded, waiting for the pipeline to
// reach PLAYING and preroll its first buffer. That call happens before any wrapper can exist if the caller
// constructs the inner capture itself, so the exact same class of problem (an unbounded block on a
// GStreamer hiccup) can freeze the main thread at open time, not just during later read()s. Taking a
// factory instead means EVERY blocking call into the backend -- open included -- happens on the background
// thread.
//
// A dedicated thread owns the ONLY access to the real capture object: it opens it, then keeps calling its
// (possibly blocking) read(). This wrapper's own read() only ever touches a small latest-frame buffer
// guarded by its own mutex, so it is always non-blocking: it returns the newest frame not yet consumed, or
// false if there isn't one yet (nothing new this tick, or not opened yet) -- the same signal the main loop
// already handles (a 5ms sleep and continue), so the existing frame-gap / video-stalled detection is
// unaffected. isOpened() is a plain atomic, set by the background thread before and after each blocking
// call, so checking it never contends with one that is in flight.
//
// UPDATE: confirmed live (not theoretical) that cv::VideoCapture's constructor can stay blocked inside the
// factory for an extreme, effectively unbounded time (one run stayed blocked for 53+ minutes). release()
// used to call inner_->release() then unconditionally thread_.join() -- correct once inner_ exists, but if
// the factory() call itself is STILL blocked, inner_ was never assigned, so there is nothing to force-release,
// and join() would wait forever. All mutable state now lives in a separate, shared_ptr-held Block so that if
// release() has to give up and detach() instead of join(), the background thread keeps its OWN reference to
// that Block and can safely keep running (and safely publish frames, if the blocked call ever does return)
// without ever touching a destroyed ThreadedCaptureWrapper object.
class ThreadedCaptureWrapper : public ICameraCapture {
public:
    using Factory = std::function<std::unique_ptr<ICameraCapture>()>;
    explicit ThreadedCaptureWrapper(Factory factory) : block_(std::make_shared<Block>()) {
        block_->running = true;
        std::shared_ptr<Block> b = block_;                     // the thread keeps its own ref, independent of this wrapper's lifetime
        thread_ = std::thread([b, factory = std::move(factory)]() mutable { run(b, factory); });
    }
    ~ThreadedCaptureWrapper() override { release(); }
    ThreadedCaptureWrapper(const ThreadedCaptureWrapper&) = delete;
    ThreadedCaptureWrapper& operator=(const ThreadedCaptureWrapper&) = delete;

    bool isOpened() const override { return block_->opened.load(); }   // plain atomic: never behind a lock a blocked open()/read() might hold

    bool read(cv::Mat& out) override {
        std::lock_guard<std::mutex> lk(block_->frame_mtx);
        if (block_->seq == block_->consumed_seq) return false;        // no new frame since the last call -- non-blocking, same as "nothing yet"
        out = block_->latest;
        block_->consumed_seq = block_->seq;
        return true;
    }

    // Any error from the factory (bad args, device not found) or from a backend that can report one;
    // empty once open succeeds. Lets the caller's own first-frame-wait loop surface a real reason instead
    // of a bare timeout if open() fails outright rather than just being slow.
    std::string open_error() const { std::lock_guard<std::mutex> lk(block_->err_mtx); return block_->open_error; }

    void release() override {
        if (block_->running.exchange(false)) {
            {   // force an in-flight blocking read() to return: calling release() on the SAME cv::VideoCapture-
                // backed object from this thread while the capture thread may be mid-call on it is the standard
                // way to unstick a blocked GStreamer pipeline (tearing it down makes the pending call return
                // rather than block forever). Does nothing if the factory() call itself is still in flight --
                // inner doesn't exist yet in that case, see the wait below.
                std::lock_guard<std::mutex> lk(block_->inner_mtx);
                if (block_->inner) block_->inner->release();
            }
            // Give the thread a bounded grace period to actually exit. The common case (inner existed, the
            // release() above unstuck it) finishes near-instantly. If the factory() call is the thing still
            // blocked, there is no handle to force it to return -- cv::VideoCapture's constructor offers no
            // cancellation until it returns on its own, confirmed live to sometimes take 53+ minutes. Rather
            // than hang this process's shutdown for an indeterminate time, detach(): the thread (and the Block
            // it shares ownership of) keeps running in the background, completely safely, until that call
            // eventually resolves or the process itself exits -- it just no longer blocks this shutdown.
            std::unique_lock<std::mutex> lk(block_->done_mtx);
            bool finished = block_->done_cv.wait_for(lk, std::chrono::seconds(2), [this] { return block_->done; });
            lk.unlock();
            if (finished) { if (thread_.joinable()) thread_.join(); }
            else if (thread_.joinable()) thread_.detach();
        }
    }

private:
    struct Block {
        std::unique_ptr<ICameraCapture> inner;    // touched by run() throughout, and by release() -- both guarded by inner_mtx
        std::mutex inner_mtx;                     // guards inner ONLY; the blocking calls through it happen OUTSIDE this lock
        std::atomic<bool> running{false};
        std::atomic<bool> opened{false};          // false until the background thread's open() actually succeeds
        std::mutex frame_mtx;                     // guards latest/seq/consumed_seq ONLY -- never inner
        cv::Mat latest;
        uint64_t seq = 0, consumed_seq = 0;
        mutable std::mutex err_mtx; std::string open_error;
        std::mutex done_mtx; std::condition_variable done_cv; bool done = false;   // signals run() has returned, for release()'s bounded wait
    };
    static void run(const std::shared_ptr<Block>& b, Factory& factory) {
        std::unique_ptr<ICameraCapture> inner;
        bool threw = false;
        try { inner = factory(); }                       // may block for a long time (pipeline open/preroll) -- fine, not the main thread
        catch (const std::exception& e) { std::lock_guard<std::mutex> lk(b->err_mtx); b->open_error = e.what(); threw = true; }
        if (b->running && !threw) {
            if (!inner || !inner->isOpened()) { std::lock_guard<std::mutex> lk(b->err_mtx); b->open_error = "camera did not open"; }
            else {
                { std::lock_guard<std::mutex> lk(b->inner_mtx); b->inner = std::move(inner); }
                b->opened.store(true);
                while (b->running) {
                    ICameraCapture* p;
                    { std::lock_guard<std::mutex> lk(b->inner_mtx); p = b->inner.get(); }
                    if (!p) break;
                    cv::Mat f;
                    bool got = p->read(f);                 // may also block for seconds (GStreamer/decoder stall) -- fine, not the main thread
                    if (!b->running) break;
                    if (got && !f.empty()) {
                        std::lock_guard<std::mutex> lk(b->frame_mtx);
                        b->latest = f; b->seq++;
                    } else {
                        std::lock_guard<std::mutex> lk(b->inner_mtx);
                        if (b->inner && !b->inner->isOpened()) { b->opened.store(false); break; }
                    }
                }
            }
        }   // else: release() ran while we were still opening -- don't publish inner, just fall through to done
        { std::lock_guard<std::mutex> lk(b->done_mtx); b->done = true; }
        b->done_cv.notify_all();
    }
    std::shared_ptr<Block> block_;
    std::thread thread_;
};

// --camera-source usb: /dev/videoN via V4L2, falling back to the default
// backend if V4L2 isn't available. Requests 1280x720 @ 30fps.
std::unique_ptr<ICameraCapture> open_usb_capture(const std::string& source);

// --camera-source csi: Jetson CSI camera via nvarguscamerasrc.
std::unique_ptr<ICameraCapture> open_csi_capture(int width, int height, double fps,
                                                  int sensor_id, int flip_method);

// --camera-source rtsp: hardware-accelerated GStreamer pipeline (TCP,
// nvv4l2decoder) with a plain FFMPEG-backend fallback (also forced TCP).
std::unique_ptr<ICameraCapture> open_rtsp_capture(const std::string& url, int latency_ms);

// --camera-source gst: read an H.264/RTP stream over UDP (typical
// Gazebo/ArduPilot FPV camera setup).
std::unique_ptr<ICameraCapture> open_gst_udp_capture(int port);


// ============================================================
// ---- video capture: backends (usb / csi / rtsp / gst-udp) and the threaded wrapper
// ============================================================
std::unique_ptr<ICameraCapture> open_usb_capture(const std::string& source) {
    std::printf("[USB] Opening USB camera: %s\n", source.c_str());

    cv::VideoCapture cap;
    bool is_index = !source.empty() &&
                     std::all_of(source.begin(), source.end(), ::isdigit);
    if (is_index) {
        cap.open(std::stoi(source), cv::CAP_V4L2);
    } else {
        cap.open(source, cv::CAP_V4L2);
    }
    if (!cap.isOpened()) {
        cap.release();
        if (is_index) cap.open(std::stoi(source));
        else cap.open(source);
    }
    if (cap.isOpened()) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
        cap.set(cv::CAP_PROP_FPS, 30);
    }
    return std::make_unique<OpenCvCapture>(std::move(cap));
}

std::unique_ptr<ICameraCapture> open_csi_capture(int width, int height, double fps,
                                                  int sensor_id, int flip_method) {
    int fps_num = std::max(1, static_cast<int>(std::round(fps)));
    std::ostringstream pipeline;
    pipeline << "nvarguscamerasrc sensor-id=" << sensor_id << " ! "
             << "video/x-raw(memory:NVMM), width=" << width << ", height=" << height
             << ", format=NV12, framerate=" << fps_num << "/1 ! "
             << "nvvidconv flip-method=" << flip_method << " ! "
             << "video/x-raw, format=BGRx ! videoconvert ! video/x-raw, format=BGR ! "
             << "appsink drop=true max-buffers=1 sync=false";

    std::printf("[CSI] Opening NVIDIA Jetson CSI camera...\n");
    std::printf("[CSI] Pipeline: %s\n", pipeline.str().c_str());

    cv::VideoCapture cap(pipeline.str(), cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        throw std::runtime_error(
            "Could not open the Jetson CSI camera with nvarguscamerasrc. "
            "Make sure the CSI camera is connected and OpenCV has GStreamer support.");
    }
    std::printf("[CSI] NVIDIA Jetson CSI camera opened: %dx%d @ %.1f FPS, sensor-id=%d\n", width,
                height, fps, sensor_id);
    return std::make_unique<OpenCvCapture>(std::move(cap));
}

std::unique_ptr<ICameraCapture> open_rtsp_capture(const std::string& url, int latency_ms) {
    std::printf("[RTSP] Connecting to stream: %s\n", url.c_str());

    // Force TCP transport for the FFMPEG fallback (matches the Python
    // port's fix -- UDP RTSP is frequently silently dropped by networks).
    setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS", "rtsp_transport;tcp", 1);

    std::ostringstream pipeline;
    pipeline << "rtspsrc location=\"" << url << "\" latency=" << latency_ms
             << " protocols=tcp ! rtph264depay ! h264parse ! nvv4l2decoder ! nvvidconv ! "
             << "video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false";

    std::printf("[RTSP] Attempting GStreamer pipeline: %s\n", pipeline.str().c_str());
    cv::VideoCapture cap(pipeline.str(), cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {                                   // not a Jetson: software H.264, then H.265 (the C12's codec is not documented)
        const char* sw[] = {"rtph264depay ! h264parse ! avdec_h264", "rtph265depay ! h265parse ! avdec_h265"};
        for (const char* chain : sw) {
            std::ostringstream p2;
            p2 << "rtspsrc location=\"" << url << "\" latency=" << latency_ms << " protocols=tcp ! " << chain
               << " ! videoconvert ! video/x-raw, format=BGR ! appsink drop=true max-buffers=1 sync=false";
            std::printf("[RTSP] Trying software pipeline: %s\n", p2.str().c_str());
            cap.open(p2.str(), cv::CAP_GSTREAMER);
            if (cap.isOpened()) break;
        }
    }
    if (!cap.isOpened()) {
        std::printf("[RTSP] Hardware GStreamer pipeline failed to open; falling back to "
                    "standard OpenCV RTSP backend (TCP transport).\n");
        cap.release();
        cap.open(url, cv::CAP_FFMPEG);
    }
    if (!cap.isOpened()) {
        throw std::runtime_error("Could not open RTSP stream at: " + url);
    }
    std::printf("[RTSP] RTSP stream opened successfully: %s\n", url.c_str());
    return std::make_unique<OpenCvCapture>(std::move(cap));
}

std::unique_ptr<ICameraCapture> open_gst_udp_capture(int port) {
    std::ostringstream pipeline;
    pipeline << "udpsrc port=" << port
             << " caps=\"application/x-rtp, media=video, encoding-name=H264, payload=96\" "
             << "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=BGR "
             << "! appsink drop=1 sync=false";

    std::printf("[VIDEO] Opening GStreamer UDP stream on port %d...\n", port);
    cv::VideoCapture cap(pipeline.str(), cv::CAP_GSTREAMER);
    if (!cap.isOpened()) {
        std::printf("[VIDEO ERROR] Could not open the UDP stream on port %d.\n", port);
    }
    return std::make_unique<OpenCvCapture>(std::move(cap));
}


// ============================================================
// ---- command line, profiles, entry point
// ============================================================
// Class-agnostic follower: operator or QGC selects a target, CSRT tracks it, the drone follows through MAVLink GUIDED velocity
// commands (image servoing, or the C12 gimbal angles). Camera in: usb / csi / rtsp / gst-udp. See the sections above.




// ---------------------------------------------------------------------
// Args
// ---------------------------------------------------------------------
struct Args {
    std::string connection = "udpin:0.0.0.0:14560";
    std::string camera_source = "usb";
    std::string source = "1";
    std::string rtsp_url;
    int rtsp_latency = 100;
    // sw (default) | hw | auto (hw when nvv4l2h264enc exists). Default stays sw: the previous (YOLO) iteration of this project
    // documented nvv4l2h264enc CRASHING the whole process on some Jetson/L4T/GStreamer combinations -- a crash is not
    // catchable here, and it would take the follow loop down with it. Switch to hw/auto only after a standalone gst-launch
    // test of the encoder succeeds on the exact board image.
    std::string rtsp_out_encoder = "sw";
    int csi_width = 1280, csi_height = 720;
    double csi_fps = 30.0;
    int csi_sensor_id = 0;
    int csi_flip = 0;
    int gst_udp_port = -1;
    // How long to wait for the FIRST usable frame before giving up. Confirmed via GStreamer debug logs
    // (GST_DEBUG=udpsrc:5,appsink:5,h264parse:4) comparing a fast run against a slow one with otherwise
    // identical pipelines: a new consumer's udpsrc binds instantly either way, but h264parse can't resolve
    // the stream's actual resolution/caps -- and appsink can't preroll a frame -- until the NEXT keyframe
    // (SPS/PPS) arrives, which depends on where in the encoder's GOP cycle the bind happened to land. One
    // observed case took just over 20s; the old fixed 15s budget (300 retries x 50ms) gave up 5s before
    // that run's first frame would have arrived. This does not block anything else: link->liveness() keeps
    // running throughout, so the MAVLink heartbeat stays healthy for the whole wait regardless of its length.
    double first_frame_timeout_s = 30.0;

    int rc_high_thresh = 1700, rc_low_thresh = 1300;

    // --- MAVLink link identity / camera component (tracker mode; see MavlinkLink)
    int sysid = 1, compid = 191, type_mask = 1479, gcs_sysid = 255;
    double link_stale_s = 0.3, link_hb_timeout_s = 2.0;
    bool cam_component = false;
    int cam_compid = 100, cam_w = 640, cam_h = 480;
    double cam_fps = 30.0;
    std::string cam_video_uri = "udp://0.0.0.0:5600";
    bool cam_video_rtsp = false;
    double link_test_s = 0;            // >0: connect, print the vehicle state for N seconds, then exit (bring-up tool)
    bool link_request_guided = false;  // with --link-test: ask for GUIDED once the vehicle is seen

    // --- tracker application (class-agnostic follower)
    // Configurable safety ceiling, actually honored (see main(): this used to be silently re-clamped to
    // 3.0/3.0/2.0/0.5 no matter what was passed here, making these flags useless above that regardless of
    // target). Defaults raised to comfortably cover a fast car/bike (~14 m/s) with margin, not just a
    // walking person -- a fixed number still can't fit every target class (a train or a fast drone could
    // exceed even this), so raise these explicitly per mission via --ceil-vx/vy/vz/yaw rather than assuming
    // one default covers everything. This is a genuine safety rail on a flying vehicle: it is intentionally
    // finite, not removed, even though no single number is "right" for every target.
    double ceil_vx = 8.0, ceil_vy = 8.0, ceil_vz = 3.0, ceil_yaw = 1.2;
    double hfov_deg = 114.6, fx = 0, fy = 0;                              // camera: Gazebo gimbal 2.0 rad; C12 visible about 100 deg. fx>0 overrides
    double track_scale = 0.5;                                              // CSRT runs on a copy scaled by this (Phase 0: 1.55x faster, small accuracy loss)
    double csrt_padding = 3.0;
    bool csrt_fast = true;                                                 // adaptive CSRT template (see TrackerEngine); --csrt-default restores OpenCV's defaults
    double csrt_template_factor = 2.5;                                     // template_size = clamp(factor * sqrt(box area), 64, 160)                                             // OpenCV's own CSRT default; raise for fast/crossing targets that overrun the search window
    double kyaw = 1.0, kz = 2.0, kx = 3.0, db_yaw = 0.02, db_z = 0.03, db_x = 0.05;   // UNTUNED starting gains: tune in the simulator
    double klat = 4.0, ki_yaw = 0.0, ki_limit = 0.3;                      // strafe gain; bearing integral gain (0 = off) and its cap (rad/s or m/s)
    std::string lateral_mode = "yaw";                                     // yaw (turn toward the target) | strafe (slide sideways, keep heading)
    // seconds without an accepted box. lost_after was 1.0: shorter than an ordinary occlusion (a car passing in front of a person
    // takes 1-2 s), so the lock was dropped before the occlusion handling could re-acquire. While COASTING the vehicle gets zero
    // velocity (it holds), so waiting 2.5 s before declaring LOST and handing back via LOITER is safe.
    double coast_after = 0.3, lost_after = 2.5, unlock_after = 6.0;
    double alt_floor = 0.0, alt_ceiling = 0;                              // m: no descent at/below the floor, no climb at/above the ceiling (0 = off)
    int detector_port = 0;                                                // >0: the ReID sidecar on 127.0.0.1:PORT (tracker/detector_sidecar.py)
    bool refine_lock = true;
    bool auto_view = false;                // move closer / zoom to keep the locked target big enough in the image (fl::decide_view)
    double view_min_px = 48, view_max_px = 120, view_min_dist = 8, zoom_max = 3, view_approach_speed = 1.5;
    bool gimbal_centre = false;            // tilt the autopilot's gimbal to keep the locked target vertically centred (sim_up.sh --gimbal)
    double gimbal_k = 0.5;                 // fraction of the vertical angle error corrected per update
    bool view_climb = true;                // view control may change height: climb for large / fast targets, back down for small ones
    // FIXED (FPV) camera: the drone itself moves so the locked target sits ON THE + (image centre): it climbs / descends until the
    // target is centred vertically (and turns to centre it sideways). false (--no-centre-target): hold the target where it was in
    // the picture when it was locked instead.
    bool centre_target = true;
    double centre_kx = 4.0;                // fixed-camera centring: m/s of forward/back per radian the target is off the + (--centre-kx)
    double view_max_alt = 30.0;            // ... never above this (m above home), never below the height at the lock
    double view_climb_rate = 1.5;          // m/s
    bool reid = false;
    std::string ai = "auto";                // auto: the models in-process (OpenVINO C++) when built with it, else the sidecar | native | sidecar
    std::string reid_model = "~/models/dinov2";   // the in-process ReID model's directory                     // --reid: DINOv2 fingerprints from the sidecar (detector_sidecar.py --reid DIR) decide re-acquisitions
    std::string yaw_ff_mode = "image";     // image: damping from the image rate (old) | world: the target's turn rate + latency prediction
    double yaw_ff_tau = 0.15;              // smoothing of the target's turn rate (s)
    double yaw_accel = 5.0;                // rad/s^2 turn-rate slew with --yaw-ff-mode world
    double predict_ahead = 0.5;            // s: how far ahead the yellow predicted box is drawn (--predict-ahead; 'k' toggles it)
    double full_res_below = 20.0;          // px (full frame; 40 hurt a 36 px target: IoU 0.89 -> 0.63): smaller targets are tracked at full resolution (--full-res-below, 0 = never)
    std::string go_guidance = "pn";        // GO: pursuit (previous: aims at where the target IS) | lead | intercept | pn (see fl::go_velocity)
    double pn_gain = 3.0;                  // proportional navigation constant N
    double stall_tolerance = 3.0;          // s: a video freeze up to this long does not count against the lock (--stall-tolerance)
    bool gimbal_follow_prediction = false; // the gimbal keeps following the prediction for 0.3 s of doubtful frames (off: holds, no waver)
    bool go_accel_ff = true;               // GO: send the commanded acceleration with the velocity (--no-go-accel-ff)
    double gimbal_db = 1.0;                // vertical centring dead-band, degrees (was 2.5: ~23 px of the 720 px sim view left uncorrected)
    double gimbal_step = 2.5;              // ... but at most this many degrees per 0.1 s update (25 deg/s): bigger steps moved the image
                                           //     faster than the tracker could follow a small person (measured live)
    double yaw_ff = 0.7;                   // 0..1: yaw damping/lead on the target's motion across the image (PD; see the yaw control)
    bool coast_yaw = false;   // (was true; operator: "don't predict or waver for a lost target" -- --coast-yaw turns it back on)                 // while the target is briefly hidden/coasting: keep TURNING toward its predicted position (yaw only)
    bool map_marker = true;                                               // TARGET / DEST icons on QGC's map (ADSB_VEHICLE; ignored by ArduPilot, see send_map_marker)
    bool lock_without_guided = false;                                     // track-only: lock/track without taking the sticks; follow starts when the pilot selects GUIDED
    bool best_view = false;                                               // opt-in autonomous altitude seeking (see fl::best_view_vz)
    double best_view_min_frac = 0.12, best_view_max_frac = 0.35, best_view_gain = 2.0;  // target box height as a fraction of frame height
    std::string log_dir, lock_box, video_out, record_dir, rtsp_out;   // rtsp_out: serve the annotated feed as RTSP (needs gst-rtsp-server at build time)
    int record_bitrate = 4000;                                            // kbps, --record
    int video_out_bitrate = 2000;
    bool no_display = false;
    std::string follow_source = "image";                                   // image = servo on the CSRT box; gimbal = follow the C12 gimbal angles (C12 tracks, CSRT verifies)
    // Camera geometry for the target-position estimate and the follow yaw. "body": fixed to the airframe (vehicle pitch applies);
    // "gimbal": a stabilized gimbal whose pitch is earth-relative. The boresight comes from the C12's attitude push when one is
    // attached and fresh, otherwise from the fixed --cam-pitch-deg / --cam-yaw-deg (+ up / + right of the nose).
    // NOTE (real C12): whether its pushed pitch is earth- or body-relative is still unconfirmed -- that is exactly this switch.
    std::string camera_mount = "body";
    double cam_pitch_deg = 0.0, cam_yaw_deg = 0.0;
    std::string vz_mode = "center";                                        // center: climb/descend to centre the target vertically | hold: altitude is the pilot's
    double min_depression_deg = 10.0;                                      // gimbal mode: ignore the distance estimate below this downward angle
    double target_height = 0;                                              // m: height of the tracked point above the ground (gimbal mode distance estimate)
    std::string c12_host; int c12_port = 5000, c12_w = 1280, c12_h = 720;  // C12 TOP protocol; --c12-host 192.168.144.108 on the real camera
    int rc_centerlock_channel = 0;                                         // 0 = off; high = lock a box at the image centre, low = unlock

    // --- go-to-target: officer confirms, drone computes the target's location and sends ONE go-to command. See the fl:: comment
    // above RescueController. Disabled unless --enable-rescue; the trigger can be a QGC channel (RC override) and/or a physical
    // RC switch -- both map to the same channel below, so either works.
    bool enable_rescue = true;
    double rescue_speed = go_settings::kSpeed, rescue_max_range = 150.0;   // GO speed: edit go_settings::kSpeed (top of file)
    bool go_through = go_settings::kFlyThrough;   // Fly through the target without braking beforehand
    bool go_confirm = false;               // --go-confirm: GO needs two presses (arm, then confirm within 3 s); default one press
    // How far above the target's estimated surface to stop (NOT 0): the raw geometry puts the destination AT the target's
    // own ground level, which for a real person/car/drone/anything the officer can select is a direct collision course, not
    double rescue_hover_height = 0.0;
    std::string height_source = "auto";    // auto | home | rangefinder -- see fl::pick_height
    double rescue_min_battery = 0;         // 0 = unrestricted in sim
    double rescue_override_margin = 5.0;
    bool goto_paced = false;               // --goto-paced: the old GO (one paced line to the point at confirm time, ends --rescue-hover-height above, then holds)
    bool lost_loiter = false;              // target lost while following: switch to LOITER (old behaviour) instead of hovering in place
    double abort_climb = 10.0;             // ABORT: climb this many metres straight up from where the drone is (--abort-climb)
    double goto_accel = go_settings::kAccel;   // GO braking / speed-up, m/s^2 (--goto-accel; edit go_settings::kAccel)
    double goto_standoff = go_settings::kStopOffset;   // GO stop offset, m: + before the target, 0 at it, - past it (--go-offset; edit go_settings::kStopOffset)
    bool zero_decel = false;
    bool fpv_shoot = false;
    bool fixed_horizontal = false;
    double rescue_decel = 0.0;
    bool rescue_lead = false;
    double rescue_max_lead = 50.0;
    bool rescue_stay_over = false;
    // frame of the autopilot gimbal's reported angles (--gimbal-centre): ArduPilot's SERVO mount reports its demanded servo angles,
    // i.e. relative to the airframe ("body"); a self-stabilising gimbal reports where the camera really points ("earth")
    std::string mount_frame = "earth";
    double video_latency = 0.0;            // s the video lags the telemetry (geometry uses the pose from then); sim measured ~0.2
    double est_q_acc = 2.0;                // target ground-track estimator: process noise (m/s^2); lower = smoother, slower velocity
    double gimbal_settle = 0.8;            // s the autopilot gimbal must be still before its reported angle is trusted for geometry
    bool speed_match = true;               // follow: fly at the target's own speed along the line of sight (--no-speed-match: off)
    double speed_match_back = 10.0;        // high-speed back-off limit
    double speed_match_accel = 12.0;       // high-G matching acceleration
    bool fpv = false;                      // FPV chase: fly with the target at its speed (see fl::chase_cmd)
    double fpv_approach = 15.0;            // FPV: max approach / back-off speed
    double fpv_yaw_ff = 0.8;               // FPV: yaw feed-forward of the bearing rate from the target's world track
    bool fpv_lateral = false;              // FPV chase also matches the target's sideways velocity (default: along the line of sight only)
    double fpv_speed = 40.0, fpv_standoff = 10.0, fpv_k = 0.5, fpv_ff = 0.8, fpv_accel = 12.0;
    // opt-in (--box-refit): measured live by the operator, the re-fit grew boxes onto the surroundings (64x20 -> 98x55) and fresh
    // locks were then rejected within seconds -- worse than CSRT alone
    bool box_refit = false;                // re-fit the tracked box with GrabCut 5x a second
    double follow_ff = 0.0;                // 0..1: follow feed-forward of the target's estimated ground velocity (0 = off, ~0.8 typical)   // metres: how far the FC's reported position target must drift from ours before we treat it as a QGC override and stop resending
    // QGC tap-to-lock latency compensation: a pilot's tap is based on the re-encoded video QGC is displaying, which lags this
    // program's live capture by roughly a decode+encode+network round trip. Without compensation, initialising the tracker
    // on the CURRENT frame with coordinates the pilot picked from an OLDER frame puts the box behind a moving target.
    // MUST be measured, not guessed, and the real value on the actual hardware link will differ from the simulator's: play a
    // video with a visible on-screen clock (the overlay always prints one, see "t=" in the first status line) through the
    // real QGC path and compare the displayed time against the wall clock at the moment of the tap.
    double tap_latency_ms = 300.0;
    // 0 = off; a real 3-position switch (ArduPilot RCx_OPTION-style low/mid/high, same PWM thresholds as
    // rc-high-thresh/rc-low-thresh): any move UP = GO (one press), back to low during a dash = stop (BRAKE). With --go-confirm:
    // low->mid = arm (1st press), mid->high = confirm (2nd press) -- a 2-position switch cannot do that (no separate confirm zone).
    int rc_rescue_channel = 0;
    bool rescue_descend = true;                                            // true (default, per the user): fly a direct 3D line to the target's real altitude (ground, or --target-height above it); false: --rescue-level-only keeps the current altitude instead
};

namespace {

std::string arg_value(int argc, char** argv, int& i) {
    if (i + 1 >= argc) throw std::runtime_error(std::string("Missing value for ") + argv[i]);
    return argv[++i];
}

// ---------------------------------------------------------------------
// Profiles: ONE switch between simulation and real hardware. Only connections and camera numbers differ; the program is identical.
//   sim              Gazebo camera (RTP/H.264 on UDP 5602) + ArduPilot SITL port gate (tcp:127.0.0.1:5770); follows the CSRT box
//   sim-gimbal       sim + the simulated C12 on 127.0.0.1:5000 (simulation/sim_c12.py); follows the gimbal angles
//   real-c12         Pi/Jetson on the Cube Orange TELEM2 UART (serial:/dev/serial0:921600) + C12 RTSP (192.168.144.108) + C12 UDP; follows the CSRT box
//   real-c12-gimbal  real-c12, following the C12 gimbal angles (enable only after the bench tests listed in docs/FPV_DASH.md, "Known open items")
// Any explicit option on the command line overrides the profile.
// ---------------------------------------------------------------------
// FPV airframe (--profile sim-fpv / real-fpv): camera FIXED to the airframe, and FPV speed.
// Matched to simulation/fpv.parm (ArduPilot: 30 m/s, 15 m/s^2, 60 deg lean).
inline void apply_fpv_flight(Args& a) {
    a.camera_mount = "body"; a.cam_pitch_deg = -15.0; a.vz_mode = "center";
    a.centre_target = true; a.kz = 2.0; a.centre_kx = 0.0; a.view_climb = false;
    a.ceil_vx = 30.0; a.ceil_vy = 20.0; a.ceil_vz = 10.0; a.ceil_yaw = 4.0;      // high-speed FPV agility
    a.kyaw = 2.0;
    // GO speed / braking / stop offset: from go_settings block at the top of the file
    a.speed_match_accel = 12.0; a.view_approach_speed = 10.0;                  // fast follow and closure
    a.fpv_speed = 30.0; a.fpv_accel = 10.0; a.fpv_approach = 15.0;             // high-speed chase
    a.enable_rescue = true;                                                     // GO enabled by default
    a.go_through = go_settings::kFlyThrough;
    a.goto_accel = go_settings::kAccel;
    a.goto_standoff = go_settings::kStopOffset;
    a.rescue_speed = go_settings::kSpeed;
    a.alt_floor = 0.0;                                                          // no safety floor
    a.rescue_hover_height = 0.0;                                                // no hover above object height
    a.target_height = 0.0;                                                      // 0 = dynamic real-time height from selection (adapts to any object size)
    a.rescue_stay_over = true;                                                  // stay locked over moving object after arrival
}

void apply_profile(Args& a, const std::string& p) {
    if (p == "sim" || p == "sim-gimbal" || p == "sim-c12cam" || p == "sim-fpv") {
        // simulation/sim_up.sh's video relay ALWAYS sends a local copy to 127.0.0.1:5601 (a local copy for the tracker and recordings),
        // regardless of --video-to (which only controls the separate remote-Jetson leg) -- 5601 is therefore the port that
        // actually has a stream on it for a local-only test, not 5600 or any other number.
        //
        // cam_video_uri does NOT default to 5600 here, even though that is QGC's usual video port and the real-c12
        // profile below uses it: in sim, port 5600 is ALREADY bound by sim_up.sh's relay (udpsrc port=5600, receiving
        // Gazebo's raw camera feed). Confirmed live via /proc/net/udp + per-inode process lookup: with QGC ALSO listening
        // on 5600 (its default video port, matching the old cam_video_uri default here) and --video-out pointed at 5600
        // to match, BOTH the relay and QGC end up bound to the same port -- Linux splits/duplicates delivery between the
        // two non-deterministically, so the relay stops reliably receiving the full camera stream, and the tracker's own
        // input on 5601 then intermittently fails to get a frame. Using 5602 for the annotated/QGC-facing output instead
        // (confirmed nothing else listens there) avoids the collision entirely; pass --video-out udp://127.0.0.1:5602 to
        // match when you want QGC to show the annotated feed.
        a.connection = "tcp:127.0.0.1:5770"; a.camera_source = "gst"; a.gst_udp_port = 5601; a.hfov_deg = 114.6; a.track_scale = 1.0;
        a.cam_component = true; a.cam_w = 640; a.cam_h = 480; a.cam_fps = 10; a.cam_video_uri = "udp://127.0.0.1:5602";
        // flat runway at home level: "home" is exact here, while SITL's simulated rangefinder reads ~1.3 m short (measured: 2.93 m
        // at a true 4.31 m), which "auto" would prefer and which pulled the dash endpoint short in testing
        a.height_source = "home";
        // centring a crossing target: measured live, P-only yaw (kyaw 1.0) + 0.7 feed-forward held a 4 m/s car 18-19 px off centre
        // (always on the side it was moving to -- lag, (1 - ff) * rate / kyaw). Firmer gain, near-full feed-forward and a small
        // anti-windup integral take the remaining lag out.
        a.kyaw = 2.0; a.yaw_ff = 0.5; a.ki_yaw = 0.0;   // PD yaw, retuned live: car kept 60 px from centre, no oscillation
        a.auto_view = true;                                               // move closer / zoom for a good view of small targets
        // SITL runs in lockstep with Gazebo: on a loaded laptop the simulated vehicle's heartbeats can arrive >2 s apart (measured),
        // which read as a link loss; the real Cube over serial does not have this, so this is for the simulation only
        a.link_hb_timeout_s = 4.0;
        if (p == "sim-gimbal") { a.follow_source = "gimbal"; a.c12_host = "127.0.0.1"; a.target_height = 0.9; }
        // sim_up.sh --c12-camera: a C12-like 1280x720 ~100 deg camera, tracked at half resolution like --profile real-c12 (so the
        // view control's digital zoom adds real detail), and the autopilot gimbal kept on the target (sim_up.sh --gimbal)
        if (p == "sim-c12cam") { a.hfov_deg = 100.0; a.track_scale = 0.5; a.cam_w = 1280; a.cam_h = 720; a.gimbal_centre = true; a.vz_mode = "hold"; a.gimbal_settle = 0.15; a.video_latency = 0.0; a.mount_frame = "body";
                                 a.yaw_ff_mode = "world"; a.kyaw = 3.0;     // centring: see "CENTRING" in the loop
                                 // digital zoom OFF by default (operator report: misses on zoom in / out -- every zoom change restarts the
                                 // tracker on a rescaled view); small targets are tracked at full resolution instead. --zoom-max 2: on
                                 a.zoom_max = 1.0;
                                 // GO as fast as the airframe allows (operator: "as fast as it should"): 12 m/s default (go SPEED overrides),
                                 // braking at 6 m/s^2. The real limit is ArduPilot's WPNAV_SPEED / WPNAV_ACCEL (simulation/fast.parm).
                                 a.rescue_speed = 12.0; a.goto_accel = 6.0;
                                 // FPV-fast follow: keep pace with an 18 m/s vehicle, turn up to 2 rad/s, close a distance error at 4 m/s
                                 a.ceil_vx = 18.0; a.ceil_vy = 10.0; a.ceil_yaw = 2.0; a.speed_match_accel = 3.5; a.view_approach_speed = 3.0; }
                                 // (6 m/s^2 tilted the drone ~31 deg per change, and the body-coupled camera lost the target: measured)
        // sim_up.sh --fpv-camera: an FPV drone -- the same 1280x720 ~100 deg camera, but FIXED to the airframe, tilted --cam-pitch-deg
        // (sim_up.sh FPV_CAM_PITCH, default -15) and no gimbal: the drone itself turns to centre the target sideways and climbs /
        // descends to hold it vertically (on the depression below the horizon: fl::body_cam_vertical_error), at FPV speed
        if (p == "sim-fpv") { a.hfov_deg = 100.0; a.track_scale = 0.5; a.cam_w = 1280; a.cam_h = 720; a.video_latency = 0.2;
                              a.gimbal_centre = false; a.yaw_ff_mode = "world"; a.kyaw = 2.0; a.zoom_max = 1.0;
                              apply_fpv_flight(a); }
    } else if (p == "real-fpv") {
        // the real FPV drone: camera fixed to the airframe (no gimbal, no C12). MEASURE the mount's tilt on the airframe and pass it
        // (--cam-pitch-deg, + up, negative looks down) and the lens's --hfov-deg: every centring and ground estimate depends on them.
        // Video defaults to a USB capture (/dev/video0, --source); --camera-source csi|rtsp|gst for other links.
        a.connection = "serial:/dev/serial0:921600"; a.camera_source = "usb"; a.source = "0";   // the capture device on the companion
        a.hfov_deg = 100.0; a.track_scale = 0.5; a.cam_component = true; a.cam_w = 1280; a.cam_h = 720; a.cam_fps = 30; a.cam_video_uri = "udp://0.0.0.0:5600";
        a.no_display = true; a.yaw_ff_mode = "world";
        apply_fpv_flight(a);
    } else if (p == "real-c12" || p == "real-c12-gimbal") {
        a.connection = "serial:/dev/serial0:921600"; a.camera_source = "rtsp"; a.rtsp_url = "rtsp://192.168.144.108:554/stream=1"; a.rtsp_latency = 50;
        a.hfov_deg = 100.0; a.track_scale = 0.5; a.cam_component = true; a.cam_w = 1280; a.cam_h = 720; a.cam_fps = 30; a.cam_video_uri = "udp://0.0.0.0:5600";
        a.c12_host = "192.168.144.108"; a.no_display = true;
        a.camera_mount = "gimbal"; a.vz_mode = "hold";   // the C12 is a stabilized gimbal; altitude stays the pilot's (see Args::vz_mode)
        if (p == "real-c12-gimbal") a.follow_source = "gimbal";
    } else {
        throw std::runtime_error("unknown --profile '" + p + "' (sim | sim-gimbal | sim-c12cam | sim-fpv | real-fpv | real-c12 | real-c12-gimbal)");
    }
}

Args parse_args(int argc, char** argv) {
    Args a;
    for (int i = 1; i + 1 < argc; i++) if (std::string(argv[i]) == "--profile") apply_profile(a, argv[i + 1]);   // first, so explicit options override it
    for (int i = 1; i < argc; i++) {
        std::string k = argv[i];
        if (k == "--profile") { arg_value(argc, argv, i); }               // already applied above
        else if (k == "--connection") a.connection = arg_value(argc, argv, i);
        else if (k == "--camera-source") a.camera_source = arg_value(argc, argv, i);
        else if (k == "--source") a.source = arg_value(argc, argv, i);
        else if (k == "--rtsp-url") a.rtsp_url = arg_value(argc, argv, i);
        else if (k == "--rtsp-latency") a.rtsp_latency = std::stoi(arg_value(argc, argv, i));
        else if (k == "--rtsp-out-encoder") a.rtsp_out_encoder = arg_value(argc, argv, i);
        else if (k == "--csi-width") a.csi_width = std::stoi(arg_value(argc, argv, i));
        else if (k == "--csi-height") a.csi_height = std::stoi(arg_value(argc, argv, i));
        else if (k == "--csi-fps") a.csi_fps = std::stod(arg_value(argc, argv, i));
        else if (k == "--csi-sensor-id") a.csi_sensor_id = std::stoi(arg_value(argc, argv, i));
        else if (k == "--csi-flip") a.csi_flip = std::stoi(arg_value(argc, argv, i));
        else if (k == "--gst-udp-port") a.gst_udp_port = std::stoi(arg_value(argc, argv, i));
        else if (k == "--first-frame-timeout") a.first_frame_timeout_s = std::stod(arg_value(argc, argv, i));
        else if (k == "--rc-high-thresh") a.rc_high_thresh = std::stoi(arg_value(argc, argv, i));
        else if (k == "--rc-low-thresh") a.rc_low_thresh = std::stoi(arg_value(argc, argv, i));
        else if (k == "--ceil-vx") a.ceil_vx = std::stod(arg_value(argc, argv, i));
        else if (k == "--ceil-vy") a.ceil_vy = std::stod(arg_value(argc, argv, i));
        else if (k == "--ceil-vz") a.ceil_vz = std::stod(arg_value(argc, argv, i));
        else if (k == "--ceil-yaw") a.ceil_yaw = std::stod(arg_value(argc, argv, i));
        else if (k == "--hfov-deg") a.hfov_deg = std::stod(arg_value(argc, argv, i));
        else if (k == "--fx") a.fx = std::stod(arg_value(argc, argv, i));
        else if (k == "--fy") a.fy = std::stod(arg_value(argc, argv, i));
        else if (k == "--track-scale") a.track_scale = std::stod(arg_value(argc, argv, i));
        else if (k == "--csrt-default") a.csrt_fast = false;
        else if (k == "--csrt-template-factor") a.csrt_template_factor = std::stod(arg_value(argc, argv, i));
        else if (k == "--csrt-padding") a.csrt_padding = std::stod(arg_value(argc, argv, i));
        else if (k == "--kyaw") a.kyaw = std::stod(arg_value(argc, argv, i));
        else if (k == "--kz") a.kz = std::stod(arg_value(argc, argv, i));
        else if (k == "--kx") a.kx = std::stod(arg_value(argc, argv, i));
        else if (k == "--klat") a.klat = std::stod(arg_value(argc, argv, i));
        else if (k == "--ki-yaw") a.ki_yaw = std::stod(arg_value(argc, argv, i));
        else if (k == "--ki-limit") a.ki_limit = std::stod(arg_value(argc, argv, i));
        else if (k == "--lateral-mode") a.lateral_mode = arg_value(argc, argv, i);
        else if (k == "--db-yaw") a.db_yaw = std::stod(arg_value(argc, argv, i));
        else if (k == "--db-z") a.db_z = std::stod(arg_value(argc, argv, i));
        else if (k == "--db-x") a.db_x = std::stod(arg_value(argc, argv, i));
        else if (k == "--coast-after") a.coast_after = std::stod(arg_value(argc, argv, i));
        else if (k == "--lost-after") a.lost_after = std::stod(arg_value(argc, argv, i));
        else if (k == "--unlock-after") a.unlock_after = std::stod(arg_value(argc, argv, i));
        else if (k == "--alt-floor") a.alt_floor = std::stod(arg_value(argc, argv, i));
        else if (k == "--alt-ceiling") a.alt_ceiling = std::stod(arg_value(argc, argv, i));
        else if (k == "--no-map-marker") a.map_marker = false;
        else if (k == "--gimbal-centre") a.gimbal_centre = true;
        else if (k == "--auto-view") a.auto_view = true;
        else if (k == "--centre-target" || k == "--center-target") a.centre_target = true;
        else if (k == "--no-centre-target" || k == "--no-center-target") a.centre_target = false;
        else if (k == "--centre-kx") a.centre_kx = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 20.0);
        else if (k == "--view-min-px") a.view_min_px = std::stod(arg_value(argc, argv, i));
        else if (k == "--view-max-px") a.view_max_px = std::stod(arg_value(argc, argv, i));
        else if (k == "--view-min-dist") a.view_min_dist = std::stod(arg_value(argc, argv, i));
        else if (k == "--zoom-max") a.zoom_max = std::max(1.0, std::stod(arg_value(argc, argv, i)));
        else if (k == "--view-approach-speed") a.view_approach_speed = std::stod(arg_value(argc, argv, i));
        else if (k == "--gimbal-k") a.gimbal_k = std::stod(arg_value(argc, argv, i));
        else if (k == "--yaw-ff-mode") a.yaw_ff_mode = arg_value(argc, argv, i);
        else if (k == "--yaw-accel") a.yaw_accel = std::clamp(std::stod(arg_value(argc, argv, i)), 0.5, 20.0);
        else if (k == "--yaw-ff-tau") a.yaw_ff_tau = std::clamp(std::stod(arg_value(argc, argv, i)), 0.02, 2.0);
        else if (k == "--full-res-below") a.full_res_below = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 400.0);
        else if (k == "--go-guidance") a.go_guidance = arg_value(argc, argv, i);
        else if (k == "--pn-gain") a.pn_gain = std::clamp(std::stod(arg_value(argc, argv, i)), 1.0, 8.0);
        else if (k == "--stall-tolerance") a.stall_tolerance = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 10.0);
        else if (k == "--no-go-accel-ff") a.go_accel_ff = false;
        else if (k == "--predict-ahead") a.predict_ahead = std::clamp(std::stod(arg_value(argc, argv, i)), 0.05, 3.0);
        else if (k == "--ai") a.ai = arg_value(argc, argv, i);
        else if (k == "--reid-model") a.reid_model = arg_value(argc, argv, i);
        else if (k == "--no-view-climb") a.view_climb = false;
        else if (k == "--view-max-alt") a.view_max_alt = std::clamp(std::stod(arg_value(argc, argv, i)), 2.0, 120.0);
        else if (k == "--view-climb-rate") a.view_climb_rate = std::clamp(std::stod(arg_value(argc, argv, i)), 0.2, 5.0);
        else if (k == "--reid") { a.reid = true; if (a.detector_port <= 0) a.detector_port = 5810; }
        else if (k == "--gimbal-db") a.gimbal_db = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 10.0);
        else if (k == "--gimbal-step") a.gimbal_step = std::stod(arg_value(argc, argv, i));
        else if (k == "--yaw-ff") a.yaw_ff = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 1.0);
        else if (k == "--no-coast-yaw") a.coast_yaw = false;
        else if (k == "--coast-yaw") a.coast_yaw = true;
        else if (k == "--gimbal-follow-prediction") a.gimbal_follow_prediction = true;
        else if (k == "--detector-port") a.detector_port = std::stoi(arg_value(argc, argv, i));
        else if (k == "--no-refine-lock") a.refine_lock = false;
        else if (k == "--lock-without-guided") a.lock_without_guided = true;
        else if (k == "--best-view") a.best_view = true;
        else if (k == "--best-view-min-frac") a.best_view_min_frac = std::stod(arg_value(argc, argv, i));
        else if (k == "--best-view-max-frac") a.best_view_max_frac = std::stod(arg_value(argc, argv, i));
        else if (k == "--best-view-gain") a.best_view_gain = std::stod(arg_value(argc, argv, i));
        else if (k == "--log-dir") a.log_dir = arg_value(argc, argv, i);
        else if (k == "--lock-box") a.lock_box = arg_value(argc, argv, i);
        else if (k == "--video-out") a.video_out = arg_value(argc, argv, i);
        else if (k == "--rtsp-out") a.rtsp_out = arg_value(argc, argv, i);
        else if (k == "--record") a.record_dir = arg_value(argc, argv, i);
        else if (k == "--record-bitrate") a.record_bitrate = std::stoi(arg_value(argc, argv, i));
        else if (k == "--video-out-bitrate") a.video_out_bitrate = std::stoi(arg_value(argc, argv, i));
        else if (k == "--no-display") a.no_display = true;
        else if (k == "--follow-source") a.follow_source = arg_value(argc, argv, i);
        else if (k == "--camera-mount") a.camera_mount = arg_value(argc, argv, i);
        else if (k == "--cam-pitch-deg") a.cam_pitch_deg = std::stod(arg_value(argc, argv, i));
        else if (k == "--cam-yaw-deg") a.cam_yaw_deg = std::stod(arg_value(argc, argv, i));
        else if (k == "--vz-mode") a.vz_mode = arg_value(argc, argv, i);
        else if (k == "--min-depression-deg") a.min_depression_deg = std::stod(arg_value(argc, argv, i));
        else if (k == "--target-height") a.target_height = std::stod(arg_value(argc, argv, i));
        else if (k == "--c12-host") a.c12_host = arg_value(argc, argv, i);
        else if (k == "--c12-port") a.c12_port = std::stoi(arg_value(argc, argv, i));
        else if (k == "--c12-width") a.c12_w = std::stoi(arg_value(argc, argv, i));
        else if (k == "--c12-height") a.c12_h = std::stoi(arg_value(argc, argv, i));
        else if (k == "--rc-centerlock-channel") a.rc_centerlock_channel = std::stoi(arg_value(argc, argv, i));
        else if (k == "--enable-dash" || k == "--enable-fpv-dash" || k == "--enable-rescue") a.enable_rescue = true;
        else if (k == "--dash-speed" || k == "--go-speed" || k == "--rescue-speed") a.rescue_speed = std::clamp(std::stod(arg_value(argc, argv, i)), 0.5, go_settings::kMaxSpeed);
        else if (k == "--go-confirm") a.go_confirm = true;
        else if (k == "--go-through") a.go_through = true;
        else if (k == "--no-go-through") a.go_through = false;
        else if (k == "--dash-max-range" || k == "--rescue-max-range") a.rescue_max_range = std::stod(arg_value(argc, argv, i));
        else if (k == "--dash-hover-height" || k == "--rescue-hover-height") a.rescue_hover_height = std::stod(arg_value(argc, argv, i));
        else if (k == "--height-source") a.height_source = arg_value(argc, argv, i);
        else if (k == "--dash-min-battery" || k == "--rescue-min-battery") a.rescue_min_battery = std::stod(arg_value(argc, argv, i));
        else if (k == "--goto-paced") a.goto_paced = true;
        else if (k == "--lost-loiter") a.lost_loiter = true;
        else if (k == "--abort-climb") a.abort_climb = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 100.0);
        else if (k == "--goto-accel") a.goto_accel = std::clamp(std::stod(arg_value(argc, argv, i)), 0.5, 20.0);
        else if (k == "--zero-decel" || k == "--no-brake") a.zero_decel = true;
        else if (k == "--fixed-horizontal") a.fixed_horizontal = true;
        else if (k == "--go-offset" || k == "--goto-standoff" || k == "--rescue-standoff")   // + before the target, - past it (m)
            a.goto_standoff = std::clamp(std::stod(arg_value(argc, argv, i)), -go_settings::kMaxOffset, go_settings::kMaxOffset);
        else if (k == "--dash-decel" || k == "--rescue-decel") a.rescue_decel = std::stod(arg_value(argc, argv, i));
        else if (k == "--dash-lead" || k == "--rescue-lead") a.rescue_lead = true;
        else if (k == "--dash-max-lead" || k == "--rescue-max-lead") a.rescue_max_lead = std::stod(arg_value(argc, argv, i));
        else if (k == "--dash-stay-over" || k == "--rescue-stay-over") a.rescue_stay_over = true;
        else if (k == "--no-box-refit") a.box_refit = false;
        else if (k == "--box-refit") a.box_refit = true;
        else if (k == "--fpv") a.fpv = true;
        else if (k == "--speed-match") a.speed_match = true;
        else if (k == "--no-speed-match") a.speed_match = false;
        else if (k == "--speed-match-back") a.speed_match_back = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 15.0);
        else if (k == "--speed-match-accel") a.speed_match_accel = std::clamp(std::stod(arg_value(argc, argv, i)), 0.5, 10.0);
        else if (k == "--mount-frame") { a.mount_frame = arg_value(argc, argv, i); if (a.mount_frame != "body" && a.mount_frame != "earth") throw std::runtime_error("--mount-frame: body | earth"); }
        else if (k == "--video-latency") a.video_latency = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 1.0);
        else if (k == "--est-q") a.est_q_acc = std::clamp(std::stod(arg_value(argc, argv, i)), 0.05, 20.0);
        else if (k == "--gimbal-settle") a.gimbal_settle = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 5.0);
        else if (k == "--no-fpv") a.fpv = false;
        else if (k == "--fpv-lateral") a.fpv_lateral = true;
        else if (k == "--fpv-approach") a.fpv_approach = std::clamp(std::stod(arg_value(argc, argv, i)), 0.5, 20.0);
        else if (k == "--fpv-yaw-ff") a.fpv_yaw_ff = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 1.0);
        else if (k == "--fpv-speed") a.fpv_speed = std::clamp(std::stod(arg_value(argc, argv, i)), 1.0, 70.0);
        else if (k == "--fpv-standoff") a.fpv_standoff = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 60.0);
        else if (k == "--fpv-gain") a.fpv_k = std::clamp(std::stod(arg_value(argc, argv, i)), 0.05, 5.0);
        else if (k == "--fpv-ff") a.fpv_ff = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 1.0);
        else if (k == "--fpv-accel") a.fpv_accel = std::clamp(std::stod(arg_value(argc, argv, i)), 1.0, 35.0);
        else if (k == "--fpv-shoot" || k == "--unrestricted" || k == "--zero-safety") {
            a.alt_floor = 0.0; a.alt_ceiling = 0.0; a.zero_decel = true; a.go_through = true;
            a.rescue_hover_height = 0.0; a.enable_rescue = true; a.go_accel_ff = true;
            a.goto_standoff = -1000.0; a.ceil_vx = 50.0; a.ceil_vy = 30.0; a.ceil_vz = 25.0; a.ceil_yaw = 8.0;
            a.rescue_speed = 40.0; a.fpv_shoot = true;
        }
        else if (k == "--follow-ff") a.follow_ff = std::clamp(std::stod(arg_value(argc, argv, i)), 0.0, 1.0);
        else if (k == "--dash-override-margin" || k == "--rescue-override-margin") a.rescue_override_margin = std::stod(arg_value(argc, argv, i));
        else if (k == "--tap-latency-ms") a.tap_latency_ms = std::stod(arg_value(argc, argv, i));
        else if (k == "--rc-dash-channel" || k == "--rc-go-channel" || k == "--rc-rescue-channel") a.rc_rescue_channel = std::stoi(arg_value(argc, argv, i));
        else if (k == "--dash-level-only" || k == "--rescue-level-only") a.rescue_descend = false;
        else if (k == "--sysid") a.sysid = std::stoi(arg_value(argc, argv, i));
        else if (k == "--gcs-sysid") a.gcs_sysid = std::stoi(arg_value(argc, argv, i));
        else if (k == "--compid") a.compid = std::stoi(arg_value(argc, argv, i));
        else if (k == "--type-mask") a.type_mask = std::stoi(arg_value(argc, argv, i));
        else if (k == "--link-stale") a.link_stale_s = std::stod(arg_value(argc, argv, i));
        else if (k == "--link-timeout") a.link_hb_timeout_s = std::stod(arg_value(argc, argv, i));
        else if (k == "--cam-component") a.cam_component = true;
        else if (k == "--cam-compid") a.cam_compid = std::stoi(arg_value(argc, argv, i));
        else if (k == "--cam-width") a.cam_w = std::stoi(arg_value(argc, argv, i));
        else if (k == "--cam-height") a.cam_h = std::stoi(arg_value(argc, argv, i));
        else if (k == "--cam-fps") a.cam_fps = std::stod(arg_value(argc, argv, i));
        else if (k == "--cam-video-uri") a.cam_video_uri = arg_value(argc, argv, i);
        else if (k == "--cam-video-rtsp") a.cam_video_rtsp = true;
        else if (k == "--link-test") a.link_test_s = std::stod(arg_value(argc, argv, i));
        else if (k == "--link-request-guided") a.link_request_guided = true;
        else throw std::runtime_error("Unknown argument: " + k);
    }
    return a;
}


}  // namespace

// ---------------------------------------------------------------------
// Global configuration (parsed once in main)
// ---------------------------------------------------------------------
Args args;



// ---------------------------------------------------------------------
// Link configuration from the command line, and the bring-up tool (--link-test)
// ---------------------------------------------------------------------
LinkConfig make_link_config(const Args& a) {
    LinkConfig c;
    c.connection = a.connection; c.sysid = (uint8_t)a.sysid; c.compid = (uint8_t)a.compid; c.type_mask = a.type_mask; c.gcs_sysid = (uint8_t)a.gcs_sysid;
    c.stale_s = a.link_stale_s; c.hb_timeout_s = a.link_hb_timeout_s;
    c.cam_enable = a.cam_component; c.cam_compid = (uint8_t)a.cam_compid; c.cam_video_uri = a.cam_video_uri;
    c.cam_video_rtsp = a.cam_video_rtsp; c.cam_w = a.cam_w; c.cam_h = a.cam_h; c.cam_fps = a.cam_fps;
    return c;
}

// First thing to run on real hardware: prove the MAVLink path (companion identity, heartbeat, message streams, RC, GUIDED request,
// QGC tracking requests) WITHOUT any camera, tracker or motion command. Sends nothing that moves the vehicle.
int run_link_test() {
    using Clock = std::chrono::steady_clock;
    std::unique_ptr<MavlinkLink> link;
    try { link = std::make_unique<MavlinkLink>(make_link_config(args)); }
    catch (const std::exception& e) { std::fprintf(stderr, "[LINK TEST] cannot open the link: %s\n", e.what()); return 1; }
    auto t0 = Clock::now(), last = t0 - std::chrono::seconds(2);
    bool asked = false, seen = false;
    while (std::chrono::duration<double>(Clock::now() - t0).count() < args.link_test_s) {
        link->liveness();
        VehicleState s = link->state();
        if (s.link_ok) seen = true;
        if (args.link_request_guided && !asked && s.link_ok) { link->request_guided(); asked = true; std::printf("[LINK TEST] requested GUIDED\n"); }
        TrackRequest tr;
        while (link->poll_track_request(tr))
            std::printf("[LINK TEST] QGC tracking request kind=%d (%.3f,%.3f)-(%.3f,%.3f)\n", (int)tr.kind, tr.x0, tr.y0, tr.x1, tr.y1);
        if (std::chrono::duration<double>(Clock::now() - last).count() >= 1.0) {
            last = Clock::now();
            std::printf("[LINK TEST] link=%s hb_age=%.1fs veh=%d:%d mode=%u guided=%d armed=%d alt=%s%.1fm gps=%d/%d ekf=%d rc[1..4,9]=%d %d %d %d %d\n",
                        s.link_ok ? "UP" : "DOWN", s.hb_age > 1e8 ? -1.0 : s.hb_age, s.sysid, s.compid, s.custom_mode, s.guided, s.armed,
                        s.alt_valid ? "" : "(n/a) ", s.alt_m, s.gps_fix, s.gps_sats, s.ekf_ok, s.rc[1], s.rc[2], s.rc[3], s.rc[4], s.rc[9]);
            std::fflush(stdout);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    std::printf("[LINK TEST] %s\n", seen ? "vehicle heartbeat seen: link OK" : "NO vehicle heartbeat seen: check wiring/ports/MAVLink version/SERIALn_PROTOCOL");
    return seen ? 0 : 3;
}

// ---------------------------------------------------------------------
// main: --selftest, --link-test, or the tracker application
// ---------------------------------------------------------------------
int run_tracker_mode();   // defined in the tracker application section
int run_replay(int argc, char** argv);   // --replay: the tracking core on a recorded video (defined after LockTracker)
static double now_s_main() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

int main(int argc, char** argv) {
    for (int i = 1; i < argc; i++)
        if (std::string(argv[i]) == "--selftest") return run_selftest();   // no vehicle, camera or display needed
    // diagnostic: the full click -> lock box path with motion:   target_tracker --lock-test CUR PREV X Y
    if (argc == 6 && std::string(argv[1]) == "--lock-test") {
        cv::Mat cur = cv::imread(argv[2]), prev = cv::imread(argv[3]);
        fl::Box b; std::string why, how; const double t0 = now_s_main();
        const bool ok = fl::lock_box_from_click(cur, prev, 1.0, std::atof(argv[4]), std::atof(argv[5]), b, why, how);
        if (ok) std::printf("OK %.0f %.0f %.0f %.0f  (%.0f ms, %s)\n", b.x, b.y, b.w, b.h, (now_s_main() - t0) * 1000, how.c_str());
        else std::printf("REFUSED: %s\n", why.c_str());
        return ok ? 0 : 1;
    }
    if (argc >= 6 && std::string(argv[1]) == "--replay") return run_replay(argc, argv);
    // diagnostic: the LIVE lock path (previous frame at the tracking scale, snapping, minimum box):
    //   target_tracker --ai-bench IMAGE X Y: the in-process ReID model on one image -- DINOv2 time per crop around (X, Y)
#ifdef HAVE_OPENVINO
    if (argc == 5 && std::string(argv[1]) == "--ai-bench") {
        cv::Mat img = cv::imread(argv[2]); const double x = std::atof(argv[3]), y = std::atof(argv[4]);
        if (img.empty()) { std::fprintf(stderr, "cannot read %s\n", argv[2]); return 2; }
        auto now_s = [] { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); };
        const std::string home = std::getenv("HOME") ? std::getenv("HOME") : ".";
        const double t0 = now_s(); NativeAI ai(home + "/models/dinov2"); const double t_load = now_s() - t0;
        const fl::Box b{x - 20, y - 20, 40, 40};
        std::vector<double> te; std::vector<float> v;
        for (int i = 0; i < 10; i++) { const double a = now_s(); const uint32_t id = ai.request_embed(img, b); while (!ai.embed_poll(id, v)) std::this_thread::sleep_for(std::chrono::microseconds(200)); te.push_back((now_s() - a) * 1000); }
        std::sort(te.begin(), te.end());
        if (const char* vf = std::getenv("AI_BENCH_VEC")) { FILE* fp = std::fopen(vf, "w"); if (fp) { for (float q : v) std::fprintf(fp, "%.6f\n", q); std::fclose(fp); } }
        std::printf("load %.1f s | DINOv2 %zu-d fingerprint: median %.1f ms, min %.1f ms\n", t_load, v.size(), te[te.size() / 2], te.front());
        return 0;
    }
#endif
    //   target_tracker --lock-job-test CUR PREV X Y [SCALE]
    if ((argc == 6 || argc == 7) && std::string(argv[1]) == "--lock-job-test") {
        cv::Mat cur = cv::imread(argv[2]), prev = cv::imread(argv[3]);
        const double sc = argc == 7 ? std::atof(argv[6]) : 0.5;
        cv::Mat ps; if (sc < 0.999) cv::resize(prev, ps, cv::Size(), sc, sc, cv::INTER_AREA); else ps = prev;
        if (std::getenv("LOCK_WARM")) (void)fl::run_lock_job(cur, ps, sc, std::atof(argv[4]), std::atof(argv[5]), 0.07 * cur.cols, 0);   // warm-up: one-time OpenCV start-up
        const fl::LockJob j = fl::run_lock_job(cur, ps, sc, std::atof(argv[4]), std::atof(argv[5]), 0.07 * cur.cols, 0);
        if (j.ok) std::printf("OK %.0f %.0f %.0f %.0f  (%.0f ms, %s)\n", j.box.x, j.box.y, j.box.w, j.box.h, j.ms, j.how.c_str());
        else std::printf("REFUSED: %s\n", j.why.c_str());
        return j.ok ? 0 : 1;
    }
    // diagnostic: what would a click at (X, Y) on this image lock?   target_tracker --grabcut-test IMG X Y
    if (argc == 5 && std::string(argv[1]) == "--grabcut-test") {
        cv::Mat img = cv::imread(argv[2]);
        if (img.empty()) { std::fprintf(stderr, "cannot read %s\n", argv[2]); return 2; }
        fl::Box b; std::string why; const double t0 = now_s_main();
        const bool ok = fl::grabcut_from_point(img, std::atof(argv[3]), std::atof(argv[4]), b, why);
        if (ok) std::printf("OK %.0f %.0f %.0f %.0f  (%.0f ms)\n", b.x, b.y, b.w, b.h, (now_s_main() - t0) * 1000);
        else std::printf("REFUSED: %s\n", why.c_str());
        return ok ? 0 : 1;
    }
    try {
        args = parse_args(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Argument error: %s\n", e.what());
        return 1;
    }
    if (args.camera_mount != "body" && args.camera_mount != "gimbal") { std::fprintf(stderr, "Argument error: --camera-mount must be body or gimbal\n"); return 1; }
    if (args.height_source != "auto" && args.height_source != "home" && args.height_source != "rangefinder") { std::fprintf(stderr, "Argument error: --height-source must be auto, home or rangefinder\n"); return 1; }
    if (args.lateral_mode != "yaw" && args.lateral_mode != "strafe") { std::fprintf(stderr, "Argument error: --lateral-mode must be yaw or strafe\n"); return 1; }
    if (args.go_guidance != "pursuit" && args.go_guidance != "lead" && args.go_guidance != "intercept" && args.go_guidance != "pn") { std::fprintf(stderr, "Argument error: --go-guidance must be pursuit, lead, intercept or pn\n"); return 1; }
    if (args.vz_mode != "center" && args.vz_mode != "hold") { std::fprintf(stderr, "Argument error: --vz-mode must be center or hold\n"); return 1; }
    if (args.link_test_s > 0) return run_link_test();
    return run_tracker_mode();
}

// ============================================================
// ---- tracker application (class-agnostic): operator/QGC selects -> CSRT -> follow
// ============================================================
// Data flow per frame:  capture -> frame watch -> (select/QGC/RC/stdin events) -> CSRT on a downscaled copy -> Kalman smoothing ->
// follow state machine -> control law (+ altitude guard) -> MavlinkLink.command() [20 Hz tick sends it, gated and slewed]
// -> status to QGC -> overlay / annotated video out / JSON log.  Velocity is only ever sent through MavlinkLink.

namespace {
std::atomic<bool> g_stop{false};
void on_signal(int) { g_stop = true; }

double now_s() {
    using namespace std::chrono;
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ---- JSON-lines log (RAII FILE*), one fixed buffer, no per-frame allocation ----------------------------
class JsonLog {
public:
    bool open(const std::string& dir) {
        if (dir.empty()) return false;
        ::mkdir(dir.c_str(), 0755);
        char name[256]; std::time_t t = std::time(nullptr); std::tm tmv; localtime_r(&t, &tmv);
        std::snprintf(name, sizeof name, "%s/follow_%04d%02d%02d_%02d%02d%02d.jsonl", dir.c_str(), tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
        f_ = std::fopen(name, "w");
        if (f_) std::printf("[LOG] %s\n", name);
        return f_ != nullptr;
    }
    ~JsonLog() { if (f_) std::fclose(f_); }
    void line(const char* s) { if (f_) { std::fputs(s, f_); std::fputc('\n', f_); std::fflush(f_); } }   // flushed per line: survives a crash
    bool on() const { return f_ != nullptr; }
private:
    FILE* f_ = nullptr;
};

// ---- drag-to-select box (left button: down -> move -> up) -------------------------------------------------
// On-screen buttons (bottom strip of the window) for operators who don't have keyboard focus on this
// window -- found live this session that 's'/'g' routinely "don't work" simply because the officer is
// looking at QGC's window when they press them, and the keypress goes there instead. A mouse click lands
// wherever the cursor is, focus-independent, same reasoning as the click-to-lock feature.
enum class Button { None, Lock, Unlock, Go, Abort };
inline std::array<std::pair<Button, cv::Rect>, 4> button_rects(int w, int h) {
    int bh = 30, margin = 6, bw = (w - 5 * margin) / 4;
    int y = h - bh - margin;
    return {{{Button::Lock, cv::Rect(margin, y, bw, bh)},
             {Button::Unlock, cv::Rect(margin * 2 + bw, y, bw, bh)},
             {Button::Go, cv::Rect(margin * 3 + bw * 2, y, bw, bh)},
             {Button::Abort, cv::Rect(margin * 4 + bw * 3, y, bw, bh)}}};
}
inline const char* button_label(Button b) {
    switch (b) { case Button::Lock: return "LOCK (centre)"; case Button::Unlock: return "UNLOCK"; case Button::Go: return "GO"; case Button::Abort: return "ABORT"; default: return ""; }
}

class BoxSelector {
public:
    static void callback(int event, int x, int y, int, void* self) { static_cast<BoxSelector*>(self)->on_mouse(event, x, y); }
    void set_frame(int w, int h) { w_ = w; h_ = h; }
    bool dragging() const { return false; }
    fl::Box drag_rect() const { return fl::Box{}; }
    bool has_pending() const { return false; }
    fl::Box pending() const { return fl::Box{}; }
    void clear() {}
    void set_pending(const fl::Box&) {}
    bool has_box() const { return false; }
    fl::Box take_box() { return fl::Box{}; }
    // Point-click lock only (no drag required): one click directly on/near the target locks immediately
    bool has_click() const { return click_; }
    cv::Point2d take_click() { click_ = false; return click_pt_; }     // a click is a POINT: GrabCut decides the object's extent
    Button take_button() { Button b = button_; button_ = Button::None; return b; }
private:
    void on_mouse(int event, int x, int y) {
        x = std::max(0, std::min(x, w_ - 1)); y = std::max(0, std::min(y, h_ - 1));
        if (event == cv::EVENT_LBUTTONDOWN) {
            button_down_ = Button::None;
            for (auto& [b, r] : button_rects(w_, h_)) if (r.contains({x, y})) button_down_ = b;
        }
        else if (event == cv::EVENT_LBUTTONUP) {
            if (button_down_ != Button::None) {
                for (auto& [b, r] : button_rects(w_, h_)) if (b == button_down_ && r.contains({x, y})) button_ = b;
                button_down_ = Button::None;
                return;
            }
            // Direct point click: locks the object at (x, y) immediately without dragging
            click_pt_ = cv::Point2d(x, y); click_ = true;
        }
    }
    int w_ = 640, h_ = 480; bool click_ = false;
    cv::Point2d click_pt_;
    Button button_down_ = Button::None, button_ = Button::None;
};

// ---- CSRT on a downscaled copy of the frame; boxes in / out are FULL-frame pixels ---------------------------
class TrackerEngine {
public:
    // padding: OpenCV's own default (3.0) unless overridden -- the Phase 0 benchmark found no safe change to CSRT's
    // DEFAULTS, but that benchmark predates the fast-crossing-traffic scenario (per-frame displacement overrunning a
    // narrow search window is a different failure mode than anything Phase 0 compared). --csrt-padding lets this be
    // tuned per scenario without silently overriding what Phase 0 actually measured.
    // fast = true: CSRT's search template scales with the target (template_size = clamp(factor * sqrt(w*h), 64, 160) at the
    // tracking scale), 17 scale steps instead of 33, no segmentation. OpenCV's default template_size is a FIXED 200 px, which
    // made every update expensive for small targets. Measured on the sim footage: small 15x21 target 142 -> 22 ms/frame, 30x42
    // 29 -> 10 ms, 45x63 37 -> 14 ms, same success (97-100%). A fixed SMALL template is not the answer either: template 64
    // dropped a 45x63 target to 48% success, template 48 to 3% -- hence scaled to the target.
    explicit TrackerEngine(double scale, double padding = 3.0, bool fast = true, double template_factor = 2.5)
        : scale_(std::max(0.1, std::min(1.0, scale))), padding_(padding), fast_(fast), tfactor_(template_factor) {}
    bool init(const cv::Mat& frame, const fl::Box& b) { shrink(frame); return init_small(small_, b); }
    // Tracking resolution per target: a SMALL object (< --full-res-below px, full frame) is tracked at full resolution -- at the sim
    // camera's half-resolution tracking a 10-20 px person was 5-10 px for CSRT, and most live losses were such targets (measured:
    // 7 of 11 in one session). Larger ones stay at the configured scale (CSRT's cost grows with the pixels).
    void set_scale(double s) { scale_ = std::max(0.1, std::min(1.0, s)); }
    double scale() const { return scale_; }
    // returns true if the tracker reports the target found; `out` is the new full-frame box
    bool update(const cv::Mat& frame, fl::Box& out) { if (!active_) return false; shrink(frame); return update_small(small_, out); }
    // Same as init()/update(), but the frame is ALREADY downscaled (by shrink_to(), same scale) -- used for frame-history
    // catch-up, where every buffered frame is stored pre-shrunk so re-running the tracker through several of them doesn't
    // re-shrink the same pixels over and over.
    bool init_small(const cv::Mat& small, const fl::Box& b) {
        cv::Rect r((int)std::lround(b.x * scale_), (int)std::lround(b.y * scale_), (int)std::lround(b.w * scale_), (int)std::lround(b.h * scale_));
        r &= cv::Rect(0, 0, small.cols, small.rows);
        if (r.width < 2 || r.height < 2) return false;
        // CSRT needs >= ~10 px to work with; a smaller object is tracked with a box grown around it, and the box REPORTED is shrunk
        // back to the object's own size (before: the padded 20x20 was the lock box -- IoU 0.28-0.45 on 8-14 px objects, review)
        fw_ = fh_ = 1.0;
        if (r.width < 10 || r.height < 10) {
            const int nw = std::max(r.width, 10), nh = std::max(r.height, 10);
            fw_ = (double)r.width / nw; fh_ = (double)r.height / nh;
            r = cv::Rect(r.x + r.width / 2 - nw / 2, r.y + r.height / 2 - nh / 2, nw, nh) & cv::Rect(0, 0, small.cols, small.rows);
            if (r.width < 8 || r.height < 8) return false;
        }
        cv::TrackerCSRT::Params p; p.padding = (float)padding_;
        if (fast_) {
            p.template_size = (float)std::clamp(tfactor_ * std::sqrt((double)r.width * r.height), 64.0, 160.0);
            // 33 size steps (OpenCV's default): measured, 17 steps disabled CSRT's size tracking entirely -- a target growing 56 -> 116 px
            // kept a 58 px box (33 steps: 112 px); on sim footage the box stayed 28x28 while the red box grew to 40x42
            p.number_of_scales = 33; p.use_segmentation = false;
        }
        p.scale_lr = (float)csrt_scale_lr();
        // benchmark-only overrides (tracker/tests/track_bench.py): CSRT_SCALES, CSRT_SCALE_STEP, CSRT_SEG, CSRT_LR (filter learning rate)
        if (const char* e = std::getenv("CSRT_SCALES")) p.number_of_scales = std::atoi(e);
        if (const char* e = std::getenv("CSRT_SCALE_STEP")) p.scale_step = (float)std::atof(e);
        if (const char* e = std::getenv("CSRT_SEG")) p.use_segmentation = std::atoi(e) != 0;
        if (const char* e = std::getenv("CSRT_LR")) p.filter_lr = (float)std::atof(e);
        if (const char* e = std::getenv("CSRT_TS")) p.template_size = (float)std::atof(e);
        tracker_ = cv::TrackerCSRT::create(p);
        try { tracker_->init(small, r); } catch (const cv::Exception& e) { std::printf("[TRACK] init failed: %s\n", e.what()); tracker_.release(); return false; }
        active_ = true; return true;
    }
    bool update_small(const cv::Mat& small, fl::Box& out) {
        if (!active_) return false;
        cv::Rect r;
        bool ok = false;
        try { ok = tracker_->update(small, r); } catch (const cv::Exception&) { ok = false; }
        if (!ok) return false;
        out = fl::Box{r.x / scale_, r.y / scale_, r.width / scale_, r.height / scale_};
        if (fw_ < 1.0 || fh_ < 1.0) { const double w = out.w * fw_, h = out.h * fh_; out = fl::Box{out.cx() - w / 2, out.cy() - h / 2, w, h}; }
        return out.valid();
    }
    void reset() { tracker_.release(); active_ = false; }
    bool active() const { return active_; }
    // Shrinks a full-frame image the same way init()/update() do internally, for a caller (frame-history buffer) that
    // wants to store/reuse the downscaled copy itself instead of paying for the resize again on every catch-up step.
    static cv::Mat shrink_to(const cv::Mat& f, double scale) {
        if (scale >= 0.999) return f;
        cv::Mat out; cv::resize(f, out, cv::Size(), scale, scale, cv::INTER_AREA); return out;
    }
private:
    void shrink(const cv::Mat& f) {
        if (scale_ >= 0.999) { small_ = f; return; }
        cv::resize(f, small_, cv::Size(), scale_, scale_, cv::INTER_AREA);          // one reused buffer, no per-frame allocation
    }
    static double csrt_scale_lr() { static const double v = std::getenv("CSRT_SCALE_LR") ? std::atof(std::getenv("CSRT_SCALE_LR")) : 0.025; return v; }   // OpenCV default
    double scale_, padding_; bool fast_; double tfactor_; cv::Ptr<cv::Tracker> tracker_; cv::Mat small_; bool active_ = false;
    double fw_ = 1.0, fh_ = 1.0;                                     // reported size / CSRT's size (a tiny object's grown box)
};

// ---- the per-frame tracking step for the operator-locked object: CSRT + appearance gate + occlusion handling -------------------
// What went wrong before (reproduced on sim footage with a real car patch passing over the target: plain CSRT 12% success, lost or
// switched at 3/3 occlusions): CSRT keeps LEARNING while the target is hidden, so it learns the occluder and follows it away.
//   * every frame CSRT proposes a box; it is accepted only if it is a plausible move AND still looks like the locked object
//     (fl::Appearance, relative to its own running level)
//   * one doubtful frame: not accepted, nothing else changes; two in a row = occlusion (or drift): CSRT is NOT updated any more
//   * hidden: the caller's Kalman prediction coasts; each frame the window around it is searched for the object's own colours
//   * re-acquired after two consistent matches in a row -> CSRT is re-initialised there
// Same footage after the change: 88% success, re-acquired after 3/3 occlusions. How long it may stay hidden before the lock is
// declared LOST is the follow state machine's --lost-after.
// [GO-7] ARRIVAL HOOK (see the GO map at the top of the file; arrival is detected in the dash loop [GO-6]).
// ==== GO arrival hook ======================================================================================================
// Called ONCE when a GO arrives at its stop point (the signed offset go_standoff from the target's current position: + before it,
// - past it -- go_settings::kStopOffset / --go-offset / "go SPEED OFFSET"; at least --alt-floor
// above its ground). The pursuit keeps holding that point (re-aimed at the target) until ABORT, a new lock, the target being gone
// for 3 s, or the pilot's mode change. Put the operator's own landing / release / next step here -- deliberately left empty.
//   link: the MAVLink link (send_goto / update_goto_point / send_mode_change / cancel_goto ...); veh: vehicle position (NED, m);
//   target: the target's estimated position (NED, m); standoff_m: the signed offset in force (+ before, - past the target).
template <class Link> void on_go_arrived(Link& link, const fl::Ned& veh, const fl::Ned& target, double standoff_m) {
    (void)link; (void)veh; (void)target; (void)standoff_m;
}

class LockTracker {
public:
    struct Step { bool accepted = false, occluded = false, reacquired = false, reanchored = false, raw_ok = false, crowded = false, ambiguous = false, reid_used = false, identity_lost = false; fl::Box box, raw; double score = 0; };
    LockTracker(double scale, double padding, bool fast, double template_factor)
        : scale_(scale), padding_(padding), fast_(fast), tf_(template_factor), engine_(scale, padding, fast, template_factor) {}
    void set_full_res_below(double px) { full_res_below_ = px; }
    void pick_scale(const fl::Box& b) { engine_.set_scale(std::min(b.w, b.h) < full_res_below_ ? 1.0 : scale_); }
    bool init(const cv::Mat& f, const fl::Box& b) {
        pick_scale(b);
        if (!engine_.init(f, b)) return false;
        ap_.init(f, b); clear_state(b); csrt_w_ = b.w; lock_contrast_ = fl::box_contrast(f, b);
        gallery_.clear(); reid_pending_ = 0; reid_next_ = 0; frame_n_ = 0; verify_pending_ = 0; verify_bad_ = 0; verify_next_ = 0;
        reanchor_run_ = 0; last_jump_frame_ = -1000; verify_ok_ = false; return true;
    }
    bool reinit(const cv::Mat& f, const fl::Box& b) {                   // a new box for the SAME object (re-anchor / re-acquisition)
        pick_scale(b);
        if (!engine_.init(f, b)) return false;
        clear_state(b); csrt_w_ = b.w; return true;
    }
    void reset() { engine_.reset(); clear_state(fl::Box()); }
    // ReID (--reid, DINOv2 in the sidecar): a gallery of the target's fingerprints, taken on clearly visible frames, and a way to
    // fingerprint a candidate. Used ONLY to decide which candidate a hidden target is (never per frame): it separates lookalikes
    // by shape -- a walker vs a box of the same colour -- where the colour check can't; the colour check still separates shapes
    // of different colours (DINOv2 alone found a red and an orange box 0.9 alike).
    struct Reid {
        std::function<uint32_t(const cv::Mat&, const fl::Box&)> request;            // async fingerprint request (0 = not sent)
        std::function<bool(uint32_t, std::vector<float>&)> poll;                    // reply in? (empty = failed)
        std::function<bool(const cv::Mat&, const std::vector<fl::Box>&, std::vector<std::vector<float>>&)> sync;   // fingerprints now (bounded wait)
    };
    // the image is being disturbed (gimbal move, zoom change, hard pitch / turn); on its END the identity check runs immediately
    void set_disturbed(bool d) { if (disturbed_ && !d) verify_next_ = 0; disturbed_ = d; }
    // the image filter's gate radius around this frame's prediction, for the hidden-target search (0 = off)
    void set_reacq_gate(double r) { reacq_gate_ = r; }
    void set_reid(Reid r) { reid_ = std::move(r); have_reid_ = (bool)reid_.sync; }
    size_t reid_gallery() const { return gallery_.size(); }
    // similarity of each box to the gallery (max cosine); false when there is no gallery or a fingerprint failed / was late. After a
    // failure ReID rests for ~1 s (frame count) rather than making every hidden frame wait on a busy sidecar.
    bool reid_sims(const cv::Mat& f, const std::vector<fl::Box>& bs, std::vector<double>& sims) {
        if (!have_reid_ || gallery_.empty() || frame_n_ < reid_rest_until_) return false;
        std::vector<std::vector<float>> vs;
        if (!reid_.sync(f, bs, vs)) {
            reid_rest_until_ = frame_n_ + 15;
            if (++reid_fails_ >= 3 && !reid_ok_once_) {                 // never answered at all: no ReID in the sidecar -- stop asking
                have_reid_ = false;
                std::printf("[REID] no fingerprints from the sidecar (start it with --reid ~/models/dinov2): ReID off\n");
            }
            return false;
        }
        reid_ok_once_ = true; reid_fails_ = 0;
        sims.assign(bs.size(), -1);
        for (size_t i = 0; i < vs.size(); i++)
            for (const auto& g : gallery_) if (g.size() == vs[i].size()) { double d = 0; for (size_t k = 0; k < g.size(); k++) d += (double)g[k] * vs[i][k]; sims[i] = std::max(sims[i], d); }
        for (double v : sims) if (v <= -1) return false;
        return true;
    }
    bool occluded() const { return occluded_; }
    int coast_frames() const { return coast_; }
    // Box re-fit (GrabCut around the tracked box, see the main loop): CSRT keeps its first box size and slowly slides towards an
    // edge or the object's shadow (measured on the sim's red cube: the box half a box off for seconds). Take a segmenter's tighter
    // box for the SAME object -- only when it does not look less like the target than CSRT's own box -- and restart CSRT on it.
    // Keeps the occlusion / crossing state (unlike reinit).
    bool refit(const cv::Mat& f, const fl::Box& cur, const fl::Box& b) {
        if (occluded_ || crowd_ || !b.valid()) return false;
        const double sb = ap_.score(f, b), sc = ap_.score(f, cur);
        if (!ap_.accepts(sb) || sb < sc - 0.05) return false;
        if (!engine_.init(f, b)) return false;
        csrt_w_ = b.w; last_ = b; if (!hist_.empty()) hist_.back() = b;
        return true;
    }
    // the camera moved (similarity A since the last frame): keep the last-seen position on the same ground spot
    void warp(const cv::Mat& A) {
        if (A.empty()) return;
        if (last_.valid()) last_ = fl::warp_box(last_, A);
        if (have_cand_) cand_ = fl::warp_box(cand_, A);
        // the recent boxes too: the velocity seen() derives from them is the TARGET's motion over the ground, not the image's -- unwarped
        // it carried the camera's own motion, which the crossing path then counted a second time (review)
        for (auto& hb : hist_) hb = fl::warp_box(hb, A);
        if (crowd_) {                                                     // the path: its start moves, the steps turn with the image
            c0_ = fl::warp_box(c0_, A);
            const double a = A.at<double>(0, 0), b = A.at<double>(0, 1), c = A.at<double>(1, 0), d = A.at<double>(1, 1);
            const double ux = cvx_, uy = cvy_; cvx_ = a * ux + b * uy; cvy_ = c * ux + d * uy;
        }
        const double a = A.at<double>(0, 0), b = A.at<double>(0, 1), c = A.at<double>(1, 0), d = A.at<double>(1, 1);
        const double vx = vx_, vy = vy_; vx_ = a * vx + b * vy; vy_ = c * vx + d * vy;
    }
    // pred: the Kalman prediction for this frame (already advanced); plausible: the caller's jump/size gate for a candidate box
    Step step(const cv::Mat& f, const fl::Box& pred, const std::function<bool(const fl::Box&)>& plausible) {
        frame_n_++;
        Step st;
        fl::Box path = pred;                                              // where the target should be (see the crossing guard)
        if (!occluded_) {
            st.raw_ok = engine_.update(f, st.raw);
            st.score = st.raw_ok ? ap_.score(f, st.raw) : 0.0;
            bool geom = st.raw_ok && plausible(st.raw);
            // CROSSING GUARD. A lookalike right next to the target (two people passing, identical sim actors) is where CSRT
            // silently swaps objects: both look right and the swap is a smooth move, so neither the appearance nor the jump gate
            // notices. While one is close, CSRT's box must also agree with the motion so far (people keep walking the same way
            // while they pass); a box pulled sideways towards the other one is not trusted -- the prediction is used instead.
            // only around a real CSRT box: around the prediction, a fast target itself (> 0.85 box from it) was taken for a lookalike (review)
            st.crowded = st.raw_ok && lookalike_near(f, st.raw);
            // The swap is gradual (a few px per frame, each step looks fine), so the box is compared with the path the target was
            // on when the lookalike came close -- its position then plus its velocity then -- accumulated over the whole crossing.
            // At full overlap the other one is not "next to" the target any more, so the guard stays on for a while after.
            if (st.crowded) { calm_ = 0; if (!crowd_ && last_.valid()) { crowd_ = true; c0_ = last_; cvx_ = vx_; cvy_ = vy_; cn_ = 0; } }
            else if (crowd_ && ++calm_ >= kCalm) crowd_ = false;
            if (crowd_) {
                cn_++; path = crowd_path();
                const double sz = std::max(path.w, path.h);
                if (geom && std::hypot(st.raw.cx() - path.cx(), st.raw.cy() - path.cy()) > std::max(3.0, 0.3 * sz)) {
                    // off the frozen straight path -- but the IMM prediction (which follows turns) agrees closely: the target TURNED, keep
                    // it (measured on an identical-twin clip: rejecting it here sent the search to the twin).
                    const bool imm_agrees = reacq_gate_ > 0 && pred.valid() &&
                                            std::hypot(st.raw.cx() - pred.cx(), st.raw.cy() - pred.cy()) <= 0.35 * sz &&
                                            std::hypot(st.raw.cx() - path.cx(), st.raw.cy() - path.cy()) <= 1.0 * sz;
                    if (!imm_agrees) geom = false;
                }
            }
            static const bool dbg = std::getenv("TRK_DEBUG") != nullptr;
            if (dbg) std::printf("[TRK] raw (%.0f,%.0f) path (%.0f,%.0f) crowd %d n %d | ", st.raw.cx(), st.raw.cy(), path.cx(), path.cy(), crowd_, cn_);
            if (dbg) std::printf("[TRK] raw_ok %d geom %d crowded %d score %.2f level %.2f -> %s  box %.0fx%.0f\n", st.raw_ok, geom, st.crowded, st.score, ap_.level(),
                                 (geom && ap_.accepts(st.score)) ? "accept" : "REJECT", st.raw.w, st.raw.h);
            if (geom && ap_.accepts(st.score)) {
                bad_ = 0; st.accepted = true; st.box = st.raw;
                // SIZE only from clearly visible frames: with its size tracking on, CSRT shrinks the box onto the visible part while
                // something covers the target (measured: 16x22 -> 9x12 under a passing car) and never grows it back. While the
                // appearance is weak, keep the last good size (position from CSRT); when it is clear again and CSRT's size has wandered
                // > 15% meanwhile, restart CSRT at the good size.
                // ... and the box still stands out from its surroundings: a box shrunk INSIDE the object has the object's own colour
                // all round it (a flat patch looks the same at any size; its contrast with the ring collapses)
                // Held only on the OCCLUSION signature -- the box's contrast with its ring collapsed (something covers it / the box sank
                // inside the object). A low appearance score alone is no reason: an oversized box is itself what lowers the score, so
                // holding on it kept a shrinking target's box at 42x42 for a 24x15 object until the track was lost (review, measured on a
                // panning clip: 78% -> 91% tracked without it, 1 -> 0 LOST; no change on the static clip).
                const bool clear = lock_contrast_ <= 0 || fl::box_contrast(f, st.raw) >= 0.6 * lock_contrast_;
                // Size: CSRT's own, always. Holding the last "good" size on unclear frames (and learning the appearance from that held
                // box) kept a really shrinking target at its old size until the track was lost: measured on a panning clip, one start
                // 44% -> 91% tracked and 1 -> 0 LOST without it, no change elsewhere, and the car-over-target self-test still passes.
                // Learning stays on every accepted frame (it is already gated by the appearance score): NOT learning on low-contrast
                // frames was measured too -- the model then never caught up with the shrinking target and the same start was lost.
                ap_.learn(f, st.box, st.score);
                seen(st.box);
                reanchor_run_ = 0;                                            // a real measurement
                // the gallery learns only from frames whose identity was just CONFIRMED (or the very first, at the operator's click), and
                // never right after a re-anchor / re-acquisition -- one fingerprint of a lookalike would let it pass every later check
                if (have_reid_ && clear && !crowd_ && (gallery_.empty() || (verify_ok_ && frame_n_ - last_jump_frame_ > 15))) reid_collect(f, st.box);
                if (have_reid_ && reid_verify(f, st.box)) {                 // the box is no longer the target (ReID): search for it
                    occluded_ = true; coast_ = 0; have_cand_ = false; streak_ = 0; st.identity_lost = true;
                    st.accepted = false; st.occluded = true;                  // and NOT reported as the target (review: the caller learned it)
                }
                return st;
            }
            // CSRT lost it or jumped -- most often because the CAMERA moved (approach, yaw, gimbal tilt) beyond its search window.
            // The prediction is camera-motion compensated: if the target still looks like itself THERE, restart CSRT on it. A real
            // occlusion scores ~0 at the prediction (measured), so this does not reopen the switch-to-the-occluder failure.
            // (While a lookalike is close: at the point on the target's own path instead -- the Kalman prediction has already
            // absorbed some of the drift towards the other one.)
            if (path.valid() && path.x >= 0 && path.y >= 0 && path.x + path.w <= f.cols && path.y + path.h <= f.rows) {
                const double ps = ap_.score(f, path);
                // (requiring the stricter re-acquisition score here was tried: a shrinking target whose CSRT box stayed oversized
                // could then never be restarted and was lost -- measured on a panning clip, 91% -> 44% tracked on one start)
                if (reanchor_run_ < 3 && ap_.accepts(ps) && (pick_scale(path), engine_.init(f, path))) {
                    reanchor_run_++; last_jump_frame_ = frame_n_; csrt_w_ = path.w; bad_ = 0; st.accepted = true; st.box = path; st.score = ps; st.reanchored = true; seen(path); return st; }
            }
            if (++bad_ < 2) return st;                                    // one doubtful frame: just don't accept it
            occluded_ = true; coast_ = 0; have_cand_ = false; streak_ = 0;   // hidden (or drifting): stop feeding CSRT
            reid_pending_ = 0;                                            // a fingerprint asked for a moment ago may already show the occluder
        }
        st.occluded = true; coast_++;
        if (crowd_) { cn_++; path = crowd_path(); if (++calm_ >= kCalm + 10) crowd_ = false; }   // hidden in the crossing: its own path
        // OCCLUSION SEARCH. The prediction assumes the target kept going straight; behind a car it may have stopped or turned
        // back. So the search covers everywhere it can have walked since it was last SEEN (a circle growing with its speed and
        // the hidden time) and a candidate counts as "close" when it is near EITHER the prediction or the last-seen spot. When
        // two lookalikes are about equally good (same colours AND about as close) it is a coin toss -- nothing is re-acquired:
        // keep searching and, if it stays ambiguous, let the track go LOST and the operator click again. A wrong lock is worse.
        const fl::Box base = crowd_ ? path : last_.valid() ? last_ : pred;
        const double sz = std::max(std::max(6.0, base.w), std::max(6.0, base.h));
        const double vmax = std::max(1.5 * std::hypot(vx_, vy_), 0.08 * sz);  // px/frame: faster than seen, or a slow walk for a static target
        const double reach = sz + 6 + vmax * coast_;
        const double r = std::min(reach, 6.0 * sz + 40);
        std::vector<fl::Appearance::Cand> cs; ap_.candidates(f, base, r, 6, cs);
        struct Scored { fl::Box b; double app, total, sim = -1; };
        std::vector<Scored> good;
        for (const auto& c : cs) {
            if (!ap_.reacquires(c.score)) continue;
            const double dl = std::hypot(c.box.cx() - base.cx(), c.box.cy() - base.cy());
            if (dl > reach) continue;                                     // it cannot have got there
            // ... and where the IMM says it can be (its prediction and uncertainty -- the uncertainty grows while hidden, and the IMM
            // follows turns): measured on a clip with an IDENTICAL twin passing close, the search otherwise re-acquired the twin on
            // 31% of tracked frames -- the only candidate it saw, beside a stale straight-line path. Nothing consistent: keep hiding
            // (then LOST and hover) rather than lock the wrong one.
            if (reacq_gate_ > 0 && pred.valid() && std::hypot(c.box.cx() - pred.cx(), c.box.cy() - pred.cy()) > reacq_gate_) continue;
            const double dp = path.valid() ? std::hypot(c.box.cx() - path.cx(), c.box.cy() - path.cy()) : dl;
            // distance from where it should be costs more (was 0.1 per box size, capped at 2): a lookalike two box widths away lost only
            // 0.2 against the target's own appearance and could win a re-acquisition (live: re-locks 87-158 px away). REACQ_W: A/B.
            static const double wd = std::getenv("REACQ_W") ? std::atof(std::getenv("REACQ_W")) : 0.2;
            good.push_back(Scored{c.box, c.score, c.score - wd * std::min(3.0, std::min(dp, dl) / sz)});
        }
        std::sort(good.begin(), good.end(), [](const Scored& x, const Scored& y) { return x.total > y.total; });
        auto tie = [&]() { return good.size() >= 2 && fl::iou(good[0].b, good[1].b) < 0.2 && good[1].total > good[0].total - 0.06; };
        // ReID: only where a decision is due -- a tie between lookalikes (every 3rd hidden frame: each fingerprint costs ~60 ms),
        // or the frame that would re-acquire. Fingerprints the top 3; the DINOv2 similarity moves the totals, and a candidate
        // that is clearly NOT the target (shape) is dropped. Without a gallery / sidecar: unchanged.
        const int need0 = good.size() >= 2 ? 3 : 2;
        const bool would_accept = !good.empty() && !tie() && have_cand_ && streak_ + 1 >= need0;
        if (have_reid_ && !gallery_.empty() && !good.empty() && ((tie() && coast_ % 3 == 1) || would_accept)) {
            std::vector<Scored> kept; std::vector<fl::Box> bs; std::vector<double> sims;
            for (size_t k = 0; k < good.size() && k < 3; k++) bs.push_back(good[k].b);
            const bool all_ok = reid_sims(f, bs, sims);
            if (all_ok) for (size_t k = 0; k < bs.size(); k++) {
                if (sims[k] < kReidMin) continue;                         // not the target's shape at all
                Scored g = good[k]; g.total += kReidW * (sims[k] - 0.8); g.sim = sims[k]; kept.push_back(g);
            }
            if (all_ok) {
                for (size_t k = 3; k < good.size(); k++) kept.push_back(good[k]);   // not fingerprinted: as they were (ranked lower anyway)
                good.swap(kept);
                std::sort(good.begin(), good.end(), [](const Scored& x, const Scored& y) { return x.total > y.total; });
                st.reid_used = true;
            }
        }
        bool ok = !good.empty();
        if (ok && tie()) { ok = false; st.ambiguous = true; }
        // STRICT RE-ACQUISITION with ReID (operator report: it re-locked another object of the same colour): with a gallery, the
        // frame that would re-acquire must have a DINOv2 confirmation of THIS candidate (>= kReidAccept); no answer = wait, never guess
        if (ok && would_accept && have_reid_ && !gallery_.empty() && (good[0].sim < kReidAccept)) { ok = false; st.ambiguous = true; }
        // DISTURBANCE (the gimbal moving, a zoom change, the drone pitching / turning hard): the whole image is moving -- no
        // re-acquisition now (a jump to whatever matches is how a lookalike got locked); CSRT + camera-motion compensation carry on,
        // and once the image is steady the ReID check runs at once (see set_disturbed)
        if (disturbed_) { ok = false; have_cand_ = false; streak_ = 0; }
        static const bool dbg2 = std::getenv("TRK_DEBUG") != nullptr;
        if (dbg2) std::printf("[TRK] hidden %d reach %.0f cands %zu good %zu %s%s\n", coast_, reach, cs.size(), good.size(), st.ambiguous ? "AMBIGUOUS " : "",
                              ok ? "candidate" : "");
        if (ok) {
            const fl::Box c{good[0].b.cx() - base.w / 2, good[0].b.cy() - base.h / 2, base.w, base.h};   // the size it had
            // consecutive consistent matches; one more while another lookalike is in view at all
            const int need = good.size() >= 2 ? 3 : 2;
            streak_ = (have_cand_ && fl::iou(c, cand_) > 0.3) ? streak_ + 1 : 1;
            cand_ = c; have_cand_ = true;
            if (streak_ >= need && (pick_scale(c), engine_.init(f, c))) {
                last_jump_frame_ = frame_n_; verify_ok_ = false; verify_next_ = 0;   // re-found: verify its identity at once
                csrt_w_ = c.w;
                occluded_ = false; bad_ = 0; have_cand_ = false; coast_ = 0; streak_ = 0;
                vx_ = vy_ = 0; last_ = c; hist_.clear(); hist_.push_back(c); crowd_ = false;   // it may be moving another way now
                st.accepted = true; st.reacquired = true; st.occluded = false; st.box = c; st.score = good[0].app; return st;
            }
        } else {
            have_cand_ = false; streak_ = 0;
        }
        return st;
    }
private:
    static constexpr int kCalm = 8;                                       // frames without a lookalike before the crossing guard ends
    fl::Box crowd_path() const { return fl::Box{c0_.x + cvx_ * cn_, c0_.y + cvy_ * cn_, c0_.w, c0_.h}; }
    // gallery upkeep (async): one fingerprint at a time, the first right after the lock, then one every ~60 clear frames; the 6
    // most recent are kept (the target's look changes as it turns and the view changes)
    void reid_collect(const cv::Mat& f, const fl::Box& b) {
        if (reid_pending_) {
            std::vector<float> v;
            if (reid_.poll(reid_pending_, v)) { reid_pending_ = 0; if (!v.empty()) { reid_ok_once_ = true; gallery_.push_back(std::move(v)); if (gallery_.size() > 6) gallery_.erase(gallery_.begin()); } }
            else if (frame_n_ > reid_next_ + 40) reid_pending_ = 0;       // no reply: give up on that one
            return;
        }
        if (frame_n_ < reid_next_ || !reid_.request) return;
        reid_pending_ = reid_.request(f, b); reid_next_ = frame_n_ + (gallery_.empty() ? 5 : 60);
    }
    // IDENTITY CHECK (ReID, asynchronous): every ~8 frames the tracked box is fingerprinted and compared with the target's gallery.
    // CSRT can slide onto a lookalike smoothly (a crossing, a same-colour object next to it) where no single frame looks wrong; the
    // shape fingerprint does. Two checks in a row clearly below the target's own similarity -> true: the caller drops to the
    // hidden-target search (which re-acquires with ReID). Needs a gallery of >= 2 fingerprints; never blocks the frame.
    bool reid_verify(const cv::Mat& f, const fl::Box& b) {
        if (!have_reid_ || gallery_.empty() || !reid_.request) return false;   // from the first fingerprint (taken at the click)
        if (verify_pending_) {
            std::vector<float> v;
            if (!reid_.poll(verify_pending_, v)) { if (frame_n_ > verify_sent_ + 40) verify_pending_ = 0; return false; }
            verify_pending_ = 0;
            if (v.empty()) return false;
            double sim = -1; for (const auto& g : gallery_) if (g.size() == v.size()) { double d = 0; for (size_t k = 0; k < v.size(); k++) d += (double)g[k] * v[k]; sim = std::max(sim, d); }
            verify_ok_ = sim >= kReidAccept;
            if (sim > -1 && sim < kVerifyMin) { if (++verify_bad_ >= 2) { verify_bad_ = 0; std::printf("[REID] the tracked box no longer looks like the target (similarity %.2f): searching\n", sim); return true; } }
            else verify_bad_ = 0;
            return false;
        }
        if (frame_n_ >= verify_next_) { verify_pending_ = reid_.request(f, b); verify_sent_ = frame_n_; verify_next_ = frame_n_ + 8; }
        return false;
    }
    void clear_state(const fl::Box& b) {
        occluded_ = false; bad_ = 0; coast_ = 0; have_cand_ = false; streak_ = 0; vx_ = vy_ = 0; hist_.clear(); crowd_ = false; calm_ = cn_ = 0;
        last_ = b; if (b.valid()) hist_.push_back(b);
    }
    static constexpr double kReidMin = 0.5, kReidW = 1.0, kVerifyMin = 0.45, kReidAccept = 0.6;
    bool disturbed_ = false; double reacq_gate_ = 0;
    int reanchor_run_ = 0; long last_jump_frame_ = -1000; bool verify_ok_ = false;
    uint32_t verify_pending_ = 0; long verify_sent_ = 0, verify_next_ = 0; int verify_bad_ = 0;
    Reid reid_; bool have_reid_ = false; std::vector<std::vector<float>> gallery_; uint32_t reid_pending_ = 0; long reid_next_ = 0, frame_n_ = 0, reid_rest_until_ = 0; int reid_fails_ = 0; bool reid_ok_once_ = false;
    void seen(const fl::Box& b) {                                         // an accepted box: last-seen spot + its speed (px/frame)
        last_ = b; hist_.push_back(b); if (hist_.size() > 8) hist_.erase(hist_.begin());
        if (hist_.size() >= 3 && !crowd_) {                               // frozen while a lookalike is close (see step)
            const double n = (double)(hist_.size() - 1);
            vx_ = (hist_.back().cx() - hist_.front().cx()) / n; vy_ = (hist_.back().cy() - hist_.front().cy()) / n;
        }
    }
    // another blob that looks like the target (same test as a re-acquisition) within ~2 target sizes of it
    bool lookalike_near(const cv::Mat& f, const fl::Box& at) const {
        if (!at.valid()) return false;
        const double sz = std::max(at.w, at.h);
        std::vector<fl::Appearance::Cand> cs; ap_.candidates(f, at, 2.0 * sz + 4, 4, cs, &at);
        for (const auto& c : cs) {
            // overlapping the target box by much is the target itself (a shifted window on the same object scores high too,
            // measured live on a car): a lookalike is a separate blob, at least ~a box width or height away
            if (std::fabs(c.box.cx() - at.cx()) < 0.85 * at.w && std::fabs(c.box.cy() - at.cy()) < 0.85 * at.h) continue;
            if (ap_.reacquires(c.score)) return true;
        }
        return false;
    }
    double scale_, padding_; bool fast_; double tf_; double full_res_below_ = 0;
    TrackerEngine engine_; fl::Appearance ap_;
    bool occluded_ = false; int bad_ = 0, coast_ = 0, streak_ = 0; bool have_cand_ = false; fl::Box cand_, last_;
    std::vector<fl::Box> hist_; double vx_ = 0, vy_ = 0; double csrt_w_ = 0;   // the box width CSRT was last started with
    double lock_contrast_ = 0;                                          // how much the locked box stood out from its surroundings
    bool crowd_ = false; int calm_ = 0, cn_ = 0; fl::Box c0_; double cvx_ = 0, cvy_ = 0;   // crossing guard: the path when it began
};

}  // namespace (closed briefly: the self-test entry point below is called from run_selftest at global scope)

// Tracking benchmark on a recording (no vehicle, no display, deterministic):
//   target_tracker --replay VIDEO X Y FRAME [TRACK_SCALE] > out.csv
// clicks (X, Y) on frame FRAME (0-based) through the real click-lock path, then runs exactly the live tracking core on every
// following frame -- camera-motion compensation, Kalman prediction, LockTracker (CSRT + appearance + occlusion search) -- and
// prints "n,state,x,y,w,h" per frame (state: 0 tracked, 1 doubtful, 2 hidden/searching, 3 lost after 2.5 s).
int run_replay(int argc, char** argv) {
    cv::VideoCapture cap(argv[2]);
    if (!cap.isOpened()) { std::fprintf(stderr, "cannot open %s\n", argv[2]); return 2; }
    const double cx = std::atof(argv[3]), cy = std::atof(argv[4]); const int f0 = std::atoi(argv[5]);
    const double scale = argc >= 7 ? std::atof(argv[6]) : 0.5;
    double fps = cap.get(cv::CAP_PROP_FPS); if (!(fps > 1 && fps < 200)) fps = 20;
    const int lost_frames = (int)std::lround(2.5 * fps);
    const double pad = std::getenv("CSRT_PAD") ? std::atof(std::getenv("CSRT_PAD")) : 3.0, tf = std::getenv("CSRT_TF") ? std::atof(std::getenv("CSRT_TF")) : 2.5;
    fl::CameraMotion cm; fl::TargetFilter kf(fl::Box(), std::getenv("KF_R") ? std::atof(std::getenv("KF_R")) : 0.05,
                                          std::getenv("KF_QP") ? std::atof(std::getenv("KF_QP")) : 1.0, std::getenv("KF_QV") ? std::atof(std::getenv("KF_QV")) : 0.5);
    LockTracker lt(scale, pad, true, tf);
    lt.set_full_res_below(std::getenv("FULL_RES_BELOW") ? std::atof(std::getenv("FULL_RES_BELOW")) : 20.0);   // as live (--full-res-below)
    std::deque<cv::Mat> hist; cv::Mat f; int n = -1; bool tracking = false; int hidden = 0;
    std::printf("n,state,x,y,w,h,px,py,pw,ph\n");   // px..ph: the Kalman prediction for this frame (before its update)
    while (cap.read(f)) {
        n++;
        const fl::Box excl = tracking ? kf.box() : fl::Box();
        const cv::Mat A = cm.step(f, tracking ? &excl : nullptr);
        hist.push_back(f.clone()); while (hist.size() > 4) hist.pop_front();
        if (n < f0) continue;
        if (n == f0) {
            const fl::LockJob j = fl::run_lock_job(f.clone(), hist.size() >= 4 ? hist.front() : cv::Mat(), std::min(1.0, scale), cx, cy, 0.07 * f.cols, 0);
            if (!j.ok || !lt.init(f, j.box)) { std::fprintf(stderr, "lock refused: %s\n", j.why.c_str()); return 1; }
            kf.reset(j.box); tracking = true;
            std::fprintf(stderr, "locked %.0fx%.0f at (%.0f,%.0f) via %s\n", j.box.w, j.box.h, j.box.cx(), j.box.cy(), j.how.c_str());
            std::printf("%d,0,%.1f,%.1f,%.1f,%.1f\n", n, j.box.x, j.box.y, j.box.w, j.box.h);
            if (std::getenv("REPLAY_LOCK_ONLY")) return 0;
            continue;
        }
        if (!tracking) { std::printf("%d,3,0,0,0,0\n", n); continue; }
        if (!A.empty()) { kf.warp(A); lt.warp(A); }
        kf.predict(1.0);
        const fl::Box pred = kf.box();
        auto plausible = [&](const fl::Box& b) { return fl::box_plausible(pred, b, f.cols, f.rows, fl::PlausCfg(), 1.0) && fl::filter_gate(kf, pred, b); };
        lt.set_reacq_gate(fl::gate_radius(kf, pred, 1.0));
        const LockTracker::Step ls = lt.step(f, pred, plausible);
        int state;
        if (ls.accepted) {
            if (ls.reacquired) kf.reset(ls.box); else if (!ls.reanchored) kf.update(ls.box); else kf.damp_velocity(0.97);
            hidden = 0; state = 0;
        } else {
            kf.damp_velocity(ls.occluded ? 0.97 : 0.9);
            state = ls.occluded ? 2 : 1;
            if (++hidden >= lost_frames) { tracking = false; state = 3; }
        }
        const fl::Box b = ls.accepted ? ls.box : kf.box();
        std::printf("%d,%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%.1f\n", n, state, b.x, b.y, b.w, b.h, pred.x, pred.y, pred.w, pred.h);
    }
    return 0;
}

// CSRT scale: does the box follow a target that GROWS (the drone approaching)? Found on recorded sim footage: the box stayed exactly
// 28x28 while the red box grew to 40x42 (IoU 0.92 -> 0.47).
void csrt_scale_selftest() {
    using namespace fl;
    cv::RNG rng(11);
    cv::Mat bg(480, 640, CV_8UC3); rng.fill(bg, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255)); cv::GaussianBlur(bg, bg, cv::Size(5, 5), 1.5);
    // a structured object (smooth colour blobs + a dark frame, like a vehicle seen from above), not pixel noise: zooming into noise
    // creates new noise, which no size estimate -- CSRT's or ours -- can follow
    cv::Mat tex(200, 200, CV_8UC3); rng.fill(tex, cv::RNG::UNIFORM, cv::Scalar(0, 0, 0), cv::Scalar(255, 255, 255)); cv::GaussianBlur(tex, tex, cv::Size(0, 0), 14.0);
    cv::normalize(tex, tex, 0, 255, cv::NORM_MINMAX); cv::rectangle(tex, cv::Rect(0, 0, 200, 200), cv::Scalar(30, 30, 30), 14); cv::rectangle(tex, cv::Rect(70, 40, 60, 120), cv::Scalar(220, 220, 230), cv::FILLED);
    auto frame = [&](int sz, Box& b) { cv::Mat f = bg.clone(), t; cv::resize(tex, t, cv::Size(sz, sz)); t.copyTo(f(cv::Rect(320 - sz / 2, 240 - sz / 2, sz, sz)));
                                      b = Box{320.0 - sz / 2, 240.0 - sz / 2, (double)sz, (double)sz}; return f; };
    if (std::getenv("TRK_DEBUG")) for (bool fast : {false, true}) {          // bare CSRT, for diagnosis
        Box b; cv::Mat f = frame(56, b); TrackerEngine e(1.0, 3.0, fast, 2.5); e.init(f, b); Box r = b;
        for (int i = 1; i <= 60; i++) { Box tb; cv::Mat g = frame(56 + i, tb); e.update(g, r); }
        std::printf("bare CSRT (%s): true 116 px, box %.0fx%.0f\n", fast ? "fast params" : "OpenCV defaults", r.w, r.h);
    }
    for (double scale : {1.0, 0.5}) {
        Box b; cv::Mat f = frame(56, b); LockTracker lt(scale, 3.0, true, 2.5); CHECK(lt.init(f, b));
        BoxKalman kf(b); Box last = b;
        for (int i = 1; i <= 60; i++) {
            Box tb; cv::Mat g = frame(56 + i, tb); kf.predict();
            LockTracker::Step st = lt.step(g, kf.box(), [&](const Box& x) { return box_plausible(kf.box(), x, g.cols, g.rows); });
            if (st.accepted) { kf.update(st.box); last = st.box; }
        }
        Box tb; frame(116, tb);
        if (std::getenv("TRK_DEBUG")) std::printf("csrt scale (track scale %.1f): true 116 px, box %.0fx%.0f, IoU %.2f\n", scale, last.w, last.h, iou(last, tb));
        CHECK(iou(last, tb) > 0.7);                                       // the box grew with the target
    }
}

// Self-test of the whole occlusion path with the REAL CSRT: a small red target moves right across textured ground; a bigger
// textured grey "car" sweeps left across it and hides it completely for several frames. The tracker must not follow the car,
// and must be back on the target after it has passed.
void occlusion_selftest() {
    using namespace fl;
    cv::RNG rng(7);
    cv::Mat ground(240, 320, CV_8UC3); rng.fill(ground, cv::RNG::NORMAL, cv::Scalar(90, 90, 90), cv::Scalar(14, 14, 14));
    cv::Mat car(44, 66, CV_8UC3); rng.fill(car, cv::RNG::NORMAL, cv::Scalar(140, 140, 145), cv::Scalar(45, 45, 45));
    cv::rectangle(car, cv::Rect(8, 6, 50, 12), cv::Scalar(40, 40, 40), cv::FILLED);   // windows: some structure
    auto frame = [&](int i, Box& tgt, Box& occ) {
        cv::Mat f = ground.clone();
        tgt = Box{60.0 + 2.0 * i, 110, 16, 22};
        // a shaded object (lit top face, darker side, a dark edge) -- a perfectly flat patch has no size information and CSRT's
        // size search drifts on it, which no real target looks like
        cv::rectangle(f, cv::Rect((int)tgt.x, (int)tgt.y, 16, 22), cv::Scalar(20, 20, 140), cv::FILLED);
        cv::rectangle(f, cv::Rect((int)tgt.x + 2, (int)tgt.y + 2, 12, 9), cv::Scalar(40, 40, 230), cv::FILLED);
        cv::rectangle(f, cv::Rect((int)tgt.x + 2, (int)tgt.y + 12, 12, 8), cv::Scalar(25, 25, 175), cv::FILLED);
        occ = Box{250.0 - 5.0 * i, 98, 66, 44};
        cv::Rect r((int)occ.x, (int)occ.y, 66, 44); cv::Rect rc = r & cv::Rect(0, 0, f.cols, f.rows);
        if (rc.area() > 0) car(cv::Rect(rc.x - r.x, rc.y - r.y, rc.width, rc.height)).copyTo(f(rc));
        return f;
    };
    Box tgt, occ; cv::Mat f0 = frame(0, tgt, occ);
    LockTracker lt(1.0, 3.0, true, 2.5); CHECK(lt.init(f0, tgt));
    BoxKalman kf(tgt); bool ever_occluded = false, followed_car = false; Box last = tgt;
    for (int i = 1; i < 45; i++) {
        cv::Mat f = frame(i, tgt, occ);
        kf.predict();
        LockTracker::Step s = lt.step(f, kf.box(), [&](const Box& b) { return box_plausible(kf.box(), b, f.cols, f.rows); });
        if (s.occluded) ever_occluded = true;
        if (s.accepted) {
            kf.update(s.box); last = s.box;
            if (iou(s.box, occ) > 0.15 && iou(s.box, tgt) < 0.2) followed_car = true;
        } else kf.damp_velocity(0.9);
    }
    CHECK(ever_occluded);                                                 // the occlusion was noticed
    CHECK(!followed_car);                                                 // never accepted a box on the occluder
    if (std::getenv("TRK_DEBUG")) std::printf("occlusion test: last box %.0f,%.0f %.0fx%.0f  target %.0f,%.0f %.0fx%.0f\n", last.x, last.y, last.w, last.h, tgt.x, tgt.y, tgt.w, tgt.h);
    CHECK(iou(last, tgt) > 0.5);                                          // back on the target after the car passed
    CHECK(!lt.occluded());

    // (2) TWO IDENTICAL people crossing (the sim's actors all share one skin): the target walks right, a lookalike walks left
    // along a slightly lower line, they overlap for several frames -- at several speeds, the other one in front or behind. The
    // lock must still be on the one that kept walking right.
    struct Cross { double va, vb; bool b_in_front; };
    for (const Cross& cfg : {Cross{3, -3, true}, Cross{3, -3, false}, Cross{2, -4, true}, Cross{4, -1.5, true}, Cross{1.5, -3, true}}) {
        auto person = [&](cv::Mat& f, double x, double y) {
            cv::rectangle(f, cv::Rect((int)x, (int)y, 14, 24), cv::Scalar(30, 30, 200), cv::FILLED);
            cv::rectangle(f, cv::Rect((int)x + 3, (int)y, 8, 7), cv::Scalar(120, 170, 220), cv::FILLED);   // head
        };
        auto fr = [&](int i, Box& a, Box& b) {
            cv::Mat f = ground.clone();
            a = Box{60.0 + cfg.va * i, 110, 14, 24}; b = Box{240.0 + cfg.vb * i, 116, 14, 24};
            if (cfg.b_in_front) { person(f, a.x, a.y); person(f, b.x, b.y); } else { person(f, b.x, b.y); person(f, a.x, a.y); }
            return f;
        };
        const int n = (int)std::lround(200.0 / (cfg.va - cfg.vb)) + 18;  // until well after they have passed
        Box a, b; cv::Mat g0 = fr(0, a, b);
        LockTracker l2(1.0, 3.0, true, 2.5); CHECK(l2.init(g0, a));
        BoxKalman k2(a); Box last2 = a; bool swapped = false;
        for (int i = 1; i < n; i++) {
            cv::Mat f = fr(i, a, b);
            k2.predict();
            LockTracker::Step st = l2.step(f, k2.box(), [&](const Box& x) { return box_plausible(k2.box(), x, f.cols, f.rows); });
            if (st.accepted) {
                if (st.reacquired) k2.reset(st.box); else k2.update(st.box);
                last2 = st.box;
                if (iou(st.box, b) > 0.3 && iou(st.box, a) < 0.1) swapped = true;
            } else k2.damp_velocity(st.occluded ? 0.97 : 0.9);
        }
        if (swapped || iou(last2, a) <= 0.4) std::printf("  crossing va %.1f vb %.1f %s: swapped %d final iou %.2f\n", cfg.va, cfg.vb, cfg.b_in_front ? "in front" : "behind", swapped, iou(last2, a));
        CHECK(!swapped);                                                  // never followed the other person
        CHECK(iou(last2, a) > 0.4);                                       // still on the target after they passed
    }
    // (3) the target walks right, goes behind a parked car and TURNS BACK there; an identical person stands still beyond the car,
    // right where the old straight-line prediction points. It must not be taken for the target.
    {
        auto person = [&](cv::Mat& f, double x, double y) {
            cv::rectangle(f, cv::Rect((int)x, (int)y, 14, 24), cv::Scalar(30, 30, 200), cv::FILLED);
            cv::rectangle(f, cv::Rect((int)x + 3, (int)y, 8, 7), cv::Scalar(120, 170, 220), cv::FILLED);
        };
        const cv::Rect carR(150, 96, 60, 50);
        auto fr = [&](int i, Box& a) {
            cv::Mat f = ground.clone();
            const double x = i <= 30 ? 80.0 + 2.5 * i : 155.0 - 2.5 * (i - 30);   // reverses at x=155, behind the car
            a = Box{x, 110, 14, 24};
            person(f, a.x, a.y); person(f, 232, 110);                     // the lookalike, waiting beyond the car
            car(cv::Rect(0, 0, 60, 44)).copyTo(f(cv::Rect(carR.x, carR.y, 60, 44)));
            cv::rectangle(f, cv::Rect(carR.x, carR.y + 44, 60, 6), cv::Scalar(140, 140, 145), cv::FILLED);
            return f;
        };
        Box a; cv::Mat g0 = fr(0, a);
        LockTracker l3(1.0, 3.0, true, 2.5); CHECK(l3.init(g0, a));
        BoxKalman k3(a); bool wrong = false;
        for (int i = 1; i < 60; i++) {
            cv::Mat f = fr(i, a);
            k3.predict();
            LockTracker::Step st = l3.step(f, k3.box(), [&](const Box& x) { return box_plausible(k3.box(), x, f.cols, f.rows); });
            if (st.accepted) {
                if (st.reacquired) k3.reset(st.box); else k3.update(st.box);
                if (iou(st.box, Box{232, 110, 14, 24}) > 0.3) wrong = true;
            } else k3.damp_velocity(st.occluded ? 0.97 : 0.9);
        }
        CHECK(!wrong);                                                    // never locked the lookalike beyond the car
    }
    // (4) ReID: the target walks right behind a parked car; a person of the SAME colours, hidden there too, walks out LEFT --
    // near where the target was last seen, before the target itself comes out on the right. Colour can't tell them apart; the
    // (stand-in) fingerprint can. With it the lock must come back on the target and never on the other one.
    for (int use_reid = 0; use_reid < 2; use_reid++) {
        auto person = [&](cv::Mat& f, double x, double y) {
            cv::rectangle(f, cv::Rect((int)x, (int)y, 14, 24), cv::Scalar(30, 30, 200), cv::FILLED);
            cv::rectangle(f, cv::Rect((int)x + 3, (int)y, 8, 7), cv::Scalar(120, 170, 220), cv::FILLED);
        };
        const cv::Rect carR(140, 96, 80, 50);
        Box a, b;
        auto fr = [&](int i) {
            cv::Mat f = ground.clone();
            a = Box{80.0 + 2.0 * i, 110, 14, 24};
            b = Box{i < 30 ? 170.0 : 170.0 - 3.0 * (i - 30), 112, 14, 24};   // waits behind the car, then walks out left
            person(f, b.x, b.y); person(f, a.x, a.y);
            car(cv::Rect(0, 0, 66, 44)).copyTo(f(cv::Rect(carR.x, carR.y, 66, 44)));
            cv::rectangle(f, cv::Rect(carR.x + 66, carR.y, 14, 50), cv::Scalar(140, 140, 145), cv::FILLED);
            cv::rectangle(f, cv::Rect(carR.x, carR.y + 44, 66, 6), cv::Scalar(140, 140, 145), cv::FILLED);
            return f;
        };
        cv::Mat g0 = fr(0);
        LockTracker l4(1.0, 3.0, true, 2.5); CHECK(l4.init(g0, a));
        int reid_calls = 0;
        if (use_reid) {
            LockTracker::Reid r;                                          // stand-in fingerprint: [1,0] on the target, [0,1] elsewhere
            r.request = [](const cv::Mat&, const Box&) { return 1u; };
            r.poll = [](uint32_t, std::vector<float>& v) { v = {1.f, 0.f}; return true; };
            r.sync = [&](const cv::Mat&, const std::vector<Box>& xs, std::vector<std::vector<float>>& vs) {
                vs.clear();
                for (const Box& x : xs) { reid_calls++; vs.push_back(std::hypot(x.cx() - a.cx(), x.cy() - a.cy()) < 8 ? std::vector<float>{1.f, 0.f} : std::vector<float>{0.f, 1.f}); }
                return true; };
            l4.set_reid(r);
        }
        BoxKalman k4(a); bool wrong = false, back = false, used = false;
        for (int i = 1; i < 95; i++) {
            cv::Mat f = fr(i);
            k4.predict();
            LockTracker::Step st = l4.step(f, k4.box(), [&](const Box& x) { return box_plausible(k4.box(), x, f.cols, f.rows); });
            used |= st.reid_used;
            if (st.accepted) {
                if (st.reacquired) k4.reset(st.box); else k4.update(st.box);
                if (iou(st.box, b) > 0.3 && iou(st.box, a) < 0.1) wrong = true;
                if (i > 45 && iou(st.box, a) > 0.4) back = true;
            } else k4.damp_velocity(st.occluded ? 0.97 : 0.9);
        }
        if (std::getenv("TRK_DEBUG") || (use_reid && (wrong || !back)))
            std::printf("  reid test (%s): wrong %d back %d gallery %zu fingerprints %d used %d\n", use_reid ? "ReID" : "no ReID", wrong, back, l4.reid_gallery(), reid_calls, used);
        if (use_reid) { CHECK(l4.reid_gallery() >= 1); CHECK(used); CHECK(!wrong); CHECK(back); }
    }
    // (5) ReID IDENTITY CHECK: the tracked box keeps looking fine to CSRT and the colour check, but its fingerprint changes (CSRT slid
    // onto a same-colour object of another shape) -- stand-in: fingerprints [1,0] until frame 100, [0,1] after. The tracker must drop
    // to the search within a few checks after 100, and never before.
    {
        cv::Mat f0 = ground.clone(); const Box tb{150, 100, 16, 22};
        cv::rectangle(f0, cv::Rect(150, 100, 16, 22), cv::Scalar(20, 20, 140), cv::FILLED); cv::rectangle(f0, cv::Rect(152, 102, 12, 9), cv::Scalar(40, 40, 230), cv::FILLED);
        LockTracker l5(1.0, 3.0, true, 2.5); CHECK(l5.init(f0, tb));
        int frame = 0; uint32_t next_id = 0; std::map<uint32_t, int> sent_at;
        LockTracker::Reid r;
        r.request = [&](const cv::Mat&, const Box&) { sent_at[++next_id] = frame; return next_id; };
        r.poll = [&](uint32_t id, std::vector<float>& v) { v = sent_at[id] < 100 ? std::vector<float>{1.f, 0.f} : std::vector<float>{0.f, 1.f}; return true; };
        r.sync = [&](const cv::Mat&, const std::vector<Box>& xs, std::vector<std::vector<float>>& vs) { vs.assign(xs.size(), {0.f, 1.f}); return true; };
        l5.set_reid(r);
        BoxKalman k5(tb); int lost_at = -1;
        for (frame = 1; frame < 160 && lost_at < 0; frame++) {
            k5.predict();
            LockTracker::Step st = l5.step(f0, k5.box(), [&](const Box& x) { return box_plausible(k5.box(), x, f0.cols, f0.rows); });
            if (st.identity_lost) lost_at = frame;
            if (st.accepted) k5.update(st.box);
        }
        if (std::getenv("TRK_DEBUG")) std::printf("  identity check: gallery %zu, dropped at frame %d\n", l5.reid_gallery(), lost_at);
        CHECK(lost_at >= 100 && lost_at < 135);
    }
}

namespace {

// ---- H.264 video out: RTP over UDP (QGC's "UDP h.264" source; default QGC port 5600), or a recording file ------------
// `sink_tail` is everything after h264parse: the RTP/UDP leg for --video-out, or a Matroska file for --record (Matroska stays
// playable up to the last written cluster even if the program is killed, unlike MP4 whose index is only written at the end).
class GstH264Out {
public:
    static std::string udp_sink(const std::string& host, int port) {
        return "rtph264pay config-interval=1 pt=96 ! udpsink host=" + host + " port=" + std::to_string(port) + " sync=false";
    }
    static std::string file_sink(const std::string& path) { return "matroskamux ! filesink location=\"" + path + "\""; }
    GstH264Out(const std::string& sink_tail, const std::string& label, int w, int h, double fps, int bitrate_kbps, bool hw) {
        gst_init(nullptr, nullptr);
        std::ostringstream p;
        p << "appsrc name=src is-live=true format=time do-timestamp=true caps=video/x-raw,format=BGR,width=" << w << ",height=" << h
          << ",framerate=" << (int)std::lround(fps) << "/1 ! queue leaky=downstream max-size-buffers=2 ! videoconvert ! ";   // a slow encoder drops frames, never queues latency
        if (hw) p << "nvvidconv ! nvv4l2h264enc bitrate=" << bitrate_kbps * 1000 << " insert-sps-pps=true profile=0 ! ";   // profile 0 = Baseline (same QGC reason as below)
        // profile=baseline (as a CAPS FILTER, not an x264enc property -- it has no such property, confirmed
        // live: "profile" only exists on x264enc's src pad template, constrained the same way any caps are,
        // not via element-property syntax): found live that QGC's bundled GStreamer (inside its AppImage, a
        // different build than the system's) was flagging nearly every slice NAL from the unconstrained
        // default profile as "broken/invalid" and dropping it -- a genuine encoder/decoder incompatibility,
        // not a load problem (recurred identically under both heavy and light system load; the system's OWN
        // gst-launch decoded the exact same stream with zero errors the whole time). Baseline disables
        // B-frames/CABAC/8x8 transform -- the most universally-compatible H.264 subset, at some cost in
        // compression efficiency that doesn't matter for a low-res real-time control feed.
        else p << "x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << bitrate_kbps << " key-int-max=30 ! video/x-h264,profile=baseline ! ";
        p << "h264parse ! " << sink_tail;
        GError* err = nullptr;
        pipe_ = gst_parse_launch(p.str().c_str(), &err);
        if (!pipe_ || err) { std::string m = err ? err->message : "unknown"; if (err) g_error_free(err); throw std::runtime_error("video-out pipeline failed: " + m); }
        src_ = gst_bin_get_by_name(GST_BIN(pipe_), "src");
        if (gst_element_set_state(pipe_, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE) throw std::runtime_error("video-out pipeline cannot start (is the encoder available?)");
        bytes_ = (size_t)w * h * 3; w_ = w; h_ = h;
        std::printf("[VIDEO OUT] H.264 -> %s (%dx%d, %s encoder)\n", label.c_str(), w, h, hw ? "hardware" : "x264");
    }
    ~GstH264Out() {
        if (!pipe_) return;
        if (src_) {                                                                 // EOS first so a file is finalized cleanly
            gst_app_src_end_of_stream(GST_APP_SRC(src_));
            GstBus* bus = gst_element_get_bus(pipe_);
            GstMessage* msg = gst_bus_timed_pop_filtered(bus, 2 * GST_SECOND, (GstMessageType)(GST_MESSAGE_EOS | GST_MESSAGE_ERROR));
            if (msg) gst_message_unref(msg);
            gst_object_unref(bus);
        }
        gst_element_set_state(pipe_, GST_STATE_NULL); if (src_) gst_object_unref(src_); gst_object_unref(pipe_);
    }
    void push(const cv::Mat& bgr) {
        if (bgr.cols != w_ || bgr.rows != h_ || !bgr.isContinuous() || bgr.type() != CV_8UC3) return;
        GstBuffer* b = gst_buffer_new_allocate(nullptr, bytes_, nullptr);
        GstMapInfo m; gst_buffer_map(b, &m, GST_MAP_WRITE); std::memcpy(m.data, bgr.data, bytes_); gst_buffer_unmap(b, &m);
        gst_app_src_push_buffer(GST_APP_SRC(src_), b);                              // takes ownership
    }
private:
    GstElement* pipe_ = nullptr; GstElement* src_ = nullptr; size_t bytes_ = 0; int w_ = 0, h_ = 0;
};

#ifdef HAVE_GST_RTSP_SERVER
// ---- annotated video as an RTSP SERVER (--rtsp-out rtsp://0.0.0.0:8554/live) ---------------------------------------------------
// Pull-based, so any number of viewers (QGC, VLC, a second laptop) connect and reconnect on their own, unlike --video-out's
// one fixed UDP destination. Design carried over from the previous (YOLO) iteration, where it was proven on the Jetson: a
// PRIVATE GLib main context on its own thread (sharing the default context with OpenCV's GTK window stalled both), frames handed
// over through a 2-deep queue with try_lock so the frame loop never blocks, last frame repeated if a client asks faster than
// frames arrive. Added here: H.264 constrained to Baseline (same QGC decoder issue as --video-out).
class GstRtspOut {
public:
    GstRtspOut(const std::string& url, int w, int h, double fps, int bitrate_kbps, bool hw) : w_(w), h_(h), fps_(fps > 0 ? fps : 15.0) {
        gst_init(nullptr, nullptr);
        std::string rest = url; auto pr = rest.find("://"); if (pr != std::string::npos) rest = rest.substr(pr + 3);
        std::string hostport = rest, path = "/live"; auto sl = rest.find('/');
        if (sl != std::string::npos) { hostport = rest.substr(0, sl); path = rest.substr(sl); }
        std::string port = "8554"; auto co = hostport.rfind(':'); if (co != std::string::npos) port = hostport.substr(co + 1);
        std::ostringstream l;
        l << "( appsrc name=src is-live=true format=time ! queue leaky=downstream max-size-buffers=2 ! videoconvert ! ";
        if (hw) l << "nvvidconv ! nvv4l2h264enc bitrate=" << bitrate_kbps * 1000 << " insert-sps-pps=true profile=0 iframeinterval=30 ! ";
        else l << "video/x-raw,format=I420 ! x264enc tune=zerolatency speed-preset=ultrafast bitrate=" << bitrate_kbps << " key-int-max=30 ! video/x-h264,profile=baseline ! ";
        l << "h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )";
        server_ = gst_rtsp_server_new();
        gst_rtsp_server_set_address(server_, "0.0.0.0");
        gst_rtsp_server_set_service(server_, port.c_str());
        GstRTSPMediaFactory* f = gst_rtsp_media_factory_new();
        gst_rtsp_media_factory_set_launch(f, l.str().c_str());
        gst_rtsp_media_factory_set_shared(f, TRUE);
        g_signal_connect(f, "media-configure", G_CALLBACK(&GstRtspOut::on_configure), this);
        GstRTSPMountPoints* mp = gst_rtsp_server_get_mount_points(server_);
        gst_rtsp_mount_points_add_factory(mp, path.c_str(), f);
        g_object_unref(mp);
        ctx_ = g_main_context_new();
        if (gst_rtsp_server_attach(server_, ctx_) == 0) throw std::runtime_error("cannot attach the RTSP server on port " + port + " (in use?)");
        loop_ = g_main_loop_new(ctx_, FALSE);
        thread_ = std::thread([this] { g_main_context_push_thread_default(ctx_); g_main_loop_run(loop_); g_main_context_pop_thread_default(ctx_); });
        std::printf("[RTSP OUT] serving rtsp://<this-host>:%s%s (%dx%d, %s encoder)\n", port.c_str(), path.c_str(), w, h, hw ? "hardware" : "x264");
    }
    ~GstRtspOut() {
        if (loop_) g_main_loop_quit(loop_);
        if (thread_.joinable()) thread_.join();
        if (loop_) g_main_loop_unref(loop_);
        if (server_) g_object_unref(server_);
        if (ctx_) g_main_context_unref(ctx_);
    }
    void push(const cv::Mat& bgr) {
        if (bgr.cols != w_ || bgr.rows != h_ || bgr.type() != CV_8UC3 || !bgr.isContinuous()) return;
        if (!mtx_.try_lock()) return;                                       // never block the frame loop
        q_.emplace_back(bgr.data, bgr.data + bgr.total() * bgr.elemSize());
        if (q_.size() > 2) q_.pop_front();
        mtx_.unlock();
    }
private:
    static void on_configure(GstRTSPMediaFactory*, GstRTSPMedia* media, gpointer self_p) {
        auto* self = static_cast<GstRtspOut*>(self_p);
        GstElement* e = gst_rtsp_media_get_element(media);
        GstElement* src = gst_bin_get_by_name_recurse_up(GST_BIN(e), "src");
        if (src) {
            GstCaps* caps = gst_caps_new_simple("video/x-raw", "format", G_TYPE_STRING, "BGR", "width", G_TYPE_INT, self->w_, "height", G_TYPE_INT, self->h_,
                                                "framerate", GST_TYPE_FRACTION, (int)std::lround(self->fps_), 1, nullptr);
            g_object_set(src, "caps", caps, nullptr); gst_caps_unref(caps);
            self->t0_ns_ = -1;                                              // each new media (first client) restarts its clock
            g_signal_connect(src, "need-data", G_CALLBACK(&GstRtspOut::on_need_data), self);
            gst_object_unref(src);
        }
        gst_object_unref(e);
    }
    static void on_need_data(GstElement* src, guint, gpointer self_p) {
        auto* self = static_cast<GstRtspOut*>(self_p);
        std::vector<uint8_t> f;
        { std::lock_guard<std::mutex> lk(self->mtx_);
          if (!self->q_.empty()) { f = std::move(self->q_.front()); self->q_.pop_front(); self->last_ = f; }
          else if (!self->last_.empty()) f = self->last_;
          else return; }
        const gint64 now_ns = g_get_monotonic_time() * 1000;
        if (self->t0_ns_ < 0) self->t0_ns_ = now_ns;
        GstBuffer* b = gst_buffer_new_allocate(nullptr, f.size(), nullptr);
        gst_buffer_fill(b, 0, f.data(), f.size());
        GST_BUFFER_PTS(b) = now_ns - self->t0_ns_;
        GST_BUFFER_DURATION(b) = gst_util_uint64_scale_int(GST_SECOND, 1, (int)std::lround(self->fps_));
        GstFlowReturn ret; g_signal_emit_by_name(src, "push-buffer", b, &ret);
        gst_buffer_unref(b);
    }
    int w_, h_; double fps_;
    GstRTSPServer* server_ = nullptr; GMainContext* ctx_ = nullptr; GMainLoop* loop_ = nullptr; std::thread thread_;
    std::mutex mtx_; std::deque<std::vector<uint8_t>> q_; std::vector<uint8_t> last_; gint64 t0_ns_ = -1;
};
#endif

// --rtsp-out-encoder auto: the Jetson hardware encoder when GStreamer has it -- software x264 on a Nano's CPU competes directly
// with CSRT for the same cores; the hardware block is otherwise idle. Opt-in only (see Args::rtsp_out_encoder for why).
bool use_hw_encoder() {
    if (args.rtsp_out_encoder != "auto") return args.rtsp_out_encoder == "hw";
    gst_init(nullptr, nullptr);
    GstElementFactory* f = gst_element_factory_find("nvv4l2h264enc");
    if (f) gst_object_unref(f);
    return f != nullptr;
}

struct Stage { double capture = 0, track = 0, control = 0, draw = 0; };

}  // namespace

int run_tracker_mode() {
    std::signal(SIGINT, on_signal); std::signal(SIGTERM, on_signal);
    std::signal(SIGPIPE, SIG_IGN);

    // ---- video in ----
    // Validate arguments synchronously (instant, no I/O) so a usage error is reported immediately, same as
    // before; the actual open (which can block for a long time -- see ThreadedCaptureWrapper's comment)
    // happens in a factory run on the capture's own background thread, never in this one.
    if (args.camera_source == "gst" && args.gst_udp_port < 0) { std::fprintf(stderr, "[VIDEO ERROR] --camera-source gst needs --gst-udp-port\n"); return 1; }
    if (args.camera_source != "usb" && args.camera_source != "csi" && args.camera_source != "gst" && args.camera_source != "rtsp")
        { std::fprintf(stderr, "[VIDEO ERROR] unknown --camera-source '%s' (usb|csi|rtsp|gst)\n", args.camera_source.c_str()); return 1; }
    auto cap = std::make_unique<ThreadedCaptureWrapper>([]() -> std::unique_ptr<ICameraCapture> {
        if (args.camera_source == "usb") return open_usb_capture(args.source);
        if (args.camera_source == "csi") return open_csi_capture(args.csi_width, args.csi_height, args.csi_fps, args.csi_sensor_id, args.csi_flip);
        if (args.camera_source == "gst") return open_gst_udp_capture(args.gst_udp_port);
        std::string url = !args.rtsp_url.empty() ? args.rtsp_url : args.source;
        return open_rtsp_capture(url, args.rtsp_latency);
    });

    // ---- MAVLink ----
    LinkConfig lc = make_link_config(args);
    // --ceil-vx/vy/vz/yaw are now honored as given (previously silently re-clamped to 3.0/3.0/2.0/0.5
    // regardless of the flag -- different target classes need very different ceilings: a person/animal
    // needs a low one for safety margin, a car/bike/fast drone needs a much higher one just to keep up,
    // and no single fixed default fits both). This sanity bound (30 m/s, 5 rad/s) only guards against an
    // obvious typo (e.g. an extra zero) reaching the flight controller -- well beyond what this airframe
    // class can do regardless, and ArduPilot's own WPNAV_SPEED/ATC_RATE_* params remain the real,
    // independent safety authority underneath whatever this program commands.
    constexpr double kSanityVel = 30.0, kSanityYaw = 5.0;
    lc.ceilings.vx = std::min(args.ceil_vx, kSanityVel); lc.ceilings.vy = std::min(args.ceil_vy, kSanityVel);
    lc.ceilings.vz = std::min(args.ceil_vz, kSanityVel); lc.ceilings.yaw = std::min(args.ceil_yaw, kSanityYaw);
    if (args.speed_match) lc.ceilings.accel = std::max(lc.ceilings.accel, args.speed_match_accel);
    // turn-rate slew: 2 rad/s^2 added ~0.25 s before the yaw reached a crossing target's rate (review); the world-rate centring wants
    // the turn now
    if (args.yaw_ff_mode == "world") lc.ceilings.yaw_accel = std::max(lc.ceilings.yaw_accel, args.yaw_accel);
    if (args.fpv) {                                                       // FPV: fast and agile -- the chase speed, firmer turns and ramps
        lc.ceilings.vx = std::min(std::max(lc.ceilings.vx, args.fpv_speed), kSanityVel); lc.ceilings.vy = std::min(std::max(lc.ceilings.vy, args.fpv_speed), kSanityVel);
        lc.ceilings.yaw = std::min(std::max(lc.ceilings.yaw, 1.5), kSanityYaw);
        lc.ceilings.accel = args.fpv_accel; lc.ceilings.yaw_accel = 4.0;
    }
    if (args.ceil_vx > kSanityVel || args.ceil_vy > kSanityVel || args.ceil_vz > kSanityVel || args.ceil_yaw > kSanityYaw)
        std::printf("[WARN] speed ceiling above the sanity bound (%.0f m/s, %.0f rad/s) -- likely a typo: clamped\n", kSanityVel, kSanityYaw);
    std::unique_ptr<MavlinkLink> link;
    try { link = std::make_unique<MavlinkLink>(lc); }
    catch (const std::exception& e) { std::fprintf(stderr, "[MAV ERROR] %s\n", e.what()); return 1; }

    // ---- first frame fixes the geometry ----
    // This same wait now also covers the capture opening (which runs asynchronously on its own thread and
    // can take a while, or fail outright) -- liveness() keeps running throughout either way, so MAVLink
    // heartbeats continue even if the camera is slow to come up or never does. Budget is wall-clock, not a
    // fixed retry count (see Args::first_frame_timeout_s for why it needs to be this generous).
    cv::Mat frame;
    double wait_until = now_s() + args.first_frame_timeout_s;
    while (!g_stop && now_s() < wait_until) { link->liveness(); if (cap->read(frame)) break; std::this_thread::sleep_for(std::chrono::milliseconds(50)); }
    if (frame.empty()) {
        std::string err = cap->open_error();
        std::fprintf(stderr, "[VIDEO ERROR] no frame received%s%s\n",
                      err.empty() ? " (is the stream running on the expected port/URL?)" : ": ", err.c_str());
        return 1;
    }
    const int W = frame.cols, H = frame.rows;
    const double hfov = args.hfov_deg * fl::kPi / 180.0;
    fl::Intrinsics K = fl::Intrinsics::from_hfov(W, H, hfov);
    if (args.fx > 0) { K.fx = args.fx; K.fy = args.fy > 0 ? args.fy : args.fx; }
    const fl::Intrinsics K0 = K;                                          // unzoomed; K = K0 with fx, fy scaled by the digital zoom
    double zoom = 1.0, pending_zoom = 1.0, view_ref_h = 0, view_next = 0, view_goal_dist = 0, view_hold_dist = 0; std::string view_why;
    double target_rate_f = 0;                       // smoothed angular rate of the target across the image (yaw damping)
    double tgt_bearing_prev = 0, tgt_bearing_t = -1e9, tgt_world_rate = 0;
    double sm_v = 0, sm_t = -1e9;
    double view_vx = 0, view_vx_t = -1e9, dist_t = -1e9;           // the view control's forward command, eased
    bool show_pred = true;
    bool view_approaching = false;                  // the view control's last decision (approach hysteresis)                          // the yellow predicted box ('k')
    int view_climb_dir = 0; double view_climb_until = 0, view_lock_alt = 0;   // view control's height change (see ViewDecision::climb)                   // speed match: smoothed forward velocity matched to the target's (m/s), last update   // --yaw-ff-mode world: the target's turn rate around us
    double dist_f = 0;                              // smoothed distance to the target (view control range hold)
    std::printf("[TRACKER] frame %dx%d, fx=%.1f (hfov %.1f deg), CSRT at scale %.2f\n", W, H, K.fx, args.hfov_deg, args.track_scale);

    fl::BearingIntegrator bearing_i; double t_ctl_prev = 0;
    double gimbal_next = 0, gimbal_cmd_pitch = 0, gimbal_moved_at = -1e9;   // --gimbal-centre
    double gimbal_last_t = 0, gimbal_sent_pitch = 1e9, gimbal_jump_at = -1e9;                       // per-frame gimbal steps
    std::deque<std::pair<double, double>> gimbal_hist;                      // (time, commanded pitch): what the camera aimed at a frame's time
    auto gimbal_hist_at = [&](double t) {
        double v = gimbal_cmd_pitch;
        for (auto it = gimbal_hist.rbegin(); it != gimbal_hist.rend(); ++it) { if (it->first <= t) return it->second; v = it->second; }
        return v;                                                           // older than the history: the oldest known command
    };
    auto gimbal_record = [&](double t) { gimbal_hist.emplace_back(t, gimbal_cmd_pitch); while (gimbal_hist.size() > 64) gimbal_hist.pop_front(); };
    fl::Gains G; G.klat = args.klat; G.kyaw = args.kyaw; G.kz = args.kz; G.kx = args.kx; G.db_yaw = args.db_yaw; G.db_z = args.db_z; G.db_x = args.db_x; G.min_depression_deg = args.min_depression_deg;
    fl::FollowFsm fsm(fl::FsmTimes{args.coast_after, args.lost_after, args.unlock_after, args.link_hb_timeout_s});
    fl::RescueCfg rcfg; rcfg.enabled = args.enable_rescue; rcfg.confirm = args.go_confirm; rcfg.speed_ms = args.rescue_speed; rcfg.max_range_m = args.rescue_max_range; rcfg.min_battery_pct = args.rescue_min_battery;
    rcfg.min_locked_s = 0.0; // FPV shoot: zero lock wait time, instantaneous trigger!
    fl::RescueController rescue(rcfg);
    int rc_rescue_zone = -1;   // -1 = no RC reading yet (0/1/2 = low/mid/high once seen)
    if (rcfg.enabled) std::printf("[FPV DASH] READY: terminal cruise shoot up to %.1f m/s (press g or click GO, x to abort)\n", rcfg.speed_ms);
    LockTracker lt(args.track_scale, args.csrt_padding, args.csrt_fast, args.csrt_template_factor);   // CSRT + appearance + occlusion handling
    lt.set_full_res_below(args.full_res_below);
    std::unique_ptr<C12Link> c12;
    const bool gimbal_mode = args.follow_source == "gimbal";
    if (gimbal_mode && args.c12_host.empty()) { std::fprintf(stderr, "--follow-source gimbal needs --c12-host\n"); return 1; }
    // image mode with a C12 attached is a real, deliberate combination (bench-testing our own CSRT, or a gimbal pointed
    // forward/fixed by the pilot rather than independently tracking) -- but if the C12's OWN tracking is also running, it
    // keeps the target centred in the image, so the follow loop sees ~0 error and barely moves (flagged in review as a
    // footgun on --profile real-c12 specifically: that profile does not imply the C12 is doing its own tracking).
    if (!gimbal_mode && !args.c12_host.empty())
        std::fprintf(stderr, "[WARN] follow-source=image with a C12 attached: if the C12's own tracking is ALSO active, "
                              "the gimbal keeps the target centred and this follow loop will barely move. Use "
                              "--follow-source gimbal (--profile real-c12-gimbal) unless the C12's tracking is off.\n");
    if (!args.c12_host.empty()) {
        try { c12 = std::make_unique<C12Link>(args.c12_host, args.c12_port); c12->want_push(20); }
        catch (const std::exception& e) { std::fprintf(stderr, "[C12 ERROR] %s\n", e.what()); return 1; }
    }
    // ---- AI models: in-process (OpenVINO C++ API: no Python, no UDP/JPEG hop) or the sidecar ----
    std::unique_ptr<AiBackend> native_ai;
#ifdef HAVE_OPENVINO
    if (args.ai != "sidecar" && args.reid) {
        auto expand = [](std::string p) { if (!p.empty() && p[0] == '~' && std::getenv("HOME")) p = std::getenv("HOME") + p.substr(1); return p; };
        try {
            const double t0 = now_s();
            native_ai = std::make_unique<NativeAI>(expand(args.reid_model));
            std::printf("[AI] %s ready in %.1f s (no sidecar needed)\n", native_ai->describe().c_str(), now_s() - t0);
            args.detector_port = 0;                                         // the sidecar link is not needed
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[AI] in-process models failed (%s)%s\n", e.what(), args.ai == "native" ? "" : ": using the sidecar");
            if (args.ai == "native") return 1;
        }
    }
#else
    if (args.ai == "native") { std::fprintf(stderr, "[AI] --ai native: this build has no OpenVINO (see CMakeLists.txt)\n"); return 1; }
#endif
    std::unique_ptr<DetectorLink> det;
    if (args.detector_port > 0) {
        try { det = std::make_unique<DetectorLink>(args.detector_port); }
        catch (const std::exception& e) { std::fprintf(stderr, "[DETECT] disabled: %s\n", e.what()); }
    }
    AiBackend* ai = native_ai ? native_ai.get() : static_cast<AiBackend*>(det.get());   // ReID, whichever serves it
    if (args.reid && !ai) std::fprintf(stderr, "[REID] needs the in-process models or the sidecar link: ReID off\n");
    if (args.reid && ai) {
        AiBackend* d = ai;
        LockTracker::Reid r;
        r.request = [d](const cv::Mat& f, const fl::Box& b) { return d->request_embed(f, b); };
        r.poll = [d](uint32_t id, std::vector<float>& v) { return d->embed_poll(id, v); };
        r.sync = [d](const cv::Mat& f, const std::vector<fl::Box>& bs, std::vector<std::vector<float>>& v) { return d->embed_many(f, bs, v, 0.2); };
        lt.set_reid(std::move(r));
        std::printf("[REID] on: DINOv2 fingerprints from the %s\n", native_ai ? "in-process model" : "sidecar (start it with --reid ~/models/dinov2)");
    }
    double ref_dist = 0; bool have_ref = false;                           // ground distance at lock (gimbal mode)
    fl::CameraMotion cam_motion;                    // camera-motion compensation of the prediction (approach, yaw, gimbal tilt)
    fl::PoseHistory pose_hist;            // recent vehicle poses, for --video-latency
    double cam_prev_gimbal = 0, cam_prev_t = -1, cam_prev_yaw = 0; bool cam_prev_have_yaw = false;   // camera-motion fallback (see below)
    fl::Cmd chase_last; double chase_last_t = -1e9;   // the last chase command, faded out if the estimate drops out
    double zoom_changed_at = -1e9;        // last digital zoom change (s)
    // per-GO overrides (stdin "go SPEED OFFSET", QGC GO param1/param2): go_speed_req 0 = the configured speed, go_standoff_req NaN =
    // the configured offset (signed: + before the target, - past it). Spent by the GO that uses them.
    double go_speed_req = 0, go_standoff_req = std::numeric_limits<double>::quiet_NaN();
    double go_speed = 0, go_standoff = 0, go_prev_range = 0, go_togo = 0;            // in force for the current GO
    // [GO-2] GO READY indicator: the same readiness checks a GO press would run, evaluated EVERY frame, so the pilot can see before
    // pressing whether GO will go (on screen: "GO READY - PRESS G" in green, else "GO NOT READY: <why>"; to QGC: a STATUSTEXT
    // whenever it changes, once it has held 0.5 s). go_predicting: the target's motion track is valid, so GO aims where it is going
    // (PN / intercept) rather than where it is now.
    bool go_ready = false, go_predicting = false, go_ready_cand = false, go_ready_said = false; double go_ready_cand_t = 0;
    std::string go_ready_why;
    double abort_flash_until = 0;         // on-screen "ABORTED" banner
    double chase_vn = 0, chase_ve = 0;   // smoothed target velocity for the chase feed-forward
    double chase_since = 0;               // FPV chase engaged since (0 = not chasing)
    double refit_next = 0, refit_zoom = 1.0, refit_t0 = 0;   // GrabCut box re-fit in the background (--no-box-refit: off)
    struct CompleteRes { bool ok = false; fl::Box whole; double t = 0; long gen = 0; };
    long lock_gen = 0, lock_job_gen = 0;
    bool abort_latched = false;
    bool go_braking = false; double go_brake_yaw = 0;   // fly-through GO: past the target (+ the coast distance), now braking straight ahead
    bool go_punctured = false;                          // aperture transit: punctured through interior geometry of object
    bool pursuit_static = false;                    // the GO target's last known track was static (keep going to its last position)
    fl::Ned go_prev_v{}, go_acc_f{}; bool go_have_prev_v = false; double go_acc_t = 0;   // GO acceleration feed-forward                     // an ABORT climb in progress: further ABORTs and the stay-over do nothing until a new lock / GO                              // bumped by every new lock / unlock: a completion applies only to its own lock
    std::future<CompleteRes> complete_job; bool complete_running = false; std::optional<CompleteRes> complete_ready; double complete_started = -1e9;
    std::future<std::pair<int, fl::Box>> refit_job; bool refit_running = false; fl::Box refit_from; double refit_last_w = 0, refit_last_h = 0;
    fl::TargetFilter kf; int occl_frames = 0;   // frames the locked target has been hidden (LockTracker coasting)
    fl::FrameGap gap; double nominal_gap = 0;
    BoxSelector selector; selector.set_frame(W, H);
    JsonLog jlog; jlog.open(args.log_dir);
    std::unique_ptr<GstH264Out> vout, rec;
#ifdef HAVE_GST_RTSP_SERVER
    std::unique_ptr<GstRtspOut> rtsp;
    if (!args.rtsp_out.empty()) {
        try { rtsp = std::make_unique<GstRtspOut>(args.rtsp_out, W, H, 15.0, args.video_out_bitrate, use_hw_encoder()); }
        catch (const std::exception& e) { std::fprintf(stderr, "[RTSP OUT] disabled: %s\n", e.what()); }
    }
#else
    if (!args.rtsp_out.empty()) std::fprintf(stderr, "[RTSP OUT] not in this build: install libgstrtspserver-1.0-dev and rebuild\n");
#endif
    if (!args.video_out.empty()) {
        try {
            std::string hp = args.video_out; if (hp.rfind("udp://", 0) == 0) hp = hp.substr(6);
            auto c = hp.rfind(':'); if (c == std::string::npos) throw std::runtime_error("--video-out expects udp://HOST:PORT");
            // Software x264 on a Jetson Nano's CPU competes directly with CSRT for the same cores; the hardware encoder is free.
            vout = std::make_unique<GstH264Out>(GstH264Out::udp_sink(hp.substr(0, c), std::stoi(hp.substr(c + 1))), args.video_out, W, H, 15.0, args.video_out_bitrate, use_hw_encoder());
        } catch (const std::exception& e) { std::fprintf(stderr, "[VIDEO OUT] disabled: %s\n", e.what()); }
    }
    if (!args.record_dir.empty()) {                                       // raw (un-annotated) camera video, for after-action review
        ::mkdir(args.record_dir.c_str(), 0755);                           // and for re-running the tracker offline on the real footage
        char name[256]; std::time_t t = std::time(nullptr); std::tm tmv; localtime_r(&t, &tmv);
        std::snprintf(name, sizeof name, "%s/cam_%04d%02d%02d_%02d%02d%02d.mkv", args.record_dir.c_str(), tmv.tm_year + 1900, tmv.tm_mon + 1, tmv.tm_mday, tmv.tm_hour, tmv.tm_min, tmv.tm_sec);
        try { rec = std::make_unique<GstH264Out>(GstH264Out::file_sink(name), name, W, H, std::max(1.0, args.cam_fps), args.record_bitrate, use_hw_encoder()); }
        catch (const std::exception& e) { std::fprintf(stderr, "[RECORD] disabled: %s\n", e.what()); }
    }
    const bool show = !args.no_display;
    const std::string win = "Drone front camera (click a target = lock; buttons LOCK/UNLOCK/GO/ABORT -- keys l/u/g/x, q = quit)";
    if (show) { cv::namedWindow(win); cv::setMouseCallback(win, BoxSelector::callback, &selector); }

    // ---- state ----
    // A selection waiting for GUIDED: either a POINT (click / tap / centre LOCK -- GrabCut finds the object around it, no box size is
    // assumed) or a RECTANGLE the operator drew (GrabCut tightens it; if it can't, the operator's own rectangle is used).
    std::optional<fl::Box> pending_box; double pending_since = 0; bool pending_point = false; int click_dbg_n = 0;
    std::string run_tag; { char tb[32]; std::time_t tt = std::time(nullptr); std::tm tmv; localtime_r(&tt, &tmv); std::strftime(tb, sizeof tb, "%H%M%S", &tmv); run_tag = tb; }   // debug image names per run
    std::future<fl::LockJob> lock_job; bool lock_job_running = false; bool pending_preseg = false; cv::Point2d click_rel(-1, -1); double lock_t = -1e9;
    bool goto_path_active = false, goto_sent = false;
    auto select_rect = [&](const fl::Box& b, double t) {
        if (goto_path_active || goto_sent) {
            link->cancel_goto(false); link->clear_hold();
            goto_path_active = false; goto_sent = false;
        }
        pending_box = b; pending_point = false; pending_preseg = false; pending_since = t; ++lock_gen;
    };
    fl::Box box;                                                         // last ACCEPTED tracker box (full-frame px)
    double lock_dep = std::numeric_limits<double>::quiet_NaN();           // FPV (fixed camera): the target's depression at lock
    double vert_err_f = 0, vert_err_t = 0;                                  // FPV centring: the smoothed vertical error (rad) and its time
    double last_accept = now_s(), ref_h = 0, last_reject_log = 0; bool tracking = false; std::vector<double> settle;
    cv::Scalar ref_color(-1, -1, -1);   // mean BGR of the box at lock time, for the appearance cross-check (fl::color_plausible)
    bool rc_lock_armed = true, prev_session = false;
    // goto_sent: once a go-to fires, the normal follow-velocity
    // control stops (one go-to command is the whole job; it must
    // not be fought by a concurrent 20 Hz velocity session)
    // Diagonal pacing: the destination is fixed at confirm time (no re-targeting), but instead of sending it once and trusting
    // ArduPilot's own path shape, we send a moving point along the straight 3D line to it, paced so horizontal and vertical
    // progress stay equal -- found live in SITL that a single position target made ArduPilot descend almost fully before
    // translating (even with WP_SPD_UP/DN raised to match the horizontal speed): this paces it ourselves instead.
    fl::Ned pursuit_tgt{}, pursuit_last{}; double pursuit_seen = 0, pursuit_floor_d = 0; bool pursuit_arrived = false;   // GO pursuit
    fl::Ned goto_start{}, goto_final{}, goto_hold{}; double goto_along = 0, goto_prev_t = 0, goto_total = 0;
    const int go_guidance_mode = args.go_guidance == "lead" ? 1 : args.go_guidance == "intercept" ? 2 : args.go_guidance == "pn" ? 3 : 0;
    fl::TargetEstimator tgt_est(args.est_q_acc);
    fl::SizeRange size_rng;                         // range from the box size, learned per target (see fl::SizeRange)   // the target's ground position + velocity (feed-forward, lead, stay-over)
    double marker_next = 0, last_marker_lat = 0, last_marker_lon = 0, last_dest_lat = 0, last_dest_lon = 0;
    bool target_marker_on = false, dest_marker_on = false;   // QGC map markers (send_map_marker)
    std::string height_used = "home";   // which height reference the destination estimate used (fl::pick_height), shown at ARM
    int override_hits = 0;   // consecutive goto_overridden() detections -- see its use below for why this needs debouncing
    fl::RState last_rescue_state = fl::RState::DISABLED;   // one frame old; used to hold still (zero velocity) once ARMED, before the dash starts
    std::string banner; double banner_until = 0;
    fl::FState prev_state = fl::FState::IDLE;
    // QGC tap-to-lock latency compensation (see Args::tap_latency_ms): ~0.6-0.8 s of recent frames with capture timestamps,
    // already downscaled to track_scale (same cost as one CSRT frame, not the full-resolution capture -- at 640x360 that is
    // well under 1 MB each), so a tap can be matched to the frame the pilot actually saw and the tracker fast-forwarded from
    // there to the live frame without re-shrinking the same pixels on every catch-up step.
    std::deque<std::pair<double, cv::Mat>> frame_hist;
    std::deque<std::pair<double, cv::Mat>> raw_hist;           // full-size frames of the last 0.6 s (late-click fix)
    double seen_t = 0;                                          // capture time of the frame on screen
    // also long enough to replay a background lock job's result (up to ~1.8 s measured with click snapping) to the live frame
    const double hist_window_s = std::max(2.5, args.tap_latency_ms / 1000.0 + 0.2);
    if (!args.lock_box.empty()) {                                        // headless / scripted lock
        double a, b, c, d; if (std::sscanf(args.lock_box.c_str(), "%lf,%lf,%lf,%lf", &a, &b, &c, &d) == 4) { pending_box = fl::Box{a, b, c, d}; pending_since = now_s(); }
    }
    int frame_no = 0; double t_prev = now_s(), fps = 0; char logbuf[1536]; Stage st;
    std::string stdin_buf; if (!show) ::fcntl(0, F_SETFL, ::fcntl(0, F_GETFL) | O_NONBLOCK);
    // Routes notable events to the pilot's QGC via STATUSTEXT, not just this console. The console always gets the full text;
    // QGC gets `qgc_text` (a short form, if given) fitted to the 50-char field, at a real severity (WARNING shows highlighted in
    // QGC), de-duplicated and rate-limited so a flickering state cannot flood the telemetry radio. qgc_text "-" = console only.
    fl::MsgThrottle msg_throttle;
    enum Sev : uint8_t { SEV_CRIT = 2, SEV_WARN = 4, SEV_NOTICE = 5, SEV_INFO = 6 };
    auto say = [&](const std::string& s, uint8_t sev = SEV_INFO, const std::string& qgc_text = "") {
        banner = s; banner_until = now_s() + 4.0; std::printf("[FOLLOW] %s\n", s.c_str()); std::fflush(stdout);
        if (qgc_text == "-") return;
        std::string q = fl::fit_statustext(qgc_text.empty() ? s : qgc_text);
        if (msg_throttle.allow(q, now_s(), sev <= SEV_WARN)) link->send_statustext(sev, q);
    };
    auto pixel_box_from_norm = [&](const TrackRequest& r) {          // a QGC rectangle in pixels (points are segmented instead)
        return fl::Box{r.x0 * W, r.y0 * H, (r.x1 - r.x0) * W, (r.y1 - r.y0) * H};
    };
    // The pilot's tap is based on the frame QGC was DISPLAYING ~tap_latency_ms ago, not the current live frame -- on a
    // moving target, initialising directly on the live frame with those coordinates puts the box behind it. Find the
    // buffered frame closest to (now - latency), init a throwaway tracker there, then run it forward through every later
    // buffered frame up to the live one so the returned box is caught up to the present. Falls back to the raw box (old
    // behaviour) if there's no history yet or the catch-up tracker fails partway.
    // catch_up_from: same, from a box known on the frame captured at `target_t` (a detection result, which is a few frames old
    // by the time it comes back from the sidecar) -- run forward through the buffered frames to the live one.
    std::function<fl::Box(const fl::Box&, double)> catch_up_from;
    auto catch_up_to_now = [&](const fl::Box& tapped_box, double capture_now) { return catch_up_from(tapped_box, capture_now - args.tap_latency_ms / 1000.0); };
    catch_up_from = [&](const fl::Box& tapped_box, double target_t) {
        if (frame_hist.empty()) return tapped_box;
        size_t start = 0; double best_dt = 1e18;
        for (size_t i = 0; i < frame_hist.size(); i++) {
            double dt = std::fabs(frame_hist[i].first - target_t);
            if (dt < best_dt) { best_dt = dt; start = i; }
        }
        // the frame the box belongs to is no longer buffered: replaying it from another frame would put it in the wrong place
        if (best_dt > std::max(0.25, 3.0 * nominal_gap)) return fl::Box();
        TrackerEngine catchup(args.track_scale, args.csrt_padding, args.csrt_fast, args.csrt_template_factor);
        if (!catchup.init_small(frame_hist[start].second, tapped_box)) return tapped_box;
        fl::Box cur = tapped_box;
        // Bound the worst-case cost of a long catch-up (e.g. a stale/slow network making tap_latency large): running ~10+
        // CSRT updates back-to-back on a Pi 5 would delay this loop's next control tick noticeably. Past kMaxCatchupFrames,
        // stride through the buffer instead of visiting every frame, but always finish on the live frame (frame_hist.back())
        // regardless of the stride, so the result is still caught up to the present, just with fewer intermediate corrections.
        constexpr size_t kMaxCatchupFrames = 15;
        size_t remaining = frame_hist.size() - 1 - start;
        size_t stride = remaining > kMaxCatchupFrames ? (remaining + kMaxCatchupFrames - 1) / kMaxCatchupFrames : 1;
        size_t last_hit = start;
        for (size_t i = start + stride; i < frame_hist.size(); i += stride) {
            fl::Box nb;
            if (!catchup.update_small(frame_hist[i].second, nb)) return cur;   // lost partway through catch-up: best estimate so far
            cur = nb; last_hit = i;
        }
        if (last_hit != frame_hist.size() - 1) {
            fl::Box nb;
            if (catchup.update_small(frame_hist.back().second, nb)) cur = nb;
        }
        return cur;
    };

    // A click (point) is segmented in the BACKGROUND: measured live, the GrabCut + motion work took up to 1.5 s at 1280x720 and froze
    // the whole loop (video, buttons, even the heartbeats). The result is replayed through the buffered frames to the live one.
    // LATE CLICKS (operator report: "click is slow for fast objects"): a click was segmented on the frame captured AFTER it -- the one the
    // operator never saw -- so a fast object had already moved off the clicked spot. Now the lock is made on the frame the operator
    // SAW (t_seen: the frame on screen for a window click; now - the tap latency for a QGC tap), kept full-size for 0.6 s, and the
    // result is replayed through the frames since to the live one (catch_up_from). A click while the previous one is still being
    // segmented is queued (the newest wins) instead of dropped.
    struct QueuedClick { double x, y, t; cv::Mat f; };                  // the frame is kept WITH the click: the buffer only holds 0.6 s
    std::optional<QueuedClick> queued_click;
    auto seen_frame = [&](double t_seen, double& tf) {
        cv::Mat f = frame; tf = now_s();
        for (auto it = raw_hist.rbegin(); it != raw_hist.rend(); ++it) { f = it->second; tf = it->first; if (it->first <= t_seen + 1e-3) break; }
        return f;
    };
    std::function<void(double, double, double, cv::Mat)> select_point_on;
    auto select_point = [&](double x, double y, double t_seen) {
        double tf = 0; cv::Mat f = seen_frame(t_seen, tf);
        if (lock_job_running) { queued_click = QueuedClick{x, y, tf, f}; return; }
        select_point_on(x, y, tf, f);
    };
    select_point_on = [&](double x, double y, double tf, cv::Mat f) {
        if (goto_path_active || goto_sent) {
            link->cancel_goto(false); link->clear_hold();
            goto_path_active = false; goto_sent = false;
            link->command(fl::Cmd(), false);
        }
        cv::Mat prev;                                                     // ~3 frames before it, at the tracking scale (motion region)
        for (auto it = frame_hist.rbegin(); it != frame_hist.rend(); ++it) if (it->first <= tf - 2.5 * std::max(0.03, nominal_gap)) { prev = it->second.clone(); break; }
        lock_job = std::async(std::launch::async, fl::run_lock_job, f.clone(), prev, std::min(1.0, args.track_scale), x, y, 0.07 * W, tf);
        lock_job_running = true; lock_job_gen = lock_gen;
    };
    // =========================== main loop ===========================
    while (!g_stop) {
        const double t_loop = now_s();
        link->liveness();                                                // heartbeats continue only while this loop turns
        double t0 = now_s();
        if (frame_no > 0 && !cap->read(frame)) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); if (!cap->isOpened()) break; continue; }
        if (frame.empty()) continue;
        if (frame.cols != W || frame.rows != H) { std::this_thread::sleep_for(std::chrono::milliseconds(5)); continue; }
        st.capture = (now_s() - t0) * 1000;
        const double now = now_s();
        frame_no++;
        if (frame_no == 2) {                                             // warm-up: OpenCV's one-time start-up (~200 ms, measured) is paid here,
            std::thread([f = frame.clone()] {                            // in the background -- not on the operator's first click
                fl::Box b; std::string w; (void)fl::ground_model_from_point(f, f.cols / 2.0, f.rows / 2.0, b);
                (void)fl::grabcut_from_point(f, f.cols / 2.0, f.rows / 2.0, b, w);
            }).detach();
        }
        if (rec) rec->push(frame);                                       // before any overlay is drawn on it
        // ---- digital zoom (view control): crop the centre, scale back up; on a change, carry the lock over to the new view ----
        if (std::fabs(pending_zoom - zoom) > 1e-6) {
            const double r = pending_zoom / zoom;
            if (tracking) {
                // where the target is in THIS frame (the filter holds the previous frame's): one step of its image velocity ahead --
                // measured: the old box carried over to a fast-moving cube landed beside it and the lock was lost a second later
                const fl::Box kb = kf.box();
                const fl::Box nb = fl::zoom_box(fl::Box{kb.x + kf.vx(), kb.y + kf.vy(), kb.w, kb.h}, r, W / 2.0, H / 2.0);
                cv::Mat zf; fl::digital_zoom(frame, pending_zoom, zf);
                if (nb.x >= 0 && nb.y >= 0 && nb.x + nb.w <= W && nb.y + nb.h <= H && lt.reinit(zf, nb)) {
                    // the filter is SCALED with the view (position about the image centre, size and velocity x r), not reset: a reset
                    // dropped the target's velocity, the next predictions stood still while it moved on, and the box was rejected --
                    // measured live, 6 of 12 losses came within 1.6 s of a zoom change
                    cv::Mat Az = (cv::Mat_<double>(2, 3) << r, 0, (1 - r) * W / 2.0, 0, r, (1 - r) * H / 2.0);
                    kf.warp(Az);
                    zoom = pending_zoom; box = nb; if (ref_h > 0) ref_h *= r; if (view_ref_h > 0) view_ref_h *= r; zoom_changed_at = now_s();
                    frame_hist.clear(); raw_hist.clear(); cam_motion.reset();   // older frames are at the old zoom
                    say("zoom " + std::to_string(zoom).substr(0, 3) + "x", SEV_INFO, "-");
                } else pending_zoom = zoom;                               // the target would leave the zoomed view: keep the zoom
            } else { zoom = pending_zoom; frame_hist.clear(); raw_hist.clear(); }
        }
        if (zoom > 1.0 + 1e-6) { cv::Mat zf; fl::digital_zoom(frame, zoom, zf); frame = zf; }
        K.fx = K0.fx * zoom; K.fy = K0.fy * zoom;
        // the buffered frames are the frames AS SHOWN (after the digital zoom): clicks, taps and catch-up boxes are in that view (review:
        // pushed before the zoom, a click at 2x was segmented on the un-zoomed frame)
        frame_hist.emplace_back(now, TrackerEngine::shrink_to(frame, args.track_scale).clone());   // store downscaled + owned (shrink_to may alias the source at scale>=1)
        raw_hist.emplace_back(now, frame.clone());                       // full size, 0.6 s: the frame a click was made on (select_point)
        while (raw_hist.size() > 1 && raw_hist.front().first < now - 0.6) raw_hist.pop_front();
        while (frame_hist.size() > 1 && frame_hist.front().first < now - hist_window_s) frame_hist.pop_front();
        const double frame_gap = gap.observe(now);                       // seconds since the previous frame
        const bool video_stalled = fl::FrameGap::stalled(frame_gap);
        const VehicleState vs = link->state();

        // [GO-1] INPUT: every GO source below sets `rescue_press` (one press = GO; --go-confirm: arm, then confirm -- see [GO-2]); "go SPEED DIST" / QGC
        // param1/param2 set this GO's speed and stop distance (go_speed_req / go_standoff_req); 'x' / "abort" / QGC ABORT set the
        // ABORT handled in [GO-8].
        // ---- operator inputs: window keys/drag, stdin, QGC, RC ----
        bool want_unlock = false, quit = false, rescue_press = false, rescue_abort = false, want_quick_lock = false;
        bool hard_abort = false;   // an EXPLICIT abort (button, key, QGC, command) -- stops whatever the drone is doing (not the RC switch's reset)
        auto handle_command_text = [&](const std::string& cmd) {
            double a, b, c, d;
            if (std::sscanf(cmd.c_str(), "lock %lf %lf %lf %lf", &a, &b, &c, &d) == 4) select_rect(fl::Box{a, b, c, d}, now);
            else if (std::sscanf(cmd.c_str(), "click %lf %lf", &a, &b) == 2) select_point(a, b, now);
            else if (cmd == "unlock") want_unlock = true;
            else if (cmd == "go") rescue_press = true;
            else if (std::sscanf(cmd.c_str(), "go %lf %lf", &a, &b) == 2) {   // go SPEED OFFSET, this GO only: "go 15 -10" = 15 m/s, 10 m PAST
                rescue_press = true;                                            // (OFFSET + before the target, 0 at it, - past it; SPEED 0 = configured)
                if (a > 0) go_speed_req = std::min(a, go_settings::kMaxSpeed);
                if (std::isfinite(b)) go_standoff_req = std::clamp(b, -go_settings::kMaxOffset, go_settings::kMaxOffset);
            }
            else if (std::sscanf(cmd.c_str(), "go %lf", &a) == 1) { rescue_press = true; if (a > 0) go_speed_req = std::min(a, go_settings::kMaxSpeed); }
            else if (cmd == "abort") rescue_abort = hard_abort = true;
            else if (cmd == "quit") quit = true;
            else if (cmd == "relock" || cmd == "lockc") want_quick_lock = true;
        };
        if (show) {
            if (selector.has_click()) { cv::Point2d p = selector.take_click(); select_point(p.x, p.y, seen_t > 0 ? seen_t : now); }   // one click = instant lock, no keypress or drag needed
            // On-screen buttons: a mouse click lands wherever the cursor is, unlike a keyboard shortcut
            // which only reaches this window if it has OS focus -- found live that 's'/'g' routinely
            // "don't work" simply because the officer is looking at QGC's window, not this one, when they
            // press them. Clicking UNLOCK/GO/ABORT works regardless of which window currently has focus.
            switch (selector.take_button()) {
                case Button::Lock: want_quick_lock = true; break;     // the object at the image centre (same as QGC "Track LOCK")
                case Button::Unlock: want_unlock = true; break;
                case Button::Go: rescue_press = true; break;
                case Button::Abort: rescue_abort = hard_abort = true; break;
                case Button::None: break;
            }
            int key = cv::waitKey(1) & 0xFF;
            if (key >= 'A' && key <= 'Z') key = key - 'A' + 'a';            // Caps Lock on: 'L' locks too (keys were silently ignored)
            if (key == 'k') { show_pred = !show_pred; say(show_pred ? "predicted box: on" : "predicted box: off", SEV_INFO, "-"); }
            else if (key == 's' || key == 13) want_quick_lock = true;       // 's' / Enter: locks the object at the image centre (same as space/'l')
            else if (key == 'u' || key == 'r') want_unlock = true;
            else if (key == 'g') rescue_press = true;
            else if (key == 'x') rescue_abort = hard_abort = true;
            else if (key == 'l') want_quick_lock = true;
            else if (key == 'q' || key == 27) quit = true;
        } else {
            char tmp[256]; ssize_t n = ::read(0, tmp, sizeof tmp);
            if (n > 0) { stdin_buf.append(tmp, (size_t)n); size_t p; while ((p = stdin_buf.find('\n')) != std::string::npos) { handle_command_text(stdin_buf.substr(0, p)); stdin_buf.erase(0, p + 1); } }
        }
        TrackRequest tr;
        while (link->poll_track_request(tr)) {                            // QGC tap / rectangle / stop
            if (tr.kind == TrackRequest::STOP) want_unlock = true;
            else if (tr.kind == TrackRequest::POINT) {
                // A tap on QGC's video: exactly the same lock as a click in the companion window -- the ground-model / GrabCut lock in
                // the background (~40 ms), replayed to the live frame, snapping to the nearest object if the tap missed slightly.
                // (Previously: the old point-GrabCut on a half-size frame, synchronous, then "tightening" a drawn box around the tap.)
                say("QGC tap: locking the object at the tap", SEV_INFO, "-");
                select_point(tr.x0 * W, tr.y0 * H, now - args.tap_latency_ms / 1000.0);   // on the frame the officer tapped (tap latency)
            } else {
                fl::Box b = catch_up_to_now(pixel_box_from_norm(tr), now);
                bool plausible = b.valid() && b.cx() >= 0 && b.cx() <= W && b.cy() >= 0 && b.cy() <= H && b.w <= 0.9 * W && b.h <= 0.9 * H;
                if (plausible) select_rect(b, now);
                else say("QGC lock refused: catch-up tracking landed off-target (try again)", SEV_WARN, "Lock refused: try again");
            }
        }
        bool qgc_brake = false;
        OpCmd op;
        double op_p1 = 0, op_p2 = 0;
        while (link->poll_op_request(op, &op_p1, &op_p2)) {                // QGC Custom Action / joystick button (MAV_CMD_USER_1..4)
            switch (op) {
                case OpCmd::PRESS: rescue_press = true;                     // param1 = GO speed (m/s), param2 = offset (m, + before / - past); 0 = configured
                    if (op_p1 > 0) go_speed_req = std::min(op_p1, go_settings::kMaxSpeed);
                    if (op_p2 != 0 && std::isfinite(op_p2)) go_standoff_req = std::clamp(op_p2, -go_settings::kMaxOffset, go_settings::kMaxOffset);
                    break;
                case OpCmd::ABORT: rescue_abort = hard_abort = true; break;
                case OpCmd::UNLOCK: want_unlock = true; break;
                case OpCmd::BRAKE: qgc_brake = true; break;                 // works even when the rescue isn't armed: a QGC panic button
                case OpCmd::LOCK: want_quick_lock = true; break;
            }
        }
        if (want_quick_lock) {
            // QGC "LOCK" button / 'l' key: the object at the image centre -- the officer points the camera (sticks / gimbal) at the target
            // first, then presses LOCK. Works with RC + QGC only: no video click needed, so it is usable from a joystick button in real time.
            select_point(W / 2.0, H / 2.0, seen_t > 0 ? seen_t : now); say("LOCK: the object at the centre of the image", SEV_NOTICE);
        }
        if (qgc_brake) {
            link->cancel_goto(); link->send_mode_change(17 /* ArduCopter BRAKE */);
            goto_path_active = false; goto_sent = false;
            say("BRAKE commanded from QGC", SEV_WARN);
        }
        // A frozen RC reading (radio dropout, companion-side staleness) must not be read as a live switch position -- flagged
        // as a real risk for the rescue trigger specifically, but it applies equally to the centre-lock switch.
        const bool rc_fresh = vs.rc_age < 1.0;
        if (args.rc_centerlock_channel > 0 && rc_fresh) {                       // RC centre-lock: the pilot centres the target, flips the switch
            int v = vs.rc[std::min(args.rc_centerlock_channel, 18)];
            if (v > args.rc_high_thresh && rc_lock_armed) { select_point(W / 2.0, H / 2.0, now); rc_lock_armed = false; }
            else if (v > 0 && v < args.rc_low_thresh) { if (!rc_lock_armed) want_unlock = true; rc_lock_armed = true; }
        }
        if (args.rc_rescue_channel > 0 && rc_fresh) {                           // rescue switch: up = GO, back to low = stop (see below)
            int v = vs.rc[std::min(args.rc_rescue_channel, 18)];
            if (v > 0) {
                int zone = v < args.rc_low_thresh ? 0 : (v > args.rc_high_thresh ? 2 : 1);
                if (rc_rescue_zone < 0) rc_rescue_zone = zone;                      // first reading: adopt it silently, no event
                else if (zone > rc_rescue_zone) rescue_press = true;                // any move UP: a press (one press = GO; --go-confirm: arm, then confirm)
                else if (zone == 0 && rc_rescue_zone != 0) rescue_abort = true;      // back to low: stop -- BRAKE during a dash ([GO-8] 2/4)
                rc_rescue_zone = zone;
            }
        }
        if (quit) break;

        // ---- the background GrabCut completion of the last click-lock: the lock box becomes the WHOLE object ----
        if (complete_running && complete_job.wait_for(std::chrono::seconds(0)) == std::future_status::ready) { complete_running = false; complete_ready = complete_job.get(); }
        if (complete_ready && (!complete_ready->ok || now - complete_started > 3.0 || complete_ready->gen != lock_gen)) complete_ready.reset();
        if (complete_ready && tracking && fsm.state() == fl::FState::LOCKED && lock_t >= complete_started - 0.01) {
            const fl::Box live = catch_up_from(complete_ready->whole, complete_ready->t);   // that whole object, in the live frame
            // the same object: the whole must contain the tracked part's centre, and not be smaller
            if (live.valid() && box.cx() >= live.x && box.cx() <= live.x + live.w && box.cy() >= live.y && box.cy() <= live.y + live.h &&
                live.w * live.h >= box.w * box.h && live.x >= 0 && live.y >= 0 && live.x + live.w <= W && live.y + live.h <= H && lt.init(frame, live)) {
                std::printf("[LOCK] completed to the whole object: %dx%d -> %dx%d (GrabCut)\n", (int)box.w, (int)box.h, (int)live.w, (int)live.h);
                box = live; kf.reset(live); last_accept = now; ref_h = 0; lock_dep = std::numeric_limits<double>::quiet_NaN(); have_ref = false; settle.clear(); view_ref_h = 0;
            }
            complete_ready.reset();
        }
        // ---- selection -> GUIDED -> lock ----
        if (lock_job_running && lock_job.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
            lock_job_running = false;
            fl::LockJob j = lock_job.get();
            if (queued_click) { const QueuedClick q = *queued_click; queued_click.reset(); select_point_on(q.x, q.y, q.t, q.f); }   // the newer click wins
            if (lock_job_running) j.ok = false, j.why = "superseded by a newer click";
            else if (lock_job_gen != lock_gen) j.ok = false, j.why = "cancelled (unlock / another selection meanwhile)";
            if (j.ok) {
                const fl::Box live = catch_up_from(j.box, j.t);                 // where that object is in the LIVE frame now
                std::printf("[LOCK] click (%.0f, %.0f): %dx%d object (%s) in %.0f ms\n", j.px, j.py, (int)j.box.w, (int)j.box.h, j.how.c_str(), j.ms);
                if (live.valid() && live.cx() >= 0 && live.cx() <= W && live.cy() >= 0 && live.cy() <= H) {
                    select_rect(live, now); pending_preseg = true;
                    // COMPLETE the object in the background (fl::complete_object): the lock goes ahead NOW with this box, and grows to
                    // the whole object (a truck's cab + cargo, a van seen from close up) ~0.1-0.5 s later
                    if (complete_running) complete_job.wait();                 // (rare) the previous one first: one worker, newest lock wins
                    complete_running = false;
                    {
                        const long g = ++lock_gen;
                        complete_job = std::async(std::launch::async, [f = j.frame, b = j.box, cx = j.used_px, cy = j.used_py, t = j.t, g]() {
                            CompleteRes r; r.t = t; r.gen = g; r.ok = fl::complete_object(f, b, cx, cy, r.whole); return r; });
                        complete_running = true; complete_ready.reset(); complete_started = now;
                    }
                    // where the click fell inside the object, carried to the live box: the segmenter's point prompt
                    click_rel = cv::Point2d((j.used_px - j.box.x) / std::max(1.0, j.box.w), (j.used_py - j.box.y) / std::max(1.0, j.box.h));
                }
                else say("lock refused: the object left the view while it was being segmented", SEV_WARN, "Lock refused: object left the view");
            } else {
                say("lock refused at (" + std::to_string((int)j.px) + ", " + std::to_string((int)j.py) + "): " + j.why + " -- no object found near the click either",
                    SEV_WARN, "Lock refused: " + j.why);
            }
            if (!args.log_dir.empty()) {                                  // what the operator clicked and what was locked
                // written on a background thread: two 720p PNGs on the loop thread stalled it 0.4-0.7 s right at the lock (review)
                char fr0[512]; std::snprintf(fr0, sizeof fr0, "%s/click_%s_%03d_frame_%d_%d.png", args.log_dir.c_str(), run_tag.c_str(), click_dbg_n + 1, (int)j.px, (int)j.py);
                cv::Mat dbg = j.frame.clone();
                cv::drawMarker(dbg, cv::Point((int)j.px, (int)j.py), j.ok ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 16, 2);
                if (j.ok) { cv::rectangle(dbg, cv::Rect((int)j.box.x, (int)j.box.y, (int)j.box.w, (int)j.box.h), cv::Scalar(0, 255, 0), 1);
                            if (j.used_px != j.px || j.used_py != j.py) cv::line(dbg, cv::Point((int)j.px, (int)j.py), cv::Point((int)j.used_px, (int)j.used_py), cv::Scalar(0, 255, 255), 1); }
                char fn[512]; std::snprintf(fn, sizeof fn, "%s/click_%s_%03d_%s.png", args.log_dir.c_str(), run_tag.c_str(), ++click_dbg_n, j.ok ? "ok" : "refused");
                std::thread([f0 = std::string(fr0), fr = j.frame, f1 = std::string(fn), dbg]() { cv::imwrite(f0, fr); cv::imwrite(f1, dbg); }).detach();
            }
        }
        bool lock_request = false;
        if (pending_box) {
            if (vs.link_ok && (vs.guided || args.lock_without_guided)) { lock_request = true; }   // track-only: never asks for GUIDED
            else if (now - pending_since > 2.5) { say(vs.link_ok ? "lock refused: GUIDED not confirmed (is the vehicle armed/GPS ok?)" : "lock refused: no vehicle link", SEV_WARN, vs.link_ok ? "Lock refused: GUIDED not confirmed" : ""); pending_box.reset(); }
            else if (vs.link_ok && !link->mode_pending()) link->request_guided();        // non-blocking; the loop keeps running
        }
        if (lock_request) {
            fl::Box pb = *pending_box; pending_box.reset();
            bool seg_ok = true;
            if (pending_preseg) {
                // already segmented in the background lock job (and caught up to the live frame)
            } else {
                // RECTANGLE drawn by the operator: GrabCut tightens it to the object inside; if it cannot, the drawn box is used
                pb.x = std::max(0.0, pb.x); pb.y = std::max(0.0, pb.y); pb.w = std::min(pb.w, W - pb.x); pb.h = std::min(pb.h, H - pb.y);
                fl::Box rb;
                if (args.refine_lock && fl::grabcut_refine(frame, pb, rb)) { std::printf("[LOCK] GrabCut tightened the drawn box %dx%d -> %dx%d\n", (int)pb.w, (int)pb.h, (int)rb.w, (int)rb.h); pb = rb; }
            }
            const bool was_preseg = pending_preseg;
            pending_point = false; pending_preseg = false;
            (void)was_preseg; click_rel = cv::Point2d(-1, -1);
            if (!seg_ok) lock_request = false;
            else if (lt.init(frame, pb)) { box = pb; kf.reset(pb); last_accept = now; tracking = true; refit_last_w = refit_last_h = 0;
                                          have_ref = false; ref_h = 0; lock_dep = std::numeric_limits<double>::quiet_NaN(); settle.clear(); goto_sent = false;   // a fresh lock re-enables normal follow control
                                          // A new lock supersedes any earlier dash: stop re-sending its position target (it would
                                          // otherwise fight the follow velocities at 1 Hz) and restart the velocity session, which
                                          // the dash ended -- the FSM may have stayed non-IDLE throughout, so the session-edge
                                          // logic below would never restart it on its own.
                                          link->cancel_goto(false); link->clear_hold(); goto_path_active = false;
                                          link->begin_session();
                                          pursuit_tgt = fl::Ned{}; pursuit_last = fl::Ned{};
                                          goto_start = fl::Ned{}; goto_final = fl::Ned{}; goto_hold = fl::Ned{};
                                          goto_along = 0; goto_total = 0;
                                          pursuit_arrived = false; pursuit_static = false;
                                          go_braking = false; go_punctured = false;
                                          go_have_prev_v = false; go_prev_v = fl::Ned{}; go_acc_f = fl::Ned{};
                                          size_rng.reset(); tgt_est.reset();
                                          tgt_bearing_t = -1e9; tgt_bearing_prev = 0; tgt_world_rate = 0;
                                          target_rate_f = 0; bearing_i.reset();
                                          chase_since = 0; chase_last = fl::Cmd(); chase_last_t = -1e9; chase_vn = 0; chase_ve = 0;
                                          sm_v = 0; sm_t = -1e9;
                                          dist_f = -1; dist_t = -1e9;
                                          view_vx = 0; view_vx_t = -1e9;
                                          vert_err_f = 0; vert_err_t = -1e9;
                                          view_ref_h = 0; view_goal_dist = 0; view_hold_dist = 0; view_approaching = false;
                                          rescue.reset();
                                          lock_t = now; view_lock_alt = vs.alt_m; abort_latched = false;
                                          if (vs.mount_known) gimbal_cmd_pitch = vs.mount_pitch_deg + (args.mount_frame == "body" ? fl::rad2deg(vs.pitch) : 0.0);   // start the gimbal servo from where it is
                                          ref_color = fl::box_mean_color(frame, pb);
                                          if (c12) c12->start_tracking((int)std::lround((pb.x + pb.w / 2) / W * args.c12_w), (int)std::lround((pb.y + pb.h / 2) / H * args.c12_h));
                                          say("LOCKED on a " + std::to_string((int)pb.w) + "x" + std::to_string((int)pb.h) + " box");
                                          // Any lock (drag, QGC tap/rectangle, RC centre-lock) hands steering to this program and the
                                          // sticks stop working -- make that explicit rather than leaving the pilot to notice the hard way.
                                          if (vs.guided) link->send_statustext(4 /* WARNING */, "TRACKING: sticks disabled, change mode to take over");
                                          else link->send_statustext(5 /* NOTICE */, "TRACK ONLY: select GUIDED to follow"); }
            else { say("lock failed: box too small", SEV_WARN); lock_request = false; }
        }

        // ---- tracking ----
        double t1 = now_s(); bool tracker_ok = false, reacquired_now = false;
        const fl::Box motion_excl = tracking ? kf.box() : fl::Box();     // the target's box last frame: its own motion is not the camera's
        // how the whole image moved since the last frame (or empty) -- only while tracking: idle it cost ~6 ms a frame for nothing
        // always stepped (an idle-time skip saved ~6 ms but left the FIRST frames after a lock without any camera-motion estimate,
        // just as the drone starts turning and the gimbal tilting -- measured live: the new filter gate then rejected the real target
        // and the lock was lost within a second)
        cv::Mat cam_A = cam_motion.step(frame, tracking ? &motion_excl : nullptr);
        int cam_src = cam_A.empty() ? 0 : 1;
        // FALLBACK when the image-based estimate failed (too few features: motion blur during a fast gimbal tilt or turn, a plain
        // runway): the camera's OWN motion between this frame and the last, from what we know -- the gimbal tilt commanded at each
        // frame's time and the vehicle's turn (pose history). Tilting up moves the scene down the image; turning right moves it
        // left. Before, such a frame got no compensation at all and the prediction stayed where the target had been -- losses
        // during gimbal turns (operator report).
        {
            const double tf = now - args.video_latency;
            const double gp = gimbal_hist_at(tf);
            fl::PoseSample pz; const bool have_pz = pose_hist.at(tf, pz);
            if (tracking && cam_A.empty() && cam_prev_t > 0 && now - cam_prev_t < 0.5) {
                const double dpitch = fl::deg2rad(gp - cam_prev_gimbal), dyaw = have_pz && cam_prev_have_yaw ? std::remainder(pz.yaw - cam_prev_yaw, 2 * M_PI) : 0.0;
                // a turn shifts the image by ~fx tan(dyaw cos(tilt)) -- the rest is rotation (review: without cos it was 2.9x at 70 deg down)
                const double dx = -K.fx * std::tan(dyaw * std::cos(fl::deg2rad(gp))), dy = K.fy * std::tan(dpitch);
                if (std::hypot(dx, dy) > 1.0) { cam_A = (cv::Mat_<double>(2, 3) << 1, 0, dx, 0, 1, dy); cam_src = 2; }
            }
            cam_prev_gimbal = gp; cam_prev_t = now; cam_prev_have_yaw = have_pz; if (have_pz) cam_prev_yaw = pz.yaw;
        }
        // Elapsed time in nominal frames for the Kalman prediction / plausibility gate: 1 at the steady rate, ~2 after a dropped frame.
        // The nominal frame period is a slow average of recent gaps, so it adapts to whatever rate the source really delivers.
        if (frame_gap > 0 && frame_gap < 0.5) nominal_gap = nominal_gap <= 0 ? frame_gap : 0.95 * nominal_gap + 0.05 * frame_gap;
        const double kf_steps = nominal_gap > 0 ? std::clamp(frame_gap / nominal_gap, 0.25, 5.0) : 1.0;
        // A VIDEO FREEZE is not the target's doing: measured in the sim with a heavy scene, Gazebo's camera stream froze 1-5 s at a
        // time (168 freezes in 85 min) and every freeze > 1 s counted as "target lost". Up to --stall-tolerance s the lock's clock
        // pauses for the freeze and tracking simply continues on the next frame (CSRT, else the hidden-target search, whose reach
        // grows with the frames that follow). Longer: as before -- the target may be anywhere.
        if (tracking && frame_gap > 0.4 && frame_gap <= args.stall_tolerance) {
            last_accept += frame_gap - std::max(0.0, nominal_gap);
            if (frame_gap > args.lost_after) say("video froze " + std::to_string(frame_gap).substr(0, 3) + " s: lock kept", SEV_INFO, "-");
        }
        if (tracking && frame_gap > args.stall_tolerance) {              // video was dark too long: the target may be anywhere
            kf.predict(kf_steps); kf.damp_velocity(0.9);                          // do NOT accept a box from this frame; the state machine runs COASTING -> LOST
        } else if (tracking) {
            if (!cam_A.empty()) { kf.warp(cam_A); lt.warp(cam_A); }      // the camera moved: move the prediction (and last-seen spot) with it
            kf.predict(kf_steps);
            const fl::Box predicted = kf.box();
            // the gate widens (x2) while the image is disturbed (gimbal moving, hard turn / pitch) or no camera-motion estimate exists
            // this frame -- the prediction cannot know where the camera went
            const double gate_k = (cam_src == 0 || now - gimbal_moved_at < 0.3 || std::fabs(vs.yawspeed) > 0.5 || now - lock_t < 1.0) ? 2.0 : 1.0;
            auto plausible = [&](const fl::Box& b) { return fl::box_plausible(predicted, b, W, H, fl::PlausCfg(), kf_steps) && fl::filter_gate(kf, predicted, b, kf_steps, gate_k); };
            const bool was_occluded = lt.occluded();
            {   // disturbance: the gimbal moved in the last 0.3 s, the zoom changed in the last 0.6 s, the airframe's attitude changed
                // > 4 deg since the frame was taken, or the drone turns > 0.8 rad/s
                fl::PoseSample pz; const bool hz = pose_hist.at(now - args.video_latency, pz);
                const bool disturbed = now - gimbal_moved_at < 0.3 || now - zoom_changed_at < 0.6 || std::fabs(vs.yawspeed) > 0.8 ||
                                       (hz && (std::fabs(vs.pitch - pz.pitch) > fl::deg2rad(4.0) || std::fabs(vs.roll - pz.roll) > fl::deg2rad(4.0)));
                // ... but not during a GO dash: the whole dash is one hard pitch, and blocking every re-find there kept the target LOCKED in
                // only 29-40% of the dash (suite). There the IMM gate (widened while disturbed) bounds where it may be found.
                lt.set_disturbed(disturbed && !goto_path_active);
            }
            lt.set_reacq_gate(gate_k * fl::gate_radius(kf, predicted, kf_steps));
            LockTracker::Step ls = lt.step(frame, predicted, plausible);
            if (ls.accepted) {
                tracker_ok = true; box = ls.box; last_accept = now; reacquired_now = ls.reacquired || ls.reanchored;
                // box re-fit, 5 Hz: GrabCut on a slightly larger box decides the object's own extent again (see LockTracker::refit).
                // In the BACKGROUND (~30 ms here, more on a Pi 5 -- never a frame stall); the answer is shifted by how far the box moved
                // meanwhile. Only a consistent answer is taken (same place within half a box, size within 0.6-1.6x), and only halfway
                // in size, so one bad cut cannot change the box much.
                if (refit_running && refit_job.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
                    refit_running = false;
                    const std::pair<int, fl::Box> res = refit_job.get(); fl::Box rb = res.second; const bool ok = res.first > 0, grows = res.first == 2;
                    if (ok && !ls.reacquired && !ls.crowded) {
                        rb.x += box.cx() - refit_from.cx(); rb.y += box.cy() - refit_from.cy();
                        const double sz = std::max(box.w, box.h), ar = (rb.w * rb.h) / std::max(1.0, box.w * box.h);
                        // two cuts in a row agreeing on the size (within 20%): take it fully -- measured on a cube driving away, the
                        // halfway step left the box 1.6x the object's area for seconds; one cut alone: halfway
                        const bool agree = refit_last_w > 0 && std::fabs(rb.w / refit_last_w - 1.0) < 0.2 && std::fabs(rb.h / refit_last_h - 1.0) < 0.2;
                        refit_last_w = rb.w; refit_last_h = rb.h;
                        const bool same = std::hypot(rb.cx() - box.cx(), rb.cy() - box.cy()) < 0.5 * sz && ar > (agree ? 0.35 : 0.6) && ar < ((agree || grows) ? 2.5 : 1.6);
                        const double a_new = (agree || grows) ? 1.0 : 0.5;
                        const double nw = (1 - a_new) * box.w + a_new * rb.w, nh = (1 - a_new) * box.h + a_new * rb.h;
                        const fl::Box nb{rb.cx() - nw / 2, rb.cy() - nh / 2, nw, nh};
                        const bool took = same && zoom == refit_zoom && nb.x >= 0 && nb.y >= 0 && nb.x + nb.w <= W && nb.y + nb.h <= H && lt.refit(frame, box, nb);
                        static const bool dbg_r = std::getenv("TRK_DEBUG") != nullptr;
                        if (dbg_r) std::printf("[REFIT] %.0fx%.0f@(%.0f,%.0f) -> cut %.0fx%.0f@(%.0f,%.0f) %s  %.1f ms\n", box.w, box.h, box.cx(), box.cy(),
                                               rb.w, rb.h, rb.cx(), rb.cy(), took ? "TAKEN" : "refused", (now - refit_t0) * 1000);
                        if (took) box = nb;
                    }
                }
                if (args.box_refit && !refit_running && !ls.reacquired && !ls.crowded && now >= refit_next) {
                    refit_next = now + 0.2;
                    const fl::Box rough{box.cx() - 0.65 * box.w, box.cy() - 0.65 * box.h, 1.3 * box.w, 1.3 * box.h};
                    // only the region GrabCut looks at is copied (1.5x the rough box), in frame coordinates
                    cv::Rect roi((int)std::floor(rough.x - 0.3 * rough.w), (int)std::floor(rough.y - 0.3 * rough.h), (int)std::ceil(1.6 * rough.w), (int)std::ceil(1.6 * rough.h));
                    roi &= cv::Rect(0, 0, W, H);
                    if (roi.width > 16 && roi.height > 16) {
                        cv::Mat crop = frame(roi).clone(); const fl::Box rr{rough.x - roi.x, rough.y - roi.y, rough.w, rough.h};
                        refit_job = std::async(std::launch::async, [crop, rr, roi]() {
                            fl::Box b; if (!fl::grabcut_refine(crop, rr, b)) return std::make_pair(0, fl::Box());
                            // the cut filling its rough box on 2+ sides: the object is at least that big (approaching, it grows
                            // faster than CSRT's scale search follows -- measured 16 -> 35 px with the box stuck at ~20)
                            const double m = 0.04 * std::max(rr.w, rr.h) + 1;
                            const int touch = (b.x <= rr.x + m) + (b.y <= rr.y + m) + (b.x + b.w >= rr.x + rr.w - m) + (b.y + b.h >= rr.y + rr.h - m);
                            return std::make_pair(touch >= 2 ? 2 : 1, fl::Box{b.x + roi.x, b.y + roi.y, b.w, b.h});
                        });
                        refit_running = true; refit_from = box; refit_zoom = zoom; refit_t0 = now;
                    }
                }
                // re-found: its old velocity may point the wrong way. A RE-ANCHOR is the prediction itself, not a measurement: feeding it
                // back gave zero innovation, the velocity never corrected and the box slid on in a straight line (review) -- not fed.
                if (ls.reacquired) kf.reset(box); else if (!ls.reanchored) kf.update(box); else kf.damp_velocity(0.97);
                const cv::Scalar c = fl::box_mean_color(frame, ls.box);
                ref_color = fl::adapt_color(ref_color, c);
                if (ls.reacquired) say("target re-acquired after " + std::to_string(occl_frames) + " hidden frames", SEV_NOTICE, "Target re-acquired");
                if (ref_h <= 0) {                                       // settle: the reference size is the median of the first accepted boxes, not the raw drag
                    settle.push_back(ls.box.h);
                    if (settle.size() >= 8) { std::sort(settle.begin(), settle.end()); ref_h = settle[settle.size() / 2]; }
                }
                occl_frames = 0;
            } else {
                kf.damp_velocity(ls.occluded ? 0.97 : 0.9);             // hidden: keep most of the motion (it is the best guess)
                if (ls.occluded) {
                    occl_frames++;
                    if (!was_occluded) say("target hidden: holding, searching for it (CSRT frozen so it cannot learn the occluder)", SEV_INFO, "Target hidden: searching");
                }
                if (ls.raw_ok && !ls.occluded && now - last_reject_log > 1.0) {
                    last_reject_log = now;
                    say(plausible(ls.raw) ? "tracker box rejected (no longer looks like the locked object)"
                                          : "tracker box rejected (implausible jump/size)", SEV_INFO, "-");   // console only
                }
            }
        }
        st.track = (now_s() - t1) * 1000;

        // ---- follow state machine ----
        if (want_unlock) { ++lock_gen; queued_click.reset(); }           // an unlock cancels a queued click and any pending completion
        fl::FsmIn fi; fi.now = now; fi.box_age = tracking ? now - last_accept : 0; fi.lock_request = lock_request; fi.unlock_request = want_unlock;
        fi.link_ok = vs.link_ok; fi.guided = vs.guided; fi.require_guided = !args.lock_without_guided;
        fsm.update(fi);
        if (fsm.state() != prev_state) {
            {   // LOCKED<->COASTING flickers on every brief occlusion: console only. LOST/LINK_LOST are warnings.
                const bool flicker = (prev_state == fl::FState::LOCKED && fsm.state() == fl::FState::COASTING) ||
                                     (prev_state == fl::FState::COASTING && fsm.state() == fl::FState::LOCKED);
                const bool bad = fsm.state() == fl::FState::LOST || fsm.state() == fl::FState::LINK_LOST;
                say(std::string(fl::to_str(prev_state)) + " -> " + fl::to_str(fsm.state()) + " (" + fsm.reason() + ")",
                    bad ? SEV_WARN : SEV_INFO, flicker ? "-" : std::string("Track ") + fl::to_str(fsm.state()) + ": " + fsm.reason());
            }
            // Previously a lost lock only printed to the companion's own console/log -- nothing the officer in the field would
            // see -- and left the vehicle sitting in GUIDED with no stick input until the unlock timeout. Tell the pilot via
            // STATUSTEXT (QGC shows these) and hand control back immediately via LOITER (ArduCopter mode 5) rather than
            // waiting; the follow FSM drops the lock on its own next tick once the mode leaves GUIDED, same as any override.
            // NOT during an active go-to dash: losing the tracker box mid-dash is handled by its own, deliberate design (the
            // last good endpoint is kept, the dash is not cancelled) -- forcing LOITER here would cut a confirmed rescue dash
            // short over a one-frame CSRT flicker, which is a worse outcome than finishing the already-armed dash.
            // Also NOT after a completed dash (goto_sent stays true until a relock/abort): the vehicle is already holding the
            // go-to point in GUIDED, and CSRT losing a now very close, large-in-frame target there is expected, not an emergency.
            // Switching to LOITER at that moment made an unattended vehicle sink (LOITER follows the throttle stick); holding the
            // reached point is the safer default, and the pilot can still take over with any mode change.
            // Target missed while following: HOVER IN THE SAME PLACE (operator's rule) -- the follow loop goes silent and the drone
            // holds a position target right where it is, in GUIDED, until a new lock / GO / ABORT or the pilot's mode change.
            // --lost-loiter restores the old hand-back (switch to LOITER: in LOITER the vehicle follows the sticks).
            if (fsm.state() == fl::FState::LOST && vs.guided && !goto_path_active && !goto_sent) {
                if (args.lost_loiter) {
                    link->send_statustext(4 /* MAV_SEVERITY_WARNING */, "Target lost: switching to LOITER");
                    link->send_mode_change(5 /* ArduCopter LOITER */);
                } else if (vs.pos_valid) {
                    link->handover_session();
                    link->send_goto(fl::Ned{vs.pos_n, vs.pos_e, vs.pos_d}, rcfg.speed_ms, vs.yaw);
                    goto_sent = true;                                     // held: no follow, no LOITER; a new lock releases it
                    say("Target lost: hovering here", SEV_WARN, "Target lost: hovering here");
                }
            } else if (fsm.state() == fl::FState::LOST && vs.guided && goto_sent && !goto_path_active) {
                link->send_statustext(4 /* MAV_SEVERITY_WARNING */, "Target lost at arrival: holding position");
            }
            prev_state = fsm.state();
        }
        if (fsm.state() == fl::FState::IDLE && tracking) { lt.reset(); tracking = false; if (c12) c12->stop_tracking(); }
        const bool session = fsm.state() != fl::FState::IDLE;
        if (session && !prev_session) link->begin_session();
        if (!session && prev_session) link->end_session();
        prev_session = session;

        // ---- control ----
        double t2 = now_s();
        fl::Cmd desired;
        double gimbal_yaw = 0, gimbal_pitch = 0, gimbal_age = 1e9; const bool gimbal_ok = c12 && c12->gimbal(gimbal_yaw, gimbal_pitch, gimbal_age);
        // Camera boresight: the C12's own attitude when it is fresh, otherwise the fixed mount angles. With a C12 attached but its
        // attitude stale, the boresight is UNKNOWN (the operator may have tilted it anywhere) -- cam_known = false then refuses a
        // destination estimate rather than silently assuming the fixed angles.
        const bool gimbal_fresh = gimbal_ok && gimbal_age < 0.5;
        const bool mount_fresh = args.gimbal_centre && vs.mount_known && vs.mount_age < 0.5;   // the autopilot's own gimbal
        double cam_yaw = gimbal_fresh ? gimbal_yaw : args.cam_yaw_deg, cam_pitch = gimbal_fresh ? gimbal_pitch : args.cam_pitch_deg;
        if (mount_fresh) { cam_yaw = vs.mount_yaw_deg; cam_pitch = vs.mount_pitch_deg; }
        const bool cam_known = gimbal_fresh || mount_fresh || (!c12 && !args.gimbal_centre);
        // measured in SITL against the truth: with the servo mount's body-frame angles taken as earth-frame, the depression error was
        // 0.94 x the vehicle pitch -- an accelerating drone (pitch -20 deg) saw the target twice as far away as it was
        const bool mount_body = mount_fresh && args.mount_frame == "body";
        const bool stabilized = (args.camera_mount == "gimbal" || args.gimbal_centre) && !mount_body;
        double h_ground = 0; const bool have_height = fl::pick_height(args.height_source, vs.alt_valid, vs.alt_m, vs.rng_m, vs.rng_age, h_ground, height_used);

        // ---- the target's ground position this frame (flat ground under it) -> world-frame estimator ----
        // Used by the follow feed-forward and the rescue destination alike.
        bool tgt_ok = false; fl::Ned tgt{}; double tgt_sigma = 1e9; double los_dep = -1;   // los_dep: how steeply down the target is seen (rad)
        const fl::Ned veh_now{vs.pos_n, vs.pos_e, vs.pos_d};
        // the pose the camera had when this frame was taken (--video-latency), for the geometry only; control uses the current one
        if (vs.pos_valid) pose_hist.push(fl::PoseSample{now, vs.pos_n, vs.pos_e, vs.pos_d, vs.yaw, vs.pitch, vs.roll, vs.alt_m,
                                                        mount_fresh ? vs.mount_pitch_deg : cam_pitch, mount_fresh ? vs.mount_yaw_deg : cam_yaw});
        fl::PoseSample ps{now, vs.pos_n, vs.pos_e, vs.pos_d, vs.yaw, vs.pitch, vs.roll, vs.alt_m, cam_pitch, cam_yaw};
        const bool have_ps = args.video_latency > 0 && pose_hist.at(now - args.video_latency, ps);
        if (have_ps && mount_fresh) { cam_pitch = ps.mpitch; cam_yaw = ps.myaw; }
        // CENTRING with a late picture: the box is where the target was when the frame was taken (--video-latency ago); the drone has
        // turned since. The bearing to steer on is the one NOW = the frame's bearing minus that turn -- without it the yaw keeps
        // turning for a picture that is already out of date and overshoots past the centre (then back: the target swings around it)
        const double yaw_since_frame = have_ps ? std::remainder(vs.yaw - ps.yaw, 2 * M_PI) : 0.0;
        const fl::Ned veh_then{ps.n, ps.e, ps.d};
        if (args.video_latency > 0 && height_used == "home") h_ground += ps.alt - vs.alt_m;   // same correction for the height above home
        if (vs.pos_valid && have_height) {
            double live_target_h = args.target_height;
            if (live_target_h <= 0.0 && tracking && tracker_ok) {
                const fl::Box sb = kf.box();
                if (sb.h > 4) {
                    if (size_rng.ready()) {
                        live_target_h = std::clamp(size_rng.H(), 0.05, 30.0);
                    } else {
                        fl::ViewAngles va_bot = fl::view_angles(sb.cx() - K.cx, (sb.y + sb.h) - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, stabilized, ps.pitch, ps.roll);
                        if (va_bot.depression > fl::deg2rad(4.0)) {
                            const double slant = h_ground / std::sin(va_bot.depression);
                            const double h_est = slant * sb.h / K.fy;
                            if (std::isfinite(h_est) && h_est > 0.05 && h_est < 30.0) live_target_h = h_est;
                        }
                    }
                }
            }
            fl::ViewAngles va_bot{}, va_cen{}, va{};
            bool have_va = false;
            if (gimbal_mode) {
                if (gimbal_fresh) {
                    va = fl::view_angles(0, 0, 1, 1, gimbal_yaw, gimbal_pitch, stabilized, vs.pitch, vs.roll);
                    va_bot = va;
                    va_cen = va;
                    have_va = true;
                }
            } else if (tracking && cam_known) {
                const fl::Box sb = kf.box();
                va_bot = fl::view_angles(sb.cx() - K.cx, (sb.y + sb.h) - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, stabilized, ps.pitch, ps.roll);
                va_cen = fl::view_angles(sb.cx() - K.cx, sb.cy() - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, stabilized, ps.pitch, ps.roll);
                va = (args.centre_target || args.go_through) ? va_cen : va_bot;
                have_va = true;
            }
            if (have_va && !gimbal_mode) los_dep = va_cen.depression;
            if (args.gimbal_centre && now - gimbal_jump_at < args.gimbal_settle) have_va = false;
            if (have_va) {
                const double tgt_h = (live_target_h > 0.05) ? live_target_h : 1.0;
                const double contact_h = std::clamp(tgt_h * 0.85, 0.85, tgt_h);
                const double ground_d = veh_then.d + h_ground;
                tgt.d = ground_d - contact_h; // z = 0.85-1.0m, landing gear clears asphalt completely

                double dist_h = 0.0;
                double hdg = ps.yaw + va_cen.bearing_right;
                const double h_to_centroid = h_ground - 0.5 * tgt_h;

                if (!gimbal_mode && va_cen.depression > fl::deg2rad(4.0) && h_to_centroid > 0.1) {
                    dist_h = h_to_centroid / std::tan(va_cen.depression); // True 3D centroid ray intersection
                    tgt.n = veh_then.n + dist_h * std::cos(hdg);
                    tgt.e = veh_then.e + dist_h * std::sin(hdg);
                    tgt_ok = true;
                    tgt_sigma = fl::geometry_sigma(h_ground, va_cen.depression);
                } else if (!gimbal_mode && va_bot.depression > fl::deg2rad(4.0) && h_ground > 0.1) {
                    dist_h = (h_ground / std::tan(va_bot.depression)) + 0.5 * tgt_h;
                    hdg = ps.yaw + va_bot.bearing_right;
                    tgt.n = veh_then.n + dist_h * std::cos(hdg);
                    tgt.e = veh_then.e + dist_h * std::sin(hdg);
                    tgt_ok = true;
                    tgt_sigma = fl::geometry_sigma(h_ground, va_bot.depression);
                } else {
                    const double height = std::max(0.05, h_ground - std::max(0.0, live_target_h) * 0.5);
                    tgt_ok = fl::target_ned_from_view(veh_then, ps.yaw, height, va_cen.bearing_right, va_cen.depression, fl::deg2rad(args.min_depression_deg), tgt, 0.05);
                    tgt.d = ground_d - contact_h;
                    tgt_sigma = fl::geometry_sigma(height, va_cen.depression);
                }
                // 3D positioning from optical extent and line-of-sight ray: fallback if ground fix unavailable
                if (!gimbal_mode && tracking && tracker_ok) {
                    const double hpx = kf.box().h;
                    if (tgt_ok && va_bot.depression >= fl::deg2rad(20.0)) {
                        const double slant_ground = (h_ground / std::sin(va_bot.depression)) + live_target_h * 0.5;
                        size_rng.learn(slant_ground, hpx, K.fy, va_cen.depression);
                    }
                    if (size_rng.ready() && !tgt_ok) {
                        const double slant = size_rng.slant(hpx, K.fy);
                        const fl::Ned ray_tgt = fl::target_ned_from_ray(veh_then, ps.yaw, slant, va_cen.bearing_right, va_cen.depression);
                        tgt = ray_tgt;
                        tgt_sigma = fl::SizeRange::sigma(slant) * size_rng.angle_factor(va_cen.depression);
                        tgt_ok = true;
                    }
                }
            }
        }
        if (!tracking || lock_request) size_rng.reset();
        if (!tracking || lock_request) tgt_est.reset();                   // a new lock is a new target
        // ... not while the airframe is pitching / rolling fast: the attitude at the frame's time is interpolated from 50 Hz samples
        // with a latency guess, and a few degrees off moves the ground point metres (measured: +-38 deg pitch swings made a slow
        // target "move" at 26 m/s, which the speed match then chased)
        else if (tgt_ok && fsm.state() == fl::FState::LOCKED && (gimbal_mode || tracker_ok)
                 && (goto_path_active || (std::fabs(vs.pitch - ps.pitch) < fl::deg2rad(4.0) && std::fabs(vs.roll - ps.roll) < fl::deg2rad(4.0)))) tgt_est.update(now - (have_ps ? args.video_latency : 0.0), tgt, tgt_sigma);   // only fresh, accepted views; stamped with the FRAME's time (the target was there ~latency ago)
        const bool tgt_vel_ok = tgt_est.valid(now);

        // ---- view control (2 Hz): move closer or zoom so the target is big enough (fl::decide_view) ----
        if (args.auto_view && tracking && fsm.state() == fl::FState::LOCKED && !goto_sent && now >= view_next && ref_h > 0) {
            view_next = now + 0.25;
            // Digital zoom only adds real detail when the camera delivers more pixels than the tracker uses (track_scale < 1: the
            // C12's 1280x720 tracked at half size). At native resolution it only upsamples -- measured live in the sim, each zoom
            // step there misplaced the small box and LOST a walker that was otherwise tracked fine -- so it stays off.
            const double zoom_cap = args.track_scale < 0.999 ? std::min(args.zoom_max, 1.0 / args.track_scale) : 1.0;
            fl::ViewCfg vc; vc.min_px = args.view_min_px; vc.max_px = args.view_max_px; vc.min_dist = args.view_min_dist; vc.zoom_max = zoom_cap;
            vc.was_approaching = view_approaching;
            const double dist = tgt_ok ? std::hypot(tgt.n - veh_now.n, tgt.e - veh_now.e) : 0.0;
            const double rate = vs.yawspeed + (nominal_gap > 0 ? (kf.vx() / nominal_gap) / K.fx : 0.0);
            // "fast" also by the target's own ground speed (a vehicle driving straight away barely moves in the image): >= 8 m/s
            const bool fast_world = tgt_est.valid(now, 1.0) && std::hypot(tgt_est.vn(), tgt_est.ve()) >= 8.0;
            const fl::ViewDecision vd = fl::decide_view(vc, kf.box().h, zoom, tgt_ok, dist, fast_world ? std::max(std::fabs(rate), 1.0) : rate);
            // zoom now -- but only for a worthwhile change (>= 10%): every change restarts the tracker on the rescaled view, and
            // measured live, small 0.1x corrections every 0.25 s kept doing that for nothing
            // not in the first 2 s after a lock: measured live, zooming within a second of the lock (track not yet settled) lost people
            // ... and only while the view is steady: the target slow in the image (< 1% of the width per frame), the drone not racing
            // (< 2 m/s) and the last zoom change 1.5 s ago -- every change restarts CSRT, and during a fast chase that lost the target
            const bool steady = std::hypot(kf.vx(), kf.vy()) / zoom < 0.01 * W && std::hypot(vs.vel[0], vs.vel[1]) < 2.0 && now - zoom_changed_at > 1.5;
            // zooming OUT is allowed during a chase too (a wider view is what a fast target needs), still >= 1.5 s apart
            const bool zoom_out = vd.zoom < zoom - 1e-6 && now - zoom_changed_at > 1.5;
            if (now - lock_t > 2.0 && (steady || zoom_out) && (std::fabs(vd.zoom / zoom - 1.0) >= 0.10 || (vd.zoom == 1.0 && zoom != 1.0 && zoom < 1.12))) pending_zoom = vd.zoom;
            if (args.view_climb && now - lock_t > 2.0 && vd.climb != 0) {
                if (vd.climb != view_climb_dir) say(vd.climb > 0 ? "view: climbing for a wider view" : "view: coming back down (target small)", SEV_INFO, "-");
                view_climb_dir = vd.climb; view_climb_until = now + 0.6;    // renewed every 0.25 s while the reason holds
            }
            view_ref_h = vd.ref_h;                                        // ... and the size the follow loop should hold (approach / back off)
            view_approaching = vd.approaching;
            // range: approach / back off to the decided distance; in the good band HOLD the distance it had when it got there --
            // measured live: holding the box SIZE instead let a car's changing aspect (side-on vs angled) drive the drone 46 m away
            if ((vd.approaching || vd.backing_off) && tgt_ok && vd.goal_dist > 0) { view_goal_dist = vd.goal_dist; view_hold_dist = 0; }
            else if (!vd.approaching && !vd.backing_off) { if (view_hold_dist <= 0 && tgt_ok) view_hold_dist = dist; view_goal_dist = view_hold_dist; }
            else view_goal_dist = 0;
            if ((vd.approaching || vd.backing_off) && vd.why.substr(0, 12) != view_why.substr(0, 12)) say("view: " + vd.why, SEV_INFO, "View: " + vd.why.substr(0, 40));
            view_why = vd.why;
        }
        if (!tracking) { view_ref_h = 0; view_goal_dist = 0; view_hold_dist = 0; pending_zoom = 1.0; target_rate_f = 0; dist_f = 0; view_vx = 0; view_approaching = false; vert_err_f = 0; }            // back to the wide view for picking the next target
        // ---- CENTRING: the target's own turn rate around the drone (rad/s), from its WORLD bearing at the time each frame was taken
        // (the vehicle's yaw then, from the pose history, + the camera pan + the box's angle in the image). Two uses below: a full
        // feed-forward (the drone turns as fast as the target goes round it: no steady lag) and the prediction across the video
        // latency (steer on where the target is NOW). Image-only control trailed a moving car by ~130 px (median 35, p90 152 px
        // in a live run); a model of the loop with the 0.2 s video delay: ~145 px -> ~5 px at a constant 0.5 rad/s, stable for a
        // real delay of 0.1-0.35 s. Only from accepted measurements; reset at every new lock / re-acquisition / zoom change.
        const bool world_ff = args.yaw_ff_mode == "world" && !gimbal_mode && cam_known && args.lateral_mode == "yaw";
        // The target's TRUE horizontal bearing from the nose at frame time: through the camera's tilt (and, body-fixed, the airframe's
        // pitch/roll) -- not atan(px/fx), which is only right for a level camera. Looking 51 deg down (the median in a live run) a
        // pixel offset is only cos(51) = 0.63 of the bearing: the yaw loop was 37% too weak, the turn-since-frame correction 1.6x too
        // strong, and a still target read as moving while the drone turned (positive feedback). Found in review.
        const fl::Box cbox = kf.box();
        const double img_b = std::atan((cbox.cx() - K.cx) / K.fx);
        const double true_b = cam_known && !gimbal_mode
            ? fl::view_angles(cbox.cx() - K.cx, cbox.cy() - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, stabilized, ps.pitch, ps.roll).bearing_right
            : fl::deg2rad(cam_yaw) + img_b;
        if (world_ff && tracking && tracker_ok && fsm.state() == fl::FState::LOCKED && !reacquired_now && now - zoom_changed_at > 0.3) {
            const double wb = ps.yaw + true_b;
            const double dtb = now - tgt_bearing_t;
            if (dtb > 0.01 && dtb < 0.5) {
                const double wr = std::remainder(wb - tgt_bearing_prev, 2 * M_PI) / dtb;
                tgt_world_rate += (dtb / (args.yaw_ff_tau + dtb)) * (std::clamp(wr, -3.0, 3.0) - tgt_world_rate);
            } else tgt_world_rate = 0;
            tgt_bearing_prev = wb; tgt_bearing_t = now;
        } else if (!tracking || fsm.state() == fl::FState::IDLE || fsm.state() == fl::FState::LOST || reacquired_now) { tgt_world_rate = 0; tgt_bearing_t = -1e9; }
        // seen nearly straight down the bearing (and its rate) is ill-conditioned -- the yaw feed-forward swung +-2 rad/s frame to frame
        // at 71-79 deg in a live log (review): faded out between 60 and 75 deg
        const double nadir_fade = los_dep > 0 ? std::clamp((fl::deg2rad(75.0) - los_dep) / fl::deg2rad(15.0), 0.0, 1.0) : 1.0;
        const double lead = world_ff ? nadir_fade * std::clamp(tgt_world_rate, -lc.ceilings.yaw, lc.ceilings.yaw) * (have_ps ? args.video_latency : 0.0) : 0.0;
        // what image_control adds atan(px/fx) to, so that the sum is the true bearing NOW (latency-corrected)
        const double yaw_offset_now = true_b - img_b - yaw_since_frame + lead;
        if (fsm.steering_allowed()) {
            if (gimbal_mode) {                                            // the C12 keeps the target centred; we follow the gimbal's angles
                if (gimbal_fresh && have_height) {
                    // effective pitch of the line of sight below the horizon (adds the vehicle's pitch for a body-relative gimbal)
                    const double los_pitch = -fl::rad2deg(fl::view_angles(0, 0, 1, 1, gimbal_yaw, gimbal_pitch, stabilized, vs.pitch, vs.roll).depression);
                    if (!have_ref) { double d = fl::ground_distance(std::max(0.2, h_ground - args.target_height), los_pitch); if (std::isfinite(d) && -los_pitch >= G.min_depression_deg) { ref_dist = d; have_ref = true; } }
                    desired = fl::gimbal_control(gimbal_yaw, los_pitch, std::max(0.2, h_ground - args.target_height), have_ref ? ref_dist : 0, G, lc.ceilings);
                }                                                         // stale gimbal data / unknown altitude -> zero command
            } else if (cam_known) {
                const bool strafe = args.lateral_mode == "strafe";
                const double hold_h = (args.auto_view && view_ref_h > 0) ? view_ref_h : (ref_h > 0 ? ref_h : kf.box().h);   // until settled: no forward/back
                // camera fixed to the airframe (FPV, no gimbal): centre vertically on the depression below the horizon, not the
                // image row -- see fl::body_cam_vertical_error (the attitude the frame was taken with, like the bearing above)
                const bool fixed_cam = args.camera_mount == "body" && !args.gimbal_centre && !c12;
                // the reference -- WHERE the drone puts the target vertically:
                //   --centre-target (FPV default): ON THE + -- the camera's hover centre (NaN reference), from the first locked frame. The
                //     drone climbs / descends until the target is centred; leaning to accelerate does not count as an error.
                //   --no-centre-target: where it was in the picture once the lock settled (ref_h set); until then hold the altitude
                if (ref_h <= 0) lock_dep = std::numeric_limits<double>::quiet_NaN();
                else if (fixed_cam && !args.centre_target && !std::isfinite(lock_dep)) lock_dep = fl::body_cam_depression(cbox.cx() - K.cx, cbox.cy() - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, ps.pitch, ps.roll);
                const double vert_err = !fixed_cam ? std::numeric_limits<double>::quiet_NaN()
                    : args.centre_target ? fl::body_cam_vertical_error(cbox.cx() - K.cx, cbox.cy() - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, ps.pitch, ps.roll, std::atan(K.cy / K.fy))
                    : !std::isfinite(lock_dep) ? 0.0
                    : fl::body_cam_vertical_error(cbox.cx() - K.cx, cbox.cy() - K.cy, K.fx, K.fy, cam_yaw, cam_pitch, ps.pitch, ps.roll, std::atan(K.cy / K.fy), lock_dep);
                desired = fl::image_control(kf.box(), hold_h, K, G, lc.ceilings, strafe ? fl::deg2rad(cam_yaw) : yaw_offset_now, args.vz_mode == "center", strafe, vert_err);
                // A turn the TRACKER can follow: the image may shift at most ~60% of the box width (>= 25 px) per frame. Measured: locks
                // made with the target far off centre turned at 1.4-2 rad/s (~40-60 px a frame on a 25-50 px box) and were lost within
                // seconds. Caps the CORRECTION only -- the target's own turn rate (feed-forward, added below) moves the scene WITH it.
                if (!strafe && nominal_gap > 0) {
                    const double px_per_frame = std::max(25.0, 0.6 * kf.box().w);
                    const double w_max = std::max(0.3, px_per_frame / (K.fx * std::max(nominal_gap, 0.03)));
                    desired.yaw_rate = fl::sym_clamp(desired.yaw_rate, w_max);
                }
                if (args.auto_view) {
                    // approaching with a distance estimate: speed from the DISTANCE still to go -- the size ratio alone gave ~1 m/s
                    // for a small person 12 -> 8 m (it was lost long before the drone got near)
                    // distance-based when the distance is known (1 m dead-band, no hunting); without it, the size servo only slowly
                    // the distance smoothed over ~1 s: measured live, a car's distance from the bottom of its box jumps by metres as it
                    // turns and moves, and the raw value made the forward/back command flip between -2 and +2 m/s
                    // the distance: from the target's track when it has one, else the raw fix; smoothed over ~0.4 s of TIME (was a fixed
                    // 0.08 per accepted fix -- with fixes dropping out while the gimbal moved it lagged by metres: review, 21.6 m used for
                    // a true 7.8 m and the drone flew on at 75-85 deg down)
                    {
                        double dnow = -1;
                        if (tgt_est.valid(now, 0.5)) { const fl::Ned tp = tgt_est.predict(now - tgt_est.last_fix()); dnow = std::hypot(tp.n - veh_now.n, tp.e - veh_now.e); }
                        else if (tgt_ok) dnow = std::hypot(tgt.n - veh_now.n, tgt.e - veh_now.e);
                        const double dtd = std::clamp(now - dist_t, 0.0, 0.5); dist_t = now;
                        if (dnow > 0) dist_f = dist_f <= 0 ? dnow : dist_f + (dtd / (0.4 + dtd)) * (dnow - dist_f);
                    }
                    double vx_view = (view_goal_dist > 0 && dist_f > 0) ? fl::sym_clamp(0.35 * fl::deadband(dist_f - view_goal_dist, 1.5), args.view_approach_speed)
                                                                        : fl::sym_clamp(desired.vx, 1.0);
                    vx_view *= std::clamp(1.0 - std::fabs(desired.yaw_rate) / std::max(0.1, lc.ceilings.yaw), 0.0, 1.0);   // turn first, then move
                    // no forward flight while the target sits near the top/bottom edge of the view: the camera (gimbal) has to catch
                    // up first, or flying on pushes the target out of the frame -- full speed when centred, none at 25 deg off
                    // (camera fixed to the airframe: the attitude-corrected error -- pitched forward, the target sits high in the
                    // frame by design, and the image row alone would stop the very forward flight that pitched it)
                    const double verr = std::fabs(fl::rad2deg(fixed_cam ? vert_err : std::atan((kf.box().cy() - K.cy) / K.fy)));
                    if (vx_view > 0) vx_view *= std::clamp(1.0 - verr / 25.0, 0.0, 1.0);
                    // eased (~0.6 s), AFTER those factors (review: applied after easing, the instantaneous yaw undid the easing --
                    // every forward-speed jump > 0.8 m/s in a live log came with a yaw swing)
                    { const double dtv = std::clamp(now - view_vx_t, 0.0, 0.5); view_vx_t = now; view_vx += (dtv / (0.6 + dtv)) * (vx_view - view_vx); }
                    desired.vx = view_vx;
                }
                // FIXED-CAMERA CENTRING (--centre-target): forward/back AND up/down from the same smoothed error (~0.3 s: one noisy
                // box must not nod the drone) -- see fl::fixed_cam_centre_vx. Replaces the size / "move closer" forward command, which
                // fought it; the speed match below still adds the target's own speed. Turn first, then move (as the view control).
                if (fixed_cam && args.centre_target) {
                    const double dtf = std::clamp(now - vert_err_t, 0.0, 0.5); vert_err_t = now;
                    vert_err_f += (dtf / (0.5 + dtf)) * (vert_err - vert_err_f);   // 0.5 s (0.3 still chased a bobbing target)
                    desired.vz = G.kz * fl::deadband(vert_err_f, G.db_z);
                    desired.vx = fl::fixed_cam_centre_vx(vert_err_f, G.db_z, args.centre_kx, args.view_approach_speed)
                               * std::clamp(1.0 - std::fabs(desired.yaw_rate) / std::max(0.1, lc.ceilings.yaw), 0.0, 1.0);
                    desired = fl::clamp_keep_direction(desired, lc.ceilings);
                }
                if (args.ki_yaw > 0) {                                    // + integral of the bearing error (anti-windup, see BearingIntegrator)
                    const double bearing = strafe ? fl::deg2rad(cam_yaw) + img_b : yaw_offset_now + img_b;
                    const bool sat = strafe ? std::fabs(desired.vy) >= lc.ceilings.vy - 1e-6 : std::fabs(desired.yaw_rate) >= lc.ceilings.yaw - 1e-6;
                    const double iv = bearing_i.step(bearing, now - t_ctl_prev, args.ki_yaw, args.ki_limit, sat);
                    if (strafe) desired.vy += iv; else desired.yaw_rate += iv;
                    desired = fl::clamp_keep_direction(desired, lc.ceilings);
                }
                // Opt-in: image_control only preserves the framing from lock time, never judges it. Added
                // on top of (not instead of) the existing centring vz, then re-clamped to the same velocity
                // ceilings -- still passes through --alt-floor/--alt-ceiling below either way.
                if (args.best_view) {
                    desired.vz += fl::best_view_vz(kf.box().h / H, args.best_view_min_frac, args.best_view_max_frac, args.best_view_gain);
                    desired = fl::clamp_keep_direction(desired, lc.ceilings);
                }
            }
        }
        // FPV chase: horizontal velocity from the target's world track (fl::chase_cmd), replacing the image-size servo's vx (and the
        // generic feed-forward). The estimator keeps predicting between geometry fixes for up to 1.5 s -- the fixes are intermittent
        // while the gimbal moves (its reported angle is only trusted once it is still, see above), a chase moves it all the time.
        // Measured against the sim's truth: the per-fix ground position is ~1.2 m off (median, 25 m range) and the velocity ~1.7 m/s
        // off on a 4 m/s target, so the chase eases in over 1.5 s (a fresh, still-noisy estimate once sent the drone off at 12 m/s)
        // and ignores a target "speed" no target could have (faster than the chase itself).
        bool chasing = false;
        const bool est_sane = tgt_est.valid(now, 1.5) && std::hypot(tgt_est.vn(), tgt_est.ve()) <= args.fpv_speed;
        if (args.fpv && fsm.steering_allowed() && !gimbal_mode && cam_known && fsm.state() == fl::FState::LOCKED && est_sane) {
            if (chase_since <= 0) chase_since = now;
            fl::ChaseCfg cc; cc.standoff = std::max(args.fpv_standoff, args.view_min_dist); cc.k = args.fpv_k; cc.ff = args.fpv_ff; cc.vmax = args.fpv_speed;
            cc.radial_only = !args.fpv_lateral;
            // approach gently while the target is small in the image: the camera motion of a hard approach shakes a 6-10 px person
            // out of the tracker (measured: walkers lost within seconds at 4 m/s, held at the old 1.5 m/s) -- 1.5 m/s at <= 15 px
            // (unzoomed), the full --fpv-approach from 40 px
            const double hpx = box.h / std::max(1.0, zoom);
            cc.pull_max = std::min(args.fpv_approach, 1.5 + (args.fpv_approach - 1.5) * std::clamp((hpx - 15.0) / 25.0, 0.0, 1.0));
            const fl::Ned tp = tgt_est.predict(now - tgt_est.last_fix());
            const double dtc = std::clamp(nominal_gap > 0 ? nominal_gap : 0.05, 0.02, 0.2), a_v = dtc / (0.4 + dtc);   // ~0.4 s smoothing
            if (chase_since == now) { chase_vn = tgt_est.vn(); chase_ve = tgt_est.ve(); }
            else { chase_vn += a_v * (tgt_est.vn() - chase_vn); chase_ve += a_v * (tgt_est.ve() - chase_ve); }
            fl::Cmd ch = fl::chase_cmd(cc, veh_now.n, veh_now.e, vs.yaw, tp, chase_vn, chase_ve);
            // yaw feed-forward: the bearing rate from the world track (target velocity relative to ours), not from the image error
            const double dn = tp.n - veh_now.n, de = tp.e - veh_now.e;
            if (std::hypot(dn, de) > 6.0 && args.lateral_mode == "yaw" && !world_ff) {   // (world_ff adds the same rate below: not twice)
                const double w = fl::bearing_rate(dn, de, chase_vn - vs.vel[0], chase_ve - vs.vel[1]);
                desired.yaw_rate = fl::sym_clamp(desired.yaw_rate + args.fpv_yaw_ff * std::clamp((now - chase_since) / 1.5, 0.0, 1.0) * w, lc.ceilings.yaw);
            }
            const double ramp = std::clamp((now - chase_since) / 1.5, 0.0, 1.0);
            desired.vx = ramp * ch.vx + (1 - ramp) * desired.vx; desired.vy = ramp * ch.vy + (1 - ramp) * desired.vy; chasing = true;
            desired = fl::clamp_keep_direction(desired, lc.ceilings);
            chase_last = desired; chase_last_t = now;
        } else if (args.fpv && chase_since > 0 && fsm.steering_allowed() && !gimbal_mode && now - chase_last_t < 1.5) {
            // the estimate dropped out for a moment (a missed geometry fix, a doubtful frame): do NOT switch to the image servo's
            // command at once -- measured: 5 m/s -> 0 in one step pitched the drone +29 deg and the target left the view. Fade the
            // last chase velocity out over 1.5 s instead (the yaw stays with the image loop).
            const double fade = 1.0 - (now - chase_last_t) / 1.5;
            desired.vx = fade * chase_last.vx; desired.vy = fade * chase_last.vy; chasing = true;
        } else chase_since = 0;
        // ---- SPEED MATCH: a fast target, a fast drone. The view control only closes a distance ERROR (<= --view-approach-speed,
        // 1.5 m/s): a car driving away at 12 m/s simply left. Add the target's own speed along the line of sight (from its world
        // track, so it is not the image's guess) to the forward command -- the drone keeps pace and the distance loop only trims.
        // Sideways motion is followed by yaw (with the turn-rate feed-forward above): the radial part grows as the target pulls
        // away, and the drone follows it round. Eased in over 1.5 s once the lock and the track are settled (a fresh, noisy
        // estimate once sent the FPV chase off at 12 m/s), and eased OUT over 1 s when the track drops (a doubtful frame or a short
        // hide): stopping from 10 m/s in one step pitched the drone 29 deg and the target left the view (measured, FPV chase).
        {
            const double dts = std::clamp(now - sm_t, 0.0, 0.5); sm_t = now;
            const double vt = std::hypot(tgt_est.vn(), tgt_est.ve());
            const bool sm_ok = args.speed_match && fsm.steering_allowed() && !gimbal_mode && cam_known && !chasing && tracking
                               && tgt_est.valid(now, 1.0) && vt <= 3.0 * lc.ceilings.vx + 10.0 && fsm.locked_for(now) > 0.5;   // (was 1 s: "slow taking up speed")   // only absurd estimates refused; fast ones are clamped below
            double want = 0;
            if (sm_ok) {
                const fl::Ned tp = tgt_est.predict(now - tgt_est.last_fix());
                const double dn = tp.n - veh_now.n, de = tp.e - veh_now.e, d = std::hypot(dn, de);
                if (d > 2.0) {
                    const double vr = (tgt_est.vn() * dn + tgt_est.ve() * de) / d;              // + = moving away from us
                    // only along the nose: a target well off to the side is turned towards first
                    const double off = std::fabs(std::remainder(std::atan2(de, dn) - vs.yaw, 2 * M_PI));
                    want = fl::deadband(vr, 0.8) * std::clamp(std::cos(off), 0.0, 1.0) * std::clamp((fsm.locked_for(now) - 0.5) / 0.8, 0.0, 1.0);   // full by 1.3 s after the lock
                    want = std::clamp(want, -args.speed_match_back, lc.ceilings.vx);
                }
                sm_v += (dts / (0.4 + dts)) * (want - sm_v);
            } else sm_v *= std::exp(-dts / 0.35);                         // eased out (~1 s to near zero)
            if (!tracking || fsm.state() == fl::FState::IDLE || fsm.state() == fl::FState::LOST) sm_v = 0;
            if (std::fabs(sm_v) > 0.05 && fsm.steering_allowed() && !gimbal_mode && !chasing) {
                desired.vx = fl::sym_clamp(desired.vx + sm_v, lc.ceilings.vx);
                if (desired.vx < -args.speed_match_back) desired.vx = -args.speed_match_back;
            }
        }
        // ---- view height (ViewDecision::climb): climb for a large / fast target up to --view-max-alt, back down to the lock height
        // for a small one. Applied on top of the follow command (the follow law holds altitude in the sim profiles). ----
        if (args.view_climb && fsm.steering_allowed() && !gimbal_mode && tracking && now < view_climb_until && vs.alt_valid) {
            if (view_climb_dir > 0 && vs.alt_m < args.view_max_alt - 0.3) desired.vz = -std::min(args.view_climb_rate, lc.ceilings.vz);
            else if (view_climb_dir < 0 && vs.alt_m > view_lock_alt + 0.5) desired.vz = std::min(0.7 * args.view_climb_rate, lc.ceilings.vz);
        }
        if (!tracking) view_climb_dir = 0;
        // NO OVERFLIGHT (operator report: lost when the drone passed over the target, front to back): once the target is seen nearly
        // straight down (> 70 deg) the drone does not fly forward any more -- past the nadir the camera cannot follow it behind --
        // and beyond 80 deg it backs off gently
        if (fsm.steering_allowed() && los_dep > fl::deg2rad(70.0)) desired.vx = std::min(desired.vx, los_dep > fl::deg2rad(80.0) ? -0.5 : 0.0);
        if (fsm.steering_allowed() && !chasing && args.follow_ff > 0 && tgt_vel_ok && !desired.is_zero())
            desired = fl::clamp_keep_direction(fl::add_feedforward(desired, tgt_est.vn(), tgt_est.ve(), vs.yaw, args.follow_ff), lc.ceilings);
        // Yaw feed-forward: the target's own angular rate around us = our yaw rate + how fast its bearing changes in the image.
        // A P-only loop turns only once an error has built up, so a crossing target drifts toward the edge of the frame.
        if (world_ff && fsm.steering_allowed() && fsm.state() == fl::FState::LOCKED)
            desired.yaw_rate = fl::sym_clamp(desired.yaw_rate + nadir_fade * std::clamp(tgt_world_rate, -lc.ceilings.yaw, lc.ceilings.yaw), lc.ceilings.yaw);
        else if (fsm.steering_allowed() && !gimbal_mode && args.yaw_ff > 0 && args.lateral_mode == "yaw" && tracker_ok && nominal_gap > 0) {
            const double bearing_rate = (kf.vx() / nominal_gap) / K.fx;  // rad/s (Kalman velocity is px per nominal frame)
            // DAMPING term (PD): how fast the target moves ACROSS THE IMAGE, lightly smoothed (0.2 s). Pushes harder while the error
            // grows, and backs off while the target is coming back to the centre. Two earlier versions fed the drone's own turn rate
            // back in (target's world rate = own yaw rate + image rate) and kept turning after the target stopped or reversed --
            // measured live on a car reversing every ~5 s: the command peaked a second AFTER the error did, overshot to the other
            // side, 25-35 reversals a minute at the +-1.2 rad/s limit.
            const double dtc = std::clamp(nominal_gap, 0.02, 0.2);
            target_rate_f += (dtc / (0.2 + dtc)) * (bearing_rate - target_rate_f);
            const double ff = args.yaw_ff * target_rate_f;
            desired.yaw_rate = fl::sym_clamp(desired.yaw_rate + ff, lc.ceilings.yaw);
        }
        // Coasting (target briefly hidden / box doubtful): keep turning toward where it is predicted to be -- yaw only, no
        // translation -- so a target moving out of view stays in view instead of the drone freezing while it leaves.
        bool coast_turn = false;
        // limited to the first second of coasting: after that the prediction is a guess, and turning after a guess can run away
        if (args.coast_yaw && tracking && fsm.state() == fl::FState::COASTING && !gimbal_mode && cam_known && now - last_accept < 1.3) {
            const fl::Box pb = kf.box();
            const double bearing = yaw_offset_now + std::atan((pb.cx() - K.cx) / K.fx);
            // capped at 0.8 rad/s: the prediction is only a guess and the video ~0.2 s late -- at the full FPV ceiling this turn
            // swung +-2 rad/s back and forth (measured)
            desired = fl::Cmd(); desired.yaw_rate = fl::sym_clamp(G.kyaw * fl::deadband(bearing, G.db_yaw), std::min(0.8, lc.ceilings.yaw));
            if (world_ff)                                                 // the target's own turn rate, fading out over the coast window
                desired.yaw_rate = fl::sym_clamp(desired.yaw_rate + nadir_fade * tgt_world_rate * std::clamp(1.0 - (now - last_accept) / 1.3, 0.0, 1.0), lc.ceilings.yaw);
            if (std::fabs(sm_v) > 0.05) desired.vx = fl::sym_clamp(sm_v, lc.ceilings.vx);   // the matched speed eases out, not a dead stop
            coast_turn = true;
        }
        if (!fsm.steering_allowed()) bearing_i.reset();
        t_ctl_prev = now;
        desired = fl::altitude_guard(desired, vs.alt_valid, vs.alt_m, args.alt_floor, args.alt_ceiling);
        // Hold still (no follow-velocity) once the officer has armed the rescue: found live that the normal follow vz (centring
        // the box) kept running through the arm/confirm window and visibly moved the drone before the dash even started. Also
        // stop once a go-to is in flight (the sole controller from there). last_rescue_state is one frame old (computed below,
        // after control runs) -- a single-frame lag at the lock-step video rate, not something the officer would notice.
        const bool rescue_holding = last_rescue_state == fl::RState::ARMED;
        // the first frame after a video stall commands nothing: the lock is kept through short freezes, but this frame's prediction has just
        // jumped by the whole gap -- the next frame steers again (suite: "stale target protection")
        if (video_stalled) desired = fl::Cmd();
        // (the no-overflight limit again, after the coasting command too: frames near nadir are the ones that turn doubtful -- review)
        if (los_dep > fl::deg2rad(70.0)) desired.vx = std::min(desired.vx, los_dep > fl::deg2rad(80.0) ? -0.5 : 0.0);
        link->command(desired, (fsm.steering_allowed() || coast_turn) && !goto_sent && !rescue_holding);

        // [GO-10] CAMERA during the dash: gimbal drones only (--gimbal-centre). With a FIXED (FPV) camera this block never runs: the
        // dash itself points the nose at the target (yaw_to in [GO-6]), which keeps the camera on it.
        // ---- GO: keep the target in the live view while the drone dives at it. The image-only gimbal loop below moves only on good
        // frames and at <= 25 deg/s; in a dash the target slid off the bottom of the view and the lock dropped (measured). During GO
        // the gimbal is aimed at the target's WORLD position (the pursuit's estimate, re-aimed every frame) -- the camera stays on it
        // even through a doubtful frame -- plus a small image correction on good frames, at up to 60 deg/s. ----
        if (args.gimbal_centre && goto_path_active && vs.link_ok && now >= gimbal_next) {
            gimbal_next = now + 0.1;
            const fl::Ned T = pursuit_last;
            const double horiz = std::hypot(T.n - vs.pos_n, T.e - vs.pos_e), down = T.d - vs.pos_d;   // NED: target below = down > 0
            double want = -fl::rad2deg(std::atan2(std::max(0.0, down), std::max(0.3, horiz)));
            if (tracker_ok) want -= 0.5 * (fl::rad2deg(std::atan((box.cy() - K.cy) / K.fy))   // + image error (target below centre -> down)
                                           + (args.video_latency > 0 ? gimbal_cmd_pitch - gimbal_hist_at(now - args.video_latency) : 0.0));
            want = std::clamp(want, -90.0, -5.0);
            const double nc = gimbal_cmd_pitch + std::clamp(want - gimbal_cmd_pitch, -6.0, 6.0);
            if (std::fabs(nc - gimbal_cmd_pitch) > 0.5) gimbal_moved_at = now;
            gimbal_cmd_pitch = nc;
            link->send_gimbal_pitchyaw(gimbal_cmd_pitch, 0.0); gimbal_record(now);
            // the target grows as the range closes: scale the tracker's expected size with it (CSRT's own scale search lagged,
            // the box stayed small and the appearance check rejected the target)
            const double range = std::sqrt(horiz * horiz + down * down);
            if (go_prev_range > 0.5 && range > 0.5) kf.scale_size(std::clamp(go_prev_range / range, 0.8, 1.25));
            go_prev_range = range;
        } else if (!goto_path_active) go_prev_range = 0;
        // ---- gimbal: keep the locked target vertically centred (autopilot mount, --gimbal-centre) ----
        // Only on an ACCEPTED measurement this frame -- never on a prediction: measured live, following the prediction while frames
        // were rejected let the gimbal chase a drifting box straight up into the sky (the image moves, the prediction moves with
        // it, runaway), and the target was gone from view for good. While the target is doubtful or hidden, the gimbal holds.
        // Every frame (was 10 Hz in steps of up to 2.5 deg: each step shifted the whole image ~20+ px at once -- a jump the tracker had
        // to absorb; operator report: misses while the gimbal moved). Same speed, in small per-frame steps: gain and slew scale with
        // the time since the last update. For the first 0.3 s of a doubtful spell it keeps following the (manoeuvre-aware) Kalman
        // prediction at half gain instead of freezing -- a fast target moving up/down the view no longer runs out of it during one
        // or two rejected frames; after that it holds (following a prediction for long once ran the gimbal into the sky).
        const bool gim_meas = tracker_ok && fsm.state() == fl::FState::LOCKED;
        const bool gim_pred = args.gimbal_follow_prediction && !gim_meas && tracking && (fsm.state() == fl::FState::LOCKED || fsm.state() == fl::FState::COASTING)
                              && now - last_accept < args.coast_after + 0.3;
        if (args.gimbal_centre && !goto_path_active && tracking && (gim_meas || gim_pred) && vs.link_ok) {
            const double dtg = std::clamp(now - gimbal_last_t, 0.02, 0.2); gimbal_last_t = now;
            const fl::Box tb = gim_meas ? box : kf.box();
            // + = target below the image centre. Corrected for the tilt the gimbal was commanded since the frame was taken (the
            // picture is --video-latency old): tilting down since then has already moved the target up the image by that much
            double err_deg = fl::rad2deg(std::atan((tb.cy() - K.cy) / K.fy));
            if (args.video_latency > 0) err_deg += gimbal_cmd_pitch - gimbal_hist_at(now - args.video_latency);
            // step from the last COMMANDED angle while it is close to the reported one, so the command leads a slowly responding
            // gimbal (measured: stepping from the reported angle let a target drop out of the bottom of the view during an approach)
            const double mount_ef = vs.mount_pitch_deg + (args.mount_frame == "body" ? fl::rad2deg(vs.pitch) : 0.0);   // earth frame, like the command
            // ... re-based toward a reported angle that is far off -- slew-limited and never against the error (review, confirmed in a
            // live log: near nadir the body servo saturates, the jump to mount_ef tilted the camera UP 15 deg a frame with the target
            // below centre, and the target was lost a frame later)
            double base = gimbal_cmd_pitch;
            if (mount_fresh && std::fabs(mount_ef - gimbal_cmd_pitch) > 15.0) {
                double stp0 = std::clamp(mount_ef - gimbal_cmd_pitch, -args.gimbal_step * dtg / 0.1, args.gimbal_step * dtg / 0.1);
                if ((err_deg > 0 && stp0 > 0) || (err_deg < 0 && stp0 < 0)) stp0 = 0;    // target below: never tilt up (and vice versa)
                base = gimbal_cmd_pitch + stp0;
            }
            if (std::fabs(err_deg) > args.gimbal_db) {                 // dead-band: no constant hunting, the image stays still
                const double k = (1.0 - std::pow(1.0 - std::clamp(args.gimbal_k, 0.0, 0.95), dtg / 0.1)) * (gim_meas ? 1.0 : 0.5);
                const double stp = args.gimbal_step * dtg / 0.1;
                // a target on the ground is below the horizon: never tilt above -5 deg while tracking it
                const double nc = std::clamp(base - std::clamp(k * err_deg, -stp, stp), -90.0, -5.0);
                if (std::fabs(nc - gimbal_cmd_pitch) > 0.4) gimbal_moved_at = now;   // (0.15: small per-frame steps blanked the geometry half the time)
                if (std::fabs(nc - gimbal_cmd_pitch) > 1.5) gimbal_jump_at = now;   // a real JUMP of the camera (the geometry waits for it)
                gimbal_cmd_pitch = nc;
            }
            if (now >= gimbal_next || std::fabs(gimbal_cmd_pitch - gimbal_sent_pitch) > 0.05) {
                link->send_gimbal_pitchyaw(gimbal_cmd_pitch, 0.0); gimbal_record(now); gimbal_sent_pitch = gimbal_cmd_pitch; gimbal_next = now + 0.1;
            }
        }
        st.control = (now_s() - t2) * 1000;

        // [GO-8] STOPPING (1/4) -- ABORT: leave the target and climb, from a dash, a hold after GO, or plain following.
        // ---- ABORT (explicit: window button, 'x', QGC ABORT, stdin "abort"): LEAVE THE TARGET AND CLIMB -- whatever the drone is doing
        // (a GO dash, following, holding after a GO). The GO and the follow loop stop at once and the drone climbs straight up
        // --abort-climb metres (default 10, never above --alt-ceiling) from where it is, and holds there in GUIDED. Nothing of the
        // follow/GO re-engages; the pilot takes over with a mode switch, or locks / GOes again. (QGC's separate BRAKE action still
        // brakes in place.) ----
        if (hard_abort && abort_latched) { say("ABORT: already climbing away", SEV_INFO, "-"); hard_abort = false; rescue_abort = false; }   // once per ABORT (review: each press climbed again)
        if (hard_abort && vs.guided && vs.pos_valid && (goto_path_active || goto_sent || tracking)) {
            abort_latched = true;
            const char* what = goto_path_active ? "GO dash" : goto_sent ? "hold after GO" : "follow";
            double up_alt = vs.alt_m + args.abort_climb;
            if (args.alt_ceiling > 0) up_alt = std::min(up_alt, std::max(vs.alt_m, args.alt_ceiling));
            const fl::Ned sp = link->stop_point();                       // brake to a stop first, then climb (no fly-back)
            const fl::Ned up{sp.n, sp.e, std::min(sp.d, vs.pos_d) - (up_alt - vs.alt_m)};
            link->handover_session();                                   // the follow loop silent now (no zero commands fighting the climb)
            link->cancel_goto(false);
            link->send_goto(up, rcfg.speed_ms, vs.yaw);                  // re-sent until the pilot leaves GUIDED
            goto_path_active = false; goto_sent = true;                 // no follow, no LOITER-on-loss: the climb point is held
            char m[96]; std::snprintf(m, sizeof m, "ABORT (%s): leaving the target, climbing to %.0f m", what, up_alt);
            say(m, SEV_WARN, "ABORT: climbing away");
            rescue_abort = false;                                         // handled here; the rescue controller still sees the abort below
            abort_flash_until = now + 4.0;
        } else if (hard_abort && vs.guided && !vs.pos_valid && (goto_path_active || goto_sent || tracking)) {
            // no position estimate: a climb point can't be placed -- still stop the follow and the GO at once (found in review: the
            // follow carried on and the operator was told "nothing to stop")
            link->handover_session(); link->cancel_goto(false);
            goto_path_active = false; goto_sent = true; rescue_abort = false; abort_flash_until = now + 4.0;
            say("ABORT: stopped (no position estimate for the climb -- take over with the sticks)", SEV_WARN, "ABORT: stopped, no position");
        } else if (hard_abort) say(vs.guided ? "ABORT: nothing to stop (not locked, no GO)" : "ABORT: not in GUIDED -- the pilot already has control", SEV_INFO, "-");

        // =============================================================================================================================
        // [GO-3] DESTINATION + [GO-2] ARM/CONFIRM + [GO-5] START + [GO-6] DASH LOOP + [GO-8] STOPPING -- the GO main-loop block.
        //   1. dest = the target's ground position (live geometry fix, or its IMM-UKF track predicted to now), turned into the
        //      end point by fl::pursuit_point: `standoff` short of it, never below --alt-floor above its ground.
        //   2. RescueController.step(): the readiness gates, then ONE press sends GO (--go-confirm: arm -> preview to QGC -> confirm).
        //   3. ro.send_goto -> the dash starts ([GO-5]); then every frame the dash loop ([GO-6]) re-aims and flies it.
        // Skipped entirely without --enable-rescue. See the GO map at the top of the file.
        // =============================================================================================================================
        // ---- go-to-target: destination from the current view, then the controller (arm/confirm -> ONE go-to command, no further handling) ----
        fl::RState rescue_state = fl::RState::DISABLED;
        bool marker_dest_ok = false; fl::Ned marker_dest{};
        if (rcfg.enabled) {
            bool dest_ok = tgt_ok; fl::Ned dest = tgt;
            // pursuit GO: one frame without a geometry fix (the gimbal just moved -- it is only trusted once still) must not refuse
            // GO -- measured: "no destination estimate yet" at the press, with a good target track a moment old. The track predicted
            // to now stands in for up to 1 s.
            // (LOCKED or briefly COASTING, the track up to 2 s old: measured live, a GO confirm was refused "no destination estimate"
            //  when the ground fixes dropped out for a moment between the two presses)
            if (!dest_ok && !args.goto_paced && tracking && (fsm.state() == fl::FState::LOCKED || fsm.state() == fl::FState::COASTING) && tgt_est.valid(now, 2.0)) {
                dest = tgt_est.predict(now - tgt_est.last_fix()); dest_ok = true;
            }
            if (dest_ok) {
                const fl::Ned veh = veh_now;
                // --rescue-lead: aim where the target will be when the drone gets there (its estimated velocity x the flight time),
                // capped at --rescue-max-lead metres so a bad velocity estimate cannot send the dash far away. Off: the current position.
                if (args.rescue_lead && tgt_vel_ok) {
                    const fl::Ned now_pos = tgt_est.pos();
                    const double t_fly = std::hypot(now_pos.n - veh.n, now_pos.e - veh.e) / std::max(0.5, rcfg.speed_ms);
                    fl::Ned ahead = tgt_est.predict(t_fly);
                    double ln = ahead.n - now_pos.n, le = ahead.e - now_pos.e, lm = std::hypot(ln, le);
                    if (lm > args.rescue_max_lead) { ln *= args.rescue_max_lead / lm; le *= args.rescue_max_lead / lm; }
                    dest.n = now_pos.n + ln; dest.e = now_pos.e + le;
                }
                // dest.d is a direct 3D go-to toward the target, per the user -- but NOT onto the target itself: the raw
                // geometry lands at the target's own ground level, which for a real person/car/anything selectable is a
                // collision course, not a rescue (flagged in review). Stop --rescue-hover-height above it instead (NED d is
                // DOWN, so "above" subtracts). --rescue-level-only overrides this to keep the current altitude instead.
                // Never stop lower than --alt-floor above the target's (estimated) ground, whatever --rescue-hover-height says.
                if (!args.goto_paced) {
                    // pursuit GO: straight at the target, ending --goto-standoff (1 m) short of it -- no hover height above it. The
                    // --alt-floor still holds (the end point is never lower than that above the target's ground); --rescue-level-only
                    // keeps the current altitude instead.
                    // the smoothed track when it has one (raw fixes jitter ~1 m) -- kept for 1.5 s without a new fix: hard pitching in the dash
                    // gates the fixes, and falling back to raw single fixes made the aim jump by metres (review)
                    if (tgt_est.valid(now, 1.5)) dest = tgt_est.predict(now - tgt_est.last_fix());
                    pursuit_tgt = dest;
                    const double floor_d = args.rescue_descend ? (dest.d - args.rescue_hover_height) : veh.d;
                    if (!args.rescue_descend) pursuit_tgt.d = veh.d;
                    // the stop point, signed offset (+ before / - past the target): this GO's override, else the configured one
                    dest = fl::pursuit_point(veh, pursuit_tgt, std::isfinite(go_standoff_req) ? go_standoff_req : args.goto_standoff, floor_d,
                                             goto_path_active ? &goto_start : nullptr);
                    pursuit_floor_d = floor_d;
                } else {
                    if (dest_ok && args.rescue_descend) dest.d -= args.rescue_hover_height;
                    if (dest_ok && !args.rescue_descend) dest.d = veh.d;
                }
            }
            fl::RescueIn ri; ri.now = now; ri.press = rescue_press; ri.abort = rescue_abort || hard_abort || want_unlock;
            ri.follow = fsm.state(); ri.locked_for = fsm.locked_for(now);
            ri.guided = vs.guided; ri.vehicle_armed = vs.armed; ri.link_ok = vs.link_ok; ri.gps_ok = vs.gps_fix >= 3; ri.pos_valid = vs.pos_valid; ri.ekf_ok = vs.ekf_ok;
            ri.dest_ok = dest_ok; ri.dest = dest; ri.veh = fl::Ned{vs.pos_n, vs.pos_e, vs.pos_d}; ri.battery_pct = vs.battery_pct;
            {   // GO READY indicator (see its state above): exactly the gates of a press, plus the range check of the dash
                std::string why = fl::not_ready_reason(ri, rcfg);
                if (why.empty() && std::hypot(dest.n - ri.veh.n, dest.e - ri.veh.e) > rcfg.max_range_m) why = "target too far (--rescue-max-range)";
                go_ready = why.empty(); go_ready_why = why;
                go_predicting = go_guidance_mode != 0 && tgt_est.valid(now, 1.0);
                if (!tracking || goto_path_active || goto_sent) { go_ready_cand = go_ready_said = false; go_ready_cand_t = now; }
                else if (go_ready != go_ready_cand) { go_ready_cand = go_ready; go_ready_cand_t = now; }
                else if (go_ready != go_ready_said && (go_ready ? (now - go_ready_cand_t > 0.1) : (now - go_ready_cand_t > 0.5))) {
                    go_ready_said = go_ready;
                    if (go_ready) say(go_predicting ? "GO READY - press GO / click GO (3D PN prediction)" : "GO READY - press GO / click GO",
                                      SEV_NOTICE, go_predicting ? "GO READY (3D PN)" : "GO READY");
                    else say("GO not ready: " + go_ready_why, SEV_INFO, "GO not ready: " + go_ready_why);
                }
            }
            fl::RescueOut ro = rescue.step(ri);
            rescue_state = ro.state;
            if (ro.state == fl::RState::ARMED && dest_ok) { marker_dest_ok = true; marker_dest = dest; }   // preview while armed
            if (ro.transitioned && ro.state == fl::RState::ARMED) {
                // Preview BEFORE the confirm press: where it will go, how long it takes, and which height reference the estimate
                // rests on (a "home" reference over uneven ground is the most likely source of a wrong endpoint). Previously the
                // officer only learned the distance after the dash had already started. Short enough for STATUSTEXT's 50 chars.
                double rng = std::hypot(dest.n - ri.veh.n, dest.e - ri.veh.e), dz = ri.veh.d - dest.d;   // dz > 0: destination is higher
                char pv[64]; std::snprintf(pv, sizeof pv, "ARMED %.0fm %s%.0fm ETA%.0fs h:%s", rng, dz >= 0 ? "up" : "dn", std::fabs(dz),
                                           std::sqrt(rng * rng + dz * dz) / std::max(0.1, rcfg.speed_ms), height_used.c_str());
                link->send_statustext(4 /* WARNING: needs the officer's attention */, pv);
                std::printf("[FPV DASH] %s -- press GO again within %.0fs to confirm\n", pv, rcfg.confirm_window_s); std::fflush(stdout);
            }
            if (ro.transitioned && ro.state != fl::RState::ARMED)        // ARMED has its own preview message above
                say(std::string("FPV DASH ") + fl::to_str(ro.state) +
                    (ro.end != fl::REnd::NONE ? std::string(" (") + fl::to_str(ro.end) + (ro.detail.empty() ? "" : ": " + ro.detail) + ")" : ""),
                    ro.end != fl::REnd::NONE ? SEV_WARN : SEV_INFO,
                    ro.end != fl::REnd::NONE ? std::string("FPV Dash: ") + (ro.detail.empty() ? fl::to_str(ro.end) : ro.detail) : "");
            // [GO-5] START: the confirm press went through -- fix this GO's speed / stop distance, take the drone from the follow
            // loop (handover_session) and send the first go-to. From here `goto_path_active` drives the dash loop [GO-6] and
            // `goto_sent` keeps the follow loop silent (no follow velocity, no LOITER-on-loss) until a new lock or ABORT.
            if (ro.send_goto) { abort_latched = false;
                pursuit_last = pursuit_tgt; pursuit_seen = now; pursuit_arrived = false;
                go_speed = go_speed_req > 0 ? go_speed_req : rcfg.speed_ms;      // this GO's speed and stop distance, then the overrides are spent
                go_standoff = std::isfinite(go_standoff_req) ? go_standoff_req : args.zero_decel ? -9999.0 : args.goto_standoff;
                go_speed_req = 0; go_standoff_req = std::numeric_limits<double>::quiet_NaN();
                goto_start = ri.veh; goto_final = ro.dest; goto_along = 0; goto_prev_t = now; goto_path_active = true; override_hits = 0; go_have_prev_v = false; go_acc_f = fl::Ned{}; pursuit_static = false; go_braking = false; go_punctured = false;
                goto_total = std::sqrt(std::pow(goto_final.n - goto_start.n, 2) + std::pow(goto_final.e - goto_start.e, 2) + std::pow(goto_final.d - goto_start.d, 2));
                link->send_goto(goto_start, args.goto_paced ? rcfg.speed_ms : go_speed, fl::bearing_to(goto_start, goto_final));   // activates the go-to at the current point
                if (args.goto_paced) link->end_session(); else link->handover_session();   // the go-to is now the sole controller
                goto_sent = true;
                if (!args.goto_paced) link->update_goto_velocity(fl::Ned{}, fl::bearing_to(goto_start, pursuit_tgt));   // velocity from the next frame
                {
                    char gm[128];
                    if (args.goto_paced) std::snprintf(gm, sizeof gm, "GO-TO sent: %d m diagonal at %.0f m/s", (int)goto_total, rcfg.speed_ms);
                    else if (args.go_through) std::snprintf(gm, sizeof gm, "GO-TO sent: THROUGH the target (%d m) at up to %.1f m/s, braking %.0f m past it", (int)goto_total, go_speed, std::max(0.0, -go_standoff));
                    else if (std::fabs(go_standoff) < 0.05) std::snprintf(gm, sizeof gm, "GO-TO sent: %d m straight at the target at %.1f m/s, to the target", (int)goto_total, go_speed);
                    else if (go_standoff > 0) std::snprintf(gm, sizeof gm, "GO-TO sent: %d m straight at the target at %.1f m/s, stopping %.1f m BEFORE it", (int)goto_total, go_speed, go_standoff);
                    else std::snprintf(gm, sizeof gm, "GO-TO sent: %d m at the target at %.1f m/s, stopping %.1f m PAST it", (int)goto_total, go_speed, -go_standoff);
                    say(gm);
                }
            }
            // Dash abort: previously nothing could stop an in-flight go-to once sent (x / abort / unlock only reached
            // RescueController, which had already returned to IDLE one tick after SENT -- the dash itself, driven by
            // goto_path_active below, was untouched by any of them). Cancel the resend loop and brake immediately; the
            // officer regains manual control once the mode switch leaves GUIDED (BRAKE is not GUIDED, so the follow FSM
            // drops the lock on its own next tick, same as any other mode change away from GUIDED).
            // In flight, or still moving with the target after arrival (--rescue-stay-over). NOT a plain completed dash: the vehicle
            // is just holding there, and an RC rescue switch flicked back to low afterwards (its natural reset) must not BRAKE it.
            // [GO-8] STOPPING (2/4) -- the RC rescue switch back to low during a dash: BRAKE mode (an explicit ABORT was handled in (1/4)).
            if ((goto_path_active || (goto_sent && args.rescue_stay_over)) && rescue_abort) {
                link->update_goto_point(veh_now, vs.yaw);                 // no velocity left attached, whatever happens to the BRAKE
                link->cancel_goto();
                link->send_mode_change(17 /* ArduCopter BRAKE */);
                goto_path_active = false; goto_sent = false;
                say("GO-TO aborted: BRAKE commanded", SEV_WARN);
            }
            // QGC's own "Go to location"/"Orbit"/"ROI" stay in GUIDED, so the mode check never catches them, and they write
            // the SAME position-target message we do -- without this check they would silently fight our 1 Hz+ resends.
            // Detected via the FC's own POSITION_TARGET_LOCAL_NED echo diverging from what we last sent (see goto_overridden);
            // cancel OUR resends only (no BRAKE -- forcing a mode change here would itself fight the pilot's deliberate QGC
            // command, the opposite of the fix) and let whichever command is freshest win.
            //
            // Debounced (3 consecutive hits, not 1): confirmed live via a clean SITL run with no QGC or any other sender
            // running at all, so nothing was actually competing -- the dash's closed-loop homing keeps refining goto_final
            // every frame as the target gets closer (and the geometry estimate correspondingly more sensitive), and the
            // FC's own echo of what we last sent is always at least one reporting interval (~200ms) behind that continuous
            // refinement. A single-frame gap between "what we just computed" and "what the FC last echoed back" is exactly
            // what that normal reporting lag looks like, not a second sender -- only a mismatch that PERSISTS across several
            // consecutive checks is actually consistent with something else also writing position targets.
            // [GO-8] STOPPING (3/4) -- someone else (QGC "Go to location", Orbit, ROI) is commanding a position: back off.
            if (goto_path_active && link->goto_overridden(args.rescue_override_margin)) {
                if (++override_hits >= 3) {
                    link->cancel_goto(false);                             // the other command has taken over: do not fight it
                    goto_path_active = false; goto_sent = false; override_hits = 0;
                    say("GO-TO cancelled: another position target was detected (QGC override?)", SEV_WARN, "GO-TO cancelled: other position cmd");
                }
            } else {
                override_hits = 0;
            }
            // [GO-6] DASH LOOP (every frame while the dash runs):
            //   T = where the target is now (live fix while LOCKED, else its track predicted <= 1.5 s, else the last point)
            //   v = fl::go_velocity(...)  [GO-4]  -> + acceleration feed-forward -> link->update_goto_velocity(v, yaw_to)  [GO-9]
            //   yaw_to points the nose at the target (with an FPV fixed camera this is what keeps it in view)
            //   arrival within ~1 m -> on_go_arrived() [GO-7];  nothing seen for 3 s -> hold the last point [GO-8 (4/4)]
            if (goto_path_active && !args.goto_paced) {
                // Pursuit: every frame, re-aim at where the target is NOW (locked: the live estimate; briefly hidden: the track predicted
                // forward up to 1.5 s; longer: the last point) and let the autopilot fly the straight line to the end point at the GO
                // speed. Nothing is paced or held in between -- the vehicle goes straight at it and stops --goto-standoff short.
                fl::Ned T = pursuit_last; bool fresh = false;
                if (dest_ok && fsm.state() == fl::FState::LOCKED) { T = pursuit_tgt; fresh = true; }
                else if (tgt_est.valid(now, 1.5)) { T = tgt_est.predict(now - tgt_est.last_fix()); if (!args.rescue_descend) T.d = veh_now.d; fresh = true; }
                if (fresh) { pursuit_last = T; pursuit_seen = now; }
                else if (T.n == 0 && T.e == 0 && T.d == 0) { T = veh_now; }
                const fl::Ned E = fl::pursuit_point(veh_now, T, go_standoff, pursuit_floor_d, &goto_start);   // the stop point (signed offset)
                // nose: at the target -- or, stopping PAST it (offset < 0), at the stop point: the same direction until the drone is over the
                // target, but it does not whip round 180 deg as the drone flies over it
                const fl::Ned& look = go_standoff < 0 ? E : T;
                // Pass direction (vector from start through target):
                double un = T.n - goto_start.n, ue = T.e - goto_start.e;
                const double uh = std::hypot(un, ue);
                if (uh > 1e-3) { un /= uh; ue /= uh; } else { un = std::cos(vs.yaw); ue = std::sin(vs.yaw); }
                const double pass_yaw = std::atan2(ue, un);

                // FLY-THROUGH (--go-through, go_settings::kFlyThrough): full speed until |offset| m past the target, then brake ahead
                const double past_m = fl::past_target_m(veh_now, T, goto_start, vs.yaw);
                const bool zero_decel = args.zero_decel || (go_standoff <= -9000.0);
                if (args.go_through && past_m >= 0.0 && !go_punctured) {
                    go_punctured = true;
                    char pm[128];
                    std::snprintf(pm, sizeof pm, "FLY-THROUGH: passed over target at %.1f m/s", std::hypot(vs.vel[0], vs.vel[1], vs.vel[2]));
                    say(pm, SEV_INFO, "FLY-THROUGH");
                }
                const double brake_past = std::max(2.0, -go_standoff);
                if (args.go_through && !zero_decel && !go_braking && past_m >= brake_past) {
                    go_braking = true; go_brake_yaw = pass_yaw;                // the heading it passed with: kept while it brakes
                    char bm[96]; std::snprintf(bm, sizeof bm, "GO: through the target at %.1f m/s -- braking", std::hypot(vs.vel[0], vs.vel[1]));
                    say(bm, SEV_INFO, "GO: through, braking");
                }
                const double to_look_h = std::hypot(look.n - veh_now.n, look.e - veh_now.e);
                const double yaw_to = (args.go_through && (go_braking || past_m >= -3.0)) ? (go_braking ? go_brake_yaw : pass_yaw)
                    : to_look_h > 1.0 ? fl::bearing_to(veh_now, look) : pass_yaw;
                const double v_now = std::sqrt(vs.vel[0] * vs.vel[0] + vs.vel[1] * vs.vel[1] + vs.vel[2] * vs.vel[2]);
                // guidance (--go-guidance): against a MOVING target the approach is relative to it -- see fl::go_velocity. The target's
                // velocity and its turn-aware future come from the IMM-UKF track; without a valid track: the previous pursuit law.
                const bool trk = tgt_est.valid(now, 1.0);
                const fl::Ned vt = trk ? fl::Ned{tgt_est.vn(), tgt_est.ve(), 0.0} : fl::Ned{};
                auto predict = [&](double t) { fl::Ned q = tgt_est.predict(now - tgt_est.last_fix() + t); q.d = T.d; return q; };
                const fl::Ned vveh{vs.vel[0], vs.vel[1], vs.vel[2]};
                fl::Ned v;
                const double pass_d = (args.fixed_horizontal || !args.rescue_descend) ? veh_now.d : pursuit_floor_d;
                if (!args.go_through) {
                    const double eff_lag = (go_standoff <= 0.0) ? 0.0 : 0.5;
                    v = fl::go_velocity(trk ? go_guidance_mode : 0, veh_now, vveh, T, vt, predict, go_standoff, pursuit_floor_d, go_speed, args.goto_accel, eff_lag, args.pn_gain, &goto_start);
                    // Proportional vertical glide slope: locks vertical closure rate to horizontal closure rate
                    // so target altitude is reached exactly at the target, preventing premature ground contact or diving
                    if (args.fixed_horizontal) {
                        v.d = 0.0;
                    } else if (args.rescue_descend) {
                        const double dh = std::hypot(T.n - veh_now.n, T.e - veh_now.e);
                        double vh = std::hypot(v.n, v.e);
                        const double dz = pass_d - veh_now.d;
                        if (dh > 0.05) {
                            const double slope = dz / dh;
                            double vd_cmd = vh * slope;
                            if (std::abs(vd_cmd) > args.ceil_vz && std::abs(slope) > 1e-4) {
                                const double vh_max = args.ceil_vz / std::abs(slope);
                                if (vh > vh_max) { v.n *= vh_max / vh; v.e *= vh_max / vh; vh = vh_max; }
                                vd_cmd = vh * slope;
                            }
                            v.d = vd_cmd;
                        } else {
                            v.d = std::clamp(dz * 2.0, -args.ceil_vz, args.ceil_vz);
                        }
                    }
                } else if (!go_braking) {
                    if (past_m < 0) {
                        // Approach: proportional navigation straight onto collision course with the target aperture
                        v = fl::go_velocity(trk ? go_guidance_mode : 0, veh_now, vveh, T, vt, predict, -1000.0, pass_d, go_speed, args.goto_accel, 0.0, args.pn_gain, &goto_start);
                        // Vertical glide: align smoothly to target altitude pass_d so it is reached AT the opening (or level flight on fixed horizontal)
                        if (args.fixed_horizontal) {
                            v.d = 0.0;
                        } else {
                            const double dh = std::hypot(T.n - veh_now.n, T.e - veh_now.e);
                            double vh = std::hypot(v.n, v.e);
                            const double dz = pass_d - veh_now.d;
                            if (dh > 0.5) {
                                const double slope = dz / dh;
                                double vd_cmd = vh * slope;
                                if (std::abs(vd_cmd) > args.ceil_vz && std::abs(slope) > 1e-4) {
                                    const double vh_max = args.ceil_vz / std::abs(slope);
                                    if (vh > vh_max) { v.n *= vh_max / vh; v.e *= vh_max / vh; vh = vh_max; }
                                    vd_cmd = vh * slope;
                                }
                                v.d = vd_cmd;
                            } else {
                                v.d = std::clamp(dz * 4.0, -args.ceil_vz, args.ceil_vz);
                            }
                        }
                        // Within 3m of entry: smoothly lock forward velocity onto the pass vector for a razor-straight aperture transit
                        if (past_m >= -3.0) {
                            const double w = std::clamp((past_m + 3.0) / 3.0, 0.0, 1.0);
                            v.n = (1.0 - w) * v.n + w * (un * go_speed);
                            v.e = (1.0 - w) * v.e + w * (ue * go_speed);
                        }
                    } else {
                        // Through & past: maintain constant cinematic flight speed straight through the negative space / interior geometry
                        v.n = un * go_speed;
                        v.e = ue * go_speed;
                        v.d = std::clamp((pass_d - veh_now.d) * 4.0, -args.ceil_vz, args.ceil_vz);
                    }
                } else {
                    v = fl::Ned{};      // FLY-THROUGH, past it: brake to a stop straight ahead (the SMOOTH GO limit makes it kAccel)
                }
                // Floor damping: only if an altitude floor is explicitly configured (> 0)
                if (!args.fpv_shoot && args.alt_floor > 0 && v.d > 0) {
                    const double eff_floor_d = (args.go_through ? pass_d : pursuit_floor_d) - 0.8;
                    v.d = std::min(v.d, std::max(0.0, (eff_floor_d - veh_now.d) / 0.8));
                }
                // SMOOTH GO (anti-wobble): the commanded velocity may change by at most --goto-accel (go_settings::kAccel) m/s^2 in any
                // direction. The guidance re-aims every frame from a noisy target estimate; unlimited, PN's sideways corrections
                // snapped the drone side to side during the dash. Braking already plans with the same rate, so the stop is unchanged.
                // The FIRST frame of a dash ramps from the drone's ACTUAL velocity (one frame's worth): without a previous command it
                // used to jump straight to the full GO speed (hover -> up to 30 m/s in one step: a lunge at the start of every GO).
                {
                    const fl::Ned prev = go_have_prev_v ? go_prev_v : vveh;
                    const double dts = go_have_prev_v ? std::clamp(now - go_acc_t, 0.0, 0.2) : 0.05, lim = args.goto_accel * dts;
                    const fl::Ned dv{v.n - prev.n, v.e - prev.e, v.d - prev.d};
                    const double dm = std::sqrt(dv.n * dv.n + dv.e * dv.e + dv.d * dv.d);
                    if (dm > lim && dm > 1e-9) v = fl::Ned{prev.n + dv.n * lim / dm, prev.e + dv.e * lim / dm, prev.d + dv.d * lim / dm};
                }
                // Floor hard clamp: only if an altitude floor is explicitly configured (> 0)
                if (!args.fpv_shoot && args.alt_floor > 0 && v.d > 0) {
                    const double eff_floor_d = (args.go_through ? pass_d : pursuit_floor_d) - 0.8;
                    v.d = std::min(v.d, std::sqrt(2.0 * 1.0 * std::max(0.0, eff_floor_d - veh_now.d)));
                }
                // acceleration feed-forward: how the commanded velocity is changing (PN's turn, the speed-up, the braking), smoothed and
                // capped at 8 m/s^2 (unrestricted high-G: up to 25 m/s^2) -- ArduPilot tilts for it at once instead of first building up a velocity error (--no-go-accel-ff: off)
                fl::Ned aff{};
                {
                    const double dta = now - go_acc_t; go_acc_t = now;
                    if (args.go_accel_ff && dta > 0.01 && dta < 0.3 && go_have_prev_v) {
                        fl::Ned araw{(v.n - go_prev_v.n) / dta, (v.e - go_prev_v.e) / dta, (v.d - go_prev_v.d) / dta};
                        const double a_max = (args.camera_mount == "body") ? 5.5 : ((args.zero_decel || args.fpv_shoot) ? 25.0 : std::max(20.0, args.goto_accel * 1.5));
                        const double a_h_max = (args.camera_mount == "body") ? (9.81 * std::tan(fl::deg2rad(30.0))) : a_max;
                        const double ah = std::hypot(araw.n, araw.e);
                        if (ah > a_h_max) { araw.n *= a_h_max / ah; araw.e *= a_h_max / ah; }
                        const double am = std::sqrt(araw.n * araw.n + araw.e * araw.e + araw.d * araw.d);
                        if (am > a_max) { araw.n *= a_max / am; araw.e *= a_max / am; araw.d *= a_max / am; }
                        const double k = dta / (0.15 + dta);
                        go_acc_f = fl::Ned{go_acc_f.n + k * (araw.n - go_acc_f.n), go_acc_f.e + k * (araw.e - go_acc_f.e), go_acc_f.d + k * (araw.d - go_acc_f.d)};
                        aff = go_acc_f;
                    }
                    go_prev_v = v; go_have_prev_v = true;
                }
                link->update_goto_velocity(v, yaw_to, aff, &E);
                static const bool dbg_p = std::getenv("TRK_DEBUG") != nullptr;
                if (dbg_p) std::printf("[PURSUIT] cmd %.1f m/s  vehicle %.1f m/s  to end %.1f m  alt %.1f\n", std::sqrt(v.n * v.n + v.e * v.e + v.d * v.d), v_now,
                    std::sqrt(std::pow(E.n - veh_now.n, 2) + std::pow(E.e - veh_now.e, 2) + std::pow(E.d - veh_now.d, 2)), -veh_now.d);
                const double togo = args.go_through ? std::max(0.0, -past_m)
                                                    : std::sqrt(std::pow(E.n - veh_now.n, 2) + std::pow(E.e - veh_now.e, 2) + std::pow(E.d - veh_now.d, 2));
                go_togo = togo;
                if (args.go_through && !zero_decel && go_braking && v_now < 0.5) {          // FLY-THROUGH: stopped -> arrived, hold here
                    pursuit_arrived = true; goto_hold = veh_now;
                    link->update_goto_point(goto_hold, go_brake_yaw);
                    goto_path_active = false;
                    char am[96]; std::snprintf(am, sizeof am, "GO-TO arrived: flew through the target, stopped %.0f m past it", past_m);
                    say(am, SEV_INFO, "GO-TO arrived (through)");
                    on_go_arrived(*link, veh_now, T, -past_m);
                }
                if (!args.go_through) {                                       // arrival at the stop point (fly-through: above, when stopped)
                    const double vt_go = tgt_est.valid(now, 1.0) ? std::hypot(tgt_est.vn(), tgt_est.ve()) : 0.0;
                    const double arrival_dist = (go_standoff <= 0.0) ? 0.2 : (std::max(0.2, go_standoff * 0.5) + 0.5 * vt_go);
                    if (!pursuit_arrived && togo < arrival_dist) {
                        pursuit_arrived = true;
                        char am[96];
                        if (std::abs(go_standoff) < 0.05) {
                            std::snprintf(am, sizeof am, "GO-TO arrived: ON TARGET (0.0 m) - actively tracking");
                        } else {
                            std::snprintf(am, sizeof am, "GO-TO arrived: %.1f m %s the target", std::fabs(go_standoff), go_standoff >= 0 ? "before" : "past");
                        }
                        say(am, SEV_INFO, "GO-TO arrived");
                        on_go_arrived(*link, veh_now, T, go_standoff);        // the operator's own landing / next step (empty here)
                    } else if (pursuit_arrived && togo > 1.5) pursuit_arrived = false;   // it moved off: chasing again
                    // Never cancel goto_path_active on arrival: keep the active 3D PN velocity loop
                    // running continuously so the drone reaches and holds directly on the object with 0 offset.
                }
                // nothing fresh for 3 s: stop re-aiming and hold the last point (the vehicle is not sent after a stale guess)
                // ... unless the target was STATIC: its last position is still where it is, so the dash carries on to it
                if (fresh) pursuit_static = !tgt_est.valid(now, 1.5) || std::hypot(tgt_est.vn(), tgt_est.ve()) < 0.5;
                // [GO-8] STOPPING (4/4) -- target not seen for 3 s: hold the last target point instead of braking short.
                if (goto_path_active && !(args.go_through && (go_braking || past_m >= -2.0)) && now - pursuit_seen > 3.0 && !pursuit_static) {
                    goto_hold = fl::Ned{pursuit_last.n, pursuit_last.e, std::min(pursuit_last.d, pursuit_floor_d)};
                    link->update_goto_point(goto_hold, vs.yaw);
                    goto_path_active = false;
                    say("GO-TO: target not seen for 3 s -- holding target point", SEV_WARN, "GO-TO: target lost, holding target");
                }
            } else if (goto_path_active) {
                // Closed-loop homing: as long as the lock holds, keep refining the endpoint from the LIVE tracked box -- a single
                // open-loop estimate taken at confirm time was found live to leave the drone short of the real point (the estimate
                // itself improves at closer range, where the viewing geometry is less sensitive to small errors). The pacing speed
                // and timing stay anchored to the ORIGINAL distance (goto_total) so the dash doesn't slow down or speed up as the
                // endpoint is refined. Once the lock is lost mid-dash, the last good endpoint is kept (the dash is not cancelled).
                if (dest_ok && fsm.state() == fl::FState::LOCKED) goto_final = dest;
                // advance along the line at the dash speed, tapering to a stop over the last metres (--rescue-decel; 0 = old
                // constant-speed pacing that stopped dead at the end)
                goto_along = fl::pace_along(goto_along, goto_total, rcfg.speed_ms, args.rescue_decel, now - goto_prev_t); goto_prev_t = now;
                double progress = goto_total > 0.3 ? std::min(1.0, goto_along / goto_total) : 1.0;
                fl::Ned wp;
                wp.n = goto_start.n + progress * (goto_final.n - goto_start.n);
                wp.e = goto_start.e + progress * (goto_final.e - goto_start.e);
                wp.d = goto_start.d + progress * (goto_final.d - goto_start.d);
                double yaw_to = fl::bearing_to(goto_start, goto_final);
                if (progress >= 1.0) { link->update_goto_point(goto_final, yaw_to); goto_hold = goto_final; goto_path_active = false; goto_prev_t = now; }   // the end of the line: hold there from here
                else link->update_goto_point(wp, yaw_to);
            } else if (goto_sent && args.rescue_stay_over && !abort_latched && dest_ok && fsm.state() == fl::FState::LOCKED && (goto_hold.n != 0 || goto_hold.e != 0 || goto_hold.d != 0)) {
                // --rescue-stay-over: after arrival, keep the hold point over a target that moves (a person walking off, a drifting
                // boat), moving it at no more than the dash speed. Off (default): hold the arrival point, as before. Losing the lock
                // simply freezes the hold point where it is.
                double dn = dest.n - goto_hold.n, de = dest.e - goto_hold.e, dd = dest.d - goto_hold.d, dist = std::sqrt(dn * dn + de * de + dd * dd);
                double step = std::min(dist, rcfg.speed_ms * std::max(0.0, now - goto_prev_t));
                if (dist > 0.5 && step > 0) { goto_hold.n += dn / dist * step; goto_hold.e += de / dist * step; goto_hold.d += dd / dist * step; link->update_goto_point(goto_hold, vs.yaw); }
                goto_prev_t = now;
            }
            last_rescue_state = ro.state;
        } else if (rescue_press) {
            // Everything above (arm/confirm, the go-to dash, all of it) is skipped entirely without --enable-rescue -- so a
            // 'g' press here previously did nothing at all: no state change, no message, nothing. Found live: this looked
            say("'g' has no effect: FPV dash is disabled (relaunch target_tracker with --enable-dash)", SEV_NOTICE, "FPV Dash disabled (--enable-dash)");
        }

        // ---- QGC map markers: the estimated target (+ its heading/speed) and the dash destination, 2 Hz ----
        if (args.map_marker && vs.geo_valid && now >= marker_next) {
            marker_next = now + 0.5;
            auto to_geo = [&](const fl::Ned& p, double& la, double& lo) { fl::offset_latlon(vs.lat_deg, vs.lon_deg, p.n - vs.pos_n, p.e - vs.pos_e, la, lo); };
            double la, lo;
            const bool have_tgt = tracking && (tgt_est.valid(now) || tgt_ok);
            if (have_tgt) {
                const fl::Ned p = tgt_est.valid(now) ? tgt_est.pos() : tgt;
                const double spd = tgt_est.valid(now) ? std::hypot(tgt_est.vn(), tgt_est.ve()) : 0.0;
                const double hdg = tgt_est.valid(now) ? fl::rad2deg(std::atan2(tgt_est.ve(), tgt_est.vn())) : NAN;
                to_geo(p, la, lo); last_marker_lat = la; last_marker_lon = lo;
                link->send_map_marker(1, la, lo, hdg, spd, fsm.state() == fl::FState::LOCKED ? "TARGET" : "TGT ?");
                target_marker_on = true;
            } else if (target_marker_on) {                                // relabel the last position: QGC keeps the icon 120 s
                link->send_map_marker(1, last_marker_lat, last_marker_lon, NAN, 0, tracking ? "TGT LOST" : "TGT OFF");
                if (!tracking) target_marker_on = false;
            }
            const bool show_dest = marker_dest_ok || goto_path_active || goto_sent;
            if (show_dest) {
                to_geo(marker_dest_ok ? marker_dest : (goto_path_active ? goto_final : goto_hold), la, lo);
                last_dest_lat = la; last_dest_lon = lo;
                link->send_map_marker(2, la, lo, NAN, 0, marker_dest_ok ? "DEST ?" : "DEST");
                dest_marker_on = true;
            } else if (dest_marker_on) { link->send_map_marker(2, last_dest_lat, last_dest_lon, NAN, 0, "DEST OFF"); dest_marker_on = false; }
        }

        // ---- status to QGC (the camera component) ----
        {
            uint8_t flags = 0;
            switch (fsm.state()) { case fl::FState::LOCKED: flags = 1; break; case fl::FState::COASTING: flags = 1 | 8; break; case fl::FState::LOST: flags = 2; break; default: flags = 0; }
            fl::Box sb = kf.box();
            if (flags) link->set_tracking_status(flags, sb.x / W, sb.y / H, (sb.x + sb.w) / W, (sb.y + sb.h) / H);
            else link->set_tracking_status(0, NAN, NAN, NAN, NAN);
        }

        // ---- overlay, video out, log ----
        double t3 = now_s();
#ifdef HAVE_GST_RTSP_SERVER
        const bool need_overlay = show || vout || rtsp;
#else
        const bool need_overlay = show || vout;
#endif
        if (need_overlay) {
            cv::Mat& vis = frame;                                         // draw in place on the frame buffer (already consumed by the tracker)
            if (tracking) {
                cv::Scalar col = fsm.state() == fl::FState::LOCKED ? cv::Scalar(0, 220, 0) : (fsm.state() == fl::FState::COASTING ? cv::Scalar(0, 200, 255) : cv::Scalar(0, 0, 255));
                cv::rectangle(vis, cv::Rect((int)box.x, (int)box.y, (int)box.w, (int)box.h), col, 2);
                if (fsm.state() != fl::FState::LOCKED)                      // not a live measurement: say so on the box
                    cv::putText(vis, fl::to_str(fsm.state()), {(int)box.x, std::max(12, (int)box.y - 4)}, cv::FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv::LINE_AA);
                // PREDICTED box (thin yellow, 'k' toggles): where the Kalman filter says the target will be --predict-ahead s from now,
                // at the tracked box's own size (the filter's own size lagged on large objects and read as a second, wrong lock --
                // operator report), with an arrow along the motion. TRK_DEBUG also draws the filter's current box.
                if (show_pred && nominal_gap > 0 && fsm.state() == fl::FState::LOCKED && tracker_ok) {   // only on a real measurement: no wandering guess
                    const double steps = args.predict_ahead / nominal_gap;
                    const double fx = box.cx() + kf.vx() * steps, fy = box.cy() + kf.vy() * steps;
                    if (std::hypot(fx - box.cx(), fy - box.cy()) > 4) {
                        const cv::Rect pr((int)std::lround(fx - box.w / 2), (int)std::lround(fy - box.h / 2), (int)box.w, (int)box.h);
                        cv::rectangle(vis, pr, {0, 255, 255}, 1);
                        cv::arrowedLine(vis, cv::Point((int)box.cx(), (int)box.cy()), cv::Point((int)fx, (int)fy), {0, 255, 255}, 1, cv::LINE_AA, 0, 0.15);
                        cv::putText(vis, "predicted", {pr.x, pr.y + pr.height + 12}, cv::FONT_HERSHEY_SIMPLEX, 0.4, {0, 255, 255}, 1, cv::LINE_AA);
                    }
                }
                static const bool dbg_kf = std::getenv("TRK_DEBUG") != nullptr;
                if (dbg_kf) { fl::Box sb = kf.box(); cv::rectangle(vis, cv::Rect((int)sb.x, (int)sb.y, (int)sb.w, (int)sb.h), {0, 160, 255}, 1); }
            }
            char txt[220];
            // t= is this frame's capture time (monotonic, seconds) -- compare it against the wall clock at the moment of a
            // QGC tap to MEASURE the real glass-to-glass latency for --tap-latency-ms, rather than guessing it.
            std::snprintf(txt, sizeof txt, "%s  link:%s mode:%s%s  alt:%.1fm  %.0ffps  t=%.3f%s%s", fl::to_str(fsm.state()), vs.link_ok ? "ok" : "DOWN", vs.guided ? "GUIDED" : "other", vs.armed ? "" : "(disarmed)", vs.alt_m, fps, now,
                         rcfg.enabled ? "  dash:" : "", rcfg.enabled ? fl::to_str(rescue_state) : "");
            cv::putText(vis, txt, {8, 18}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 255, 255}, 1, cv::LINE_AA);
            if (gimbal_mode) std::snprintf(txt, sizeof txt, "cmd vx %+.2f vz %+.2f yaw %+.2f | gimbal yaw %+.1f pitch %+.1f %s", desired.vx, desired.vz, desired.yaw_rate, gimbal_yaw, gimbal_pitch, gimbal_ok && gimbal_age < 0.5 ? "" : "(NO DATA)");
            else std::snprintf(txt, sizeof txt, "cmd vx %+.2f vy %+.2f vz %+.2f yaw %+.2f", desired.vx, desired.vy, desired.vz, desired.yaw_rate);
            cv::putText(vis, txt, {8, 38}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {200, 255, 200}, 1, cv::LINE_AA);
            // A clear "what to do next" prompt instead of making the officer infer it from small state text --
            // found live this session that the small console/HUD text was easy to miss while watching QGC instead.
            const char* hint = ""; std::string hint_s; bool hint_ready = false;
            char gbuf[128] = "";
            if (now < abort_flash_until) {
                hint = "ABORTED - CLIMBING AWAY FROM THE TARGET";
            } else if (goto_path_active) {
                if (pursuit_arrived) {
                    std::snprintf(gbuf, sizeof gbuf, "GO ARRIVED: DIRECTLY ON TARGET (%.1f m) - TRACKING [X TO DISENGAGE]", go_togo);
                } else {
                    std::snprintf(gbuf, sizeof gbuf, "GO DASHING: %.0f m/s  |  %.1f m TO TARGET [3D PN]  |  PRESS X TO ABORT", go_speed, go_togo);
                }
                hint_s = gbuf; hint = hint_s.c_str(); hint_ready = true;
            } else if (goto_sent && !goto_path_active) {
                hint = "GO COMPLETE: TRACKING DIRECTLY ON TARGET - PRESS X TO DISENGAGE";
                hint_ready = true;
            } else if (lock_job_running) {
                hint = "LOCKING...";
            } else if (!tracking) {
                hint = "CLICK A TARGET TO LOCK";
            } else if (fsm.state() == fl::FState::LOST) {
                hint = "TARGET LOST - CLICK TO RELOCK";
            } else if (lt.occluded()) {
                hint = "TARGET HIDDEN - SEARCHING...";
            } else if (fsm.state() == fl::FState::COASTING) {
                hint = "TARGET COASTING...";
            } else if (rcfg.enabled && rescue_state == fl::RState::ARMED) {
                hint = "ARMED - CLICK GO AGAIN TO CONFIRM";
            } else if (rcfg.enabled && fsm.state() == fl::FState::LOCKED) {   // the GO READY indicator
                hint_s = go_ready ? (go_predicting ? "GO READY - PRESS G / CLICK GO (3D PN PREDICTION)" : "GO READY - PRESS G / CLICK GO")
                                  : "GO NOT READY: " + go_ready_why;
                hint = hint_s.c_str(); hint_ready = go_ready;
            }
            {   // live telemetry: what the drone is doing and the speeds in force
                const double gs = std::hypot(vs.vel[0], vs.vel[1]), vz = -vs.vel[2];
                char gos[64] = "";
                if (goto_path_active) std::snprintf(gos, sizeof gos, "  [GO %.1f m/s, %.1f m to go%s]", go_speed, go_togo, pursuit_arrived ? ", ARRIVED" : "");
                else if (goto_sent) std::snprintf(gos, sizeof gos, "  [TRACKING ON TARGET]");
                std::snprintf(txt, sizeof txt, "SPEED %.1f m/s  climb %+.1f m/s  ALT %.1f m  %s  | GO %.0f m/s stop %+.1f m  ABORT climb %.0f m%s",
                              gs, vz, vs.alt_m, vs.guided ? "GUIDED" : "not GUIDED", go_speed_req > 0 ? go_speed_req : rcfg.speed_ms,
                              std::isfinite(go_standoff_req) ? go_standoff_req : args.goto_standoff, args.abort_climb, gos);
                cv::putText(vis, txt, {8, H - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {255, 255, 255}, 2, cv::LINE_AA);
                cv::putText(vis, txt, {8, H - 12}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {40, 40, 40}, 1, cv::LINE_AA);
            }
            if (hint[0]) cv::putText(vis, hint, {8, 62}, cv::FONT_HERSHEY_SIMPLEX, 0.6, now < abort_flash_until ? cv::Scalar(0, 0, 255)
                                     : hint_ready ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 200, 255), 2, cv::LINE_AA);   // green = GO ready/active, amber = not ready
            if (args.auto_view && tracking) {
                std::snprintf(txt, sizeof txt, "ZOOM %.1fx  view: %s", zoom, view_why.c_str());
                cv::putText(vis, txt, {8, 84}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {255, 200, 0}, 1, cv::LINE_AA);
            }
            if (now < banner_until) cv::putText(vis, banner, {8, H - 40}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0, 255, 255}, 1, cv::LINE_AA);
            if (video_stalled) cv::putText(vis, "VIDEO WAS STALLED", {W / 2 - 70, 60}, cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 0, 255}, 2, cv::LINE_AA);
            if (show && !tracking) cv::drawMarker(vis, cv::Point(W / 2, H / 2), {255, 255, 255}, cv::MARKER_CROSS, 18, 1);   // what LOCK (centre) takes
            if (tracking && box.valid()) {                               // centring: the image centre and how far the target is off it
                const cv::Point c0(W / 2, H / 2), tc((int)std::lround(box.cx()), (int)std::lround(box.cy()));
                const double off = std::hypot(tc.x - c0.x, tc.y - c0.y);
                const cv::Scalar col = off < 0.04 * W ? cv::Scalar(0, 220, 0) : off < 0.12 * W ? cv::Scalar(0, 220, 255) : cv::Scalar(0, 0, 255);
                cv::drawMarker(vis, c0, col, cv::MARKER_CROSS, 22, 1);
                if (off > 6) cv::line(vis, c0, tc, col, 1, cv::LINE_AA);
                std::snprintf(txt, sizeof txt, "CENTRE off %+d,%+d px", tc.x - c0.x, tc.y - c0.y);
                cv::putText(vis, txt, {W - 220, 84}, cv::FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv::LINE_AA);
            }
            if (show) {                                                  // buttons: interactive only in this window, not the QGC feed
                for (auto& [b, r] : button_rects(W, H)) {
                    cv::Scalar fill = {55, 55, 55}, txtcol = {150, 150, 150};
                    std::string lbl = button_label(b);
                    if (b == Button::Lock && !tracking) { fill = {120, 60, 0}; txtcol = {255, 255, 255}; }
                    else if (b == Button::Go) {
                        if (goto_path_active) { fill = {200, 140, 0}; txtcol = {255, 255, 255}; lbl = "DASHING"; }
                        else if (rescue_state == fl::RState::ARMED) { fill = {0, 180, 255}; txtcol = {255, 255, 255}; lbl = "CONFIRM"; }
                        else if (rcfg.enabled && fsm.state() == fl::FState::LOCKED && go_ready) { fill = {0, 180, 0}; txtcol = {255, 255, 255}; }
                        else if (rcfg.enabled && fsm.state() == fl::FState::LOCKED) { fill = {0, 100, 160}; txtcol = {255, 255, 255}; lbl = "GO (WAIT)"; }
                    }
                    else if (b == Button::Unlock && tracking) { fill = {90, 70, 0}; txtcol = {255, 255, 255}; }
                    else if (b == Button::Abort && (rescue_state == fl::RState::ARMED || goto_path_active || goto_sent || tracking)) { fill = {0, 0, 160}; txtcol = {255, 255, 255}; }
                    cv::rectangle(vis, r, fill, cv::FILLED);
                    cv::rectangle(vis, r, {230, 230, 230}, 1);
                    int baseline = 0; cv::Size tsz = cv::getTextSize(lbl, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
                    cv::putText(vis, lbl, {r.x + (r.width - tsz.width) / 2, r.y + (r.height + tsz.height) / 2},
                                cv::FONT_HERSHEY_SIMPLEX, 0.5, txtcol, 1, cv::LINE_AA);
                }
            }
            if (vout) vout->push(vis);
#ifdef HAVE_GST_RTSP_SERVER
            if (rtsp) rtsp->push(vis);
#endif
            if (show) { cv::imshow(win, vis); seen_t = now; }                // what the operator is looking at now (late-click fix)
        }
        st.draw = (now_s() - t3) * 1000;
        double dtf = now - t_prev; t_prev = now; if (dtf > 0.001) fps = (fps == 0 ? 1.0 / dtf : 0.9 * fps + 0.1 * (1.0 / dtf));
        if (jlog.on()) {
            std::snprintf(logbuf, sizeof logbuf,
                "{\"t\":%.3f,\"n\":%d,\"state\":\"%s\",\"box\":[%.1f,%.1f,%.1f,%.1f],\"ok\":%d,\"age\":%.3f,\"cmd\":[%.3f,%.3f,%.3f,%.3f],\"link\":%d,\"guided\":%d,\"armed\":%d,\"alt\":%.2f,\"gimbal\":[%.2f,%.2f],\"stalled\":%d,\"rescue\":\"%s\",\"pos\":[%.2f,%.2f,%.2f],"
                "\"ms\":{\"cap\":%.1f,\"trk\":%.1f,\"ctl\":%.1f,\"draw\":%.1f,\"loop\":%.1f},\"fps\":%.1f,\"zoom\":%.2f,\"view_ref\":%.1f,\"tgt\":[%d,%.2f,%.2f,%.1f,%.1f,%.1f],\"tvel\":[%d,%.2f,%.2f],\"att\":[%.2f,%.2f,%.3f],\"chase\":%d,\"gim\":[%.1f,%.1f],\"cm\":%d,\"kfv\":[%.1f,%.1f]}",
                now, frame_no, fl::to_str(fsm.state()), box.x, box.y, box.w, box.h, tracker_ok ? 1 : 0, tracking ? now - last_accept : 0.0,
                desired.vx, desired.vy, desired.vz, desired.yaw_rate, vs.link_ok ? 1 : 0, vs.guided ? 1 : 0, vs.armed ? 1 : 0, vs.alt_m, gimbal_yaw, gimbal_pitch, video_stalled ? 1 : 0,
                fl::to_str(rescue_state), vs.pos_n, vs.pos_e, vs.pos_d,
                st.capture, st.track, st.control, st.draw, (now_s() - t_loop) * 1000, fps, zoom, view_ref_h,
                tgt_ok ? 1 : 0, tgt.n, tgt.e, dist_f, view_goal_dist, tgt_sigma < 1e8 ? tgt_sigma : -1.0, tgt_vel_ok ? 1 : 0, tgt_est.vn(), tgt_est.ve(), fl::rad2deg(vs.pitch), fl::rad2deg(vs.roll), vs.yaw, chase_since > 0 ? 1 : 0,
                gimbal_cmd_pitch, vs.mount_pitch_deg, cam_src, kf.vx(), kf.vy());   // gim: [commanded, reported] pitch; cm: camera motion 0 none / 1 image / 2 fallback
            jlog.line(logbuf);
        }
    }

    // ---- exit: end the session (zeros), then the link destructor sends 3 more zero commands ----
    link->command(fl::Cmd(), false);
    link->end_session();
    std::this_thread::sleep_for(std::chrono::milliseconds(120));
    link->shutdown();
    cap->release();
    if (show) cv::destroyAllWindows();
    std::printf("[FOLLOW] stopped (zero velocity sent)\n");
    return 0;
}

/*
Build: cmake -S . -B build && cmake --build build   (needs OpenCV with the contrib tracking module, GStreamer, MAVLink headers: scripts/fetch_mavlink.sh)
Self-test (no vehicle, camera or display): ./build/target_tracker --selftest
Bring-up check of the MAVLink path: ./build/target_tracker --link-test 10 --connection <conn>
*/
