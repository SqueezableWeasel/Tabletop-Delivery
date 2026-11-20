/**
 * @file
 * Portable observation types, clocks, freshness rules, and simulator interface.
 * Receive times are steady-clock seconds. Camera sensor timestamps are metadata
 * in nanoseconds; they are never compared directly with host receive times.
 */

#pragma once // Include this header only once per translation unit.
#include "../native/core.h"
#include <algorithm> // Sorting, bounded copies, searches, and min/max/clamp operations on observations and geometry.
#include <chrono> // Monotonic timing and duration conversion for freshness, integration, and pacing.
#include <cmath> // Finite-value checks, trigonometry, interpolation, and metric geometry calculations.
#include <cstdint> // Explicit-width integer types for binary contracts, timestamps, and sequence values.
#include <memory> // RAII smart pointers that define ownership and deterministic cleanup of native resources.
#include <string> // Owned text values for paths, diagnostic messages, and configuration names.
#include <vector> // Owned variable-length storage for pixels, measurements, routes, or detections.

namespace tabletop {
inline double clock_now() { // Use a monotonic clock for age checks and integration; wall-clock corrections must not change sensor freshness or virtual movement.
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()) // Convert the monotonic clock's native duration to floating-point seconds.
        .count();
}
struct Frame { // Immutable packed 640x480 BGR pixels, application sequence, host receive time, and optional sensor timestamp. A default frame represents missing input.
    std::shared_ptr<const std::vector<unsigned char>> pixels; // Shared immutable pixels keep asynchronous consumers independent of camera buffer reuse.
    double received = -1; // Negative sentinel means no host receive timestamp has been recorded.
    std::uint64_t sequence = 0, sensor_ns = 0; // Application frame sequence identifies unique frames; sensor_ns retains device metadata only.
    explicit operator bool() const { // Frame validity requires owned pixels of the exact packed 640x480 BGR size.
        return pixels && pixels->size() == 640 * 480 * 3; // Short-circuit before dereferencing missing storage; three bytes are required per pixel.
    }
};
struct Scan { // One completed scan in sensor-relative degrees/meters with a host receive time and sequence. A negative sequence denotes no scan received.
    std::vector<RawPoint> points; // Owned scan measurements in raw sensor degrees and metric range.
    double received = -1; // Negative sentinel means no host receive timestamp has been recorded.
    int sequence = -1; // Negative sequence distinguishes missing data from a valid first scan or inference.
};
struct Inputs { // Bounded camera history plus the most recent complete scan and latched error. Producers return a coherent copy to the main application thread.
    std::vector<Frame> frames; // Recent immutable camera frames provide candidates for nearest receive-time pairing.
    Scan scan; // Most recent complete scan; earlier scans are not queued for delayed processing.
    std::string error; // First latched acquisition error for the main thread to report and terminate on.
};
inline Frame paired_frame(const Inputs &inputs) { // Choose nearest host receive time; return newest frame before a scan exists so the display remains useful. The core still rejects absent/stale LiDAR.
    if (inputs.frames.empty()) // With no camera history there is no frame to pair or display.
        return {};
    if (inputs.scan.sequence < 0) // Before the first scan, allow the newest camera frame to remain visible diagnostically.
        return inputs.frames.back(); // Use the latest image for display while the core still holds motion without LiDAR.
    return *std::min_element(inputs.frames.begin(), inputs.frames.end(), // Select the frame whose host receive time is closest to the current scan.
                             [&](const Frame &a, const Frame &b) {
                                 return std::abs(a.received - inputs.scan.received) <
                                        std::abs(b.received - inputs.scan.received); // Compare absolute receive-time differences, not sensor-clock timestamps.
                             });
}
struct Detection { // One COCO detection: zero-based class ID, confidence, and normalized XYXY box.
    int id; // Zero-based COCO class identifier returned by the supplied SSD model.
    float score, x0, y0, x1, y1; // Confidence and normalized image bounds ordered left, top, right, bottom.
};
struct CoralState { // Ready describes the worker/interpreter; healthy additionally requires recent source and result timestamps. A sequence identifies one completed inference.
    bool ready = false, healthy = false, stop_sign = false; // Interpreter availability, freshness-qualified health, and current stop-sign observation.
    int sequence = -1; // Negative sequence distinguishes missing data from a valid first scan or inference.
    double frame_time = -1, received = -1, inference_ms = 0; // Source-frame receive time (s), inference completion time (s), and Invoke duration (ms).
    std::string detail; // Human-readable readiness or worker-failure detail for diagnostics.
    std::vector<Detection> detections; // Accepted detections from one completed inference result.
};
inline bool coral_fresh(const CoralState &state, double now) { // Check both result age and source-image age so fast delivery of an already stale inference cannot satisfy the motion gate.
    return state.ready && state.sequence >= 0 && state.received <= now && state.frame_time <= now &&
           now - state.received < .6 && now - state.frame_time < .8; // Require a real result, nonfuture timestamps, result age <0.6 s, and image age <0.8 s.
}
Config default_config(); // Supply the calibrated tabletop defaults used by configuration and synthetic tests.
const char *track_name(int state); // Map a perception status code to stable display/log text, with UNKNOWN fallback.
const char *robot_name(int state); // Map a delivery status code to stable display/log text, with UNKNOWN fallback.
Frame synthetic_frame(const Config &, XY figure, double now, std::uint64_t sequence,
                      bool marker = true); // Draw a calibrated synthetic marker frame; marker=false intentionally simulates visual loss.
Scan synthetic_scan(const Config &, XY figure, double now, int sequence); // Generate a small figure-like LiDAR cluster at the supplied metric position.
class DemoSource { // Generate a repeating 30-second scene at independent camera/scan rates, including a crossing and marker dropout. No physical device is opened.
    Config config_; // Simulator-owned copy of geometry and camera-to-bearing calibration.
    Inputs inputs_; // Simulator's bounded frame history and latest synthetic scan.
    int camera_tick_ = -1, scan_tick_ = -1; // Last emitted camera/scan tick; negative values force initial samples.

  public:
    explicit DemoSource(Config config) : config_(config) {} // Copy the experiment settings without opening any physical sensor or TPU.
    Inputs update(double now, double elapsed); // Advance independent synthetic sensor clocks and return a coherent observation snapshot.
};
} // namespace tabletop
