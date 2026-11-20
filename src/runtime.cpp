/**
 * @file
 * Default experiment parameters, status labels, and deterministic sensor synthesis.
 * The simulator exercises the same image segmentation and range association as
 * hardware input. It generates observations, not precomputed tracking results.
 */

#include "runtime.h"
#include <array> // Fixed-size standard arrays used where element count is part of the data contract.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
namespace tabletop {
Config default_config() { // Initialize every scalar/array with the reference rig geometry. JSON startup loading supplies required experiment fields and validates them together.
    Config c{}; // Zero-initialize the complete fixed-layout configuration before supplying every default field.
    c.forward = 269.065; // LiDAR sensor azimuth corresponding to tabletop forward, in degrees.
    const double anchors[]{155.326484, -12.678333, 333.378556, 0, 517.592068, 13.3075}; // Three measured [camera-u pixels,bearing degrees] pairs define piecewise linear calibration.
    std::copy(std::begin(anchors), std::end(anchors), c.anchors); // Copy all six calibration entries into the engine's fixed-size anchor array.
    c.xmin = -.25; // Left edge of the metric tracking region, in meters.
    c.xmax = .25; // Right edge of the metric tracking region, in meters.
    c.ymin = .4; // Near edge of the metric tracking region, in meters forward of the LiDAR.
    c.ymax = .8; // Far edge of the metric tracking region, in meters forward of the LiDAR.
    c.cluster_gap = .045; // Maximum spatial gap between neighboring returns in one cluster, in meters.
    c.angle_gap = 2.5; // Maximum angular gap between neighboring cluster returns, in degrees.
    c.max_width = .10; // Maximum accepted cluster bounding-box diagonal, in meters.
    c.bearing_gate = 3.5; // Allowed camera-to-cluster bearing difference for association, in degrees.
    c.min_aspect = .45; // Minimum marker bounding-box width divided by height, dimensionless.
    c.max_aspect = 2.2; // Maximum marker bounding-box width divided by height, dimensionless.
    c.extrapolation = 65; // Allowed horizontal extrapolation beyond camera calibration anchors, in pixels.
    c.fresh = .5; // Maximum camera/scan receive age allowed for tracking, in seconds.
    c.pair_gap = .25; // Maximum absolute receive-time difference between paired camera and LiDAR, in seconds.
    c.reacquire = .7; // Gap after which a position must be confirmed again instead of continuing the old filter, in seconds.
    c.max_speed = .4; // Maximum plausible figure motion used by the jump gate, in meters per second.
    c.jump = .035; // Fixed positional tolerance added to the speed-times-age jump envelope, in meters.
    c.smoothing = .12; // Exponential position filter time constant, in seconds; smaller values respond faster.
    c.trail_s = 5; // Maximum age of displayed position history, in seconds.
    const int low[]{75, 100, 120}, high[]{115, 255, 255}, roi[]{0, 170, 640, 450}; // Define blue-marker HSV bounds and the calibrated camera ROI's exclusive upper edges.
    std::copy(std::begin(low), std::end(low), c.hsv_low); // Populate all three lower HSV limits.
    std::copy(std::begin(high), std::end(high), c.hsv_high); // Populate all three upper HSV limits.
    std::copy(std::begin(roi), std::end(roi), c.roi); // Populate the four camera ROI bounds in full-image pixel coordinates.
    c.min_area = 80; // Minimum accepted connected marker area, in foreground pixels.
    c.max_area = 4000; // Maximum accepted connected marker area, in foreground pixels.
    c.min_points = 3; // Minimum number of valid LiDAR returns required for a cluster.
    c.confirmation = 2; // Consecutive consistent scans required before a new track authorizes motion.
    c.min_quality = 1; // Minimum decoded LiDAR return quality; the core also rejects zero quality.
    c.lane_width = .48; // Full width of the wide virtual lane, in meters before footprint insetting.
    c.narrow_width = .16; // Full width of the narrow virtual lane, in meters before footprint insetting.
    c.lane_ymin = .4; // Near boundary of the virtual lane, in meters forward of the LiDAR.
    c.lane_ymax = .8; // Far boundary of the virtual lane, in meters forward of the LiDAR.
    c.robot_radius = .022; // Circular virtual robot footprint radius used for collision checking, in meters.
    c.figure_radius = .045; // Assumed figure footprint radius used by this tabletop model, in meters.
    c.margin = .005; // Additional nonnegative clearance beyond both body radii, in meters.
    c.boundary_radius = 0; // Start at the nominal 7.2 cm sum until a live or saved total-radius selection overrides it.
    c.speed = .028; // Nominal virtual travel speed in meters per second; Config's 0.8 speed scale gives 0.0224 m/s of actual motion.
    c.yield_s = 1; // Wait before attempting a detour around a persistent obstruction, in seconds.
    c.grid_m = .01; // A* search-grid spacing, in meters; continuous edge checks still guard movement.
    c.start = {0, .435}; // Exact depot coordinate [right,forward], in meters.
    c.goal = {0, .765}; // Exact delivery coordinate [right,forward], in meters.
    return c; // Return a fully initialized default experiment; file loading still validates its effective settings.
}
const char *track_name(int i) { // Translate bounded enum values to readable UI/log states; unknown values receive a neutral label rather than indexing beyond the lookup table.
    static const char *names[]{// Keep labels in the exact order of the corresponding status enumeration.
                               "WAITING", "TRACKING", "ACQUIRING", "NO BLUE MARKER",
                               "MULTIPLE MARKERS", "MARKER ONLY", "AMBIGUOUS LIDAR", "OUTSIDE CALIBRATION",
                               "POSITION JUMP", "STALE CAMERA", "STALE LIDAR", "TIMING GAP"};
    return i >= 0 && i < 12 ? names[i] : "UNKNOWN"; // Check the enum range before indexing; unknown status codes receive a safe fallback label.
}
const char *robot_name(int i) {
    static const char *names[]{"READY", "DRIVING", "YIELDING", "REROUTING",
                               "BLOCKED", "DELIVERED", "PAUSED", "TRACKING LOST",
                               "TPU OFFLINE", "STOP SIGN", "TOO CLOSE"}; // Keep labels in the exact order of the corresponding status enumeration.
    return i >= 0 && i < 11 ? names[i] : "UNKNOWN"; // Check the enum range before indexing; unknown status codes receive a safe fallback label.
}
Frame synthetic_frame(const Config &c, XY figure, double now, std::uint64_t sequence, bool marker) { // Invert the same local bearing calibration to draw a cyan marker at the expected image coordinate. Other rectangles provide visual context only.
    auto p =
        std::make_shared<std::vector<unsigned char>>(640 * 480 * 3, static_cast<unsigned char>(42)); // Allocate owned packed 640x480 BGR pixels initialized to a dark neutral background.
    const double a = std::atan2(figure.x, figure.y) * 180 / 3.141592653589793; // Convert the synthetic figure's metric position into bearing degrees measured from forward.
    const int k = a < c.anchors[3] ? 0 : 2; // Select the left-center or center-right calibration interval for inverse interpolation.
    const double u = c.anchors[k] + (a - c.anchors[k + 1]) / (c.anchors[k + 3] - c.anchors[k + 1]) *
                                        (c.anchors[k + 2] - c.anchors[k]); // Invert the local bearing calibration to find the image column for the synthetic marker.
    auto rect = [&](int x0, int y0, int x1, int y1, std::array<unsigned char, 3> color) { // Draw a clipped solid BGR rectangle directly into the owned pixel buffer.
        for (int y = std::max(0, y0); y < std::min(480, y1); ++y) // Clip vertical drawing bounds to valid image rows before writing pixels.
            for (int x = std::max(0, x0); x < std::min(640, x1); ++x) // Clip horizontal drawing bounds to valid image columns.
                std::copy(color.begin(), color.end(), p->begin() + (y * 640 + x) * 3); // Store three BGR bytes at the row-major pixel offset.
    };
    const int x = int(std::lround(u)); // Round the projected marker column to the nearest integer image coordinate.
    rect(0, 370, 640, 480, {65, 60, 53}); // Draw the synthetic tabletop/background region for visual context.
    rect(x - 20, 260, x + 21, 365, {95, 95, 95}); // Draw the neutral figure body independently of the identifiable blue marker.
    rect(x - 15, 233, x + 16, 263, {145, 163, 180}); // Draw a small head region for a readable synthetic figure silhouette.
    if (marker) // Omit only the identifier during intentional marker-loss scenarios.
        rect(x - 13, 313, x + 14, 337, {240, 220, 40}); // Draw the cyan-blue marker whose HSV values pass the normal segmentation gates.
    return {p, now, sequence, 0}; // Publish the synthetic image with host time and application sequence; no sensor timestamp exists.
}
Scan synthetic_scan(const Config &c, XY figure, double now, int sequence) { // Generate four quality returns spanning a compact surface at the specified metric position. The core must still cluster and associate them normally.
    Scan s; // Create an owned synthetic scan with the same contract as hardware acquisition.
    s.received = now; // Stamp the synthetic scan on the caller-controlled host clock.
    s.sequence = sequence; // Preserve the supplied scan identity for duplicate-scan suppression in the core.
    const double a = std::atan2(figure.x, figure.y) * 180 / 3.141592653589793; // Convert the synthetic figure's metric position into bearing degrees measured from forward.
    for (double d : {-1.05, -.35, .35, 1.05}) // Generate four nearby azimuth offsets forming one compact detectable cluster.
        s.points.push_back({c.forward + a + d, std::hypot(figure.x, figure.y), 47}); // Transform back to raw LiDAR azimuth and use radial distance plus a valid quality value.
    return s; // Return generated raw observations; association and filtering still run through the normal core.
}
Inputs DemoSource::update(double now, double elapsed) { // Sample a scripted crossing, hold, return, and brief marker occlusion. Advance sequences only when a 15 Hz camera or 10 Hz LiDAR tick changes.
    const double phase = std::fmod(elapsed, 30.); // Repeat the scripted tabletop scene every 30 elapsed seconds.
    double x = .15; // Begin with the figure 15 cm right of the lane centerline.
    if (phase >= 2 && phase < 4) // Move the figure toward the center over the two-second interval from 2 to 4 seconds.
        x = .15 * (4 - phase) / 2; // Linear interpolation decreases rightward position to zero as the figure crosses into the path.
    else if (phase >= 4 && phase < 18) // Hold the figure on the centerline long enough for yield and detour behavior.
        x = 0; // Centerline position blocks the direct depot-to-delivery route.
    else if (phase >= 18 && phase < 20) // Return the figure to the right over the interval from 18 to 20 seconds.
        x = .15 * (phase - 18) / 2; // Linear interpolation moves the obstruction away from the centerline.
    int ct = int(elapsed * 15), st = int(elapsed * 10); // Independent 15 Hz camera and 10 Hz scan ticks exercise receive-time pairing.
    if (ct != camera_tick_) { // Emit a camera sample only when its simulated acquisition tick changes.
        camera_tick_ = ct; // Remember the emitted camera tick to avoid producing duplicates at the main-loop rate.
        inputs_.frames.push_back(
            synthetic_frame(config_, {x, .63}, now, ct, !(phase > 7 && phase < 8))); // Append a frame at y=0.63 m; the scripted 7-8 second interval hides its blue marker.
        if (inputs_.frames.size() > 24) // Keep simulator history at the same 24-frame capacity as physical acquisition.
            inputs_.frames.erase(inputs_.frames.begin()); // Remove the oldest frame when the bounded history is full.
    }
    if (st != scan_tick_) { // Emit a scan only when the independent 10 Hz sensor tick advances.
        scan_tick_ = st; // Remember the newly published scan tick.
        inputs_.scan = synthetic_scan(config_, {x, .63}, now, st); // Replace the latest synthetic LiDAR scan at the figure's current metric position.
    }
    return inputs_; // Return a coherent snapshot with shared immutable image storage and owned scan data.
}
} // namespace tabletop
