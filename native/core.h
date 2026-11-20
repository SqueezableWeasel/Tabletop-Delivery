/**
 * @file
 * Public data contract for the portable tracking/planning engine.
 * All XY coordinates and radii are meters; bearings are degrees; times are
 * steady-clock seconds. Call one engine instance from a single thread.
 */

#pragma once // Include this header only once per translation unit.
#include <cstdint> // Explicit-width integer types for binary contracts, timestamps, and sequence values.
#ifdef _WIN32
#define API extern "C" __declspec(dllexport)
#else
#define API extern "C" __attribute__((visibility("default")))
#endif

struct XY { // Fixed-layout native engine interface, linked directly into the executable. Metric point in the sensor plane: x right, y forward.
    double x, y; // Rightward x and forward y coordinates, measured in meters.
};
struct RawPoint { // Sensor-relative azimuth in degrees, distance in meters, and decoded quality.
    double angle, range; // Sensor azimuth in degrees and radial distance in meters, before forward calibration.
    int quality; // SDK-decoded return quality; zero denotes an unusable measurement.
};
struct Marker { // Camera centroid (u,v), integer pixel bounds, and component area in pixels.
    double u, v; // Component centroid in camera pixels: u across the image, v down the image.
    int x, y, w, h, area; // Pixel bounding-box origin, width, height, and foreground pixel count.
};
struct Group { // Accepted LiDAR cluster summary; bearing is relative to calibrated forward.
    XY xy; // Cartesian position in the calibrated right/forward sensor frame, in meters.
    double bearing, range, width; // Mean bearing in degrees, median range in meters, and bounding-box diagonal in meters.
    int count; // Number of accepted LiDAR returns contributing to this cluster.
};
struct TrailPoint { // One timestamped filtered position; segment separates discontinuous tracks.
    double stamp; // Host receive time of the associated scan, in monotonic seconds.
    XY xy; // Cartesian position in the calibrated right/forward sensor frame, in meters.
    int segment; // Discontinuity identifier prevents trail lines from crossing tracking gaps.
};
struct Config { // Experiment parameters. Arrays: anchors = three (pixel u, bearing degree) pairs; roi = [xmin,ymin,xmax,ymax] pixels; HSV uses OpenCV H=0..179 and S/V=0..255.
    double forward, anchors[6], xmin, xmax, ymin, ymax; // LiDAR forward offset (degrees), three pixel/bearing pairs, and metric ROI limits.
    double cluster_gap, angle_gap, max_width, bearing_gate; // Neighbor distance (m), neighbor angle (degrees), cluster size (m), and association gate (degrees).
    double min_aspect, max_aspect, extrapolation; // Marker width/height limits and allowed horizontal calibration extrapolation in pixels.
    double fresh, pair_gap, reacquire, max_speed, jump, smoothing, trail_s; // Freshness/pair/reacquisition times (s), max motion (m/s), jump allowance (m), filter/trail times (s).
    int hsv_low[3], hsv_high[3], roi[4], min_area, max_area, min_points, confirmation, min_quality; // HSV limits, exclusive-upper-bound pixel ROI, component areas, cluster/confirmation counts, quality floor.
    double lane_width, narrow_width, lane_ymin, lane_ymax; // Wide/narrow lane widths and near/far lane bounds, all in meters.
    double robot_radius, figure_radius, margin, speed, yield_s, grid_m; // Body radii and margin (m), nominal robot speed (m/s), yield duration (s), and grid spacing (m).
    XY start, goal; // Depot and delivery coordinates in the same metric frame as the tracked figure.
    double boundary_radius = 0; // Explicit robot-center exclusion radius in meters; zero uses the nominal figure + robot + margin sum.
    double speed_scale = .8; // Fraction of nominal robot speed, in (0,1]; applies to saved calibrations that omit this optional setting.
};

inline constexpr int BOUNDARY_MIN_MM = 30, BOUNDARY_MAX_MM = 100; // Shared inclusive 3–10 cm limits for UI requests, configuration, and collision geometry.
inline double exclusion_radius(const Config &c) { // Resolve the selected total radius without subtracting footprints or allowing a negative clearance margin.
    return c.boundary_radius == 0 ? c.figure_radius + c.robot_radius + c.margin : c.boundary_radius; // A live or saved selection overrides the nominal starting geometry.
}
inline bool valid_boundary_radius(double radius) { // Ordered comparisons reject NaN and infinity as well as values outside the supported range.
    return radius >= BOUNDARY_MIN_MM / 1000. && radius <= BOUNDARY_MAX_MM / 1000.; // Use meters internally and include both exact endpoints.
}
enum TrackStatus { // Perception result for the current observation. Only TRACKING permits motion.
    WAITING, // No usable perception observation has established a tracking state yet.
    TRACKING, // Unique, fresh camera/LiDAR association has passed confirmation and motion checks.
    ACQUIRING, // A plausible position is waiting for enough consistent scans to initialize the filter.
    NO_MARKER, // No accepted visual target: a blue component or an eligible detector observation.
    MULTIPLE_MARKERS, // More than one visual candidate prevents unique target identification.
    MARKER_ONLY, // A visual target is present but no LiDAR cluster matches its expected bearing.
    AMBIGUOUS_LIDAR, // Multiple clusters match the marker bearing; no arbitrary cluster is selected.
    OUTSIDE_CALIBRATION, // Marker position exceeds the calibrated interval and allowed extrapolation.
    POSITION_JUMP, // Observation timing or displacement violates the configured tracking motion allowance.
    STALE_CAMERA, // Camera pixels are missing or older than the allowed freshness interval.
    STALE_LIDAR, // A scan is missing or older than the allowed freshness interval.
    TIMING_GAP
}; // Paired frame and scan receive times are too far apart for association.
enum RobotStatus { // Delivery decision reported to the UI; pause and health gates precede planning.
    READY, // Robot is at its reset state waiting for a start request.
    DRIVING, // Robot is following a direct, continuously checked route toward delivery.
    YIELDING, // Robot is temporarily waiting for stable sensing or for an obstruction to clear.
    DETOUR, // Robot is traveling on a delivery that has selected an indirect route.
    BLOCKED, // No collision-free route fits the selected lane and current figure boundary.
    DELIVERED, // Robot has reached the goal and stopped the current delivery.
    PAUSED, // User has paused a delivery that was previously started.
    TRACK_LOST, // Fused tracking cannot currently authorize virtual movement.
    TPU_OFFLINE, // Required Coral inference is unavailable or stale.
    TPU_STOP, // A confirmed stop-sign detection is holding virtual movement.
    TOO_CLOSE
}; // The robot's current position overlaps the figure's exclusion boundary.
enum Command { NONE,
               TOGGLE_RUN,
               RESET,
               TOGGLE_LANE,
               CLEAR_TRAIL }; // Single-iteration actions; NONE leaves the user's run/lane selections unchanged.
struct Control { // One-shot command plus independently timestamp-validated Coral observation. coral_sequence prevents repeated reads from counting as new sign confirmations.
    int command, coral_required, coral_healthy, coral_sequence, stop_sign; // Command, semantic-gate enable/health flags, unique inference sequence, and stop-sign observation.
};
struct Result { // Bounded output snapshot: counts delimit arrays; selected=-1 means no associated cluster. xy/raw are filtered/measured positions; heading is radians from forward.
    int track_status, marker_count, group_count, selected, trail_count, scan_number; // Perception state, valid array counts, selected cluster index, and last consumed scan number.
    Marker markers[16]; // Fixed-capacity camera candidates; only marker_count entries are valid.
    Group groups[64]; // Fixed-capacity LiDAR clusters; only group_count entries are valid.
    TrailPoint trail[256]; // Chronological history; only trail_count entries and matching segments may be connected.
    XY xy, raw; // Smoothed and measured figure positions; their validity depends on track_status/last_seen.
    double last_seen, pair_delta, expected_bearing; // Last accepted scan time (s), signed receive-time gap (s), and camera-derived bearing (degrees).
    int robot_status, running, narrow, path_count, reroutes, coral_held; // Delivery state, run/lane flags, valid route length, detour count, and stop-sign latch flag.
    XY robot, goal, path[128]; // Current robot, delivery goal, and remaining waypoint prefix in meters.
    double heading, lane_width, obstacle_radius; // Robot heading from forward (radians), full lane width (m), and total exclusion radius (m).
};
API int core_abi(); // Return fixed-layout interface version 2, which includes the explicit boundary radius in Config.
API int core_size(int which); // Return sizeof(Config/Result/Control/RawPoint) for selectors 0/1/2/3; other selectors return zero.
API void *core_create(const Config *config); // Allocate one engine; return null for invalid settings or construction failure.
API void core_destroy(void *core); // Destroy an engine created by core_create; passing null is harmless.
API int core_set_boundary(void *core, double radius_m); // Call before core_step on the same thread. Preserve pose/tracking, clear route.
API int core_step(void *core, const unsigned char *bgr, int width, int height, int stride,
                  double camera_time, const RawPoint *points, int count, double scan_time,
                  int scan_number, double now, const Control *control, Result *result); // Consume one observation snapshot and command; return 0, -1 invalid input, or -2 internal failure.
// Evaluation entry: supply detector boxes already mapped into the original 640x480 camera coordinates.
// A unique detector sequence and unique scan are both required to advance tracking confirmation.
API int core_step_observations(void *core, const Marker *observations, int observations_count,
                              std::uint64_t camera_sequence, double camera_at, const RawPoint *points,
                              int count, double scan_at, int scan_number, double now,
                              const Control *control, Result *result);
API int core_plan(const Config *config, XY start, XY goal, const XY *obstacles, int obstacle_count,
                  int narrow, XY *path, int capacity); // Planner-only entry point for portable C++ tests and diagnostics.
