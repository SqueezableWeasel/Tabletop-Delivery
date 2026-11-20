/**
 * @file
 * Strict JSON configuration and boundary persistence.
 * Parsing checks types and sizes before copying values into Config. Validation
 * enforces geometric/timing invariants before the engine can accept the settings.
 */

#include "app.h"
#include <array> // Fixed-size standard arrays used where element count is part of the data contract.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
namespace tabletop {
namespace {
double number(const cv::FileNode &n, const char *name) { // Scalar readers reject missing values, type mismatches, and nonfinite data. Integer fields additionally reject fractional values before narrowing.
    auto v = n[name]; // Retrieve a named scalar node without assuming the field exists or has the correct type.
    if (v.empty() || (!v.isInt() && !v.isReal())) // Reject absent values and nonnumeric JSON types before conversion.
        throw std::runtime_error(std::string("Missing/nonnumeric configuration: ") + name);
    double d = double(v); // Read the numeric value as double before applying finite/range/integer validation.
    if (!std::isfinite(d)) // Reject NaN or infinity instead of allowing them into geometry or timing comparisons.
        throw std::runtime_error(std::string("Nonfinite configuration: ") + name);
    return d; // Return the validated finite numeric scalar.
}
int integer(const cv::FileNode &n, const char *name) {
    double d = number(n, name); // Reuse numeric type/finite checks before enforcing an integral destination.
    if (d != std::floor(d) || std::abs(d) > 1000000) // Reject fractional values and bound magnitude before narrowing to int.
        throw std::runtime_error(std::string("Expected integer: ") + name);
    return int(d); // Convert only after verifying integrality and a safely representable magnitude.
}
template <class T>
void array(const cv::FileNode &n, const char *name, T *out, int count) { // Copy fixed-size numeric arrays only after checking length and element type. Integral destinations must not silently truncate a floating-point setting.
    auto a = n[name]; // Retrieve the requested fixed-size numeric array node.
    if (!a.isSeq() || int(a.size()) != count) // Require an array with exactly the number of elements expected by the destination buffer.
        throw std::runtime_error(std::string("Wrong array size: ") + name);
    for (int i = 0; i < count; ++i) { // Validate and copy each element without exceeding the fixed destination capacity.
        auto v = a[i]; // Select this array element before checking its scalar type and numeric range.
        double d = double(v); // Read the numeric value as double before applying finite/range/integer validation.
        if ((!v.isReal() && !v.isInt()) || !std::isfinite(d) || std::abs(d) > 1000000) // Reject nonnumeric, nonfinite, or excessively large array elements.
            throw std::runtime_error(std::string("Invalid array: ") + name);
        if constexpr (std::is_integral_v<T>) // Perform fractional-value rejection only when the destination element type is integral.
            if (d != std::floor(d)) // Do not silently truncate a fractional JSON value assigned to an integer array.
                throw std::runtime_error(std::string("Expected integer array: ") + name);
        out[i] = T(d); // Copy the validated value into the requested destination element type.
    }
}
void read_tracking(Config &c, const cv::FileNode &root) { // Read calibration, marker, LiDAR, and temporal settings. Robot geometry is loaded separately so an explicitly requested tracking overlay preserves it.
    if (integer(root, "schema_version") != 1) // Require schema version 1 before interpreting the tracking configuration layout.
        throw std::runtime_error("Unsupported configuration version");
    c.forward = number(root, "lidar_forward_deg"); // LiDAR sensor azimuth corresponding to tabletop forward, in degrees.
    auto anchors = root["camera_bearing_anchors"]; // Read the three camera pixel/bearing calibration pairs as a nested JSON sequence.
    if (!anchors.isSeq() || anchors.size() != 3) // Exactly left, center, and right calibration pairs are required.
        throw std::runtime_error("Exactly three bearing anchors required");
    for (int i = 0; i < 3; ++i) { // Visit each of the three calibration pairs or HSV channels, as selected by this loop's body.
        auto a = anchors[i]; // Select one calibration pair before validating its two scalar coordinates.
        if (!a.isSeq() || a.size() != 2) // Each anchor must contain exactly one pixel coordinate and one bearing value.
            throw std::runtime_error("Invalid bearing anchor");
        for (int j = 0; j < 2; ++j) { // Read pixel u and bearing degrees into adjacent entries of the flattened anchor array.
            if (!a[j].isReal() && !a[j].isInt()) // Reject a nonnumeric anchor element before assigning it to the calibration table.
                throw std::runtime_error("Nonnumeric anchor");
            c.anchors[i * 2 + j] = double(a[j]); // Flatten the three two-element pairs into [u0,b0,u1,b1,u2,b2].
            if (!std::isfinite(c.anchors[i * 2 + j])) // Ensure no invalid floating-point anchor enters interpolation or slope calculation.
                throw std::runtime_error("Nonfinite anchor");
        }
    }
    auto m = root["marker"], l = root["lidar"], t = root["tracking"]; // Separate marker, LiDAR, and temporal tracking sections from robot geometry.
    array(m, "hsv_low", c.hsv_low, 3); // Load exactly three integer lower bounds for H, S, and V.
    array(m, "hsv_high", c.hsv_high, 3); // Load exactly three integer upper bounds for H, S, and V.
    array(m, "roi_xyxy_px", c.roi, 4); // Load the four integer image ROI bounds before checking image containment.
    c.min_area = integer(m, "min_area_px"); // Minimum accepted connected marker area, in foreground pixels.
    c.max_area = integer(m, "max_area_px"); // Maximum accepted connected marker area, in foreground pixels.
    c.min_aspect = number(m, "min_aspect"); // Minimum marker bounding-box width divided by height, dimensionless.
    c.max_aspect = number(m, "max_aspect"); // Maximum marker bounding-box width divided by height, dimensionless.
    c.extrapolation = number(m, "anchor_extrapolation_px"); // Allowed horizontal extrapolation beyond camera calibration anchors, in pixels.
    double x[2], y[2]; // Temporary two-element buffers hold metric ROI limits before copying them to named fields.
    array(l, "roi_x_m", x, 2); // Read the minimum and maximum rightward coordinates in meters.
    array(l, "roi_y_m", y, 2); // Read the near and far forward coordinates in meters.
    c.xmin = x[0]; // Left edge of the metric tracking region, in meters.
    c.xmax = x[1]; // Right edge of the metric tracking region, in meters.
    c.ymin = y[0]; // Near edge of the metric tracking region, in meters forward of the LiDAR.
    c.ymax = y[1]; // Far edge of the metric tracking region, in meters forward of the LiDAR.
    c.min_quality = integer(l, "min_quality"); // Minimum decoded LiDAR return quality; the core also rejects zero quality.
    c.min_points = integer(l, "min_cluster_points"); // Minimum number of valid LiDAR returns required for a cluster.
    c.cluster_gap = number(l, "cluster_gap_m"); // Maximum spatial gap between neighboring returns in one cluster, in meters.
    c.angle_gap = number(l, "cluster_angle_gap_deg"); // Maximum angular gap between neighboring cluster returns, in degrees.
    c.max_width = number(l, "max_cluster_width_m"); // Maximum accepted cluster bounding-box diagonal, in meters.
    c.bearing_gate = number(l, "bearing_gate_deg"); // Allowed camera-to-cluster bearing difference for association, in degrees.
    c.fresh = number(t, "freshness_s"); // Maximum camera/scan receive age allowed for tracking, in seconds.
    c.pair_gap = number(t, "max_pair_delta_s"); // Maximum absolute receive-time difference between paired camera and LiDAR, in seconds.
    c.confirmation = integer(t, "confirmation_scans"); // Consecutive consistent scans required before a new track authorizes motion.
    c.reacquire = number(t, "reacquire_after_s"); // Gap after which a position must be confirmed again instead of continuing the old filter, in seconds.
    c.max_speed = number(t, "max_speed_m_s"); // Maximum plausible figure motion used by the jump gate, in meters per second.
    c.jump = number(t, "jump_allowance_m"); // Fixed positional tolerance added to the speed-times-age jump envelope, in meters.
    c.smoothing = number(t, "smoothing_time_s"); // Exponential position filter time constant, in seconds; smaller values respond faster.
    c.trail_s = number(t, "trail_s"); // Maximum age of displayed position history, in seconds.
}
} // namespace
void validate_config(const Config &c) { // Check cross-field relationships: ordered anchors, image/metric regions, threshold ranges, lane containment, and endpoints valid even in the narrow lane.
    auto fail = [](bool bad, const char *why) { // Centralize field-specific validation exceptions without modifying the configuration.
        if (bad) // Convert a failed invariant into a descriptive startup error.
            throw std::runtime_error(std::string("Invalid configuration: ") + why);
    };
    fail(!valid_boundary_radius(exclusion_radius(c)), "figure boundary radius must be between 3 and 10 cm"); // Enforce the same inclusive total-radius limits as live controls and the core API.
    fail(!(c.speed_scale > 0 && c.speed_scale <= 1), "robot speed_scale must be greater than 0 and at most 1"); // Reject nonfinite pacing and unsupported fractions before constructing the engine.
    fail(c.min_area < 1 || c.max_area < c.min_area || c.max_area > 640 * 480, "marker areas"); // Require positive ordered component areas bounded by the total image pixel count.
    fail(c.min_aspect <= 0 || c.max_aspect < c.min_aspect || c.extrapolation < 0, "marker shape"); // Require ordered positive marker aspect limits and nonnegative extrapolation allowance.
    fail(c.angle_gap <= 0 || c.max_width <= 0 || c.bearing_gate <= 0 || c.bearing_gate > 45,
         "LiDAR association"); // Require meaningful cluster/association limits and cap the bearing gate at 45 degrees.
    fail(c.min_quality < 0 || c.min_quality > 63 || c.min_points > 16384 || c.confirmation > 100,
         "scan thresholds"); // Bound quality and observation-count thresholds to supported data capacities.
    fail(c.max_speed <= 0 || c.jump < 0 || c.trail_s < 0 || c.yield_s < 0,
         "tracking/planner timing"); // Require positive figure-speed allowance and nonnegative jump/history/yield values.
    fail(!(c.anchors[1] < c.anchors[3] && c.anchors[3] < c.anchors[5]),
         "bearing anchors must increase"); // Bearing values must increase from left through center to right for this calibration model.
    fail(c.lane_width > 1.5 || c.narrow_width > c.lane_width || c.lane_ymax - c.lane_ymin > 1.5,
         "tabletop lane dimensions"); // Keep lane geometry within tabletop scale and ensure narrow width does not exceed wide width.
    fail(c.lane_ymin < c.ymin || c.lane_ymax > c.ymax || -c.lane_width / 2 < c.xmin ||
             c.lane_width / 2 > c.xmax,
         "lane must fit configured tracking area"); // Require the virtual lane to lie inside the configured metric tracking rectangle.
    for (int i = 0; i < 3; ++i) // Visit each of the three calibration pairs or HSV channels, as selected by this loop's body.
        fail(c.hsv_low[i] < 0 || c.hsv_low[i] > c.hsv_high[i] || c.hsv_high[i] > (i ? 255 : 179),
             "HSV bounds"); // Validate ordered HSV channel limits using 179 for hue and 255 for saturation/value.
    for (XY p : {c.start, c.goal}) // Validate both depot and delivery against the more restrictive narrow lane.
        fail(!std::isfinite(p.x) || !std::isfinite(p.y) ||
                 std::abs(p.x) > c.narrow_width / 2 - c.robot_radius ||
                 p.y < c.lane_ymin + c.robot_radius || p.y > c.lane_ymax - c.robot_radius,
             "depot/delivery outside narrow lane"); // Require finite endpoints with their entire circular robot footprint inside the narrow lane.
    void *h = core_create(&c); // Exercise the core's own configuration invariants as the final application validation step.
    fail(!h, "native engine rejected settings"); // Reject settings that passed parser checks but fail the portable engine's contract.
    core_destroy(h); // Release the temporary validation engine immediately after successful creation.
}
Config load_config(const std::filesystem::path &path) { // Start with a fully initialized Config, load the required schema sections, and validate the entire result before returning it to application startup.
    cv::FileStorage f(path.string(), cv::FileStorage::READ); // Open the selected JSON file through OpenCV's structured configuration reader.
    if (!f.isOpened()) // Detect file-open failure before attempting to read or write any fields.
        throw std::runtime_error("Cannot open config: " + path.string());
    Config c = default_config(); // Initialize every Config member before overwriting the required serialized values.
    read_tracking(c, f.root()); // Parse calibration, marker, LiDAR, and temporal fields using checked readers.
    auto r = f["robot"]; // Read robot-specific geometry and motion settings independently from tracking overlays.
#define READ(field) c.field = number(r, #field) // Stringize the Config field name to read the matching numeric key from the robot JSON object.
    READ(lane_width); // Full width of the wide virtual lane, in meters before footprint insetting.
    READ(narrow_width); // Full width of the narrow virtual lane, in meters before footprint insetting.
    READ(lane_ymin); // Near boundary of the virtual lane, in meters forward of the LiDAR.
    READ(lane_ymax); // Far boundary of the virtual lane, in meters forward of the LiDAR.
    READ(robot_radius); // Circular virtual robot footprint radius used for collision checking, in meters.
    READ(figure_radius); // Assumed figure footprint radius used by this tabletop model, in meters.
    READ(margin); // Additional nonnegative clearance beyond both body radii, in meters.
    if (!r["boundary_radius"].empty()) // The optional total-radius selection overrides the nominal figure + robot + margin sum.
        READ(boundary_radius); // Read through the same finite numeric validation used by all other geometry settings.
    READ(speed); // Nominal virtual robot travel speed before applying the independent pacing fraction, in meters per second.
    if (!r["speed_scale"].empty()) // Omitted pacing uses Config's 80% default, including automatically selected saved calibrations.
        READ(speed_scale); // Read a finite fraction; core validation requires a value greater than zero and at most one.
    READ(yield_s); // Wait before attempting a detour around a persistent obstruction, in seconds.
    READ(grid_m); // A* search-grid spacing, in meters; continuous edge checks still guard movement.
#undef READ // Restrict the helper macro's lifetime to this serialization block.
    double p[2]; // Reuse a validated two-coordinate buffer for depot and delivery positions.
    array(r, "start", p, 2); // Load the depot's [x,y] metric position with exact array-length checking.
    c.start = {p[0], p[1]}; // Exact depot coordinate [right,forward], in meters.
    array(r, "goal", p, 2); // Load the delivery point's [x,y] metric position with exact array-length checking.
    c.goal = {p[0], p[1]}; // Exact delivery coordinate [right,forward], in meters.
    validate_config(c); // Verify all cross-field and core invariants before these settings are used or persisted.
    return c; // Return the fully initialized and validated effective configuration by value.
}
void overlay_tracking(Config &c, const std::filesystem::path &path) { // Apply only an explicitly selected calibration file, then validate against the configured lane/robot geometry. No neighboring directories are searched.
    cv::FileStorage f(path.string(), cv::FileStorage::READ); // Open the selected JSON file through OpenCV's structured configuration reader.
    if (!f.isOpened()) // Detect file-open failure before attempting to read or write any fields.
        throw std::runtime_error("Cannot open tracking calibration: " + path.string());
    read_tracking(c, f.root()); // Parse calibration, marker, LiDAR, and temporal fields using checked readers.
    validate_config(c); // Verify all cross-field and core invariants before these settings are used or persisted.
}
bool load_boundary_setting(Config &c, const std::filesystem::path &path) { // A missing sidecar is normal. Parse and validate a temporary Config before commit so malformed saved values cannot partially overwrite active settings.
    if (!std::filesystem::exists(path)) // Treat an absent optional radius sidecar as normal first-run behavior.
        return false;
    cv::FileStorage f(path.string(), cv::FileStorage::READ); // Open the selected JSON file through OpenCV's structured configuration reader.
    if (!f.isOpened()) // Detect file-open failure before attempting to read or write any fields.
        throw std::runtime_error("Cannot read saved figure boundary");
    if (integer(f.root(), "schema_version") != 1) // Require the saved boundary's own supported schema before reading its radius.
        throw std::runtime_error("Unsupported boundary settings version");
    const double radius = number(f.root(), "radius_m"); // Read the selected total exclusion radius in meters.
    if (!valid_boundary_radius(radius)) // A saved preference cannot bypass the live control's inclusive 3–10 cm limits.
        throw std::runtime_error("Saved boundary radius must be between 3 and 10 cm");
    Config updated = c; // Validate changes on a copy so malformed sidecars cannot partially mutate active settings.
    updated.boundary_radius = radius; // Restore the total radius directly while retaining nominal body sizes, margin, and calibration.
    validate_config(updated); // Check the proposed copy using the same invariants as ordinary configuration loading.
    c = updated; // Commit only after the complete modified configuration has passed validation.
    return true;
}
void save_boundary_setting(const Config &c, const std::filesystem::path &path) { // Persist only schema ID and selected total radius; calibration and nominal body sizes stay in the main config.
    validate_config(c); // Verify all cross-field and core invariants before these settings are used or persisted.
    cv::FileStorage f(path.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Open a JSON writer for the selected output path.
    if (!f.isOpened()) // Detect file-open failure before attempting to read or write any fields.
        throw std::runtime_error("Cannot save figure boundary: " + path.string());
    f << "schema_version" << 1 << "radius_m" << exclusion_radius(c); // Serialize the exact radius used by the display and collision checker.
    f.release(); // Close and flush the structured writer before reporting that the boundary was saved.
}
void save_config(const Config &c, const std::filesystem::path &path, double calibration_camera_distance_m, double calibration_camera_height_m) { // Record optional camera placement geometry without changing measured LiDAR ranges or the robot's 2D coordinates.
    if (!std::isfinite(calibration_camera_distance_m) || calibration_camera_distance_m < 0 ||
        !std::isfinite(calibration_camera_height_m) || calibration_camera_height_m < 0) // Reject invalid provenance before creating an output file.
        throw std::runtime_error("Invalid camera calibration distance or height");
    cv::FileStorage f(path.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Open a JSON writer for the selected output path.
    if (!f.isOpened()) // Detect file-open failure before attempting to read or write any fields.
        throw std::runtime_error("Cannot write configuration");
    f << "schema_version" << 1 << "lidar_forward_deg" << c.forward << "camera_bearing_anchors"
      << "["; // Begin the effective settings document with schema, forward calibration, and anchor array.
    for (int i = 0; i < 3; ++i) // Visit each of the three calibration pairs or HSV channels, as selected by this loop's body.
        f << "[" << c.anchors[i * 2] << c.anchors[i * 2 + 1] << "]"; // Serialize one [pixel_u,bearing_degrees] calibration pair.
    f << "]" << "marker" << "{" << "hsv_low" << "[" << c.hsv_low[0] << c.hsv_low[1] << c.hsv_low[2]
      << "]"
      << "hsv_high" << "[" << c.hsv_high[0] << c.hsv_high[1] << c.hsv_high[2] << "]"
      << "roi_xyxy_px" << "[" << c.roi[0] << c.roi[1] << c.roi[2] << c.roi[3] << "]"
      << "min_area_px" << c.min_area << "max_area_px" << c.max_area << "min_aspect" << c.min_aspect
      << "max_aspect" << c.max_aspect << "anchor_extrapolation_px" << c.extrapolation << "}"
      << "lidar" << "{" << "roi_x_m" << "[" << c.xmin << c.xmax << "]" << "roi_y_m" << "[" << c.ymin
      << c.ymax << "]"
      << "min_quality" << c.min_quality << "cluster_gap_m" << c.cluster_gap
      << "cluster_angle_gap_deg" << c.angle_gap << "min_cluster_points" << c.min_points
      << "max_cluster_width_m" << c.max_width << "bearing_gate_deg" << c.bearing_gate << "}"
      << "tracking" << "{" << "freshness_s" << c.fresh << "max_pair_delta_s" << c.pair_gap
      << "confirmation_scans" << c.confirmation << "reacquire_after_s" << c.reacquire
      << "max_speed_m_s" << c.max_speed << "jump_allowance_m" << c.jump << "smoothing_time_s"
      << c.smoothing << "trail_s" << c.trail_s << "}" << "robot" << "{"; // Serialize marker thresholds, metric LiDAR limits, and temporal settings under their named sections.
#define WRITE(field) f << #field << c.field // Stringize each robot field name so serialization uses the same key as parsing.
    WRITE(lane_width); // Full width of the wide virtual lane, in meters before footprint insetting.
    WRITE(narrow_width); // Full width of the narrow virtual lane, in meters before footprint insetting.
    WRITE(lane_ymin); // Near boundary of the virtual lane, in meters forward of the LiDAR.
    WRITE(lane_ymax); // Far boundary of the virtual lane, in meters forward of the LiDAR.
    WRITE(robot_radius); // Circular virtual robot footprint radius used for collision checking, in meters.
    WRITE(figure_radius); // Assumed figure footprint radius used by this tabletop model, in meters.
    WRITE(margin); // Additional nonnegative clearance beyond both body radii, in meters.
    WRITE(boundary_radius); // Preserve an explicit radius selection when saving the complete effective configuration.
    WRITE(speed); // Save nominal speed without multiplying it, so repeated saves preserve the same actual travel rate.
    WRITE(speed_scale); // Store pacing separately for reproducible calibration and session-configuration roundtrips.
    WRITE(yield_s); // Wait before attempting a detour around a persistent obstruction, in seconds.
    WRITE(grid_m); // A* search-grid spacing, in meters; continuous edge checks still guard movement.
#undef WRITE // End the local write helper's scope before serializing coordinate arrays.
    f << "start" << "[" << c.start.x << c.start.y << "]" << "goal" << "[" << c.goal.x << c.goal.y
      << "]" << "}"; // Finish the robot section with exact depot and delivery [x,y] coordinates.
    if (calibration_camera_distance_m > 0) { // Include physical-placement provenance only when saving a newly captured calibration.
        f << "reference" << "{" << "camera_to_figure_center_m" << calibration_camera_distance_m
          << "distance_origin" << "Camera lens; horizontal forward distance to the figure center on the tabletop"
          << "calibration" << "Measured stationary center, left, and right camera/LiDAR pairs on the reference row"
          << "range_note" << "Observed LiDAR surfaces retain their measured ranges from the LiDAR origin";
        if (calibration_camera_height_m > 0) // Zero means the optional lens-center height was not supplied by the caller.
            f << "camera_center_height_m" << calibration_camera_height_m; // Store the measured height independently of the horizontal placement distance.
        f << "}"; // Close the reference object after all supplied setup measurements are recorded.
    }
}
} // namespace tabletop
