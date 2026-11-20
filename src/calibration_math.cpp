/**
 * @file
 * Calibration validation, aggregation, fitting, and persistence.
 * Anchors come from observed marker pixels and manually selected LiDAR clusters.
 * The 16-inch camera reference is provenance, never an invented range offset.
 */
#include "calibration.h"
#include <limits> // Initialize extrema before measuring observation spread.
#include <stdexcept> // Explain why a capture or completed calibration cannot be accepted.

namespace tabletop {
namespace {
constexpr double pi = 3.14159265358979323846; // Convert between sensor degrees and trigonometric radians.
double wrapped(double degrees) { // Use signed angular differences that remain continuous across raw zero degrees.
    return std::remainder(degrees, 360.); // Return the equivalent signed difference in [-180,180].
}
std::filesystem::path temporary_path(const std::filesystem::path &path, const char *kind) { // Keep staging and backup files on the destination filesystem so final renames remain local.
    auto candidate = path; // Retain the complete user-selected filename and directory.
    candidate += std::string(kind) + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()); // Use a session-unique suffix without depending on wall-clock timestamps.
    if (std::filesystem::exists(candidate)) // Never reuse a staging or backup filename that already contains data.
        throw std::runtime_error("Calibration temporary filename already exists");
    return candidate; // Return an unused sibling filename for this operation.
}
void publish_file(const std::filesystem::path &pending, const std::filesystem::path &destination) { // Publish a fully written file with an atomic rename, preserving the previous file first.
    if (std::filesystem::exists(destination)) { // A completed calibration may replace another completed calibration on a subsequent run.
        if (!std::filesystem::is_regular_file(destination)) // Refuse directories and other non-file destinations without moving or deleting them.
            throw std::runtime_error("Calibration destination is not a regular file");
        std::filesystem::copy_file(destination, temporary_path(destination, ".backup.")); // Keep the previous bytes in a unique backup; a backup failure leaves the active file untouched.
    }
    std::filesystem::rename(pending, destination); // Replace without a delete-then-write gap; failure leaves the existing destination in place.
}
struct PendingFiles { // Clean up only this operation's temporary files when validation or publication throws.
    std::filesystem::path first, second; // Empty or already-published temporary names are harmless to remove.
    ~PendingFiles() { std::error_code ignored; if (!first.empty()) std::filesystem::remove(first, ignored); if (!second.empty()) std::filesystem::remove(second, ignored); } // Never remove the published configuration or a user's backup during cleanup.
};
} // namespace
std::filesystem::path startup_config_path(const std::filesystem::path &requested, bool explicitly_selected) { // Resolve one deterministic startup choice rather than searching unrelated directories.
    if (explicitly_selected) return requested; // An explicit path is authoritative, including when it is missing or invalid.
    const auto directory = requested.has_parent_path() ? requested.parent_path() : std::filesystem::path("."); // Resolve automatic preferences beside the default project configuration.
    const auto record = directory / "active_calibration.json"; // A successful calibration records its selected output here.
    if (std::filesystem::exists(record)) { // Prefer the most recently activated calibration, including a custom output filename.
        cv::FileStorage saved(record.string(), cv::FileStorage::READ); // Read a structured local preference without shell parsing.
        if (!saved.isOpened() || !saved["schema_version"].isInt() || int(saved["schema_version"]) != 1 ||
            !saved["config_path"].isString() || std::string(saved["config_path"]).empty()) // Reject malformed preferences instead of silently using an unrelated calibration.
            throw std::runtime_error("Invalid active_calibration.json; use --config delivery_config.json to select the seed settings");
        std::filesystem::path selected = std::string(saved["config_path"]); // Recover the filename explicitly recorded by calibration completion.
        if (selected.is_relative()) selected = directory / selected; // Resolve relative paths against the preference file, preserving relocatable project folders.
        load_config(selected); // Surface a missing or invalid active calibration before opening sensors or inference.
        return selected.lexically_normal(); // The selected file also determines the boundary sidecar used by startup.
    }
    const auto calibration = directory / "calibration_16in.json"; // Automatically use an existing default-name calibration even before an activation record is present.
    if (std::filesystem::exists(calibration)) { load_config(calibration); return calibration; } // Validate the saved measurements rather than quietly falling back on failure.
    return requested; // A first run with no saved calibration uses the seed configuration.
}
void activate_calibration(const std::filesystem::path &config, const std::filesystem::path &record) { // Publish a validated calibration selection after its complete configuration has been saved.
    const auto target = std::filesystem::absolute(config).lexically_normal(), active = std::filesystem::absolute(record).lexically_normal(); // Compare normalized absolute paths before writing the preference.
    if (target == active) throw std::runtime_error("Calibration output cannot be active_calibration.json"); // Prevent the activation record from overwriting the configuration it is supposed to select.
    load_config(target); // Never change the active selection to an unreadable or invalid configuration.
    auto relative = target.lexically_relative(active.parent_path()); // Store a portable project-relative path whenever both files share a filesystem root.
    const auto stored = relative.empty() ? target : relative; // Preserve an absolute path for an explicitly selected file on another root.
    PendingFiles pending{temporary_path(active, ".pending."), {}}; // The temporary preference is removed automatically if writing or publication fails.
    cv::FileStorage output(pending.first.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Write the activation record as a complete new JSON document.
    if (!output.isOpened()) throw std::runtime_error("Cannot save active calibration selection"); // Leave the previous selection intact on an open failure.
    output << "schema_version" << 1 << "config_path" << stored.generic_string(); // Store only the schema and chosen file; actual measured values remain in the calibration.
    output.release(); // Flush the entire preference before exposing it to a later process.
    publish_file(pending.first, active); // Replace the active record only after the calibration and record are ready.
}
Config calibration_capture_config(const Config &base) { // Diagnostic bounds must admit the closer figure before valid new anchors exist.
    Config c = base; // Preserve measured sensor orientation, thresholds, and robot settings as a starting point.
    c.xmin = std::min(c.xmin, -.60); // Include nearby left placements and potential camera/LiDAR lateral separation.
    c.xmax = std::max(c.xmax, .60); // Include the corresponding right-hand capture area.
    c.ymin = std::min(c.ymin, .12); // Admit surfaces nearer than the nominal 16-inch center placement.
    c.ymax = std::max(c.ymax, 1.20); // Keep surrounding scan context visible for explicit cluster selection.
    c.roi[0] = c.roi[1] = 0; // Detect the marker anywhere in the full image during calibration.
    c.roi[2] = 640; c.roi[3] = 480; // Keep the established camera resolution and exclusive image bounds.
    c.max_area = std::max(c.max_area, 12000); // A closer marker occupies more pixels; retain the same HSV and shape checks.
    validate_config(c); // Validate the expanded diagnostic configuration before opening the engine.
    return c; // Return separate capture settings without changing the user's working configuration.
}
CalibrationPose summarize_calibration(const std::vector<CalibrationSample> &samples) { // Average only a sufficiently long, stationary set of real paired observations.
    if (samples.size() < CALIBRATION_SAMPLES) // A single snapshot does not provide a stability check.
        throw std::runtime_error("Hold still until 20 paired samples are available");
    CalibrationPose pose; // Initialize all accumulated fields and marker bounds.
    double sin_sum = 0, cos_sum = 0; // Circular averaging handles a cluster straddling raw zero degrees.
    double min_u = 640, max_u = 0, min_x = 10, max_x = -10, min_y = 10, max_y = -10; // Track independent pixel and metric spread.
    for (const auto &s : samples) { // Validate each manually associated marker/cluster pair before averaging.
        if (!std::isfinite(s.marker.u) || s.marker.u < 0 || s.marker.u >= 640 ||
            !std::isfinite(s.raw_angle) || !std::isfinite(s.group.range) || s.group.range <= 0 ||
            !std::isfinite(s.group.xy.x) || !std::isfinite(s.group.xy.y) || s.group.count < 2 ||
            s.marker.area < 1 || s.marker.area > 640 * 480 || s.marker.y < 0 || s.marker.h < 1 || s.marker.h > 480 || s.marker.y + s.marker.h > 480) // Reject malformed observations instead of producing plausible-looking anchors.
            throw std::runtime_error("Invalid calibration observation");
        pose.u += s.marker.u; pose.range += s.group.range; // Accumulate observed pixel/range values without imposing the ruler distance.
        pose.surface.x += s.group.xy.x; pose.surface.y += s.group.xy.y; // Preserve mean measured surface coordinates for diagnostics.
        sin_sum += std::sin(s.raw_angle * pi / 180); cos_sum += std::cos(s.raw_angle * pi / 180); // Accumulate bearings on the unit circle.
        min_u = std::min(min_u, s.marker.u); max_u = std::max(max_u, s.marker.u); // Measure horizontal marker jitter across the sample window.
        min_x = std::min(min_x, s.group.xy.x); max_x = std::max(max_x, s.group.xy.x); // Measure lateral movement of the selected surface.
        min_y = std::min(min_y, s.group.xy.y); max_y = std::max(max_y, s.group.xy.y); // Measure forward movement independently of camera pixels.
        pose.top = std::min(pose.top, s.marker.y); pose.bottom = std::max(pose.bottom, s.marker.y + s.marker.h); // Retain all observed marker rows.
        pose.max_area = std::max(pose.max_area, s.marker.area); // Record the largest real component for runtime area limits.
    }
    if (max_u - min_u > 8 || std::hypot(max_x - min_x, max_y - min_y) > .012) // Require less than 8 pixels and 12 mm of total spread while the figure is stationary.
        throw std::runtime_error("Figure or selected cluster is moving; hold still");
    if (std::hypot(sin_sum, cos_sum) < .99 * samples.size()) // Reject inconsistent bearings even if other malformed fields happen to look stationary.
        throw std::runtime_error("Selected LiDAR bearings are inconsistent");
    pose.samples = int(samples.size()); // Record how many unique paired observations support this pose.
    pose.u /= pose.samples; pose.range /= pose.samples; // Compute the measured horizontal centroid and average surface range.
    pose.surface.x /= pose.samples; pose.surface.y /= pose.samples; // Compute mean surface coordinates in the capture frame.
    pose.raw_angle = std::fmod(std::atan2(sin_sum, cos_sum) * 180 / pi + 360., 360.); // Normalize the circular-mean raw bearing into [0,360).
    return pose; // Return stable measured evidence rather than an inferred 16-inch point.
}
Config fit_calibration(const Config &base, const std::array<CalibrationPose, 3> &poses) { // The physical center placement defines forward; side placements determine image-to-bearing slopes.
    for (const auto &p : poses) // Require a valid aggregate for all three independently captured placements.
        if (p.samples < CALIBRATION_SAMPLES || !std::isfinite(p.u) || p.u < 0 || p.u >= 640 ||
            !std::isfinite(p.raw_angle) || !std::isfinite(p.range) || p.range <= 0)
            throw std::runtime_error("Capture all three stationary poses first");
    const auto &center = poses[0], &left = poses[1], &right = poses[2]; // Document the capture order separately from the serialized anchor order.
    const double left_angle = wrapped(left.raw_angle - center.raw_angle), right_angle = wrapped(right.raw_angle - center.raw_angle); // Express side bearings relative to the measured center.
    if (left.u + 30 >= center.u || center.u + 30 >= right.u || left_angle >= -2 || right_angle <= 2 ||
        left_angle < -45 || right_angle > 45) // Require useful, ordered anchor spacing and reject swapped poses or implausible cluster choices.
        throw std::runtime_error("Left/center/right pixels and bearings must be ordered and well separated");
    for (const auto &p : poses) { // Side placements should remain on the same measured forward row.
        const double y = p.range * std::cos(wrapped(p.raw_angle - center.raw_angle) * pi / 180); // Compare observed forward surfaces without assuming the camera and LiDAR share an origin.
        if (std::abs(y - center.range) > .04) // Allow small prop-surface/orientation differences but reject large depth changes or an incorrect cluster.
            throw std::runtime_error("Keep all three figure placements on the same forward row");
    }
    Config c = base; // Preserve user-selected boundaries, timing, lane, depot, and delivery geometry.
    c.forward = center.raw_angle; // Derive LiDAR forward from the actual center capture rather than the ruler length.
    const double anchors[]{left.u, left_angle, center.u, 0., right.u, right_angle}; // Construct the piecewise camera-pixel to LiDAR-bearing map from measured pairs.
    std::copy(std::begin(anchors), std::end(anchors), c.anchors); // Replace all three anchors atomically in the proposed configuration.
    for (const auto &p : poses) { // Ensure the measured closer row and marker bounds remain usable during normal tracking.
        const double a = wrapped(p.raw_angle - c.forward) * pi / 180, x = p.range * std::sin(a), y = p.range * std::cos(a); // Rotate the measured surface into the newly defined table frame.
        c.xmin = std::min(c.xmin, x - .10); c.xmax = std::max(c.xmax, x + .10); // Extend only the observation region when a measured side pose requires it.
        c.ymin = std::min(c.ymin, std::max(.05, y - .12)); c.ymax = std::max(c.ymax, y + .12); // Admit surfaces around the new row while retaining the configured lane.
        c.roi[1] = std::min(c.roi[1], std::max(0, p.top - 40)); c.roi[3] = std::max(c.roi[3], std::min(480, p.bottom + 40)); // Expand vertical camera bounds around the closer marker.
        c.max_area = std::max(c.max_area, std::min(640 * 480, p.max_area * 2)); // Leave room for ordinary area changes around the measured closer pose.
    }
    c.roi[0] = 0; c.roi[2] = 640; // Keep side-to-side marker travel visible across the calibrated image width.
    validate_config(c); // Reject any incompatible combined settings before allowing Save to publish them.
    return c; // Return a proposed configuration; fitting itself never writes files or changes live tracking.
}
void save_calibration(const Config &config, const std::array<CalibrationPose, 3> &poses, const std::filesystem::path &path) { // Stage and validate complete calibration outputs before replacing any existing configuration.
    auto report = path; report += ".measurements.json"; // Keep the measured evidence beside the new usable configuration.
    if ((std::filesystem::exists(path) && !std::filesystem::is_regular_file(path)) ||
        (std::filesystem::exists(report) && !std::filesystem::is_regular_file(report))) // Reject non-file destinations before writing or replacing any data.
        throw std::runtime_error("Calibration output must be a regular file");
    validate_config(config); // Validate configuration before creating either output file.
    PendingFiles pending{temporary_path(path, ".pending."), temporary_path(report, ".pending.")}; // Stage both outputs beside their destinations for local atomic publication.
    cv::FileStorage evidence(pending.second.string(), cv::FileStorage::WRITE | cv::FileStorage::FORMAT_JSON); // Serialize numeric evidence without exposing a partially written report.
    if (!evidence.isOpened()) // Surface path/permission errors before reporting success.
        throw std::runtime_error("Cannot write calibration measurements");
    evidence << "camera_to_figure_center_m" << CALIBRATION_CAMERA_DISTANCE_M
             << "camera_center_height_m" << CALIBRATION_CAMERA_HEIGHT_M
             << "distance_basis" << "User positions the figure center 16 inches forward of the camera lens, measured horizontally."
             << "range_note" << "LiDAR ranges describe observed surfaces from the LiDAR origin; no range bias or scaling applied."
             << "poses" << "["; // Distinguish the physical placement instruction from independently observed ranges.
    const char *names[]{"center", "left", "right"}; // Preserve the capture order for later review of manual selections.
    for (int i = 0; i < 3; ++i) // Record all three actual aggregates instead of claiming a calculated ruler-based calibration.
        evidence << "{" << "pose" << names[i] << "marker_u_px" << poses[i].u << "raw_bearing_deg" << poses[i].raw_angle
                 << "observed_range_m" << poses[i].range << "paired_samples" << poses[i].samples << "}";
    evidence << "]"; evidence.release(); // Flush measurement provenance before writing the configuration that will be used for tracking.
    save_config(config, pending.first, CALIBRATION_CAMERA_DISTANCE_M, CALIBRATION_CAMERA_HEIGHT_M); // Write measured anchors, the horizontal reference, and the 28 mm camera height to the staged file.
    load_config(pending.first); // Verify the complete configuration before replacing any working calibration.
    publish_file(pending.second, report); // Retain a backup of existing measurement evidence before publishing the new report.
    publish_file(pending.first, path); // Publish the operational configuration last; no partially written configuration becomes active.
}
} // namespace tabletop
