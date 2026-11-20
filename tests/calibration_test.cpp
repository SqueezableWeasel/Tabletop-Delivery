/**
 * @file
 * Device-free checks of the measured 16-inch calibration workflow.
 * Synthetic camera/LiDAR origins deliberately differ so a fake distance
 * substitution or range scaling cannot pass the geometry assertions.
 */
#include "../src/calibration.h"
#include <opencv2/imgcodecs.hpp> // Write a deterministic capture-window preview for visual inspection.
#include <iostream> // Report completed checks and the specific failed invariant.
#include <limits> // Exercise invalid nonfinite observation rejection.
#include <stdexcept> // Use exceptions for concise independent assertions.

using namespace tabletop;
namespace {
void require(bool value, const char *message) { // Fail immediately on an unmet calibration invariant.
    if (!value) throw std::runtime_error(message); // Preserve an actionable reason for CTest output.
}
template<class F> void rejects(F action) { // Require invalid input to fail rather than quietly create a calibration.
    bool failed = false; // Distinguish an expected exception from successful processing.
    try { action(); } catch (const std::exception &) { failed = true; } // Record rejection without skipping later independent checks.
    require(failed, "Invalid calibration input was accepted"); // A silent fallback is not a valid measured calibration.
}
std::vector<CalibrationSample> observations(double x) { // Create a stationary target with intentionally different camera and LiDAR distances.
    constexpr double y = CALIBRATION_CAMERA_DISTANCE_M + .060 - .015; // Fixture: camera is 6 cm forward of LiDAR and the sensed surface is 1.5 cm in front of the figure center.
    const double angle = 359.6 + std::atan2(x, y) * 180 / 3.141592653589793; // Include wraparound at raw zero degrees in the captured bearings.
    const double u = 320 + 600 * x / CALIBRATION_CAMERA_DISTANCE_M; // Camera pixels depend on camera-relative depth, independently of LiDAR range.
    std::vector<CalibrationSample> values; // Return complete observed pairs, not injected fitted anchors.
    for (int i = 0; i < CALIBRATION_SAMPLES; ++i) { // Provide exactly the required number of distinct stationary pairs.
        const double noise = i % 2 ? .1 : -.1; // Add small symmetric marker/angle noise to exercise real aggregation.
        Marker m{u + noise, 450, int(u) - 25, 425, 50, 50, 2500}; // Place the larger nearby marker partly below the nominal image ROI.
        Group g{{x, y}, angle - 359.6, std::hypot(x, y), .02, 4}; // Keep the true measured surface range rather than substituting 16 inches.
        values.push_back({m, g, angle + noise * .2}); // Use independent pixel and raw-bearing observations for each sample.
    }
    return values; // Return synthetic measured evidence with known geometry for exact checks.
}
} // namespace
int main() {
    try {
        require(std::abs(CALIBRATION_CAMERA_DISTANCE_M - .4064) < 1e-12, "Incorrect inch conversion"); // Verify the exact requested camera reference distance.
        require(std::abs(CALIBRATION_CAMERA_HEIGHT_M - .028) < 1e-12, "Incorrect camera center height"); // Verify the measured 28 mm lens-center height is represented in meters.
        Config base = default_config(); base.boundary_radius = .03; // Preserve a user-selected minimum boundary through calibration.
        const Config capture = calibration_capture_config(base); // Check that acquisition admits the closer image and scan area before alignment is known.
        require(capture.ymin <= .12 && capture.roi[3] == 480, "Capture region excludes nearby marker/surface"); // Nearby data must be observable without relying on the old association.
        std::array<CalibrationPose, 3> poses{summarize_calibration(observations(0)),
            summarize_calibration(observations(-.10)), summarize_calibration(observations(.10))}; // Aggregate center, left, and right placements from actual pair records.
        const Config fitted = fit_calibration(base, poses); // Run production calibration math on the known fixture.
        require(std::abs(std::remainder(fitted.forward - 359.6, 360.)) < 1e-8, "Circular-mean forward incorrect"); // Raw wraparound must not corrupt the center direction.
        require(std::abs(fitted.anchors[2] - 320) < 1e-9 && fitted.anchors[1] < -10 && fitted.anchors[5] > 10,
                "Measured pixel/bearing anchors incorrect"); // Verify both camera centroids and signed LiDAR bearings contribute to the map.
        require(std::abs(poses[0].range - .4514) < 1e-12, "Surface range was forced to camera distance"); // The camera's 0.4064 m reference must not replace LiDAR's observed 0.4514 m range.
        require(fitted.boundary_radius == .03 && fitted.lane_ymin == base.lane_ymin && fitted.start.y == base.start.y,
                "Calibration changed robot geometry"); // New alignment must preserve the selected boundary and delivery layout.
        require(fitted.ymin < .38 && fitted.roi[3] == 480 && fitted.max_area >= 5000,
                "Closer-row observation bounds were not expanded"); // Admit measured closer surfaces and larger/lower marker images during delivery tracking.
        auto bad_samples = observations(0); bad_samples.pop_back(); // A short window must not count as a stationary calibration.
        rejects([&] { summarize_calibration(bad_samples); }); // Require all twenty unique pairs before acceptance.
        bad_samples = observations(0); bad_samples.back().marker.u += 12; // Simulate visible motion despite otherwise plausible measurements.
        rejects([&] { summarize_calibration(bad_samples); }); // Marker spread must reject a moving capture.
        bad_samples = observations(0); bad_samples.back().group.xy.x += .03; // Simulate LiDAR movement or target switching.
        rejects([&] { summarize_calibration(bad_samples); }); // Metric spread must also reject a moving capture.
        bad_samples = observations(0); bad_samples.back().raw_angle = std::numeric_limits<double>::quiet_NaN(); // Supply unusable numeric sensor data.
        rejects([&] { summarize_calibration(bad_samples); }); // Invalid data cannot become plausible serialized anchors.
        auto swapped = poses; std::swap(swapped[1], swapped[2]); // Deliberately capture the side placements in the wrong order.
        rejects([&] { fit_calibration(base, swapped); }); // Pixel and bearing ordering must expose the mistake.
        auto wrong_row = poses; wrong_row[1].range += .15; // Move a side target to a materially different forward depth.
        rejects([&] { fit_calibration(base, wrong_row); }); // Same-row checking must reject an unrelated cluster or placement.
        auto incomplete = poses; incomplete[2].samples = 0; // Leave the right placement unmeasured.
        rejects([&] { fit_calibration(base, incomplete); }); // Fitting must not publish a partial calibration.
        const auto directory = std::filesystem::current_path() / ("calibration_test_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count())); // Isolate all generated settings and backups from any user's active calibration.
        std::filesystem::create_directory(directory); // Create one uniquely named test-owned directory without touching other configurations.
        const auto path = directory / "calibration_16in.json", seed = directory / "delivery_config.json", active = directory / "active_calibration.json"; // Exercise the production filenames within this isolated project.
        save_config(base, seed); // Provide seed settings for the no-calibration first-run case.
        require(startup_config_path(seed, false) == seed, "First run did not select seed configuration"); // No saved calibration must retain the normal first-run fallback.
        save_calibration(fitted, poses, path); // Exercise the real configuration and measured-evidence writers.
        const Config restored = load_config(path); // Read the generated calibration with the production startup parser.
        require(std::abs(restored.anchors[0] - fitted.anchors[0]) < 1e-9 && restored.boundary_radius == .03,
                "Saved calibration lost alignment or boundary"); // The resulting file must be directly usable by the delivery executable.
        cv::FileStorage metadata(path.string(), cv::FileStorage::READ); // Inspect camera-origin provenance separately from operational geometry.
        require(std::abs(double(metadata["reference"]["camera_to_figure_center_m"]) - .4064) < 1e-12,
                "Saved calibration lost camera reference"); // Save the requested physical reference without altering measured ranges.
        require(std::abs(double(metadata["reference"]["camera_center_height_m"]) - .028) < 1e-12,
                "Saved calibration lost camera height"); // Record camera height separately from the horizontal distance and measured LiDAR surface range.
        metadata.release(); // Close the reader before checking replacement protection and cleanup.
        cv::FileStorage evidence(path.string() + ".measurements.json", cv::FileStorage::READ); // Inspect the companion report used to review physical setup and observed poses.
        require(std::abs(double(evidence["camera_center_height_m"]) - .028) < 1e-12,
                "Measurement report lost camera height"); // Both saved artifacts must describe the same mounted camera geometry.
        evidence.release(); // Release the report before replacement-protection checks and cleanup.
        require(startup_config_path(seed, false) == path, "Existing default calibration was not selected automatically"); // A previously saved calibration must work without requiring the user to capture again.
        activate_calibration(path, active); // Exercise the same activation operation called after the third accepted pose.
        require(startup_config_path(seed, false) == path, "Active calibration was not loaded"); // A normal launch must prefer the recorded calibration over the seed.
        Config revised = fitted; revised.boundary_radius = .08; // Make a valid second calibration distinguishable after replacing the same output file.
        save_calibration(revised, poses, path); activate_calibration(path, active); // Repeated calibration must succeed while retaining backups.
        require(load_config(startup_config_path(seed, false)).boundary_radius == .08, "Repeated calibration was not stored or selected"); // Verify real persisted values rather than only the selected filename.
        bool original_backup = false; // Locate a backup containing the original 3 cm selected boundary.
        for (const auto &file : std::filesystem::directory_iterator(directory)) // Inspect only files in this test's unique output directory.
            if (file.path().filename().string().find("calibration_16in.json.backup.") == 0)
                original_backup = original_backup || load_config(file.path()).boundary_radius == .03; // Verify the old configuration bytes still reconstruct the prior settings.
        require(original_backup, "Repeated calibration did not preserve the previous configuration"); // Backups must be useful configurations, not empty placeholder files.
        const auto custom = directory / "custom profile.json"; // Include a space to verify path serialization and relative-path handling.
        save_calibration(fitted, poses, custom); activate_calibration(custom, active); // A custom output must also become the active default automatically.
        require(startup_config_path(seed, false) == custom, "Custom calibration was not activated"); // No --config argument should be needed after a custom calibration output.
        require(startup_config_path(seed, true) == seed, "Explicit --config did not override saved selection"); // Honor deliberate user selection even when an active record exists.
        const auto absent = directory / "absent.json"; // Model an explicit missing path rather than an implicit default.
        require(startup_config_path(absent, true) == absent, "Explicit missing config was silently replaced"); // Startup must retain missing explicit paths so the normal parser reports an error.
        rejects([&] { activate_calibration(absent, active); }); // Invalid activation attempts must not replace the currently working preference.
        require(startup_config_path(seed, false) == custom, "Failed activation replaced the valid selection"); // Confirm the last successful calibration remains selected after an error.
        Config invalid = fitted; invalid.boundary_radius = .2; // Reject a new configuration that violates the supported boundary range.
        rejects([&] { save_calibration(invalid, poses, custom); }); // Failed validation must happen before replacing the working output.
        require(load_config(custom).boundary_radius == .03, "Invalid save changed the existing calibration"); // Verify the previous operational configuration survives unchanged.
        { std::ofstream corrupt(active); corrupt << "{\"schema_version\":1,\"config_path\":\"absent.json\"}"; } // Simulate an active calibration that has been moved or deleted externally.
        rejects([&] { startup_config_path(seed, false); }); // Missing active data must not silently revert to seed alignment.
        require(startup_config_path(seed, true) == seed, "Explicit config could not bypass a missing active file"); // An explicit seed selection remains a recovery route.
        { std::ofstream corrupt(active); corrupt << "{\"schema_version\":1,\"config_path\":12}"; } // Simulate a malformed active preference.
        rejects([&] { startup_config_path(seed, false); }); // Reject a non-string configuration path before treating it as a filename.
        for (const auto &file : std::filesystem::directory_iterator(directory)) std::filesystem::remove(file.path()); // Remove only files in this uniquely created, nonrecursive test directory.
        std::filesystem::remove(directory); // Remove the now-empty test directory without touching any shared project paths.
        std::unique_ptr<void, decltype(&core_destroy)> core(core_create(&fitted), core_destroy); // Validate the fitted settings through the actual portable tracking engine.
        require(bool(core), "Fitted configuration cannot initialize tracking"); // A nominally fitted file must be accepted by the runtime.
        Result result{}; Inputs inputs; Frame frame; Control control{}; // Prepare real segmentation/clustering observations without hardware.
        for (int sequence = 1; sequence <= 2; ++sequence) { // Supply enough unique paired scans to pass normal acquisition confirmation.
            const double now = 100 + sequence * .1; // Advance deterministic host receive times without waiting.
            frame = synthetic_frame(fitted, {0, .38}, now, sequence); // Render a marker for a position nearer than the original metric ROI admitted.
            inputs.scan = synthetic_scan(fitted, {0, .38}, now, sequence); // Generate unaltered sensor-relative ranges for that nearby surface.
            require(core_step(core.get(), frame.pixels->data(), 640, 480, 1920, now,
                inputs.scan.points.data(), int(inputs.scan.points.size()), now, sequence, now, &control, &result) == 0,
                "Fitted tracking update failed"); // Exercise the full production association path with the new anchors.
        }
        require(result.track_status == TRACKING && std::abs(result.xy.y - .38) < .001,
                "Fitted tracker rejected or rescaled nearby surface"); // Verify usable near-row tracking and preservation of physical LiDAR distances.
        auto preview = calibration_preview(calibration_capture_config(fitted), inputs, frame, result, 0, 0, 20, "Synthetic test preview: stationary measured-pair window ready."); // Render coherent synthetic camera/scan coordinates without opening a desktop or device.
        require(preview.cols == 1280 && preview.rows == 800, "Calibration preview dimensions"); // Preserve the fixed mouse-coordinate/display contract.
        require(cv::imwrite("calibration_preview.png", preview), "Cannot save calibration preview"); // Produce a local visual QA artifact from the actual renderer.
        std::cout << "Calibration tests passed: measured anchors, separate sensor origins, stability, invalid poses, persistence, near-row tracking, rendering\n";
        return 0; // Return success only after all physical-reference and measured-data invariants pass.
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n'; // Report the first failed invariant to CTest.
        return 1; // Propagate validation failures to the build rather than publishing an unverified package.
    }
}
