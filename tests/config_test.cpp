/**
 * @file
 * Application-layer checks for parsing, persistence, and rendering.
 * Temporary files exercise roundtrip serialization, invalid settings, and
 * boundary sidecars. Rendering is tested without opening a desktop window.
 */

#include "../src/app.h"
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
using namespace tabletop;
int main(int argc, char **argv) {
    try {
        if (argc != 2) // Require the fixture configuration path supplied by CTest.
            throw std::runtime_error("Expected configuration path");
        Config c = load_config(argv[1]); // Parse and validate the repository's actual JSON configuration before testing persistence.
        auto file = std::filesystem::current_path() / "config_roundtrip_test.json"; // Keep the temporary roundtrip fixture inside the test's working directory.
        save_config(c, file); // Serialize resolved settings through the production writer.
        Config copy = load_config(file); // Read the serialized file through the production parser for a real roundtrip check.
        std::filesystem::remove(file); // Remove only this test's named temporary roundtrip file.
        if (copy.forward != c.forward || copy.anchors[4] != c.anchors[4] ||
            copy.margin != c.margin || copy.roi[3] != c.roi[3]) // Check representative calibration, geometry, and ROI fields survive serialization unchanged.
            throw std::runtime_error("Roundtrip mismatch");
        int rejected = 0; // Count invalid fixtures that actually throw from production validation.
        auto invalid = [&](Config bad) { // Validate a supplied modified copy and count rejection without terminating the remaining cases.
            try {
                validate_config(bad); // Exercise production cross-field validation for one deliberately malformed fixture.
            } catch (const std::exception &) {
                ++rejected; // Count this fixture only when validation throws as expected.
            }
        };
        auto bad = c; // Start with a valid configuration so each case isolates one invalid field.
        bad.roi[2] = 641; // Place the camera ROI one pixel beyond the 640-pixel image boundary.
        invalid(bad); // Check rejection of the one deliberately invalid field in this fixture.
        bad = c; // Restore all valid fields before introducing the next independent error.
        bad.hsv_high[0] = 255; // Hue uses 0..179, so 255 must fail even though it is valid for S/V channels.
        invalid(bad); // Check rejection of the one deliberately invalid field in this fixture.
        bad = c; // Restore all valid fields before introducing the next independent error.
        bad.min_area = -1; // A negative connected-component area threshold is invalid.
        invalid(bad); // Check rejection of the one deliberately invalid field in this fixture.
        bad = c; // Restore all valid fields before introducing the next independent error.
        bad.goal.y = 2; // Place the delivery point outside the configured tabletop lane.
        invalid(bad); // Check rejection of the one deliberately invalid field in this fixture.
        bad = c; // Restore all valid fields before introducing the next independent error.
        bad.anchors[3] = -30; // Make the middle bearing violate the required left-to-right increasing order.
        invalid(bad); // Check rejection of the one deliberately invalid field in this fixture.
        if (rejected != 5) // All five independent invalid fixtures must have been rejected.
            throw std::runtime_error("Invalid configuration accepted");
        auto boundary_file = std::filesystem::current_path() / "boundary_roundtrip_test.json"; // Use a separate named temporary file for radius-only persistence checks.
        Config adjusted = c; // Copy calibration before changing only the boundary margin.
        adjusted.margin = .023; // Choose 23 mm of extra clearance as a distinct roundtrip value.
        save_boundary_setting(adjusted, boundary_file); // Persist the total radius using the production sidecar writer.
        Config loaded = c; // Start sidecar loading from the untouched reference calibration.
        if (!load_boundary_setting(loaded, boundary_file) ||
            std::abs(loaded.margin - .023) > 1e-12 || loaded.forward != c.forward ||
            loaded.anchors[4] != c.anchors[4] || loaded.roi[3] != c.roi[3]) // Require successful radius recovery while preserving forward calibration, anchors, and image ROI.
            throw std::runtime_error("Boundary persistence changed calibration or lost radius");
        {
            std::ofstream invalid_boundary(boundary_file); // Replace only this temporary test sidecar with a deliberately invalid radius fixture.
            invalid_boundary << "{\"schema_version\":1,\"radius_m\":0.001}"; // A 1 mm total radius is smaller than the two configured body radii.
        }
        bool rejected_boundary = false; // Track whether the malformed sidecar raises a validation exception.
        try {
            load_boundary_setting(loaded, boundary_file); // Attempt to load the invalid fixture into an already valid configuration.
        } catch (const std::exception &) {
            rejected_boundary = true; // Record the expected exception without replacing the existing valid radius.
        }
        std::filesystem::remove(boundary_file); // Remove the test sidecar so the next check also exercises normal missing-file behavior.
        if (!rejected_boundary || std::abs(loaded.margin - .023) > 1e-12 || // Require rejection, no partial mutation, and a false return for the now-missing optional file.
            load_boundary_setting(loaded, boundary_file)) // Attempt to load the invalid fixture into an already valid configuration.
            throw std::runtime_error("Invalid/missing boundary settings mishandled");
        Inputs in; // Prepare a device-free observation snapshot for headless rendering validation.
        in.frames.push_back(synthetic_frame(c, {0, .63}, 100, 1)); // Generate a valid synthetic camera image at a known centerline figure position.
        in.scan = synthetic_scan(c, {0, .63}, 100, 1); // Generate the matching raw LiDAR scan at the same host receive time.
        auto frame = paired_frame(in); // Select the camera frame through the same pairing helper used by the live application.
        Result result{}; // Initialize a minimal result for a render-only test without running a delivery.
        result.robot = c.start; // Place the displayed robot at the depot from the fixture configuration.
        result.lane_width = c.lane_width; // Supply the configured wide-lane geometry to the renderer.
        result.selected = -1; // Ensure the renderer does not read a nonexistent selected cluster.
        auto screen = render(c, in, frame, result, {}, false, true, false, 100, 1); // Render simulation mode into memory without opening an OpenCV window.
        if (screen.rows != 800 || screen.cols != 1280) // Require the stable 1280x800 canvas contract used by controls and saved previews.
            throw std::runtime_error("Render dimensions");
        std::cout << "Config roundtrip, five invalid-config checks, boundary persistence and "
                     "native rendering passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
