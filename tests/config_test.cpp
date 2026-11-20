/**
 * @file
 * Application-layer checks for parsing, persistence, and rendering.
 * Temporary files exercise roundtrip serialization, invalid settings, and
 * boundary sidecars. Rendering is tested without opening a desktop window.
 */

#include "../src/app.h"
#include <opencv2/highgui.hpp> // Mouse event constants let widget tests exercise the same click and drag inputs as the live window.
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
#include <iterator> // Stream-buffer iterators read a complete saved configuration for the omitted-field fixture.
#include <limits> // Nonfinite values exercise pacing validation before settings can reach the core.
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
            copy.margin != c.margin || copy.roi[3] != c.roi[3] || copy.speed != c.speed ||
            copy.speed_scale != c.speed_scale) // Check calibration, geometry, ROI, and separate motion settings survive serialization unchanged.
            throw std::runtime_error("Roundtrip mismatch");
        copy.speed_scale = .5; // Select a custom pace so persistence cannot accidentally substitute the 80% default.
        for (int i = 0; i < 2; ++i) { // Repeat saving/loading to catch accidental multiplication of already-scaled speed.
            save_config(copy, file); // Use the same writer as calibrated and effective session configurations.
            copy = load_config(file); // Restore both nominal speed and its separate pacing fraction.
            if (copy.speed != c.speed || copy.speed_scale != .5) // Saving must not reduce the nominal speed or reset a deliberate custom pace.
                throw std::runtime_error("Speed compounded or custom pace lost on save");
        }
        std::string omitted; // Hold a complete otherwise-valid configuration without the optional pacing field.
        { std::ifstream input(file); omitted.assign(std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()); } // Close the reader before rewriting the fixture on every platform.
        const auto key = omitted.find("\"speed_scale\""); // Locate only the optional JSON key emitted by the production writer.
        const auto comma = omitted.find(',', key); // Preserve all neighboring settings while removing that single member.
        if (key == std::string::npos || comma == std::string::npos) // Fail explicitly if serialization no longer supports this fixture construction.
            throw std::runtime_error("Cannot construct omitted pacing fixture");
        omitted.erase(key, comma - key + 1); // Leave a valid robot object with speed but no speed_scale member.
        { std::ofstream output(file); output << omitted; } // Publish the fixture after the writer is closed so the parser sees all bytes.
        copy = load_config(file); // Exercise automatic default pacing for a saved calibration without an explicit fraction.
        std::filesystem::remove(file); // Remove only the test's temporary roundtrip fixture.
        if (copy.speed != c.speed || copy.speed_scale != .8 || copy.anchors[4] != c.anchors[4]) // Default pacing must preserve measured alignment and nominal speed.
            throw std::runtime_error("Omitted pacing did not preserve calibration and select 80%");
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
        for (double scale : {0., -.1, 1.01, std::numeric_limits<double>::infinity(), std::numeric_limits<double>::quiet_NaN()}) { // Pacing accepts only finite fractions in (0,1].
            bad = c; // Isolate each invalid pacing value from all other valid settings.
            bad.speed_scale = scale; // Stage an invalid fraction without mutating the reference fixture.
            invalid(bad); // Exercise the production validator for each unsupported value.
        }
        if (rejected != 10) // All five geometry/calibration and five pacing fixtures must have been rejected.
            throw std::runtime_error("Invalid configuration accepted");
        auto boundary_file = std::filesystem::current_path() / "boundary_roundtrip_test.json"; // Use a separate named temporary file for radius-only persistence checks.
        Config loaded = c; // Start sidecar loading from the untouched reference calibration.
        for (double radius : {.03, .072, .10}) { // Roundtrip both inclusive endpoints and the usual starting selection.
            Config adjusted = c; // Preserve the experiment's calibration and nominal body dimensions.
            adjusted.boundary_radius = radius; // Select the total radius directly, including values below the nominal body sum.
            save_boundary_setting(adjusted, boundary_file); // Persist with the same sidecar writer used by Save/B.
            if (!load_boundary_setting(loaded, boundary_file) ||
                std::abs(exclusion_radius(loaded) - radius) > 1e-12 || loaded.forward != c.forward ||
                loaded.anchors[4] != c.anchors[4] || loaded.roi[3] != c.roi[3] ||
                loaded.figure_radius != c.figure_radius || loaded.robot_radius != c.robot_radius || loaded.margin != c.margin) // Restoring a total-radius preference must not rewrite calibration or nominal footprints.
                throw std::runtime_error("Boundary persistence changed calibration or lost radius");
            save_config(loaded, file); // Session configuration must also retain the resolved explicit selection.
            if (std::abs(exclusion_radius(load_config(file)) - radius) > 1e-12) // Verify startup parsing reconstructs the effective boundary from a full saved configuration.
                throw std::runtime_error("Effective configuration lost selected boundary");
            std::filesystem::remove(file); // Remove only the named temporary full-config fixture.
        }
        for (double radius : {0., .029999, .100001, .2}) { // Check immediate neighbors of the allowed range and other invalid persisted values.
            {
                std::ofstream invalid_boundary(boundary_file); // Replace only the temporary preference with a deliberately invalid total radius.
                invalid_boundary << "{\"schema_version\":1,\"radius_m\":" << radius << "}"; // Close the fixture before production parsing reads it.
            }
            bool rejected_boundary = false; // Each invalid value must fail independently without mutating the current 10 cm preference.
            try {
                load_boundary_setting(loaded, boundary_file); // Exercise the actual restart preference loader.
            } catch (const std::exception &) {
                rejected_boundary = true; // Record the expected validation failure.
            }
            if (!rejected_boundary || std::abs(exclusion_radius(loaded) - .10) > 1e-12) // Verify transactional failure preserves the previously accepted geometry.
                throw std::runtime_error("Invalid boundary settings changed active geometry");
        }
        std::filesystem::remove(boundary_file); // Remove the test sidecar so the next check also exercises normal missing-file behavior.
        if (load_boundary_setting(loaded, boundary_file)) // Missing preferences are normal and must not report a successful load.
            throw std::runtime_error("Missing boundary settings mishandled");
        BoundaryUi ui(c); // Exercise the actual widget used by keyboard, buttons, and mouse dragging.
        if (ui.minimum_mm != 30 || ui.maximum_mm != 100 || ui.current_mm.load() != 72) // Limits must be fixed while the default remains 7.2 cm.
            throw std::runtime_error("Wrong initial boundary controls");
        ui.adjust(-1000); // Simulate enough keyboard/button decrements to move beyond the lower endpoint.
        if (ui.requested_mm.load() != 30) // Requests must clamp at exactly 3 cm.
            throw std::runtime_error("Boundary decrement escaped minimum");
        ui.adjust(1000); // Simulate repeated increments beyond the upper endpoint.
        if (ui.requested_mm.load() != 100) // Requests must clamp at exactly 10 cm.
            throw std::runtime_error("Boundary increment escaped maximum");
        ui.mouse(cv::EVENT_LBUTTONDOWN, 445, 736, 0); // Select the visible left endpoint of the real slider.
        if (ui.requested_mm.load() != 30) // Clicking the minimum must request the same radius as keyboard adjustments.
            throw std::runtime_error("Slider minimum mismatch");
        ui.mouse(cv::EVENT_MOUSEMOVE, 900, 736, cv::EVENT_FLAG_LBUTTON); // Drag past the right endpoint while holding the mouse button.
        if (ui.requested_mm.load() != 100) // Out-of-track drags must remain bounded at 10 cm.
            throw std::runtime_error("Slider maximum mismatch");
        ui.mouse(cv::EVENT_LBUTTONUP, 900, 736, 0); // Release the gesture so later mouse movement cannot keep changing the radius.
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
        std::cout << "Config and pacing roundtrips, ten invalid-config checks, boundary persistence and "
                     "native rendering passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
