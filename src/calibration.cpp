/**
 * @file
 * Interactive native capture for a figure centered 16 inches from the camera.
 * The user explicitly selects the figure's LiDAR cluster for each placement.
 * Existing camera-bearing anchors are not used to select calibration targets.
 */
#include "calibration.h"
#include <opencv2/highgui.hpp> // Window events and keyboard input for selecting and accepting measured poses.
#include <opencv2/imgproc.hpp> // Draw the camera overlay, cluster map, and capture instructions.
#include <iostream> // Report saved calibration paths and cancellation to the launching terminal.
#include <optional> // Represent an explicitly selected target separately from an arbitrary default position.
#include <thread> // Pace sensor polling without keeping the CPU continuously busy.

namespace tabletop {
namespace {
const cv::Scalar white(225, 230, 232), muted(145, 153, 158), cyan(225, 195, 80), green(140, 225, 135), amber(90, 190, 245); // Match the application's presentation colors.
const cv::Rect map_box(730, 105, 535, 565); // Only clicks inside this map can select a LiDAR cluster.
cv::Point point(XY p) { // Map metric coordinates into the calibration-only selection panel.
    return {1000 + int(std::lround(p.x * 420)), 660 - int(std::lround(p.y * 450))}; // Display x right and y forward with enough range to include the closer row.
}
void label(cv::Mat &image, const std::string &text, int x, int y, double scale = .5, cv::Scalar color = white) { // Draw one short status or instruction line.
    cv::putText(image, text, {x, y}, cv::FONT_HERSHEY_SIMPLEX, scale, color, 1, cv::LINE_AA); // Keep text independent of capture and planner state.
}
struct Click { // The callback only posts a selection request; the sensor loop owns capture state.
    bool pending = false; // True until the latest click is consumed by the main calibration loop.
    cv::Point location; // Cursor position in the fixed calibration canvas.
};
bool fresh_pair(const Config &c, const Inputs &in, const Frame &frame, double now) { // Require currently usable observations before selecting or recording a target.
    return frame && in.scan.sequence >= 0 && frame.received <= now && in.scan.received <= now &&
           now - frame.received < c.fresh && now - in.scan.received < c.fresh &&
           std::abs(frame.received - in.scan.received) <= c.pair_gap; // Compare host receive times using the same freshness/pairing contract as normal tracking.
}
} // namespace
cv::Mat calibration_preview(const Config &capture, const Inputs &inputs, const Frame &frame,
                            const Result &r, int stage, int selected, int samples,
                            const std::string &message) { // Compose the calibration UI without starting inference or a virtual delivery.
    cv::Mat image(800, 1280, CV_8UC3, cv::Scalar(24, 26, 29)); // Keep a fixed canvas so mouse hit testing matches displayed coordinates.
    label(image, "CALIBRATION | 16 INCH CAMERA ROW", 18, 34, .75); // Identify the physical reference used for this capture workflow.
    label(image, "Figure center: 16 in / 40.64 cm forward of the camera lens, measured horizontally.", 18, 65, .51, cyan); // State the origin and direction of the ruler measurement explicitly.
    label(image, "Camera center height: " + std::to_string(int(std::lround(CALIBRATION_CAMERA_HEIGHT_M * 1000))) + " mm above the tabletop", 18, 83, .38, muted); // Show the physical camera height without adding a false height correction to LiDAR measurements.
    label(image, "CAMERA / BLUE MARKER", 18, 99, .44, muted); // Identify the image used for measured marker centroids.
    label(image, "CLICK THE FIGURE'S LIDAR CLUSTER", 740, 99, .44, cyan); // Make manual association explicit instead of relying on the existing bearing map.
    if (frame) { // Display only an owned, correctly sized frame.
        cv::Mat camera(480, 640, CV_8UC3, const_cast<unsigned char *>(frame.pixels->data())); // Wrap immutable pixels as a read-only source for the canvas copy.
        camera.copyTo(image(cv::Rect(18, 110, 640, 480))); // Copy into the view before adding any overlays.
    }
    for (int i = 0; i < r.marker_count; ++i) { // Show every candidate so multiple-marker ambiguity is visible.
        const auto &m = r.markers[i]; // Use measured component geometry from the portable segmentation engine.
        cv::rectangle(image, {18 + m.x, 110 + m.y, m.w, m.h}, r.marker_count == 1 ? green : amber, 2); // A unique green marker is required for recording.
    }
    for (int cm = 0; cm <= 120; cm += 20) { // Draw context lines in LiDAR-relative centimeters, separate from the camera placement instruction.
        const int y = point({0, cm / 100.}).y; // Convert metric grid depth to panel pixels.
        cv::line(image, {748, y}, {1250, y}, cv::Scalar(55, 58, 61)); // Draw a subdued horizontal reference row.
        label(image, std::to_string(cm) + " cm", 745, y - 4, .32, muted); // Label LiDAR-map distances without claiming they equal camera-to-center distance.
    }
    cv::line(image, point({0, 0}), point({0, 1.2}), cv::Scalar(75, 78, 82)); // Mark the provisional LiDAR-forward axis used only to select clusters.
    for (const auto &p : inputs.scan.points) { // Show surrounding raw returns so the user can identify the moving figure cluster.
        if (!std::isfinite(p.angle) || !std::isfinite(p.range) || p.range <= 0 || p.range > 1.3 || p.quality < 1) // Skip invalid or out-of-panel context points before coordinate conversion.
            continue;
        const double a = (p.angle - capture.forward) * 3.141592653589793 / 180; // Rotate raw sensor angles into the capture map's provisional frame.
        const auto pixel = point({p.range * std::sin(a), p.range * std::cos(a)}); // Convert the measured polar surface into map pixels.
        if (map_box.contains(pixel)) // Avoid drawing off-panel returns over instructions or the camera image.
            cv::circle(image, pixel, 1, muted, cv::FILLED);
    }
    for (int i = 0; i < r.group_count; ++i) { // Give each eligible compact cluster a visible selection target.
        const auto p = point(r.groups[i].xy); // Locate the cluster's observed surface centroid.
        if (!map_box.contains(p)) // A valid off-screen cluster is not selectable in this view.
            continue;
        cv::circle(image, p, i == selected ? 12 : 6, i == selected ? amber : green, 2, cv::LINE_AA); // Highlight only the cluster explicitly chosen for this placement.
        label(image, std::to_string(i + 1), p.x + 10, p.y - 8, .38, i == selected ? amber : green); // Label cluster IDs for visual confirmation.
    }
    label(image, "Map distances use the LiDAR origin; the ruler starts at the camera.", 18, 616, .45, muted); // Keep the two origins distinct during physical setup.
    const char *instructions[]{"CENTER: place figure center on the 16-inch camera-forward mark.",
                               "LEFT: slide about 10 cm left along the same forward row.",
                               "RIGHT: slide about 10 cm right of center on the same row.",
                               "Three poses captured. S retries a failed save; R restarts capture."}; // Completed captures save automatically; this state remains visible only if saving needs attention.
    label(image, instructions[std::clamp(stage, 0, 3)], 18, 653, .53, cyan); // Show exactly the currently requested action.
    label(image, "Center " + std::string(stage > 0 ? "OK" : "--") + "    Left " + (stage > 1 ? "OK" : "--") +
                 "    Right " + (stage > 2 ? "OK" : "--") + "    Paired samples: " + std::to_string(samples) + "/20", 18, 691, .52); // Summarize completed placements; the message separately reports whether this window is stable.
    label(image, message.substr(0, 112), 18, 731, .48, amber); // Keep actionable capture feedback within the fixed canvas width.
    label(image, "Click cluster | SPACE capture (third pose auto-saves) | R restart | S retry save | Q/Esc cancel", 18, 775, .47); // Explain automatic persistence while retaining retry/cancel controls and no robot start command.
    return image; // Return an owned preview suitable for display and capture evidence.
}
int run_calibration(const Config &base, const std::string &port, int camera,
                    const std::filesystem::path &destination, const std::filesystem::path &logs,
                    const volatile std::sig_atomic_t &quit) { // Capture real data without loading Coral or allowing virtual motion.
    if (std::filesystem::absolute(destination).lexically_normal() == std::filesystem::absolute("active_calibration.json").lexically_normal()) // The selected configuration must remain separate from its startup preference record.
        throw std::runtime_error("Calibration output cannot be active_calibration.json");
    if (std::filesystem::exists(destination) && !std::filesystem::is_regular_file(destination)) // Repeated regular-file saves are supported, but a directory cannot be a calibration output.
        throw std::runtime_error("Calibration output must be a regular file");
    const Config capture = calibration_capture_config(base); // Admit the closer row and enlarged image marker for diagnostic acquisition.
    std::unique_ptr<void, decltype(&core_destroy)> core(core_create(&capture), core_destroy); // Reuse segmentation and clustering but not the existing automatic target association.
    if (!core) // Fail before using an invalid opaque engine handle.
        throw std::runtime_error("Cannot initialize calibration observation engine");
    Sensors sensors(port, camera); // Own the real camera and LiDAR for the lifetime of this calibration session.
    SessionLog log(logs / "calibration_16in", capture, false, false); // Retain raw scans and camera snapshots when the user accepts each pose.
    const std::string window = "Tabletop Delivery - 16 inch calibration"; // Use a separate window title to distinguish capture from a delivery run.
    cv::namedWindow(window, cv::WINDOW_AUTOSIZE); // Fix canvas geometry so click positions correspond to the visible map.
    Click click; // Keep the callback request alive until after the window is destroyed.
    cv::setMouseCallback(window, [](int event, int x, int y, int, void *context) { // Post clicks without touching sensors, samples, or the planner inside the callback.
        if (event == cv::EVENT_LBUTTONDOWN) { // Ignore releases and passive motion when choosing a cluster.
            auto &request = *static_cast<Click *>(context); // Recover the session-owned click request.
            request.location = {x, y}; request.pending = true; // Record the next user-selected map location for the main loop.
        }
    }, &click);
    struct CloseWindow { // Destroy the window before callback state and sensor ownership leave scope, including on exceptions.
        std::string name; // Own the title needed during deterministic cleanup.
        ~CloseWindow() { try { cv::destroyWindow(name); } catch (const cv::Exception &) {} } // Remove the callback before session cleanup; tolerate a window already closed by its title-bar button.
    } close{window};
    std::array<CalibrationPose, 3> poses{}; // Accumulate center, left, and right measurements separately.
    std::vector<CalibrationSample> samples; // Maintain a rolling set of distinct, stationary paired observations.
    std::optional<XY> selection; // No cluster is selected until the user explicitly clicks one.
    Result result{}; // Hold the latest segmentation/clustering diagnostics from the portable core.
    int stage = 0, used_scan = -1, selected = -1; // Track the requested pose and suppress repeated scan observations.
    std::uint64_t used_frame = 0; // A camera image may contribute at most once to the stability window.
    std::string message = "Hold the figure still, remove your hand, then click its green LiDAR cluster."; // Initial guidance avoids collecting hand-contaminated moving samples.
    while (!quit) { // Honor the application's orderly Ctrl+C/SIGTERM request.
        Inputs inputs = sensors.snapshot(); // Copy coherent scan and image history without keeping hardware locks during UI work.
        if (!inputs.error.empty()) // Report acquisition errors rather than continuing to collect stale evidence.
            throw std::runtime_error(inputs.error);
        Frame frame = paired_frame(inputs); // Select the nearest host-receive image for the latest completed scan.
        const double now = clock_now(); // Use one monotonic timestamp for freshness and core processing.
        Control ctl{}; // Never request start, lane changes, or semantic motion during calibration.
        if (core_step(core.get(), frame ? frame.pixels->data() : nullptr, 640, 480, 1920,
                      frame ? frame.received : -1, inputs.scan.points.data(), int(inputs.scan.points.size()),
                      inputs.scan.received, inputs.scan.sequence, now, &ctl, &result)) // Obtain candidate markers/clusters even if the existing bearing association is wrong.
            throw std::runtime_error("Calibration observation processing failed");
        const bool fresh = fresh_pair(capture, inputs, frame, now); // A visible historical cluster alone cannot authorize capture.
        if (click.pending) { // Consume a queued user selection on the same thread that owns the sample window.
            click.pending = false; // Handle each click once.
            if (stage < 3 && fresh && map_box.contains(click.location)) { // Accept selections only while a placement still needs real, fresh evidence.
                double best = 22; int chosen = -1; // Require a click close to an actual plotted compact cluster.
                for (int i = 0; i < result.group_count; ++i) { // Pick the nearest displayed cluster to the explicit click.
                    const double distance = cv::norm(point(result.groups[i].xy) - click.location); // Measure screen distance rather than assuming a raw cluster index stays stable.
                    if (distance < best) { best = distance; chosen = i; } // Retain the closest eligible selection target.
                }
                selection.reset(); samples.clear(); // Clear old evidence when selecting a new surface.
                if (chosen >= 0) selection = result.groups[chosen].xy; // Anchor future samples to the actual clicked surface position.
                message = chosen >= 0 ? "Selected cluster. Hold still while fresh pairs accumulate." : "Click closer to the figure's green cluster."; // Explain whether selection succeeded.
            }
        }
        selected = -1; // Re-identify the selected stationary surface on each scan; cluster indices can change.
        if (selection && fresh && stage < 3) { // Association during calibration is based on the user's stationary map selection, not old image-bearing anchors.
            int count = 0; // Multiple nearby surfaces are ambiguous even after an explicit click.
            for (int i = 0; i < result.group_count; ++i)
                if (std::hypot(result.groups[i].xy.x - selection->x, result.groups[i].xy.y - selection->y) < .025) { // Follow only a compact cluster within 2.5 cm of the manually chosen location.
                    selected = i; ++count; // Count matches instead of guessing if the figure merges with another surface.
                }
            if (count != 1) selected = -1; // Require a unique stationary cluster near the selection.
        }
        if (stage < 3 && (!fresh || result.marker_count != 1 || selected < 0)) { // Break the stability window on missing, stale, multiple, or ambiguous evidence.
            samples.clear(); // Do not combine observations across a loss or selection ambiguity.
            message = !fresh ? "Waiting for fresh paired camera and LiDAR observations." : result.marker_count != 1 ?
                "Exactly one blue marker is required. Check lighting and remove other blue objects." :
                "Click the figure's cluster; keep it separated from nearby objects."; // Give feedback for the actual capture gate that failed.
        } else if (stage < 3 && inputs.scan.sequence != used_scan && frame.sequence != used_frame) { // Count each scan and image at most once, regardless of UI refresh rate.
            used_scan = inputs.scan.sequence; used_frame = frame.sequence; // Mark the consumed pair before updating the rolling window.
            if (samples.size() == CALIBRATION_SAMPLES) samples.erase(samples.begin()); // Keep the most recent twenty pairs for stationary-spread checks.
            samples.push_back({result.markers[0], result.groups[selected], capture.forward + result.groups[selected].bearing}); // Preserve directly observed pixel and raw-angle pairs.
            try { summarize_calibration(samples); message = "Stable capture ready. Press SPACE to accept this placement."; } // Enable acceptance only after the complete window passes validation.
            catch (const std::exception &e) { message = e.what(); } // Keep collecting so transient motion can age out of the stability window.
        }
        auto screen = calibration_preview(capture, inputs, frame, result, stage, selected, int(samples.size()), message); // Render exactly the candidate state and instruction currently used by capture.
        cv::imshow(window, screen); // Display the live paired image, map, and capture progress.
        const int key = cv::waitKey(20) & 255; // Service window events while bounding the polling rate to about 50 Hz.
        bool save_requested = false; // Request one save after the third accepted pose, avoiding repeated writes after an error.
        if (key == 'q' || key == 'Q' || key == 27 || cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) break; // Cancel without changing the working configuration.
        if (key == 'r' || key == 'R') { // Let the user repeat all measurements after an incorrect selection or placement.
            stage = 0; poses = {}; samples.clear(); selection.reset(); // Discard incomplete calibration state while keeping devices and the input configuration intact.
            message = "Restarted. Place the figure center 16 inches forward of the camera lens."; // Restore center-placement guidance.
        }
        if (key == ' ' && stage < 3) { // Accept a physical placement only on the user's explicit capture action.
            try {
                if (!fresh || selected < 0 || result.marker_count != 1) throw std::runtime_error("Fresh, unique marker and selected cluster required"); // Recheck the live gate at acceptance time.
                poses[stage] = summarize_calibration(samples); // Reject incomplete or unstable evidence before committing a pose.
                log.snapshot(screen, frame, inputs.scan, result, {}); // Preserve camera pixels and unaltered scan returns for this accepted placement.
                ++stage; samples.clear(); selection.reset(); // Require a new manual selection and stability window for the next position.
                save_requested = stage == 3; // The third accepted placement automatically triggers validation, persistence, and activation.
                message = save_requested ? "Validating and saving calibration automatically..." : "Move to the next position, remove your hand, and click its LiDAR cluster."; // Advance physical instructions without requiring a separate successful-save key press.
            } catch (const std::exception &e) { message = e.what(); } // Keep the interactive session alive so the user can correct invalid evidence.
        }
        if (save_requested || key == 's' || key == 'S') { // Automatically save a complete capture; S explicitly retries a failed save without recapturing measurements.
            try {
                if (stage != 3) throw std::runtime_error("Capture center, left, and right before saving"); // Never publish partially measured anchors.
                Config calibrated = fit_calibration(base, poses); // Check ordering, stability, common row, and complete configuration invariants.
                save_calibration(calibrated, poses, destination); // Publish complete measured outputs, retaining backups when replacing an existing calibration.
                activate_calibration(destination); // Make this validated output the configuration selected by a normal launch without --config.
                std::cout << "Calibration saved: " << std::filesystem::absolute(destination) << '\n'
                          << "Calibration is now active. Run: ./build/tabletop_delivery\n"; // Confirm both persistence and activation before reporting a successful completion.
                return 0; // Normal scope exit stops acquisition and destroys the capture window.
            } catch (const std::exception &e) { message = std::string(e.what()) + " | S retry, R restart"; std::cerr << "Calibration save/activation failed: " << e.what() << '\n'; } // Retain measurements and report any failure in both the window and terminal without claiming activation succeeded.
        }
    }
    std::cout << "Calibration capture closed.\n"; // Do not claim a successful save, or deny a completed file write if a later activation step failed.
    return 0; // Leave the user's working configuration untouched on cancellation.
}
} // namespace tabletop
