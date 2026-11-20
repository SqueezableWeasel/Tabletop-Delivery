/**
 * @file
 * Application entry point and single-threaded coordinator.
 * Loads explicit/local settings, starts selected backends, and consumes bounded
 * sensor snapshots. Only this thread mutates the tracker/planner or applies UI
 * geometry changes. All times passed to the core use the steady clock.
 */

#include "app.h"
#include "evaluation.h" // Exact detector-frame pairing, independent comparison, and measurement logs.
#include "calibration.h" // Native stationary capture derives new anchors at the camera-referenced 16-inch row.
#include <opencv2/highgui.hpp>
#include <csignal> // Signal-safe termination flag and registration of interrupt/termination handlers.
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <thread> // Background acquisition/inference workers or main-loop pacing.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
namespace {
volatile std::sig_atomic_t quit_requested = 0; // Signal-handler communication uses a signal-safe integer rather than performing cleanup in the handler.
void signal_handler(int) { // The signal handler only assigns a sig_atomic_t flag; cleanup runs normally on the main thread after the loop observes the request.
    quit_requested = 1; // Ask the normal main loop to exit so destructors can release hardware safely.
}
struct Options { // CLI values are independent of neighboring folders. Relative paths resolve against the launch directory; run from the repository root by default.
    std::string tracking_mode = "marker"; // marker controls the working demo; compare shadows YOLO; yolo permits detector control.
    tabletop::DetectorOptions detector; // Select the native inference tensor contract and person threshold.
    bool model_explicit = false, detector_explicit = false, profile_explicit = false, confidence_explicit = false;
    bool demo = false, headless = false, auto_start = false, no_coral = false, check_coral = false; // Default to live, windowed operation with required Coral and manual robot start.
    bool calibrate = false; // Select live calibration without starting a delivery or opening the Coral runtime.
    bool config_explicit = false; // Distinguish a deliberate --config choice from default automatic calibration selection.
    std::string calibration_output = "calibration_16in.json"; // Default measured-calibration output; repeat saves replace it after validation and retain backups.
    double seconds = 0; // Zero means no time limit in windowed mode; headless mode requires a positive duration.
    int camera = 0; // Select the first enumerated camera unless the user provides another index.
    std::string config = "delivery_config.json", tracking, port,
                model = "models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite"; // Repository-relative settings and detector paths; optional overrides begin empty.
    std::string library = "native_runtime/libedgetpu.so.1", output = "delivery_runs"; // Use the project-private matching delegate and keep session output under the run directory.
};
void help() { // Describe the live, simulated, and diagnostic entry points without starting devices.
    std::cout
        << "Tabletop Delivery\n"
           "  ./build/tabletop_delivery                   live run using the saved active calibration\n"
           "  ./build/tabletop_delivery --check-coral     real native TPU warmup, then exit\n"
           "  ./build/tabletop_delivery --demo            synthetic sensors, Coral OFF\n"
           "  ./build/tabletop_delivery --calibrate       capture three poses, automatically save and activate\n"
           "  --calibration-output FILE                  saved calibration file (default calibration_16in.json)\n"
           "  --headless --seconds N                     bounded run, save final snapshot\n"
           "  --auto-start                               start robot without pressing Space\n"
           "  --no-coral                                 explicitly disable semantic hold\n"
           "  --config PATH  --tracking-config PATH       configuration/calibration override\n"
           "  --port /dev/serial/by-id/... --camera-index N\n"
           "  --model FILE --edgetpu-lib FILE --output DIR\n"
           "Detector and tracking modes:\n"
           "  --tracking marker|compare|yolo              marker is default; compare keeps marker control\n"
           "  --detector ssd|yolov8                       compare/yolo select YOLO automatically\n"
           "  --yolo-profile coco|figure                  figure selects the custom 2-class model\n"
           "  --target-confidence 0.25                    figure threshold; alias: --person-confidence\n"
           "  --person-confidence 0.35                    minimum COCO person score\n"
           "  --person-max-width 0.30 --person-max-height 0.65   target box fractions of camera image\n"
           "  --yolo-coordinates normalized|pixels        explicit exported box convention\n"
           "Keys: Space start/pause, R reset, N lane, M mask, S snapshot, Q/Esc quit\n"
           "Figure boundary: click/drag slider or -/+; [/] adjust 0.1 cm; B/Save remembers it\n"; // Report application progress or state to the launching terminal; output does not drive decisions.
}
Options parse(int argc, char **argv) { // Parse options and reject unsafe/contradictory combinations before acquiring hardware. A headless run needs a positive duration so it terminates predictably.
    Options o; // Begin parsing from the explicit live-mode defaults.
    for (int i = 1; i < argc; ++i) { // Skip argv[0], which is the executable name, and examine user-supplied arguments.
        std::string a = argv[i]; // Copy this argument so comparisons and error messages do not depend on a mutable index.
        auto value = [&]() { // Consume exactly one following argument for an option that requires a value.
            if (++i >= argc) // Advance to the value and reject a missing argument before indexing argv.
                throw std::runtime_error("Missing value for " + a);
            return std::string(argv[i]); // Return an owned copy of the consumed command-line value.
        };
        if (a == "--demo") // Select synthetic observations without opening physical sensors or the USB accelerator.
            o.demo = true; // Enable deterministic synthetic inputs for demonstrations and desktop tests.
        else if (a == "--headless") // Suppress window creation while retaining logs and a final rendered snapshot.
            o.headless = true; // Mark this run as noninteractive; duration validation below ensures termination.
        else if (a == "--auto-start") // Allow the caller to request the initial robot start command.
            o.auto_start = true; // Request startup motion, still subject to tracking, Coral, and collision gates.
        else if (a == "--no-coral") // Explicitly opt out of semantic inference while retaining geometric tracking.
            o.no_coral = true; // Disable the Coral requirement for this run only.
        else if (a == "--check-coral") // Select a real model/delegate warm-up diagnostic that exits before sensor startup.
            o.check_coral = true; // Remember the diagnostic-only mode for validation and startup dispatch.
        else if (a == "--calibrate") // Select live measured calibration independently of existing camera-bearing association.
            o.calibrate = true; // Dispatch to the dedicated stationary-capture UI after loading user settings.
        else if (a == "--calibration-output") // Allow a new destination when retaining multiple measured calibrations.
            o.calibration_output = value(); // Keep capture output separate from the chosen input configuration.
        else if (a == "--seconds") { // Parse an optional duration in floating-point seconds.
            auto v = value(); // Consume the numeric text before checking complete conversion.
            std::size_t n = 0; // The numeric parser reports how many characters it actually consumed.
            o.seconds = std::stod(v, &n); // Convert duration text and retain the consumed-character count for trailing-junk rejection.
            if (n != v.size()) // Reject values such as 10abc instead of silently accepting a numeric prefix.
                throw std::runtime_error("Invalid --seconds");
        } else if (a == "--camera-index") { // Parse the selected libcamera device index as an integer.
            auto v = value(); // Consume the numeric text before checking complete conversion.
            std::size_t n = 0; // The numeric parser reports how many characters it actually consumed.
            o.camera = std::stoi(v, &n); // Convert camera index text, recording where conversion ended.
            if (n != v.size()) // Reject values such as 10abc instead of silently accepting a numeric prefix.
                throw std::runtime_error("Invalid camera index");
        } else if (a == "--config") { // An explicit main configuration bypasses automatic saved-calibration selection.
            o.config = value(); // Store the selected main configuration path for startup loading and boundary sidecar naming.
            o.config_explicit = true; // Preserve the user's deliberate choice even when a saved active calibration is available.
        } else if (a == "--tracking-config") // Allow an explicitly selected tracking calibration overlay.
            o.tracking = value(); // Store the optional tracking-only overlay path without searching neighboring directories.
        else if (a == "--port") // Override automatic LiDAR serial-device selection.
            o.port = value(); // Pass the chosen serial path through to the native sensor backend.
        else if (a == "--tracking") o.tracking_mode = value(); // Select the visual source used by fusion while preserving explicit marker/comparison modes.
        else if (a == "--detector") {
            const auto v=value(); o.detector_explicit=true;
            if(v=="ssd") o.detector.kind=tabletop::DetectorKind::Ssd;
            else if(v=="yolov8") o.detector.kind=tabletop::DetectorKind::YoloV8;
            else throw std::runtime_error("--detector must be ssd or yolov8");
        } else if (a == "--yolo-profile") {
            const auto v = value(); o.profile_explicit = true; // Bind the tensor contract and class meanings together.
            if (v == "coco") o.detector.profile = tabletop::YoloProfile::Coco;
            else if (v == "figure") o.detector.profile = tabletop::YoloProfile::Figure;
            else throw std::runtime_error("--yolo-profile must be coco or figure");
        } else if (a == "--person-confidence" || a == "--target-confidence") {
            o.confidence_explicit = true; // A caller threshold takes precedence over either profile's default.
            const auto v=value(); std::size_t n=0; o.detector.person_confidence=std::stof(v,&n);
            if(n!=v.size() || !std::isfinite(o.detector.person_confidence) || o.detector.person_confidence<=0 || o.detector.person_confidence>1)
                throw std::runtime_error("--person-confidence must be in (0,1]");
        } else if(a=="--person-max-width" || a=="--person-max-height") {
            const auto v=value(); std::size_t n=0; const float fraction=std::stof(v,&n);
            if(n!=v.size() || !std::isfinite(fraction) || fraction<=0 || fraction>1)
                throw std::runtime_error("Person size limits must be in (0,1]");
            (a=="--person-max-width" ? o.detector.max_person_width : o.detector.max_person_height)=fraction;
        } else if(a=="--yolo-coordinates") {
            const auto v=value();
            if(v!="normalized" && v!="pixels") throw std::runtime_error("--yolo-coordinates must be normalized or pixels");
            o.detector.normalized_boxes=v=="normalized";
        } else if (a == "--model") { o.model=value(); o.model_explicit=true; }
        else if (a == "--edgetpu-lib") // Select an alternative delegate library that must still pass ABI/version checks.
            o.library = value(); // Store the explicit native delegate library path.
        else if (a == "--output") // Choose the parent directory for session logs and snapshots.
            o.output = value(); // Store the output root; SessionLog creates a unique child directory within it.
        else
            throw std::runtime_error("Unknown argument: " + a);
    }
    if (!std::isfinite(o.seconds) || o.seconds < 0 || o.camera < 0) // Reject invalid durations and negative camera indices before acquiring resources.
        throw std::runtime_error("Invalid duration/camera index");
    if (o.headless && !o.check_coral && o.seconds <= 0) // Require a finite stopping point for a noninteractive run, except the diagnostic-only check.
        throw std::runtime_error("--headless requires --seconds N");
    if (o.check_coral && (o.demo || o.no_coral)) // Prevent contradictory requests to check real Coral while simultaneously disabling it.
        throw std::runtime_error("--check-coral requires real Coral, not --demo/--no-coral");
    if (o.calibrate && (o.demo || o.headless || o.auto_start || o.check_coral || o.seconds > 0)) // Calibration requires manual physical placement and a visible live-sensor selection window.
        throw std::runtime_error("--calibrate requires live interactive capture without demo, auto-start, or a time limit");
    if(o.tracking_mode!="marker" && o.tracking_mode!="compare" && o.tracking_mode!="yolo")
        throw std::runtime_error("--tracking must be marker, compare, or yolo");
    if (o.profile_explicit) {
        if (o.detector_explicit && o.detector.kind != tabletop::DetectorKind::YoloV8)
            throw std::runtime_error("--yolo-profile requires --detector yolov8");
        o.detector.kind = tabletop::DetectorKind::YoloV8; // An explicit YOLO profile selects its backend even for --check-coral.
    }
    if(o.tracking_mode!="marker") {
        if(o.detector_explicit && o.detector.kind!=tabletop::DetectorKind::YoloV8)
            throw std::runtime_error("compare/yolo tracking requires --detector yolov8");
        o.detector.kind=tabletop::DetectorKind::YoloV8;
        if(o.demo || o.no_coral || o.calibrate)
            throw std::runtime_error("compare/yolo requires live Coral; calibrate separately using marker mode");
    }
    if (o.detector.profile == tabletop::YoloProfile::Figure && !o.confidence_explicit)
        o.detector.person_confidence = .25f; // Match the training notebook's initial preview threshold.
    if(o.detector.kind==tabletop::DetectorKind::YoloV8 && !o.model_explicit)
        o.model = o.detector.profile == tabletop::YoloProfile::Figure ? "models/figure_yolov8n_320_edgetpu.tflite" : "models/yolov8n_320_edgetpu.tflite";
    return o; // Return validated command-line options to startup.
}
} // namespace
int main(int argc, char **argv) { // Program entry point; one outer exception boundary converts failures to a nonzero exit code.
    using namespace tabletop; // Bring application types into this local scope without affecting public headers.
    try {
        for (int i = 1; i < argc; ++i) // Skip argv[0], which is the executable name, and examine user-supplied arguments.
            if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") { // Handle help before parsing or opening files/devices.
                help(); // Print usage information and supported keys to the terminal.
                return 0;
            }
        auto o = parse(argc, argv); // Startup: limit OpenCV CPU workers, register termination handlers, and decide whether live TPU inference is required. --check-coral exits after real warm-up. Validate all command-line options before selecting hardware or simulator backends.
        cv::setNumThreads(1); // Limit OpenCV CPU workers to reduce contention with acquisition and inference threads.
        std::signal(SIGINT, signal_handler); // Convert Ctrl+C into a request for orderly shutdown.
        std::signal(SIGTERM, signal_handler); // Handle process termination through the same orderly shutdown path.
        const bool coral_required = !o.demo && !o.no_coral; // Semantic inference is required only for live runs without the explicit disable flag.
        std::unique_ptr<Coral> coral; // Optional exclusive owner of the inference service; null means it is disabled.
        if (o.check_coral) { // Run the hardware diagnostic independently of camera, LiDAR, and tracking configuration.
            Coral check(o.model, o.library, o.detector); // Construction validates model/runtime compatibility and invokes the real USB delegate once.
            std::cout << "PASS: USB delegate attached and model invoked in C++.\n"; // Report application progress or state to the launching terminal; output does not drive decisions.
            return 0;
        }
        o.config = startup_config_path(o.config, o.config_explicit).string(); // Prefer the active saved calibration unless the caller explicitly chose a configuration.
        Config c = load_config(o.config); // Load the resolved configuration, then apply any explicit tracking overlay and its matching boundary preference.
        std::cout << "Configuration: " << std::filesystem::absolute(o.config) << '\n'; // Report application progress or state to the launching terminal; output does not drive decisions.
        if (!o.tracking.empty()) { // Overlay tracking only when the user selected a calibration file.
            overlay_tracking(c, o.tracking); // Apply camera/LiDAR/tracking fields while preserving the selected robot geometry.
            std::cout << "Tracking calibration: " << std::filesystem::absolute(o.tracking) << '\n'; // Report application progress or state to the launching terminal; output does not drive decisions.
        }
        const std::filesystem::path boundary_file = o.config + ".boundary.json"; // Keep the saved radius beside the selected main configuration, not in a global location.
        bool boundary_saved = false; // Remember whether startup successfully loaded a saved figure exclusion radius.
        try {
            boundary_saved = load_boundary_setting(c, boundary_file); // A missing sidecar is normal; invalid saved values throw without mutating the active config.
        } catch (const std::exception &e) {
            std::cerr << "Saved boundary ignored: " << e.what() << '\n'; // Report a malformed optional sidecar while continuing with the validated main settings.
        }
        if (o.calibrate) // Capture measured 16-inch-row anchors before creating an inference service or any delivery engine.
            return run_calibration(c, o.port, o.camera, o.calibration_output, o.output, quit_requested); // Preserve the loaded boundary preference and save a separate calibrated configuration.
        BoundaryUi boundary(c); // Initialize slider limits and requested/applied values from the resolved radius.
        if (boundary_saved) // Show persisted status only when a valid saved boundary was actually loaded.
            boundary.notice = "Saved"; // Indicate that the applied radius agrees with the persisted boundary setting.
        std::cout << "Figure boundary: " << exclusion_radius(c) * 100
                  << " cm radius\n"; // Report application progress or state to the launching terminal; output does not drive decisions.
        std::cout << "Robot speed: " << c.speed * c.speed_scale * 100 << " cm/s ("
                  << c.speed_scale * 100 << "% pace)\n"; // Show the actual travel rate after resolving the selected calibration's nominal speed and pacing fraction.
        std::unique_ptr<void, decltype(&core_destroy)> core(core_create(&c), core_destroy); // Ownership: the engine, inference worker, sensor stack, and window all have RAII cleanup. Constructor exceptions also unwind resources already acquired. Own the opaque core handle with its required custom destructor for exception-safe cleanup.
        if (!core) // Stop startup if the engine rejected configuration or could not allocate its state.
            throw std::runtime_error("Native engine creation failed");
        if (coral_required) // Initialize the USB service only when semantic sensing is part of this run.
            coral = std::make_unique<Coral>(o.model, o.library, o.detector); // Load, check, and warm up Coral before live camera frames are submitted.
        std::unique_ptr<Sensors> sensors; // Optional RAII owner of the physical acquisition stack.
        DemoSource demo(c); // Keep a simulator available through the same observation contract as the hardware backend.
        if (!o.demo) // Open the real acquisition stack only for a live run.
            sensors = std::make_unique<Sensors>(o.port, o.camera); // Start camera callbacks and the LiDAR worker using the selected device options.
        SessionLog log(o.output, c, o.demo, coral_required);
        const bool evaluating=o.tracking_mode!="marker", yolo_controls=o.tracking_mode=="yolo";
        std::unique_ptr<void,decltype(&core_destroy)> shadow(evaluating?core_create(&c):nullptr,core_destroy);
        if(evaluating && !shadow) throw std::runtime_error("Cannot create comparison tracker");
        DetectionPairer pairer;
        EvaluationDisplay evaluation;
        std::unique_ptr<EvaluationLog> evaluation_log;
        if(evaluating) evaluation_log=std::make_unique<EvaluationLog>(log.path(),o.tracking_mode,o.model,o.detector);
        std::cout<<"Tracking mode: "<<o.tracking_mode<<'\n'; // Create a unique run directory and record the resolved configuration before entering the loop.
        std::cout << "Session: " << log.path() << '\n'; // Report application progress or state to the launching terminal; output does not drive decisions.
        const double start = clock_now(); // Establish the monotonic session origin for elapsed time and watchdog deadlines.
        double last_log = -1, last_display = -1, step_ms = 0, last_sensor_ok = start; // Initialize logging/display schedules, measured core cost, and the last healthy sensor time.
        bool mask = false, snapshot = false, ever_paired = false, ever_coral = false; // Initialize display/snapshot toggles and watchdog evidence that streams have ever been usable.
        int command = (o.auto_start || o.headless) ? TOGGLE_RUN : NONE, old_track = -1, // Headless or explicitly auto-started runs emit one start request; old statuses force initial logging.
            old_robot = -1; // Remember the reported delivery state for the next transition comparison.
        Result result{}; // Zero-initialize the public engine result before any observation has been processed.
        result.last_seen = -1; // Mark the figure position unknown until the first accepted track.
        result.selected = -1; // Prevent an uninitialized result from appearing to select LiDAR cluster zero.
        Inputs inputs; // Own the observation snapshot currently consumed by the main loop.
        Frame frame; // Hold the camera frame selected for the current LiDAR receive time.
        CoralState state; // Hold the latest copied inference result and read-time health assessment.
        cv::Mat screen; // Store the most recent rendered canvas for display and snapshots.
        const char *window = "Tabletop Delivery"; // Stable window identifier used for creation, callbacks, display, and close detection.
        struct WindowCleanup { // Scope guard closes GUI resources when startup or the main loop exits through an exception.
            bool active; // Cleanup is relevant only when windowed mode was requested.
            ~WindowCleanup() { // Destructors must not let a secondary GUI cleanup exception escape during unwinding.
                if (active) // Skip GUI teardown for headless runs.
                    try {
                        cv::destroyAllWindows(); // Release OpenCV windows and their associated GUI resources.
                    } catch (...) {
                    }
            }
        } window_cleanup{!o.headless};
        if (!o.headless) { // Perform GUI-only work when an interactive display was requested.
            cv::namedWindow(window, cv::WINDOW_AUTOSIZE); // Create a fixed-size display whose pixel coordinates match the boundary controls.
            cv::setMouseCallback( // Forward mouse events to the boundary widget without mutating core geometry in the callback.
                window,
                [](int event, int x, int y, int flags, void *data) {
                    static_cast<BoundaryUi *>(data)->mouse(event, x, y, flags); // Recover the widget passed as callback user data and translate this mouse event.
                },
                &boundary);
        }
        while (!quit_requested) { // Main loop: consume UI requests, pair observations, check health, step the engine, log/render, then process keyboard events and pace the next iteration. Continue until a signal, key/window close, error, or duration limit ends the run.
            const double now = clock_now(), elapsed = now - start; // Sample one iteration time and derive session elapsed seconds from the monotonic origin.
            const bool save_boundary = boundary.save_requested.exchange(false); // Apply geometry on the engine thread before any motion step. Atomic UI requests cannot race the planner. Saving records the newly applied radius. Consume the save request exactly once, including requests arriving from GUI callbacks.
            if (const int requested = boundary.requested_mm.load();
                requested != boundary.current_mm.load()) { // Apply only a changed requested millimeter value, on the same thread that steps the engine.
                const double radius = requested / 1000.; // Convert the integer slider value from millimeters to the engine's meters.
                if (std::abs(radius - exclusion_radius(c)) > 1e-12) { // Ignore rounding-equivalent radius values rather than invalidating a valid route unnecessarily.
                    if (core_set_boundary(core.get(), radius)) // Validate and apply geometry before this iteration's robot movement.
                        throw std::runtime_error("Invalid figure boundary requested");
                    if (shadow && core_set_boundary(shadow.get(),radius)) throw std::runtime_error("Comparison boundary update failed");
                    c.boundary_radius = radius; // Keep display and persistence synchronized with the exact total radius accepted by the engine.
                    boundary.notice = "Unsaved"; // Mark a live radius edit as pending persistence until the user requests Save/B.
                    last_display = -1; // Force prompt rendering of changed radius or persistence feedback.
                    std::cout << "Figure boundary changed to " << radius * 100 << " cm radius\n"; // Report application progress or state to the launching terminal; output does not drive decisions.
                }
                boundary.current_mm = requested; // Acknowledge the radius that was applied so callbacks and display share the same value.
            }
            if (save_boundary) { // Persist only when requested; changing the slider does not automatically write a file.
                try {
                    save_boundary_setting(c, boundary_file); // Write the effective total radius beside the chosen main configuration.
                    boundary.notice = "Saved"; // Indicate that the applied radius agrees with the persisted boundary setting.
                    std::cout << "Boundary saved: " << std::filesystem::absolute(boundary_file) // Report application progress or state to the launching terminal; output does not drive decisions.
                              << '\n';
                } catch (const std::exception &e) {
                    boundary.notice = "Save failed; see terminal"; // Preserve live operation while making a persistence failure visible in the window.
                    std::cerr << e.what() << '\n'; // Send the detailed persistence failure to the terminal for diagnosis.
                }
                last_display = -1; // Force prompt rendering of changed radius or persistence feedback.
            }
            inputs = o.demo ? demo.update(now, elapsed) : sensors->snapshot(); // Use the same observation contract for simulated and live input. Tracking uses the frame nearest the scan; inference receives the newest camera frame. Obtain one coherent observation snapshot from the selected backend.
            if (!inputs.error.empty()) // A latched acquisition failure is a device error, not merely a missing figure marker.
                throw std::runtime_error(inputs.error);
            frame = paired_frame(inputs); // Select the nearest receive-time camera frame for geometric fusion.
            if (coral && !inputs.frames.empty()) // Submit inference only when the service exists and at least one frame is available.
                coral->submit(inputs.frames.back()); // Prefer the newest frame for inference rather than delaying behind the tracking pair.
            state = coral ? coral->status(now) : CoralState{}; // Copy read-time inference health or an explicit empty state when semantic sensing is disabled.
            if (!coral) // Give the display an unambiguous reason for absent semantic inference.
                state.detail = "Explicitly disabled"; // Report the intentionally disabled service instead of presenting it as a hardware failure.
            if(evaluating) {
                pairer.observe(inputs.scan); // Store every arriving scan before pairing the most recent completed inference.
                inputs=pairer.pair(state); // Use original detector pixels and the closest buffered scan.
                frame=state.source_frame;
            }
            const bool pair_fresh = frame && inputs.scan.sequence >= 0 &&
                                    now - frame.received < c.fresh &&
                                    now - inputs.scan.received < c.fresh; // Acquisition watchdogs distinguish temporary tracking loss from a missing stream. Marker loss holds motion in the core; persistent device failure exits. Require both acquired streams to exist and have receive ages below the configured freshness limit.
            if (pair_fresh) { // Healthy streams reset the device-staleness watchdog regardless of marker visibility.
                last_sensor_ok = now; // Remember the most recent main-loop iteration with fresh camera and LiDAR data.
                ever_paired = true; // Record that live acquisition has succeeded at least once in this session.
            }
            if (state.healthy) // A recent real inference result supplies evidence that semantic acquisition has started.
                ever_coral = true; // Record successful inference for startup and final headless health checks.
            if (!o.demo) { // Open the real acquisition stack only for a live run.
                if (elapsed > 15 && !ever_paired) // Give live camera/LiDAR startup at most 15 seconds to produce fresh observations.
                    throw std::runtime_error("No fresh camera/LiDAR pair within 15 seconds");
                if (ever_paired && now - last_sensor_ok > 5) // After successful startup, fail a stream outage lasting more than five seconds.
                    throw std::runtime_error("Camera/LiDAR stream stale for five seconds");
                if (coral_required && !state.ready) // A failed required inference worker is fatal rather than an implicit CPU/disabled fallback.
                    throw std::runtime_error("Coral stopped: " + state.detail);
                if (coral_required && elapsed > 15 && !ever_coral) // Require fresh inference evidence within the live startup deadline.
                    throw std::runtime_error("No fresh native Coral result within 15 seconds");
            }
            Control control{command, int(coral_required), int(state.healthy), state.sequence,
                            int(state.stop_sign)}; // The core sees one-shot commands plus the current semantic gate. Clear the command after the step so a held key event is not replayed each loop. Package the one-shot command and timestamp-qualified semantic observation for this core step.
            const double before = clock_now(); // Start timing just the tracking/planning call, excluding rendering and sensor acquisition.
            const auto marker_step=[&](void *engine, const Control &ctl, Result &out) {
                return core_step(engine,frame?frame.pixels->data():nullptr,640,480,1920,
                    frame?frame.received:now-100,inputs.scan.points.data(),int(inputs.scan.points.size()),
                    inputs.scan.sequence>=0?inputs.scan.received:now-100,inputs.scan.sequence,now,&ctl,&out);
            };
            const int rc=yolo_controls ? step_detector(core.get(),inputs,state,o.detector,now,control,result)
                                      : marker_step(core.get(),control,result);
            if(evaluating) {
                Result other{};
                Control shadow_control{NONE,0,0,-1,0}; // Shadow tracks only; it never starts a second virtual delivery.
                const int shadow_rc=yolo_controls ? marker_step(shadow.get(),shadow_control,other)
                    : step_detector(shadow.get(),inputs,state,o.detector,now,shadow_control,other);
                if(shadow_rc) throw std::runtime_error("Comparison tracker rejected observation");
                evaluation=evaluation_display(yolo_controls,yolo_controls?other:result,yolo_controls?result:other,state,o.detector);
                evaluation_log->record(elapsed,now,inputs,state,evaluation);
            } // Both trackers consume the same source image and paired scan during evaluation.
            if (rc) // A nonzero core return indicates invalid input or an internal exception and ends the run.
                throw std::runtime_error("Native tracking engine rejected frame/scan: " +
                                         std::to_string(rc));
            step_ms = (clock_now() - before) * 1000; // Convert the core call's monotonic duration to the milliseconds shown in diagnostics.
            command = NONE; // Consume the command once so a toggle cannot repeat on every main-loop iteration.
            const bool changed =
                result.track_status != old_track || result.robot_status != old_robot; // Write transitions immediately and periodic records at roughly 10 Hz. Render independently at a 30 Hz target, or on snapshot/final-state requests. Compare current tracking and robot statuses against the last reported transition.
            if (changed || now - last_log >= .1) { // Log every state transition immediately and stable states at approximately 10 Hz.
                log.record(elapsed, frame, inputs.scan, result, state); // Write the current paired observation, robot decision, and inference status to CSV.
                last_log = now; // Advance the periodic logging schedule after a record was written.
            }
            if (changed) { // Emit a concise terminal transition only when tracking or robot status changed.
                std::cout << (yolo_controls ? detector_track_name(result.track_status, o.detector.profile == YoloProfile::Figure) : track_name(result.track_status)) << " | robot "
                          << robot_name(result.robot_status) << " | reroutes " << result.reroutes
                          << '\n'; // Report application progress or state to the launching terminal; output does not drive decisions.
                old_track = result.track_status; // Remember the reported perception state to suppress duplicate transition messages.
                old_robot = result.robot_status; // Remember the reported delivery state for the next transition comparison.
            }
            bool finished = o.seconds > 0 && elapsed >= o.seconds; // A positive duration limit ends the run once elapsed time reaches the requested seconds.
            if ((!o.headless && now - last_display >= 1. / 30) || snapshot || finished) { // Render for the GUI's 30 Hz target, a requested snapshot, or the final timed state.
                screen = render(c, inputs, frame, result, state, coral_required, o.demo, mask, now,
                                step_ms, &boundary, evaluating ? &evaluation : nullptr); // Build a coherent display from the observation and result consumed in this iteration.
                last_display = now; // Record when the current canvas was rendered for GUI pacing.
                if (!o.headless) // Perform GUI-only work when an interactive display was requested.
                    cv::imshow(window, screen); // Present the rendered canvas under the existing window identifier.
            }
            if (snapshot) { // Save a user-requested snapshot once the current canvas has been rendered.
                log.snapshot(screen, frame, inputs.scan, result, state); // Persist the canvas, paired camera frame, raw scan, and structured state together.
                snapshot = false; // Consume the snapshot request so subsequent iterations do not save duplicates.
                std::cout << "Snapshot saved\n"; // Report application progress or state to the launching terminal; output does not drive decisions.
            }
            if (finished) // Leave the loop after rendering the duration-limited final state.
                break;
            if (!o.headless) { // Perform GUI-only work when an interactive display was requested.
                int key = cv::waitKey(1) & 255; // Pump GUI events and normalize the returned key to its low byte.
                if (key == 27 || key == 'q' || key == 'Q' ||
                    cv::getWindowProperty(window, cv::WND_PROP_VISIBLE) < 1) // Escape, Q, or closing the window requests orderly application shutdown.
                    break;
                if (key == ' ') // Space sends one start/pause toggle to the next core step.
                    command = TOGGLE_RUN; // Queue the run-toggle action without directly modifying engine state.
                if (key == 'r' || key == 'R') // R restores the virtual robot to its depot through the core command interface.
                    command = RESET; // Queue a delivery reset for the next iteration.
                if (key == 'n' || key == 'N') // N switches wide/narrow lane mode and resets the delivery inside the core.
                    command = TOGGLE_LANE; // Queue lane selection as a one-shot core command.
                if (key == 'm' || key == 'M') // M toggles the diagnostic HSV mask display.
                    mask = !mask; // Change only visualization; the tracking algorithm receives the original paired image.
                if (key == 's' || key == 'S') // S requests a coherent snapshot on the next rendered iteration.
                    snapshot = true; // Latch a one-shot snapshot request for the next pass through the loop.
                if (key == '[') // Left bracket reduces the requested figure boundary by one millimeter.
                    boundary.adjust(-1); // Apply the widget's bounded decrement; the core will receive it next iteration.
                if (key == ']') // Right bracket increases the requested figure boundary by one millimeter.
                    boundary.adjust(1); // Apply the widget's bounded increment without bypassing its minimum/maximum limits.
                if (key == 'b' || key == 'B') // B requests persistence of the applied figure boundary.
                    boundary.save_requested = true; // Publish a one-shot save request through the same channel as the mouse Save button.
            }
            const double remaining = .02 - (clock_now() - now); // Target a 20 ms main-loop period after subtracting work already performed.
            if (remaining > 0) // Sleep only when the iteration finished before its target period.
                std::this_thread::sleep_for(std::chrono::duration<double>(remaining)); // Yield the CPU for the remaining portion of the approximately 50 Hz loop period.
        }
        const double final_now = clock_now(); // Save a final diagnostic view before releasing sensors. In bounded live checks, unavailable/stale required devices produce a nonzero exit status. Sample shutdown time for the final display and required-stream freshness checks.
        screen = render(c, inputs, frame, result, state, coral_required, o.demo, mask, final_now,
                        step_ms, &boundary, evaluating ? &evaluation : nullptr); // Build a coherent display from the observation and result consumed in this iteration.
        log.snapshot(screen, frame, inputs.scan, result, state); // Persist the canvas, paired camera frame, raw scan, and structured state together.
        if(evaluation_log) evaluation_log->finish(); // Publish aggregate evidence before shutting down devices.
        sensors.reset(); // Stop physical acquisition and join sensor workers before leaving the application scope.
        coral.reset(); // Join the inference worker and release its interpreter, delegate, and loaded runtime.
        if (!o.headless) // Perform GUI-only work when an interactive display was requested.
            cv::destroyAllWindows(); // Release OpenCV windows and their associated GUI resources.
        if (o.headless && !o.demo &&
            (!ever_paired || !frame || final_now - frame.received >= c.fresh ||
             final_now - inputs.scan.received >= c.fresh || (coral_required && !state.healthy))) // A bounded live check must end with fresh required streams, not merely have succeeded earlier.
            throw std::runtime_error("Headless check ended without fresh required sensors/Coral");
        std::cout << "Stopped. Logs and snapshot: " << log.path() << '\n'; // Report application progress or state to the launching terminal; output does not drive decisions.
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "ERROR: " << e.what() << '\n'; // Surface the caught failure and return a failure status to scripts or CTest.
        return 1;
    }
}
