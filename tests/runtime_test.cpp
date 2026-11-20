/**
 * @file
 * Behavioral tests using synthetic images/scans and an explicit clock.
 * Each case drives the public engine interface and checks observable motion
 * or hold behavior. No hardware, sleeping, or inference runtime is required.
 */

#include "../src/runtime.h"
#include <functional> // Comparison/function utilities used by the native implementation.
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <random>
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
#include <limits> // Numeric limits, including the unreachable-cost infinity used by grid search.
using namespace tabletop;
void expect(bool v, const char *message) { // Convert a failed behavioral expectation into a test failure with its scenario-specific message.
    if (!v) // A false condition ends this test immediately instead of allowing later checks to obscure the cause.
        throw std::runtime_error(message);
}
double distance(XY a, XY b) {
    return std::hypot(a.x - b.x, a.y - b.y); // Independently measure metric displacement for pose and nonmovement assertions.
}
struct Harness { // Own an isolated core and explicit simulated clock for each behavior scenario.
    Config c = default_config(); // Start each scenario with the calibrated tabletop geometry and tracking thresholds.
    std::unique_ptr<void, decltype(&core_destroy)> core{core_create(&c), core_destroy}; // Own one engine per harness and always destroy it even if an expectation throws.
    Result r{}; // Store the most recent public result for behavior assertions.
    double now = 100; // Begin at a positive deterministic monotonic time, independent of wall-clock execution speed.
    int sequence = 0; // Count synthetic observations so acquisition confirmation sees distinct scans.
    Frame f; // Hold the latest synthetic image, which tests can deliberately leave stale or replace independently.
    Scan s; // Hold the latest synthetic scan for normal updates or controlled dropout/ambiguity scenarios.
    explicit Harness(Config config = default_config()) : c(config) { // Allow a scenario to select motion settings before the core's member initializer copies them.
        expect(bool(core), "core_create"); // Require successful engine construction before any scenario exercises the public API.
    }
    void update(Control ctl = {}) { // Step with the current observations and supplied command without advancing the synthetic clock.
        expect(core_step(core.get(), f ? f.pixels->data() : nullptr, 640, 480, 1920,
                         f ? f.received : now - 100, s.points.data(), int(s.points.size()),
                         s.received, s.sequence, now, &ctl, &r) == 0,
               "core_step"); // Require the public engine API to accept the test's observation dimensions, timestamps, and buffers.
    }
    void step(XY xy = {.15, .63}, int command = NONE, bool marker = true, Control ctl = {}) { // Produce a fresh 50 ms observation pair and optionally change command, marker visibility, or semantic state.
        now += .05; // Advance deterministic sensor time by 50 ms; tests do not need to sleep in real time.
        ++sequence; // Assign a distinct scan/frame identity so the core consumes this observation once.
        f = synthetic_frame(c, xy, now, sequence, marker); // Draw an image that exercises real marker segmentation rather than injecting a tracked position.
        s = synthetic_scan(c, xy, now, sequence); // Generate raw polar returns that must pass normal clustering and bearing association.
        ctl.command = command; // Attach this step's one-shot user command to any supplied Coral observation state.
        update(ctl); // Run the public core with the newly generated observations and command.
    }
    void acquire(XY xy = {.15, .63}) { // Supply the normal two-scan confirmation sequence at a chosen metric figure position.
        step(xy); // Feed a fresh consistent observation; two successive calls satisfy the default confirmation threshold.
        step(xy); // Feed a fresh consistent observation; two successive calls satisfy the default confirmation threshold.
        expect(r.track_status == TRACKING, "acquire"); // Check that confirmation produced a valid fused track before the scenario proceeds.
    }
};
int main() {
    int tests = 0; // Count only scenarios that finish without throwing an expectation failure.
    try {
        auto test = [&](const char *name, const std::function<void()> &run) { // Execute a named independent scenario and print its pass marker after all assertions complete.
            run(); // Invoke the scenario body; an exception skips the pass counter and reaches the outer failure report.
            ++tests; // Count this scenario only after it completes successfully.
            std::cout << "PASS " << name << '\n';
        };
        test("native image segmentation and metric fusion", [] { // Independent behavior scenario: native image segmentation and metric fusion.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.acquire({-.15, .63}); // Confirm a figure left of the centerline to test camera-bearing conversion and metric sign.
            expect(distance(h.r.xy, {-.15, .63}) < .001, "wrong figure position"); // Verify fused position is within 1 mm of the synthetic metric target.
            expect(h.r.groups[h.r.selected].count == 4, "return count"); // Verify all four synthetic returns support the selected cluster.
        });
        test("robot pacing preserves tracking cadence and pause", [] { // Measure one second of actual travel at default and full pacing, then verify pause remains immediate.
            for (double scale : {.8, 1.}) { // Exercise the slower default and an explicit full-speed selection independently.
                Config c = default_config(); // Keep the same geometry, tracking settings, and nominal speed for both runs.
                c.speed_scale = scale; // Change only virtual movement pacing; synthetic observation cadence stays at 20 Hz.
                Harness h(c); // Construct the engine with the selected pacing fraction.
                h.step({.15, .63}, TOGGLE_RUN); // Place the figure beside the direct route and request movement.
                for (int i = 0; i < 20; ++i) // Allow initial tracking confirmation and the recovery settling interval to finish.
                    h.step(); // Supply one fresh camera/scan pair every 50 ms.
                XY before = h.r.robot; // Measure from an already-moving position so startup delays do not enter the speed check.
                for (int i = 0; i < 20; ++i) // Measure displacement across exactly one second of sensor time.
                    h.step(); // Keep the route clear and the figure stationary during measurement.
                expect(std::abs(distance(before, h.r.robot) - (scale == .8 ? .0224 : .028)) < 1e-9,
                       "wrong paced travel distance"); // Check measured metric displacement against the requested physical rates.
                expect(h.r.track_status == TRACKING && h.r.robot_status == DRIVING, "pacing interrupted tracking"); // Slower motion must not delay or invalidate perception.
                before = h.r.robot; // Retain the last moving pose to check same-iteration pause behavior.
                h.step({.15, .63}, TOGGLE_RUN); // Request pause while valid sensing continues at its normal cadence.
                expect(h.r.robot_status == PAUSED && distance(before, h.r.robot) < 1e-12, "paced pause moved"); // A pause must consume no motion budget at either pace.
            }
        });
        test("direct delivery latches", [] { // Independent behavior scenario: direct delivery latches.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 300; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            expect(h.r.robot_status == DELIVERED, "delivery failed");
            expect(distance(h.r.robot, h.c.goal) < .002, "wrong goal");
            for (int i = 0; i < 10; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            expect(h.r.robot_status == DELIVERED, "delivery unlatched");
        });
        test("central prop yields and detours", [] { // Independent behavior scenario: central prop yields and detours.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({0, .63}, TOGGLE_RUN); // Start with a centerline obstruction that requires yielding before a detour.
            bool yield = false, detour = false; // Track whether the scenario actually visits both expected intermediate states before delivery.
            for (int i = 0; i < 450; ++i) {
                XY before = h.r.robot; // Save the robot pose before an update so subsequent checks can detect actual movement.
                h.step({0, .63}); // Keep the obstruction centered while the robot decides whether to yield, detour, or remain blocked.
                yield |= h.r.robot_status == YIELDING; // Record that at least one update reported the required temporary yield state.
                detour |= h.r.robot_status == DETOUR; // Record that at least one update selected travel along an indirect route.
                if (distance(before, h.r.robot) > 1e-10) // Check obstacle clearance whenever this update actually advances the robot.
                    expect(distance(h.r.robot, {0, .63}) >
                               h.c.robot_radius + h.c.figure_radius + h.c.margin,
                           "obstacle collision"); // Require the moved robot to remain outside the combined figure/robot/clearance radius.
            }
            expect(yield && detour && h.r.robot_status == DELIVERED, "detour behavior"); // Require yield, detour, and eventual delivery within this scenario's bounded simulated duration.
        });
        test("narrow lane blocks then resumes", [] { // Independent behavior scenario: narrow lane blocks then resumes.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({0, .63}, TOGGLE_LANE); // Select the narrow lane, where the central prop should eliminate a feasible passage.
            h.step({0, .63}, TOGGLE_RUN); // Start with a centerline obstruction that requires yielding before a detour.
            for (int i = 0; i < 100; ++i)
                h.step({0, .63}); // Keep the obstruction centered while the robot decides whether to yield, detour, or remain blocked.
            expect(h.r.robot_status == BLOCKED, "narrow lane should block");
            for (int i = 1; i <= 60; ++i)
                h.step({.15 * i / 60, .63}); // Move the figure gradually out of the narrow lane without triggering the implausible-jump gate.
            for (int i = 0; i < 300; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            expect(h.r.robot_status == DELIVERED, "clear lane resume");
        });
        test("marker loss immediately holds", [] { // Independent behavior scenario: marker loss immediately holds.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            XY p = h.r.robot; // Save the pre-failure pose to verify that a sensing outage or loop stall cannot cause unsafe motion.
            h.step({.15, .63}, NONE, false); // Remove the visual marker while leaving a fresh LiDAR observation available.
            expect(h.r.robot_status == TRACK_LOST && distance(p, h.r.robot) == 0,
                   "lost marker moved robot"); // Missing visual identification must hold the robot at exactly its previous position.
        });
        test("stale scan holds with fresh camera", [] { // Independent behavior scenario: stale scan holds with fresh camera.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            XY p = h.r.robot; // Save the pre-failure pose to verify that a sensing outage or loop stall cannot cause unsafe motion.
            h.now += .6; // Advance past the 0.5-second freshness limit while deliberately leaving one stream unchanged.
            h.f = synthetic_frame(h.c, {.15, .63}, h.now, 100); // Refresh only the camera so the test isolates a stale LiDAR stream.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.track_status == STALE_LIDAR && distance(p, h.r.robot) == 0,
                   "stale scan moved robot"); // Fresh camera data cannot authorize motion using an expired scan.
        });
        test("stale camera and pair gap hold", [] { // Independent behavior scenario: stale camera and pair gap hold.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.acquire(); // Establish a valid track before deliberately changing timing or input validity.
            h.now += .6; // Advance past the 0.5-second freshness limit while deliberately leaving one stream unchanged.
            h.s = synthetic_scan(h.c, {.15, .63}, h.now, 100); // Refresh only LiDAR so the camera-staleness path is isolated.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.track_status == STALE_CAMERA, "stale camera accepted"); // Require explicit stale-camera rejection while LiDAR remains fresh.
            h.f = synthetic_frame(h.c, {.15, .63}, h.now - .3, 100); // Supply a fresh-enough image whose 300 ms pairing gap exceeds the 250 ms association limit.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.track_status == TIMING_GAP, "bad pair accepted"); // Reject incompatible host receive times even when each stream is individually fresh.
        });
        test("Coral offline and stop-sign latch", [] { // Independent behavior scenario: Coral offline and stop-sign latch.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            Control ctl{}; // Start with no healthy inference result, then enable the semantic requirement explicitly.
            ctl.coral_required = 1; // Require Coral evidence so missing inference cannot be treated as an optional disabled service.
            h.step({.15, .63}, TOGGLE_RUN, true, ctl); // Start a clear-path delivery with the scenario's explicit inference state.
            for (int i = 0; i < 20; ++i)
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            expect(h.r.robot_status == TPU_OFFLINE, "offline TPU allowed motion"); // Required but unhealthy inference must hold an otherwise geometrically clear delivery.
            ctl.coral_healthy = 1; // Supply healthy inference after proving that its absence prevents motion.
            for (int i = 0; i < 20; ++i) {
                ctl.coral_sequence = i; // Give each semantic observation a distinct identity so confirmation/release can progress.
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            }
            ctl.stop_sign = 1; // Introduce a positive stop-sign observation without changing geometric sensing.
            for (int i = 20; i < 22; ++i) {
                ctl.coral_sequence = i; // Give each semantic observation a distinct identity so confirmation/release can progress.
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            }
            expect(h.r.robot_status == TPU_STOP, "stop sign not held"); // Require two distinct healthy positive results to latch the semantic hold.
            XY held = h.r.robot; // Remember the pose at the start of a semantic or manual hold.
            for (int i = 22; i < 30; ++i) {
                ctl.coral_sequence = i; // Give each semantic observation a distinct identity so confirmation/release can progress.
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            }
            expect(distance(held, h.r.robot) == 0, "stop hold moved"); // Confirm the held robot has not moved despite continued fresh geometric observations.
            ctl.stop_sign = 0; // Begin healthy clear observations so the timed semantic release path can run.
            for (int i = 30; i < 60; ++i) {
                ctl.coral_sequence = i; // Give each semantic observation a distinct identity so confirmation/release can progress.
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            }
            expect(h.r.robot.y > held.y, "stop sign did not clear"); // Verify movement resumes after sign clearance and recovery-settling intervals.
        });
        test("duplicate Coral sequence cannot confirm sign", [] { // Independent behavior scenario: duplicate Coral sequence cannot confirm sign.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            Control ctl{NONE, 1, 1, 1, 1}; // Reuse one healthy positive inference sequence to test duplicate-result suppression.
            for (int i = 0; i < 10; ++i)
                h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            expect(h.r.robot_status != TPU_STOP, "duplicate confirmed sign"); // Repeated reads of one positive result must not count as two sign confirmations.
        });
        test("pause reset", [] { // Independent behavior scenario: pause reset.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            XY held = h.r.robot; // Remember the pose at the start of a semantic or manual hold.
            for (int i = 0; i < 10; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            expect(h.r.robot_status == PAUSED && distance(held, h.r.robot) == 0, "pause moved"); // A user pause preserves pose despite ongoing fresh observations.
            h.step({.15, .63}, RESET); // Send a one-shot reset after testing pause behavior.
            expect(h.r.robot_status == READY && distance(h.r.robot, h.c.start) == 0,
                   "reset failed"); // Reset must restore both the depot pose and waiting-to-start status.
        });
        test("camera-only never authorizes motion", [] { // Independent behavior scenario: camera-only never authorizes motion.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.acquire(); // Establish a valid track before deliberately changing timing or input validity.
            h.now += .05; // Advance one observation interval before staging a camera-only scan update.
            h.s.sequence = ++h.sequence; // Mark the empty scan as a genuinely new observation rather than a duplicate.
            h.s.received = h.now; // Keep the deliberately modified scan fresh so rejection depends on its geometry, not age.
            h.f = synthetic_frame(h.c, {.15, .63}, h.now, h.sequence); // Keep the blue marker visible while removing usable range evidence.
            h.s.points.clear(); // Remove every LiDAR return without removing the fresh scan timestamp.
            h.update({TOGGLE_RUN, 0, 0, 0, 0}); // Request motion with camera-only evidence to verify that fusion is mandatory.
            expect(h.r.track_status == MARKER_ONLY && h.r.robot_status == TRACK_LOST,
                   "camera-only motion"); // Visual identification alone must report MARKER_ONLY and hold the virtual robot.
        });
        test("ambiguous range rejected", [] { // Independent behavior scenario: ambiguous range rejected.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.f = synthetic_frame(h.c, {0, .63}, h.now, 1); // Draw a centered marker whose bearing can match two separately staged range clusters.
            h.s.sequence = 1; // Identify the ambiguity fixture as a new scan.
            h.s.received = h.now; // Keep the deliberately modified scan fresh so rejection depends on its geometry, not age.
            for (double a : {-2.1, -1.4, -.7}) // Build the first compact cluster just left of the camera-derived center bearing.
                h.s.points.push_back({h.c.forward + a, .52, 47}); // Place the first cluster at 0.52 m with valid quality.
            for (double a : {.7, 1.4, 2.1}) // Build a separate compact cluster just right of the same expected bearing.
                h.s.points.push_back({h.c.forward + a, .70, 47}); // Place the competing cluster at 0.70 m so the range ambiguity is real.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.track_status == AMBIGUOUS_LIDAR, "ambiguous match"); // Two bearing-compatible clusters must be rejected instead of choosing an arbitrary distance.
        });
        test("loop stall caps motion", [] { // Independent behavior scenario: loop stall caps motion.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            XY p = h.r.robot; // Save the pre-failure pose to verify that a sensing outage or loop stall cannot cause unsafe motion.
            h.now += 2; // Simulate a long application stall without sleeping in real time.
            h.step(); // Advance one fresh observation at the default clear-side position.
            expect(distance(p, h.r.robot) <= h.c.speed * .05 + 1e-9, "teleport after stall"); // Require travel after a stall to remain within one capped 50 ms integration budget.
        });
        test("stationary noise keeps one detour", [] { // Independent behavior scenario: stationary noise keeps one detour.
            for (unsigned seed : {42u, 77u, 123u, 999u}) { // Repeat the noise scenario with several reproducible random sequences.
                Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
                std::mt19937 rng(seed); // Seed deterministic pseudorandom noise so a failure can be reproduced exactly.
                std::uniform_real_distribution<double> noise(-.003, .003); // Model small stationary-prop measurement jitter uniformly within +/-3 mm.
                int last = -1; // Sentinel forces the first synthetic scan to be generated.
                double delivered = -1; // Record the first simulated arrival time, retaining -1 if delivery never occurs.
                for (int tick = 0; tick < 1500; ++tick) { // Run a 50 Hz application loop for a bounded 30-second synthetic interval.
                    h.now = 100 + tick * .02; // Advance the explicit clock by 20 ms per application iteration.
                    int scan = tick / 5; // Emit a 10 Hz scan sequence while the application steps five times faster.
                    if (scan != last) { // Add new measurement noise only when a new scan arrives, not at every application iteration.
                        last = scan; // Remember which simulated scan has already been generated.
                        h.s = synthetic_scan(h.c, {-.032 + noise(rng), .558 + noise(rng)}, h.now,
                                             scan); // Perturb a stationary figure's LiDAR position while preserving the base obstacle placement.
                        h.f = synthetic_frame(h.c, {-.032, .558}, h.now, scan); // Keep visual identification stable while testing route robustness to small range-position noise.
                    }
                    h.update({tick == 0 ? TOGGLE_RUN : NONE, 0, 0, 0, 0}); // Start only on the first tick; later iterations cannot repeatedly toggle motion.
                    if (h.r.robot_status == DELIVERED && delivered < 0) // Record only the first arrival, not subsequent iterations of the latched delivered state.
                        delivered = tick * .02; // Convert arrival tick into elapsed seconds for the bounded-completion assertion.
                }
                expect(delivered >= 0 && delivered < 20 && h.r.reroutes <= 1, "jitter regression"); // Require timely arrival with at most one detour despite stationary measurement noise.
            }
        });
        test("bounded heading change", [] { // Independent behavior scenario: bounded heading change.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({0, .63}, TOGGLE_RUN); // Start with a centerline obstruction that requires yielding before a detour.
            double prev = h.r.heading; // Save the prior display heading for per-update angular-rate checks.
            for (int i = 0; i < 400; ++i) {
                h.step({0, .63}); // Keep the obstruction centered while the robot decides whether to yield, detour, or remain blocked.
                double diff =
                    std::atan2(std::sin(h.r.heading - prev), std::cos(h.r.heading - prev)); // Compute shortest wrapped heading change rather than a discontinuity across +/-pi.
                expect(std::abs(diff) <= 3 * .05 + 1e-8, "heading snapped"); // Enforce the 3 rad/s display turn limit over a 50 ms test step.
                prev = h.r.heading; // Use this heading as the reference for the next angular-rate check.
            }
            h.step({0, .63}, RESET); // Reset after the turning scenario to verify heading initialization independently.
            expect(h.r.heading == 0, "reset heading"); // Reset must restore the display's forward-facing heading.
        });
        test("new raw intrusion stops current detour", [] { // Independent behavior scenario: new raw intrusion stops current detour.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({0, .63}, TOGGLE_RUN); // Start with a centerline obstruction that requires yielding before a detour.
            for (int i = 0; i < 100 && h.r.robot_status != DETOUR; ++i)
                h.step({0, .63}); // Keep the obstruction centered while the robot decides whether to yield, detour, or remain blocked.
            expect(h.r.robot_status == DETOUR, "no detour"); // Ensure a real active detour exists before testing a new raw-position intrusion.
            XY robot = h.r.robot, raw = h.r.raw, next = h.r.path[0]; // Capture robot position, raw figure position, and next waypoint before moving the obstruction.
            double dx = next.x - robot.x, dy = next.y - robot.y; // Construct the current route segment direction for the intrusion geometry.
            double t = std::clamp(
                ((raw.x - robot.x) * dx + (raw.y - robot.y) * dy) / (dx * dx + dy * dy), 0., 1.); // Project the raw figure position onto the current segment and clamp to its finite endpoints.
            XY v{robot.x + t * dx - raw.x, robot.y + t * dy - raw.y}; // Form a vector from the figure toward the closest point on the active route segment.
            double length = std::hypot(v.x, v.y); // Normalize the intrusion direction using its Euclidean metric length.
            h.step({raw.x + .04 * v.x / length, raw.y + .04 * v.y / length}); // Move the measured figure 4 cm toward the route while the smoothed estimate still lags.
            expect(h.r.track_status == TRACKING, "intrusion not tracked"); // Verify the intrusion remains a valid track rather than relying on tracking loss to stop motion.
            expect((h.r.robot_status == YIELDING || h.r.robot_status == TOO_CLOSE) &&
                       distance(robot, h.r.robot) == 0,
                   "raw intrusion ignored"); // A new raw conflict must yield or report overlap without advancing along the old detour.
        });
        test("live boundary expansion holds and shrinking resumes without reset", [] { // Independent behavior scenario: live boundary expansion holds and shrinking resumes without reset.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            const XY figure{.05, .50}; // Keep the near-route prop within 9 cm of the depot so a supported 1 cm expansion can overlap it.
            h.step(figure, TOGGLE_RUN); // Start delivery with this near-route figure placement.
            for (int i = 0; i < 20; ++i)
                h.step(figure); // Maintain a consistent near-route observation while testing boundary changes.
            const XY pose = h.r.robot; // Save the exact robot pose before a live boundary mutation or persistent hold check.
            const int scan = h.r.scan_number, reroutes = h.r.reroutes; // Preserve tracking identity and detour count to detect unintended resets on geometry edits.
            const double larger = distance(pose, h.r.xy) + .01; // Choose an expanded boundary that overlaps the saved robot pose by 1 cm.
            expect(core_set_boundary(h.core.get(), larger) == 0, "live boundary update rejected"); // Apply an expanded live radius through the public engine interface.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.track_status == TRACKING && h.r.scan_number == scan,
                   "boundary reset tracking"); // Boundary edits must preserve existing tracking state and scan identity.
            expect(h.r.robot_status == TOO_CLOSE && distance(pose, h.r.robot) == 0,
                   "expanded boundary failed to stop"); // Expanding into the robot must stop movement without relocating its pose.
            expect(h.r.path_count == 0 && h.r.reroutes == reroutes,
                   "boundary failed to discard invalidated route"); // Invalidated geometry must clear the route without inventing an additional detour.
            expect(std::abs(h.r.obstacle_radius - larger) < 1e-12,
                   "drawn radius differs from collision radius"); // Report exactly the radius used for collision checks so the displayed boundary is truthful.
            expect(core_set_boundary(h.core.get(), .072) == 0, "shrink rejected"); // Restore the default combined radius while keeping the current delivery state.
            for (int i = 0; i < 100; ++i)
                h.step(figure); // Maintain a consistent near-route observation while testing boundary changes.
            expect(distance(pose, h.r.robot) > .01, "shrinking did not resume delivery"); // After shrinking clears the obstruction, a running delivery should progress without a reset.
            expect(std::abs(h.r.obstacle_radius - .072) < 1e-12, "shrink radius not applied"); // Confirm the shrunk radius became the active reported collision boundary.
        });
        test("boundary change preserves pause", [] { // Independent behavior scenario: boundary change preserves pause.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            for (int i = 0; i < 20; ++i)
                h.step(); // Advance one fresh observation at the default clear-side position.
            h.step({.15, .63}, TOGGLE_RUN); // Toggle start/pause with the figure beside the route; the current run state selects the action.
            const XY pose = h.r.robot; // Save the exact robot pose before a live boundary mutation or persistent hold check.
            expect(core_set_boundary(h.core.get(), .10) == 0, "paused boundary rejected"); // Edit a paused delivery's boundary at the supported upper endpoint without a run-toggle command.
            h.step(); // Advance one fresh observation at the default clear-side position.
            expect(h.r.robot_status == PAUSED && distance(pose, h.r.robot) == 0,
                   "boundary resumed paused robot"); // A user pause preserves pose despite ongoing fresh observations.
        });
        test("boundary change preserves stop-sign hold", [] { // Independent behavior scenario: boundary change preserves stop-sign hold.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            Control ctl{TOGGLE_RUN, 1, 1, 1, 1}; // Stage the first healthy stop-sign detection together with a start request.
            h.step({.15, .63}, TOGGLE_RUN, true, ctl); // Start a clear-path delivery with the scenario's explicit inference state.
            ctl.coral_sequence = 2; // Supply the second distinct positive inference required to latch the sign hold.
            h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            expect(h.r.robot_status == TPU_STOP, "stop-sign setup failed"); // Require two distinct healthy positive results to latch the semantic hold.
            const XY pose = h.r.robot; // Save the exact robot pose before a live boundary mutation or persistent hold check.
            expect(core_set_boundary(h.core.get(), .1) == 0, "held boundary rejected"); // Change geometry while the semantic latch is active.
            ctl.coral_sequence = 3; // Continue healthy positive semantic evidence after the radius update.
            h.step({.15, .63}, NONE, true, ctl); // Keep camera/LiDAR fresh while varying only inference identity, health, or sign detection.
            expect(h.r.robot_status == TPU_STOP && distance(pose, h.r.robot) == 0,
                   "boundary cleared stop-sign hold"); // Require two distinct healthy positive results to latch the semantic hold.
        });
        test("invalid boundary leaves current geometry intact", [] { // Independent behavior scenario: invalid boundary leaves current geometry intact.
            Harness h; // Create fresh perception, route, and semantic-latch state for this independent scenario.
            h.acquire(); // Establish a valid track before deliberately changing timing or input validity.
            const double old = h.r.obstacle_radius; // Capture the valid boundary before attempting invalid updates.
            for (double radius : {-.1, 0., .029999, .100001, .2, std::numeric_limits<double>::quiet_NaN(),
                                  std::numeric_limits<double>::infinity()}) // Exercise negative, undersized, NaN, and infinite radius rejection.
                expect(core_set_boundary(h.core.get(), radius) != 0, "invalid radius accepted"); // Invalid values must fail before mutating the existing geometry.
            expect(core_set_boundary(nullptr, .1) != 0, "null engine accepted"); // A null engine handle must be rejected without dereferencing it.
            h.update(); // Evaluate the current staged data without automatically refreshing either sensor.
            expect(h.r.obstacle_radius == old, "invalid radius changed geometry"); // Rejected updates must leave the previously valid radius unchanged.
            for (double radius : {.03, .10}) { // Both inclusive endpoints must work even when smaller than the nominal body-radius sum.
                expect(core_set_boundary(h.core.get(), radius) == 0, "endpoint radius rejected"); // Select the endpoint through the public API used by the live UI.
                h.update(); // Evaluate the unchanged observations without needing a new scan to publish geometry.
                expect(std::abs(h.r.obstacle_radius - radius) < 1e-12, "endpoint radius not applied"); // Verify the result reports the requested radius exactly.
            }
        });
        test("nearest receive pairing and Coral freshness", [] { // Independent behavior scenario: nearest receive pairing and Coral freshness.
            Inputs in; // Build an explicit receive-time history independently of hardware acquisition.
            Config c = default_config(); // Start each scenario with the calibrated tabletop geometry and tracking thresholds.
            in.frames = {synthetic_frame(c, {0, .6}, 10, 1), synthetic_frame(c, {0, .6}, 10.1, 2),
                         synthetic_frame(c, {0, .6}, 10.2, 3)}; // Stage three camera receive times around the scan timestamp to test nearest-time selection.
            in.scan.received = 10.12; // Choose a scan time closest to the second camera frame rather than the newest one.
            in.scan.sequence = 1; // Mark the scan present so pairing uses nearest-time selection instead of startup fallback.
            expect(paired_frame(in).sequence == 2, "nearest frame wrong"); // Require the closest receive-time frame's identity, not simply the latest image.
            CoralState s; // Construct an inference status whose timestamp fields can be manipulated independently.
            s.ready = true; // Indicate an available interpreter so freshness depends on sequence and time checks.
            s.sequence = 1; // Mark the semantic state as a completed real observation rather than a missing-result sentinel.
            s.received = 10.2; // Set the inference completion time for the result-age test.
            s.frame_time = 10; // Source image predates inference and must also satisfy its own freshness limit.
            expect(coral_fresh(s, 10.3), "fresh TPU rejected"); // Accept a result whose source and completion times are both recent and not in the future.
            expect(!coral_fresh(s, 10.81), "stale TPU accepted"); // Reject the same result after its completion and source ages exceed allowed limits.
            s.frame_time = 11; // Stage a future source timestamp to verify clock-consistency rejection.
            expect(!coral_fresh(s, 10.3), "future TPU accepted"); // Reject future-dated source evidence even when interpreter readiness is true.
        });
        std::cout << tests << " native runtime tests passed\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "FAIL after " << tests << " tests: " << e.what() << '\n';
        return 1;
    }
}
