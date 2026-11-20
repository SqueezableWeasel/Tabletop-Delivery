# Code walkthrough

This guide explains the application's execution order and each owned source component. The source contains statement-level explanations as well as file/function notes, ownership rules, and units. See [Reading the annotated source](ANNOTATED_SOURCE.md) for comment conventions and a suggested reading order. Start with `src/main.cpp`, then follow the observations into `native/core.cpp`.

## Layers and dependencies

| Layer | Files | External dependencies |
| --- | --- | --- |
| Tracking and planning | `native/core.h`, `native/core.cpp` | C++ standard library |
| Observations and simulation | `src/runtime.h`, `src/runtime.cpp` | C++ standard library |
| Configuration and presentation | `src/app.h`, `src/config.cpp`, `src/view.cpp` | OpenCV |
| Sensor acquisition | `src/sensors.cpp`, `src/camera_controls.h` | libcamera, SLAMTEC SDK, Linux DMA APIs |
| Inference | `src/coral.cpp`, `src/coral_engine.h`, `src/coral_abi.h`, `src/model_check.h` | OpenCV, TensorFlow Lite, project-local libedgetpu |
| Detector decoding and comparison | `src/yolo.h`, `src/evaluation.h` | Core contracts and the C++ standard library |
| Application coordination | `src/main.cpp` | The interfaces above |
| Desktop backend | `src/no_hardware.cpp` | Rejecting implementations of hardware interfaces |

The portable core does not include OpenCV or hardware headers. Its tests can build independently. `app.h` exposes Pimpl classes so Linux types do not leak into the desktop build.

## 1. Application lifecycle — `src/main.cpp`

### Options and resource selection

`Options` holds mode flags, duration, camera index, and paths. `parse()` requires a value after each value-taking flag, rejects malformed numeric text, requires a positive duration in headless mode, and disallows a simulated or disabled-TPU `--check-coral` run. `--help` returns before any device is opened.

Defaults name resources within the working directory. `--config`, `--model`, `--edgetpu-lib`, and `--output` select explicit alternatives. `startup_config_path()` honors an explicit configuration; otherwise it reads `active_calibration.json`, falls back to an existing `calibration_16in.json`, then uses `delivery_config.json` on a first run. Relative active-file paths resolve against the preference directory. `--tracking-config` applies an explicitly supplied overlay. `--calibrate` opens live stationary capture at the 16-inch camera row and `--calibration-output` chooses its output. This mode rejects simulation, auto-start, headless operation, and time limits.

### Startup and ownership

The program limits OpenCV's internal workers, installs SIGINT/SIGTERM handlers, and determines whether Coral is required. The signal handler only sets a `sig_atomic_t`; normal control flow performs cleanup.

The Coral-only check constructs the inference service, performs warm-up, and exits. Normal startup loads/validates configuration, applies an optional tracking overlay, and attempts the matching boundary sidecar. Invalid saved preferences produce a warning without partially changing configuration. Calibration branches after these settings are loaded and before Coral or the delivery loop starts, so it preserves user preferences while opening only the capture services.

The core handle uses a custom `unique_ptr` deleter. Sensor and inference services use RAII objects, and a window guard closes the GUI on exceptions. Initialization failures therefore unwind resources acquired earlier in that startup sequence.

### Main-loop order

1. Read pending radius/save requests. Apply geometry through `core_set_boundary()` before motion can occur.
2. Obtain a snapshot from physical sensors or `DemoSource`.
3. Submit the newest camera frame for inference. Marker mode pairs a frame to the newest scan; detector/comparison modes use the completed inference source frame and a buffered scan near its host receive time.
4. Inspect acquisition and inference health. Track successful input startup and persistent stream failures.
5. Call `core_step()` for marker observations or `step_detector()` / `core_step_observations()` for detector candidates. Comparison runs an independent shadow track without a second moving robot.
6. Clear the consumed command, log state changes, and render when the display/snapshot schedule requires it.
7. Translate keyboard events into requests for the next iteration and sleep for the remaining loop budget.

The loop targets 50 Hz; rendering targets 30 Hz. These are scheduling intervals, not guaranteed device throughput. State changes are logged immediately, with periodic samples at about 10 Hz. Sensor startup has a 15-second freshness deadline; an established camera/LiDAR stream that stays stale for five seconds terminates acquisition. Missing a marker is handled separately as a tracking hold.

### Shutdown

A final snapshot is written before releasing acquisition and inference. A bounded live run returns failure if required streams/results are not fresh at its end. Exceptions print one error and return a nonzero process status.

## 2. Observation contracts — `src/runtime.h`

| Type/helper | Meaning |
| --- | --- |
| `Frame` | Immutable shared 640 × 480 packed BGR pixel vector; host receive seconds; application sequence; sensor timestamp in nanoseconds. |
| `Scan` | One complete vector of angle/range/quality returns, host receive seconds, and a sequence. Negative sequence means missing input. |
| `Inputs` | Bounded camera history, newest scan, and a latched acquisition error. |
| `Detection` | Zero-based class ID, score, and normalized XYXY box. |
| `CoralState` | Worker readiness, result freshness, source/result timestamps, inference duration, result sequence, and detections. |
| `clock_now()` | Monotonic seconds; wall-clock changes cannot affect freshness or motion integration. |
| `paired_frame()` | Nearest camera host receive timestamp to the scan. Before a scan exists, returns the newest frame for display. |
| `coral_fresh()` | Requires a ready worker, completed sequence, result age below 0.6 s, and source-frame age below 0.8 s. Future timestamps are rejected. |

Shared immutable pixels remain valid after the acquisition buffer is reused. Sensor timestamps are retained for diagnosis but are not treated as synchronized with host receive times.

## 3. Defaults and simulation — `src/runtime.cpp`

`default_config()` initializes every field, including the reference mount's calibration, marker thresholds, tracking gates, and robot geometry. Configuration loading replaces required fields and validates relationships.

`track_name()` and `robot_name()` convert enum values to UI/CSV labels and return `UNKNOWN` for out-of-range inputs.

`synthetic_frame()` inverts the local bearing map to draw a cyan marker where a figure at the requested XY position should appear. `synthetic_scan()` generates four quality returns spanning a compact surface. The normal segmentation and association code must recover the target from these inputs.

`DemoSource::update()` maintains a 24-frame history and advances camera and LiDAR sequences at 15 and 10 Hz respectively. Its 30-second scene moves the figure toward the lane, briefly removes the marker, holds the obstruction, then moves it aside. Simulation creates observations, not precomputed tracking or robot decisions.

## 4. Physical acquisition — `src/sensors.cpp`

### Shared state and error handling

`Sensors::Impl` owns the camera manager, acquired camera, stream configuration, buffer allocator, memory mappings, requests, serial channel, LiDAR driver, and scan worker. A mutex protects published `Inputs`. `error()` retains the first error so callbacks/threads never throw into a foreign event loop.

### Camera setup

`start_camera()` selects the requested camera index and requires a 640 × 480 RGB888 stream with four buffers. On the little-endian Pi, this format's bytes are BGR for downstream OpenCV use. It requests sYCC color space, approximately 15 FPS, and the maximum sensor crop when available.

The function validates buffer plane count, lengths, offsets, and stride; maps each packed buffer; attaches it to a request; connects the completion callback; starts capture; and queues the requests. Calibration assumes the selected image geometry remains stable.

### Completion callback

`camera_done()` ignores cancellation during shutdown. A successful frame uses `SensorTimestamp`, falling back to buffer metadata. Repeated nonzero sensor timestamps are not republished.

The callback starts DMA read synchronization, copies 480 rows into an owned packed vector while honoring actual source stride, ends synchronization, and publishes the frame under the mutex. It retains at most 24 frames, then reuses and requeues the request. Copying is essential: the producer may refill a mapped buffer while the tracker or TPU still reads its owned frame.

### LiDAR setup and worker

`start_lidar()` uses an explicit port or locates one CP2102N serial link. Multiple candidates require `--port`; no candidate uses `/dev/ttyUSB0`. The SDK opens 460800 baud, checks health, starts the motor and scanning, and reads HQ nodes with a 500 ms timeout. Ten consecutive retrieval failures become an acquisition error.

Each HQ node converts to degrees and meters:

```text
angle_deg = angle_z_q14 × 90 / 16384
range_m   = dist_mm_q2 / 4000
quality   = quality_byte >> SDK quality shift
```

Zero ranges are skipped. Sensor-relative angles remain intact until the core applies calibration. Only the newest completed scan is published.

### Cleanup and `camera_controls.h`

Destruction sets the stop flag, joins the LiDAR worker, stops its motor/connection, stops camera callbacks, frees requests, unmaps buffers, and releases the allocator/camera/manager. Startup-state flags make this work after partial construction.

`set_camera_frame_duration()` supplies two explicit `int64_t` microsecond values via a typed span. This matches `FrameDurationLimits` storage and avoids integer-type ambiguity. The libcamera test verifies the actual serialized representation and value lifetime without opening a camera.

## 5. Core data interface — `native/core.h`

`XY` values and all geometric lengths are meters. Raw and calibrated bearings are degrees; robot heading is radians from forward. Times are monotonic seconds. `Config` contains the complete experiment, and `Control` carries a command plus independently validated inference state.

`Result` uses bounded arrays: 16 markers, 64 clusters, 256 trail points, and 128 route waypoints. Counts delimit valid prefixes. `selected = -1` indicates no associated group; position values must be interpreted with tracking/selection validity.

The C-shaped API is linked directly into the executable:

- `core_create()` / `core_destroy()` allocate and release one engine.
- `core_step()` validates observations, tracks, advances the robot, and copies the result.
- `core_set_boundary()` updates clearance without resetting pose or perception.
- `core_plan()` exposes stateless planning to tests and diagnostic callers.
- `core_abi()` / `core_size()` describe the fixed-layout data contract.

One engine is used from one thread. Input memory is borrowed for the duration of the call; internal state and returned arrays own their values. Negative return codes indicate invalid arguments, insufficient path capacity, or internal exceptions, depending on the function.

## 6. Fusion and tracking — `native/core.cpp`

### Marker segmentation: `markers()`

Pixels inside the image ROI are converted to HSV using H = 0–179 and S/V = 0–255. A binary mask identifies pixels inside all three threshold intervals. Eight-connected flood filling accumulates component area, centroid, and bounds. Area/aspect gates produce marker candidates. The core implements this directly and does not require OpenCV.

### Metric clustering: `groups()`

For each finite, positive, adequate-quality range, the calibrated transform is:

```text
theta = radians(wrap(raw_angle - forward_angle))
x = range × sin(theta)
y = range × cos(theta)
```

Returns outside the metric ROI are discarded. Remaining returns are angle-sorted and split when either spatial or angular continuity fails. A valid group has enough points and a bounded diagonal extent; it reports mean XY, mean bearing, median range, count, and width.

### Association and confirmation: `Engine::track()`

Freshness is checked even when the scan sequence is unchanged. New sequences additionally require an acceptable camera/scan receive-time difference. Exactly one eligible visual observation is allowed: a marker centroid or the horizontal center of a detector box. Its horizontal centroid is interpolated between neighboring calibration anchors, with bounded extrapolation.

Exactly one group must lie within the bearing gate. No group is camera-only evidence; two groups are ambiguous. Both conditions invalidate tracking. Successful initialization requires consecutive spatially consistent scans. Long gaps force reacquisition; impossible jumps are rejected using a fixed allowance plus a maximum-speed term.

### Filtering and trail

Accepted observations update the filtered position using `alpha = 1 - exp(-dt / smoothing_time)`. The measured/raw position is retained independently. A trail records filtered positions for the configured duration, bounded to 256 points. Recovery after missing input starts a new segment, so the display does not draw a false connecting line through a gap.

## 7. Collision geometry and route planning — `native/core.cpp`

`Space` expresses allowable robot-center positions. Lane walls are inset by the robot radius. Each raw/filtered figure position is surrounded by the selected total exclusion radius returned by `exclusion_radius()`. A zero `boundary_radius` uses the nominal figure + robot + margin sum; a nonzero value overrides that sum. `BOUNDARY_MIN_MM` and `BOUNDARY_MAX_MM` define the shared inclusive 3–10 cm limits. Both the core API and application configuration reject effective radii outside this range.

`segment_distance()` projects an obstacle center onto a finite segment and clamps the projection to its endpoints. `Space::clear()` uses this distance to check the full swept path, not just sampled endpoints. Touching the modeled boundary is rejected. Because the lane rectangle is convex, in-lane endpoints also keep the straight segment within the lane.

`plan()` rejects occupied endpoints, accepts a clear direct segment, or runs A* on an eight-connected grid. Start and goal remain exact continuous coordinates and connect to nearby nodes only through checked segments. Edge costs and the heuristic use Euclidean distance. Stale priority-queue entries are ignored. Reconstructed routes are shortened with greedy visibility checks using the same geometry predicate.

`planning_space()` prefers up to 2 cm additional clearance, reduced near otherwise valid endpoints. `stable_plan()` retries that preference at 100%, 50%, 25%, and 0%. The hard collision radius never shrinks.

## 8. Robot decisions — `Engine::robot()`

The decision order is significant:

1. Cap elapsed integration time at 50 ms; apply reset, lane, trail, and run commands.
2. Update stop-sign confirmation only for distinct healthy inference sequences. Two positives latch a hold; fresh clear observations must persist 0.6 s to release it.
3. Honor pause, invalid tracking, required-TPU health, and the sign latch before planning.
4. Require 0.25 s of healthy settling after a recovery.
5. Hold if the robot overlaps either raw or filtered exclusion circle; latch delivery within 2 mm of the destination.
6. Preserve a valid route. Otherwise clear it, prefer a direct segment, yield, then plan a detour or report blocked.
7. Move by at most `speed × speed_scale × capped elapsed time`, checking each proposed segment before applying it. The default pacing fraction is 0.8, giving 2.24 cm/s at a nominal 2.8 cm/s. Pacing only changes the movement budget; sensor acquisition and decision timing remain independent. Configuration saves retain both values separately to preserve the same pace across calibration and restarts.

Heading turns at no more than 3 rad/s. The footprint is circular, so display orientation can smooth independently of positional collision checks. `core_set_boundary()` changes only the explicit total `boundary_radius`, clears the route/path count/yield timer, and preserves pause, pose, perception, and the semantic latch. Values outside 3–10 cm fail without changing state. The display and collision checker use this same selected total without adding another robot radius.

## 9. Coral runtime and inference

### Lifecycle — `src/coral_engine.h`

`Engine` owns the loaded library, delegate, model bytes/model handle, interpreter options, and interpreter. Startup checks the model, loads the private library, verifies its layout fingerprint and resolved TensorFlow Lite version, enumerates USB devices, and creates a USB delegate.

The interpreter uses one CPU worker and disables delegate fallback. SSD expects uint8 `[1,300,300,3]` input and four float outputs. YOLO validates its integer input/output tensors and selected profile: six output channels for figure/stop_sign, or 84 for COCO. The included custom model uses `[1,320,320,3]` INT8 input and `[1,6,2100]` INT8 output. Capacity, byte lengths, quantization, and class contracts are checked before a warm-up invocation can report readiness.

Cleanup destroys interpreter/options/model before the delegate and unloads the shared library last. Model bytes outlive the TFLite model handle. Errors use the same cleanup path.

### Inference and results

`infer()` prepares SSD with a direct resize. YOLO uses aspect-preserving RGB letterboxing and integer quantization from the tensor scale/zero point. The invocation timer surrounds `TfLiteInterpreterInvoke`, including USB/runtime and any CPU operators inside the graph. Separate measurements record preprocessing, decoding/NMS, and total worker time.

SSD parsing checks finite/in-range counts, scores, classes, and boxes and filters scores below 0.45. `src/yolo.h` dequantizes either supported tensor orientation, reads four box channels plus the selected class count, filters scores, reverses letterboxing, and performs class-aware non-maximum suppression. Its `person_observations()` helper is shared by COCO person and custom figure class zero; the name does not imply another detector. Supported neural-network operations execute on the TPU; the Pi handles remaining graph operations and application processing.

### Worker — `src/coral.cpp`

Construction warms up the engine, then starts one worker. `submit()` ignores invalid/duplicate frames and replaces the single pending frame. The worker moves that frame out under a mutex and releases the mutex during inference. This permits concurrent submission without accumulating a backlog.

Results carry a new sequence and both source-frame and completion times. Stop-sign observations use score at least 0.60 and the selected model's class ID: custom figure 1, COCO YOLO 11, or SSD 12. The core, rather than the worker, owns confirmation/release timing. `status()` copies the result and recomputes freshness after locking. Destruction wakes and joins the worker before destroying its engine.

### Identity/model helpers

`coral_abi.h` hashes sizes and offsets of TensorFlow Lite structures crossing the delegate boundary. `native/coral_runtime/runtime_identity.cpp` exports that hash and the linked TFLite version. These are structural checks, not a complete proof of binary compatibility.

`model_check.h` reads bounded FlatBuffer offsets, vtables, operator codes, and strings to require `edgetpu-custom-op`. It accepts the schema's byte and wider code fields. Full graph validation remains TensorFlow Lite's responsibility. Model files are size-bounded before allocation.

## 10. JSON and UI persistence — `src/config.cpp`

Scalar readers require numeric, finite values; integer readers reject fractions. Fixed-size array readers validate count and element type before copying. `read_tracking()` handles calibration, marker, LiDAR, and temporal sections. `load_config()` separately reads robot geometry, then validates all relationships. `overlay_tracking()` is invoked only for an explicit CLI path and preserves robot geometry.

Validation covers ordered anchors, HSV bounds, marker shapes, ROI/lane containment, thresholds, timing, and endpoints that fit even the narrow lane. The core's own invariants are checked through a temporary engine.

Boundary loading uses a temporary Config and commits only after validation. Saving stores schema ID and combined radius in a sidecar. `save_config()` records resolved runtime parameters for a session; descriptive `reference` metadata is not used by the engine.

## 11. Display and logging — `src/view.cpp`

Drawing helpers map meters to screen coordinates with Y forward/up. Fixed-point OpenCV drawing retains subpixel movement. `render()` composes the paired image, visual boxes, raw scan context, configured lane, selected target, trail, route, robot, status, ages, and boundary controls. Marker/comparison control draws the independent green marker candidates. Detector control skips that pass and draws one purple box per detection on its exact source frame, with the profile-appropriate figure/person label. This prevents duplicate tracker/detector boxes. HSV mask preview applies only to marker/comparison control.

Detector boxes are not painted over an unrelated newer image; the display reports inference age instead. Other LiDAR returns remain context rather than planner obstacles. Rendering only consumes state.

`BoundaryUi` stores integer millimeters, using atomics for requested/current values and save/drag flags. Button clicks and key events request changes; the main thread applies them. Slider limits enforce the inclusive 3–10 cm selected total radius. The notice string is owned by the application/render thread.

`SessionLog` creates a unique UTC-named directory and writes startup configuration and mode metadata. CSV uses empty fields for invalid positions. Snapshots contain rendered PNG, camera PNG, raw LiDAR CSV, and structured JSON with validity, route, boundary, timing, and detections. Write failures propagate as exceptions.

## 12. Build, support scripts, and tests

`CMakeLists.txt` builds the portable core first. `TABLETOP_BUILD_APP=OFF` leaves only dependency-light tests. The application adds OpenCV and either Linux hardware sources or `no_hardware.cpp`. Hardware-off stubs reject live calls explicitly. The selected backend is a build decision; `--demo` is a runtime choice.

`build_pi.sh` builds the chosen SDK, configures hardware dependencies, compiles, and runs CTest. `fetch_model.sh` downloads the optional SSD detector; the custom compiled figure model is bundled with its checksum. The export notebooks/scripts prepare models offline and are not part of the live C++ process. `build_coral_runtime.sh` downloads pinned source, verifies it, builds into `native_runtime/`, and preserves its license notice. `diagnose.sh` collects a read-only dependency/device inventory; `debug_coral.sh` rebuilds symbols and captures the Coral-only debugger trace.

The private runtime's CMake recipe derives the upstream C++ source list, prepares two delegate wrappers against the installed public TFLite API, generates FlatBuffer and firmware headers, and links TensorFlow Lite, USB, FlatBuffers, and Abseil. Adjusted wrapper copies stay in the build directory; upstream source files remain intact. Required symbols must resolve at link time.

The [testing guide](TESTING.md) describes each test file and the difference between desktop verification, target-header compilation checks, and real hardware execution.

## 13. Measured camera-row calibration — `src/calibration.*`

`calibration.h` defines the 0.4064 m horizontal camera reference, 0.028 m camera-center height, twenty-pair stability window, individual observation records, and aggregated poses. Distance and height are user-measured setup geometry shown in the capture window and recorded in both saved artifacts. They never replace or correct a LiDAR surface range; actual image/bearing alignment comes from new captures at that mounted height.

`calibration_capture_config()` widens diagnostic metric/image bounds so the nearby figure can be observed before new anchors exist. `run_calibration()` opens the real sensor service and a separate capture window without a Coral instance. The ordinary core supplies candidate markers and clusters, but its automatic bearing association is ignored for capture. A mouse click chooses a surface position; subsequent samples require exactly one nearby cluster, one image marker, fresh receive-time pairing, and distinct camera/scan identities. No start command is sent to the robot.

`summarize_calibration()` checks finite data, at least twenty observations, pixel spread, metric spread, and angular consistency. Pixel/range values use arithmetic means; raw bearings use a circular mean to handle zero-degree wraparound. Pressing Space commits the stable aggregate for CENTER, LEFT, then RIGHT and stores a diagnostic camera/scan snapshot. Each placement requires a new explicit selection.

`fit_calibration()` defines LiDAR forward from the measured center bearing. The two side bearings are expressed relative to that direction, then paired with measured image columns. It rejects incomplete or swapped placements, inadequate anchor spacing, and excessive forward-depth differences. It preserves robot geometry and the selected boundary while extending observation bounds for the measured nearer row and marker size.

Accepting the third pose automatically fits, saves, and activates the calibration. `save_calibration()` stages the configuration and numeric `.measurements.json` report beside their destinations, validates the staged configuration through the production reader, retains backups, and publishes complete files by rename. Cleanup removes only temporary files. `activate_calibration()` validates the published configuration, stores a project-relative path in `active_calibration.json`, and publishes that preference through the same backup/rename mechanism. The UI reports success only after both operations complete. S retries a failure while preserving the measured poses; R starts over. Existing output filenames can be reused.

Explicit `--config` bypasses saved selection even when the active record is invalid. Automatic startup validates the selected calibration and never silently falls back from corrupt or missing active data. Boundary preferences are loaded beside the resolved calibration file. The capture window's scope guard handles close-button cancellation, and sensor ownership stops acquisition on every exit path. The [physical walkthrough](CALIBRATION.md) describes ruler placement and subsequent delivery validation.
