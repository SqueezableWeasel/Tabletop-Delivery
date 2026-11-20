# Reading the annotated source

The C++ source places explanations on the same line as the code, after `//`. The comments explain what the statement does and, where useful, why it is needed: units, coordinate conventions, timing limits, resource lifetime, failure handling, and the behavior visible in the demo. Test sources use the same format for input fixtures and expected outcomes. Shell and CMake files use trailing `#` comments to explain dependency discovery, verification, compilation, and packaging operations.

```cpp
const auto &m = mappings.at(buffer); // Retrieve the previously validated mapping for this completed buffer.
```

## What a comment covers

A trailing comment applies to the statement or construct on that line. For a multiline expression, the explanation usually follows its final line; a block's explanation follows its opening code line. A comment beside an `if`, loop, or exception guard explains the condition and the operation it controls. For example, an error guard's explanation also covers the following `throw`; the exception's message supplies the specific failure detail.

Blank lines, closing braces, continuation lines, and repeated string fragments do not each receive a separate description. Those lines belong to the surrounding documented statement or scope. Function and section explanations sit beside the code that introduces their scope. File-level `@file` overview blocks remain at the top of each C++ file.

The formatter uses `ColumnLimit: 0`, `ReflowComments: false`, and a single space before trailing comments so it keeps explanations beside their code. Long explanations may require horizontal scrolling. Shell comments follow the final line of a backslash-continued command so the continuation character remains the last character on its line.

JSON cannot contain comments. Every configuration field is documented separately in [Configuration](CONFIGURATION.md), and the C++ configuration readers, defaults, and writers also describe the fields they handle. Downloaded upstream SDK/runtime sources, generated build products, and model bytes are not rewritten by this project's annotations.

## Suggested reading order

| File | Follow these details |
| --- | --- |
| [native/core.h](../native/core.h) | Units, calibration arrays, tracking and robot states, fixed-capacity result arrays, and the exported engine interface. |
| [src/runtime.h](../src/runtime.h) | Immutable camera pixels, scan identity, monotonic timestamps, receive-time pairing, and inference freshness. |
| [src/main.cpp](../src/main.cpp) | Command-line validation, configuration loading, backend lifetime, the main loop, live boundary requests, logging, and shutdown. |
| [native/core.cpp](../native/core.cpp) | HSV conversion, connected components, LiDAR clustering, camera association, confirmation/filtering, continuous collision checks, A* search, and virtual movement. |
| [src/sensors.cpp](../src/sensors.cpp) | Camera setup, typed controls, DMA mappings and copies, serial decoding, publication locks, and cleanup order. |
| [src/coral_engine.h](../src/coral_engine.h) | Model validation, ABI/version checks, USB selection, tensor contracts, warm-up, preprocessing, inference, and output decoding. |
| [src/coral.cpp](../src/coral.cpp) | The single pending-frame slot, worker wakeup, immutable source ownership, result publication, and shutdown. |
| [src/view.cpp](../src/view.cpp) | Metric-to-pixel transforms, valid-state overlays, slider requests, and diagnostic output. |
| [src/config.cpp](../src/config.cpp) | Checked numeric parsing, cross-field validation, explicit overlays, and radius-only persistence. |
| [src/calibration.h](../src/calibration.h) | Sixteen-inch horizontal reference, 28 mm camera-center height, paired-sample and pose contracts, and capture/fit/persistence interfaces. |
| [src/calibration_math.cpp](../src/calibration_math.cpp) | Stability checks, circular bearing averages, ordered measured anchors, observation-region adaptation, and separate configuration output. |
| [src/calibration.cpp](../src/calibration.cpp) | Live cluster selection, fresh distinct-pair collection, three physical placements, capture-window rendering, and explicit saving. |
| [tests/runtime_test.cpp](../tests/runtime_test.cpp) | Deterministic clock advancement, sensor fixtures, scenario setup, and externally visible movement/hold assertions. |

The [code walkthrough](CODE_GUIDE.md) explains all modules and their relationships. The [test inventory](TESTING.md) identifies the remaining test files and their scope.

## Conventions used in the explanations

**Position and angle.** Core positions and distances use meters. Positive x points right; positive y points forward from the calibrated LiDAR reference. Raw/calibrated bearings use degrees, while the displayed robot heading uses radians from forward. Image coordinates use pixels, with v increasing downward. The slider requests integer millimeters; the display commonly labels centimeters.

**Time.** Acquisition, pairing, freshness, and integration use monotonic host seconds. Camera sensor timestamps are separate nanosecond metadata. Receiving two observations near each other does not establish hardware synchronization. Wall-clock UTC is used for session directory names only.

**Ownership.** `unique_ptr` owns an object exclusively and releases it at scope exit. Camera frames use shared immutable pixel storage so rendering and inference can retain a frame after the capture request is reused. A borrowed pointer is valid only while its owning buffer, model, interpreter, or library remains alive. Cleanup comments identify those dependencies explicitly.

**Shared state.** Scope-bound mutex locks protect published observation/result snapshots. Atomics carry small boundary/shutdown requests where appropriate. The main thread alone updates tracking/planning state and applies live geometry changes; callbacks request changes rather than directly editing the planner.

**Validity and bounds.** Counts identify the valid prefixes of fixed-capacity arrays. Negative sequence or selection values represent missing observations. A stored coordinate is not necessarily currently valid: consumers must inspect its tracking/selection status. Comments around early exits describe why an observation cannot authorize movement.

**Geometry.** The figure boundary is the selected total exclusion radius, constrained to 3–10 cm. Without an explicit selection, it starts at the nominal figure radius plus robot radius plus extra margin. Live changes update the total independently of those nominal settings. Planning may prefer additional clearance, but movement is checked against the mandatory boundary. Both raw and filtered figure positions constrain motion. The on-screen rectangular robot artwork does not replace the circular collision footprint.

## Keeping explanations accurate

When changing behavior, update the nearby explanation and the relevant configuration or algorithm guide together. Check numerical units and whether a value is a measurement, a filtered estimate, a display conversion, or an assumption of this tabletop demo. Tests should continue to check observable behavior and independent geometry rather than simply reproducing the implementation.

Annotations themselves do not change the executable algorithms or configuration. The package verification described in [Testing](TESTING.md) includes a comparison of C++ token streams and executable script text, plus compilation and the existing behavior tests.
