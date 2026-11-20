# Testing and validation

## Build profiles

The core is independent of OpenCV and hardware libraries:

```bash
cmake -S . -B build-core -DCMAKE_BUILD_TYPE=Release -DTABLETOP_BUILD_APP=OFF
cmake --build build-core --parallel 2
ctest --test-dir build-core --output-on-failure
```

For a Linux desktop application with OpenCV:

```bash
cmake -S . -B build-desktop -DCMAKE_BUILD_TYPE=Release -DTABLETOP_HARDWARE=OFF
cmake --build build-desktop --parallel 2
ctest --test-dir build-desktop --output-on-failure
./build-desktop/tabletop_delivery --demo
```

With a Windows multi-configuration generator, use `--config Release` when building and `-C Release` with CTest. The executable is then typically `build-desktop/Release/tabletop_delivery.exe`. Set `OpenCV_DIR` to the directory containing `OpenCVConfig.cmake`, and make its runtime DLL directory available on `PATH`.

For the Pi, follow the root README and run `bash build_pi.sh`. The script builds hardware backends and executes CTest. Compilation and device-independent tests do not prove physical acquisition works; run the hardware checks below as well.

## Test inventory

| CTest entry | Source | What it checks |
| --- | --- | --- |
| `yolo` | `tests/yolo_test.cpp` | Quantized decoding, both tensor orientations, class-aware NMS, letterbox inversion, profile mismatch rejection, and time-paired detector observations. |
| `evaluation_ui` | `tests/evaluation_ui_test.cpp` | Comparison logs, profile-specific headings, one detector box per observation, and suppression on mismatched source frames. |
| `evaluation_cli` | `tests/evaluation_cli_test.cmake` | Model/profile selection, threshold aliases, and conflicting mode rejection. |
| `planner` | `tests/planner_test.cpp` | Direct paths, detours, blocked passages, occupied endpoints, and 252 obstacle positions; validates each returned segment. |
| `runtime` | `tests/runtime_test.cpp` | 22 behavior cases covering segmentation/fusion, measured travel speed and pause at different pacing fractions, movement, loss/staleness, ambiguity, sign latching, duplicate inference, stalls, jitter, raw intrusion, heading limits, and boundary changes. |
| `model` | `tests/model_test.cpp` | Rejects non-model input, invalid offsets, and an operator name appearing only in unrelated data. Optional CLI argument checks a real compiled model. |
| `config` | `tests/config_test.cpp` | Configuration roundtrip and rejection; omitted pacing defaults, custom pacing persistence, repeated saves without compounding speed; boundary persistence at 3, 7.2, and 10 cm; rejection outside the limits without mutation; keyboard/button and mouse request clamping; preservation of calibration; rendering without a window. |
| `calibration` | `tests/calibration_test.cpp` | Stable measured poses, angular wraparound, sensor-origin separation, saved 16-inch reference and 28 mm camera height, repeated saves/backups, default and custom activation, explicit override, failed-save preservation, invalid active-record rejection, closer-row tracking, and rendering. Synthetic evidence does not certify physical placement. |
| `camera_controls` | `tests/camera_controls_test.cpp` | Actual libcamera storage type, element count, byte length, lifetime, and typed retrieval for frame duration. Hardware-enabled builds only; no device is opened. |
| `app_help` | Application CLI | Help exits successfully without loading devices. |
| `app_demo` | Application CLI | Bounded headless simulation using an explicit configuration. |
| `app_resources` | `tests/resource_paths_test.cmake` | Launch from isolated directories, save a final snapshot, select active calibration without --config and apply default pacing when omitted, honor explicit overrides, and reject missing explicit or active configurations. |

The core-only profile contains four entries, the desktop application eleven, and the hardware application twelve. Runtime behavior cases are subtests within the `runtime` entry.

To inspect the downloaded model independently on the Pi or a desktop build:

```bash
./build/model_test models/figure_yolov8n_320_edgetpu.tflite
```

That parser check does not invoke the TPU. The inference check below does.

## Hardware acceptance checks

1. Enumerate the camera and serial adapter; ensure your user can access both and the Coral.
2. Run `./build/tabletop_delivery --yolo-profile figure --check-coral` and require the native USB warm-up PASS message.
3. Run `./build/tabletop_delivery --yolo-profile figure --tracking yolo --headless --seconds 20` for bounded acquisition and inspect the resulting session records.
4. Open the live window, verify figure/cluster alignment at several calibrated positions, then start virtual motion.
5. Exercise direct movement, intrusion, detour, blocked narrow lane, figure occlusion, and boundary expansion/shrinking.
6. Save a radius, restart, and verify restoration; inspect a snapshot's radius and validity flags.

## Package verification

Verification performed on 20 November 2026 for the merged package:

- Built the complete desktop application with MSVC and OpenCV 4.10.
- Passed all 11 desktop CTest entries, including custom-model decoding, detector CLI selection, the duplicate-overlay regression, calibration persistence, 22 runtime cases, and the 252-position planner sweep.
- Checked the bundled compiled model with the bounded model parser and its SHA-256 checksum.
- Checked documentation/media links and ZIP/tar contents, excluding machine-specific configuration and build/runtime caches.
- Reviewed both supplied physical demo recordings: nominal/narrow-lane delivery and stop-sign hold/release, using the custom model and corrected overlay.

Desktop tests do not invoke physical Pi camera buffers, serial LiDAR, or USB
inference. The recordings provide qualitative hardware evidence for the user's
setup; run the acceptance checks after deployment to another installation.
Calibration depends on the physical mount. No end-to-end latency distribution,
generalization guarantee, or CPU-versus-TPU speedup is asserted.

## Maintaining the code

Source style is defined in `.clang-format`; shell scripts use LF line endings through `.gitattributes`. Keep algorithm changes covered by behavior or geometry tests rather than tests that only reproduce implementation details. Keep C++ comments focused on units, ownership, state transitions, invariants, and failure conditions. The [annotation guide](ANNOTATED_SOURCE.md) explains how statement comments cover wrapped expressions, guards, and scopes.

Build output, SDK/runtime downloads, optional generated detectors, run data, and local preferences are ignored by Git. The custom figure model, checksum, labels, manifest, and demo assets are deliberately tracked. Documentation images are deliberate example assets and remain tracked. Dependency identities and notices are listed in [THIRD_PARTY.md](../THIRD_PARTY.md).
