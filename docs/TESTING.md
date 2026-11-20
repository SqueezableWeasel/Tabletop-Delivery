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
| `planner` | `tests/planner_test.cpp` | Direct paths, detours, blocked passages, occupied endpoints, and 252 obstacle positions; validates each returned segment. |
| `runtime` | `tests/runtime_test.cpp` | 21 behavior cases covering segmentation/fusion, movement, loss/staleness, ambiguity, sign latching, duplicate inference, stalls, jitter, raw intrusion, heading limits, and boundary changes. |
| `model` | `tests/model_test.cpp` | Rejects non-model input, invalid offsets, and an operator name appearing only in unrelated data. Optional CLI argument checks a real compiled model. |
| `config` | `tests/config_test.cpp` | Configuration roundtrip and rejection; saved boundary load/save and preservation of calibration; rendering without a window. |
| `camera_controls` | `tests/camera_controls_test.cpp` | Actual libcamera storage type, element count, byte length, lifetime, and typed retrieval for frame duration. Hardware-enabled builds only; no device is opened. |
| `app_help` | Application CLI | Help exits successfully without loading devices. |
| `app_demo` | Application CLI | Bounded headless simulation using an explicit configuration. |
| `app_resources` | `tests/resource_paths_test.cmake` | Launch from an isolated directory with local config, save a final snapshot, and reject a missing explicit config. |

The core-only profile contains three entries, the desktop application seven, and the hardware application eight. Runtime behavior cases are subtests within the `runtime` entry.

To inspect the downloaded model independently on the Pi or a desktop build:

```bash
./build/model_test models/ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite
```

That parser check does not invoke the TPU. The inference check below does.

## Hardware acceptance checks

1. Enumerate the camera and serial adapter; ensure your user can access both and the Coral.
2. Run `./build/tabletop_delivery --check-coral` and require the native USB warm-up PASS message.
3. Run `./build/tabletop_delivery --headless --seconds 20` for bounded acquisition and inspect the resulting session records.
4. Open the live window, verify marker/cluster alignment at several calibrated positions, then start virtual motion.
5. Exercise direct movement, intrusion, detour, blocked narrow lane, marker occlusion, and boundary expansion/shrinking.
6. Save a radius, restart, and verify restoration; inspect a snapshot's radius and validity flags.

## Package verification

Verification performed on 20 November 2026:

- Compiled the complete desktop application and tests using MSVC and OpenCV 4.10.
- Passed all seven desktop CTest entries, including 21 runtime cases and the 252-position planner sweep.
- Passed ARM64 compiler checks for sensor, Coral, main-loop, display, configuration, core, and camera-control-test sources against the available Pi headers.
- Ran a bounded 20-second simulation and inspected its final snapshot and state transitions.
- Confirmed that relocating explanations to trailing comments preserves the C++ token streams in all 21 source/header files, the executable command text in all eight shell/CMake files, every explanation's text, and the configuration values.
- Checked local documentation links, code formatting, shell helper paths, and archive contents.

These checks do not execute physical Pi camera buffers, LiDAR scans, USB inference, or Linux build/download scripts for this package. Complete the hardware acceptance checks on the target Pi. The included calibration is specific to the reference mount. No measured end-to-end frame-rate guarantee, latency distribution, or CPU-versus-TPU speedup is asserted.

## Maintaining the code

Source style is defined in `.clang-format`; shell scripts use LF line endings through `.gitattributes`. Keep algorithm changes covered by behavior or geometry tests rather than tests that only reproduce implementation details. Keep C++ comments focused on units, ownership, state transitions, invariants, and failure conditions. The [annotation guide](ANNOTATED_SOURCE.md) explains how statement comments cover wrapped expressions, guards, and scopes.

Build output, SDK/runtime downloads, detector binaries, run data, and local preferences are ignored by Git. Documentation images are deliberate example assets and remain tracked. Dependency identities and notices are listed in [THIRD_PARTY.md](../THIRD_PARTY.md).
