# Dependencies and notices

The repository contains application source, tests, documentation, and build recipes. Dependency checkouts, runtime source downloads, compiled libraries, and model binaries are generated locally and excluded from Git. Keep upstream license and attribution files with any dependency artifacts you redistribute.

| Dependency | Use | Upstream |
| --- | --- | --- |
| OpenCV 4 | Rendering, JSON storage, image preparation | [OpenCV](https://github.com/opencv/opencv) |
| libcamera | CSI camera acquisition | [Raspberry Pi libcamera](https://github.com/raspberrypi/libcamera) |
| SLAMTEC RPLIDAR SDK | Serial LiDAR scan acquisition | [SDK source](https://github.com/Slamtec/rplidar_sdk) |
| TensorFlow Lite | Interpreter and delegate API | [TensorFlow source](https://github.com/tensorflow/tensorflow/tree/v2.20.0/tensorflow/lite) |
| libedgetpu | Coral USB execution | [Pinned source](https://github.com/google-coral/libedgetpu/tree/e35aed18fea2e2d25d98352e5a5bd357c170bd4d) |
| FlatBuffers, Abseil, libusb | Private runtime build/runtime dependencies | Installed development packages and their notices |
| SSD MobileNet V2 COCO | Edge TPU detector | [Coral test_data](https://github.com/google-coral/test_data) |

## Download identities

The private runtime helper pins libedgetpu commit:

```text
e35aed18fea2e2d25d98352e5a5bd357c170bd4d
```

The expected source archive SHA-256 is:

```text
4e0a49c767104438e5eebd638fe814c9c5776f5173a27684ed4ecb401173872d
```

The model SHA-256 is documented in [models/README.md](models/README.md) and enforced by its fetch helper. Record the LiDAR SDK commit used for each reproducible build. The installed TensorFlow Lite headers and runtime must match the private delegate build.

`native/coral_runtime/CMakeLists.txt` generates adjusted delegate wrapper copies in its build directory and embeds the upstream firmware for compilation. The downloaded upstream files remain intact, and the install helper copies libedgetpu's license into `native_runtime/`.

## Project assets

The hardware photograph is supplied by the project owner. The interface screenshot is generated from the included simulator. The diagrams and documentation describe this implementation. No affiliation with a robotics company is asserted.

A license for the project's own source has not been selected. This file records dependencies and attribution; it does not assign a project license.
