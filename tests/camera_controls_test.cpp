/**
 * @file
 * Validate camera-control representation using the installed libcamera.
 * Check the stored element type, count, byte length, and typed roundtrip.
 * No camera device is acquired by this test.
 */

#include "../src/camera_controls.h"
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.

int main() {
    try {
        libcamera::ControlList controls(libcamera::controls::controls); // Exercise the actual installed libcamera ControlList serialization; no camera, USB accelerator or LiDAR is opened by this regression. Use the installed library's real control definitions instead of mocking serialization.
        tabletop::set_camera_frame_duration(controls); // Invoke the production helper that requests a fixed approximately 15 fps period.
        const auto &raw = controls.get(libcamera::controls::FrameDurationLimits.id()); // Retrieve the untyped stored value before any typed accessor could assert on a mismatch.
        if (raw.type() != libcamera::ControlTypeInteger64 || !raw.isArray() ||
            raw.numElements() != 2 || raw.data().size() != 2 * sizeof(std::int64_t)) // Require an int64 array with exactly two elements and 16 bytes of payload.
            throw std::runtime_error("FrameDurationLimits must store two 64-bit integers");
        const auto values = controls.get(libcamera::controls::FrameDurationLimits); // Check the representation before typed retrieval, which asserts on an incorrect element type. Verify that values outlive the helper's array. Read through the typed control accessor only after validating the raw representation.
        if (!values || (*values)[0] != 66666 || (*values)[1] != 66666) // Confirm both 66,666 microsecond values survived the helper's local array lifetime.
            throw std::runtime_error("Frame-duration roundtrip changed the requested 15 fps");
        std::cout << "PASS: libcamera frame-duration type, size and roundtrip\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
