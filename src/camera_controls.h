/**
 * @file
 * Typed camera-control construction shared with the serialization test.
 * FrameDurationLimits expects a pair of int64 microsecond values. The explicit
 * span ensures both storage type and element count match libcamera's contract.
 */

#pragma once // Include this header only once per translation unit.
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <array> // Fixed-size standard arrays used where element count is part of the data contract.
#include <cstdint> // Explicit-width integer types for binary contracts, timestamps, and sequence values.
#include <span> // Non-owning typed array views for libcamera controls with explicit element counts.

namespace tabletop {
inline void set_camera_frame_duration(libcamera::ControlList &controls) { // Populate the fixed-frame-duration control using libcamera's expected int64 element type.
    const std::array<std::int64_t, 2> durations{66666, 66666}; // About 15 fps, in microseconds. The initializer-list overload in the Pi's libcamera deduces {66666,66666} as int32, although this control requires int64. Pass a typed span to select the type-safe overload. Two equal 66,666 microsecond limits request a fixed frame period near 15 fps.
    controls.set(libcamera::controls::FrameDurationLimits,
                 std::span<const std::int64_t, 2>{durations}); // Pass a fixed-length typed span so libcamera stores two int64 values, not an int32 initializer list.
}
} // namespace tabletop
