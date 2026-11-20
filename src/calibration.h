/**
 * @file
 * Measured camera/LiDAR calibration on a row 16 inches from the camera.
 * The ruler distance describes physical placement; it never rescales LiDAR ranges.
 */
#pragma once // Share calibration contracts without duplicate definitions.
#include "app.h"
#include <array> // Store the three independently measured poses in center, left, right order.
#include <csignal> // Reuse the application's signal-safe shutdown flag in the interactive capture loop.

namespace tabletop {
inline constexpr double CALIBRATION_CAMERA_DISTANCE_M = 16 * .0254; // Sixteen inches is exactly 0.4064 meters from the camera lens along the table.
inline constexpr double CALIBRATION_CAMERA_HEIGHT_M = .028; // The camera lens center is 28 mm above the tabletop; this is setup geometry, not a LiDAR range correction.
inline constexpr int CALIBRATION_SAMPLES = 20; // Require twenty distinct paired observations at each stationary placement.
struct CalibrationSample { // One manually identified LiDAR cluster paired with the unique blue camera marker.
    Marker marker{}; // Preserve measured centroid, bounds, and area without using the existing bearing anchors.
    Group group{}; // Preserve observed surface position, range, and bearing in the capture configuration's frame.
    double raw_angle = 0; // Sensor-relative cluster bearing in degrees, before choosing the new forward direction.
};
struct CalibrationPose { // Stable aggregate of actual observations for one center, left, or right placement.
    double u = 0, raw_angle = 0, range = 0; // Mean image column, circular-mean raw bearing, and mean measured surface range.
    XY surface{}; // Mean measured surface position in the capture coordinate system; no base-center range correction is applied.
    int top = 480, bottom = 0, max_area = 0, samples = 0; // Bounds/area support expanding the image ROI for the closer marker.
};
Config calibration_capture_config(const Config &base); // Widen observation bounds to include a nearby prop without relying on its previous bearing association.
CalibrationPose summarize_calibration(const std::vector<CalibrationSample> &samples); // Reject insufficient, invalid, or moving samples before aggregating a pose.
Config fit_calibration(const Config &base, const std::array<CalibrationPose, 3> &poses); // Fit measured anchors in center/left/right order and preserve robot settings.
void save_calibration(const Config &config, const std::array<CalibrationPose, 3> &poses,
                      const std::filesystem::path &path); // Validate, save, and replace a calibration through temporary files, retaining backups of existing outputs.
std::filesystem::path startup_config_path(const std::filesystem::path &requested, bool explicitly_selected); // Honor explicit --config; otherwise prefer the active calibration, the default calibration file, then the seed configuration.
void activate_calibration(const std::filesystem::path &config, const std::filesystem::path &record = "active_calibration.json"); // Remember a validated calibration for normal startup, storing its path relative to the activation record.
cv::Mat calibration_preview(const Config &capture, const Inputs &inputs, const Frame &frame,
                            const Result &result, int stage, int selected, int samples,
                            const std::string &message); // Render capture instructions and selectable LiDAR clusters without modifying sensor state.
int run_calibration(const Config &base, const std::string &port, int camera,
                    const std::filesystem::path &destination, const std::filesystem::path &logs,
                    const volatile std::sig_atomic_t &quit); // Space accepts each pose; the third pose automatically saves/activates, S retries failures, and Q/Esc cancels.
} // namespace tabletop
