/**
 * @file
 * Application-layer contracts shared by hardware, UI, and the main loop.
 * Pimpl classes hide Linux-specific dependencies from desktop builds. Frames
 * own their pixels; rendering/logging consume snapshots rather than driver buffers.
 */

#pragma once // Include this header only once per translation unit.
#include "runtime.h"
#include <filesystem> // Portable paths, directory creation, resource discovery, and session output locations.
#include <fstream> // File streams for configuration, detector bytes, and run diagnostics.
#include <atomic> // Atomic flags and values used for communication without unsynchronized shared writes.
#include <opencv2/core.hpp>
namespace tabletop {
Config load_config(const std::filesystem::path &path); // Load and validate a complete experiment; throws with a field-specific error.
void overlay_tracking(Config &config, const std::filesystem::path &path); // Apply a caller-selected tracking section while preserving robot geometry.
void validate_config(const Config &config); // Throw if any supported field type, range, or cross-field geometry invariant is invalid.
void save_config(const Config &config, const std::filesystem::path &path); // Serialize the effective operational settings for reproducible run diagnostics.
bool load_boundary_setting(Config &config, const std::filesystem::path &path); // Return false when absent; invalid sidecars throw without changing Config.
void save_boundary_setting(const Config &config, const std::filesystem::path &path); // Save only the selected total figure exclusion radius and its sidecar schema version.
struct BoundaryUi { // Widget state: callbacks post atomics, the main thread applies and saves values. notice is owned by the application/render thread, not written by callbacks.
    int minimum_mm, maximum_mm; // Slider limits in integer millimeters; the minimum preserves both physical body radii.
    std::atomic<int> current_mm, requested_mm; // Applied and requested radii are separate so callbacks never mutate engine state directly.
    std::atomic<bool> save_requested{false}, dragging{false}; // One-shot save request and current mouse-drag state shared with the UI callbacks.
    std::string notice = "B or Save to remember"; // Main/render-thread-owned text indicates whether the applied boundary has been saved.
    explicit BoundaryUi(const Config &config); // Initialize widget limits and requested/applied radius from validated settings.
    void adjust(int delta_mm); // Request a bounded radius change in millimeters; the main loop applies it before motion.
    void mouse(int event, int x, int y, int flags); // Translate display mouse events into radius changes or a save request.
};
class Sensors { // RAII owner of the physical camera and LiDAR. snapshot() is thread-safe; construction starts acquisition and destruction stops workers/motor/buffers.
    struct Impl; // Forward declaration keeps hardware/thread implementation details out of this public header.
    std::unique_ptr<Impl> impl_; // Exclusive implementation ownership ensures deterministic cleanup through the destructor.

  public:
    Sensors(const std::string &port, int camera_index); // Open the selected camera and serial LiDAR, or throw after releasing partial resources.
    ~Sensors(); // Stop acquisition and join workers before freeing their buffers and device handles.
    Inputs snapshot(); // Return a mutex-protected copy; immutable pixel storage is shared rather than recopied.
};
class Coral { // RAII inference service. submit() keeps only the newest pending frame; status() returns a copy with freshness evaluated against the current host clock.
    struct Impl; // Forward declaration keeps hardware/thread implementation details out of this public header.
    std::unique_ptr<Impl> impl_; // Exclusive implementation ownership ensures deterministic cleanup through the destructor.

  public:
    Coral(const std::string &model, const std::string &library); // Load the detector and matching USB runtime, warm up, then start the inference worker.
    ~Coral(); // Signal and join the worker before destroying its interpreter and USB delegate.
    void submit(const Frame &frame); // Replace the single pending frame; invalid and duplicate sequences are ignored.
    CoralState status(double now); // Copy the latest result and recompute freshness against the current host clock.
};
cv::Mat render(const Config &config, const Inputs &inputs, const Frame &frame, const Result &result,
               const CoralState &coral, bool coral_required, bool demo, bool mask, double now,
               double step_ms, const BoundaryUi *boundary = nullptr); // Create a standalone 1280x800 BGR visualization from a coherent input/result pair.
class SessionLog { // Own one run directory and CSV stream. Snapshot numbering is per session. Methods are called from the main thread and propagate filesystem errors.
    std::filesystem::path path_; // Unique output directory for this live or simulated session.
    std::ofstream csv_; // Open session CSV stream; destructor closes the underlying file automatically.
    int snapshot_ = 0; // Per-run monotonically increasing snapshot directory number.

  public:
    SessionLog(const std::filesystem::path &root, const Config &, bool demo, bool coral_required); // Create a unique session directory and write configuration, metadata, and CSV headers.
    const std::filesystem::path &path() const { // Expose the session directory without transferring ownership or permitting mutation.
        return path_; // Return a reference valid for the lifetime of this SessionLog instance.
    }
    void record(double elapsed, const Frame &, const Scan &, const Result &, const CoralState &); // Write one timestamped tracking/robot/Coral observation to the session CSV.
    void snapshot(const cv::Mat &screen, const Frame &, const Scan &, const Result &,
                  const CoralState &); // Save the displayed image, paired camera image, raw scan, and structured result together.
};
} // namespace tabletop
