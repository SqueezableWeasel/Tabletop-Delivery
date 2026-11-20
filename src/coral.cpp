/**
 * @file
 * Asynchronous Coral inference and latest-result publication.
 * One pending frame bounds latency and memory. The inference worker owns all
 * interpreter calls; callers only submit shared frames or copy a result snapshot.
 */

#include "coral_engine.h"
#include <condition_variable> // Blocking wakeup for the inference worker instead of continuously polling for a frame.
#include <mutex> // Scope-bound locks protecting snapshots exchanged between acquisition/inference and the main loop.
#include <thread> // Background acquisition/inference workers or main-loop pacing.

namespace tabletop {
struct Coral::Impl { // Construct and warm up the interpreter before marking ready. Only the worker invokes it after startup; shutdown joins that worker before destroying Engine.
    Engine engine; // Own the checked interpreter and USB delegate; it outlives the worker using it.
    std::mutex mutex; // Protect the pending frame, result state, and shutdown flag.
    std::condition_variable wake; // Wake the worker only for a submitted frame or shutdown request.
    bool stopping = false; // Shutdown state is read and written while holding the mutex.
    Frame pending; // One waiting frame bounds inference backlog; a newer submission replaces it.
    std::uint64_t submitted = 0; // Last submitted camera sequence prevents reinference of the same image.
    CoralState state; // Latest published inference result and startup/error information.
    std::thread worker; // Exclusive post-startup caller of the interpreter's inference routine.
    Impl(const std::string &model, const std::string &library) : engine(model, library) { // Warm up the interpreter before launching any worker or publishing readiness.
        state.ready = true; // Engine construction succeeded, including a real USB-delegate model invocation.
        state.detail = "USB delegate ready; waiting for a camera frame"; // Tell consumers that the service is ready but no camera inference result exists yet.
        worker = std::thread([this] { // Start asynchronous inference while the main application continues tracking and rendering.
            try {
                int sequence = 0; // Count completed inference results separately from camera frame sequences.
                for (;;) { // Run until an explicit shutdown request or a caught inference error ends the worker.
                    Frame frame; // Wait without polling. Move the pending frame out under lock, then release the lock during inference so camera submission can replace the waiting frame. Retain the chosen source image independently of the replaceable waiting slot.
                    {
                        std::unique_lock<std::mutex> lock(mutex); // Use a lock that the condition variable can release while waiting and reacquire on wakeup.
                        wake.wait(lock, [&] { return stopping || bool(pending); }); // Sleep until shutdown or a valid pending frame; the predicate handles spurious wakeups.
                        if (stopping) // Exit without starting another inference after shutdown was requested.
                            return;
                        frame = std::move(pending); // Transfer the pending image into worker-local ownership while holding the queue lock.
                        pending = {}; // Empty the waiting slot so new submissions are independent of the in- flight image.
                    }
                    CoralState next; // Attach the source receive time and a new result sequence. Class ID 12 is the zero-based stop-sign class; confirmation/release timing belongs to the core. Build a complete result privately before publishing it to readers.
                    next.ready = true; // This result comes from a successfully initialized inference engine.
                    next.sequence = ++sequence; // Assign a new completed-result identity for stop-sign confirmation and duplicate suppression.
                    next.frame_time = frame.received; // Preserve source image age so a newly delivered result cannot hide stale pixels.
                    next.detections = engine.infer(frame, next.inference_ms); // Invoke native inference outside the publication lock and record its measured Invoke duration.
                    next.received = clock_now(); // Stamp inference completion on the shared monotonic host clock.
                    next.detail = "Coral USB native inference"; // Supply a concise diagnostic label for a successfully completed native USB inference.
                    for (const auto &d : next.detections) // Examine accepted model detections for the project's semantic stop condition.
                        if (d.id == 12 && d.score >= .60f) // Zero-based COCO stop-sign ID 12 requires at least 0.60 confidence for this gate.
                            next.stop_sign = true; // Flag a positive observation; consecutive confirmation and release timing occur in the core.
                    std::lock_guard<std::mutex> lock(mutex); // Serialize state publication or shutdown changes with submissions and status readers.
                    state = std::move(next); // Publish the whole completed observation together so timestamps and detections cannot be mixed.
                }
            } catch (const std::exception &e) {
                std::lock_guard<std::mutex> lock(mutex); // Serialize state publication or shutdown changes with submissions and status readers.
                state.ready = false; // A worker exception makes required inference unavailable rather than silently disabling its gate.
                state.detail = e.what(); // Preserve the underlying inference failure for the main loop's error message.
            }
        });
    }
    ~Impl() { // Request shutdown and join before engine member destruction releases the interpreter.
        {
            std::lock_guard<std::mutex> lock(mutex); // Serialize state publication or shutdown changes with submissions and status readers.
            stopping = true; // Set the shutdown predicate while holding the same mutex used by the worker.
        }
        wake.notify_one(); // Wake a sleeping worker so it can observe shutdown rather than waiting forever for a frame.
        if (worker.joinable()) // Join only a thread that was actually launched.
            worker.join(); // Wait for in-flight inference and worker exit before destroying native runtime resources.
    }
};
Coral::Coral(const std::string &model, const std::string &library) // Own the warmed-up service behind the public platform-neutral interface.
    : impl_(std::make_unique<Impl>(model, library)) {}
Coral::~Coral() = default; // Default destruction invokes the implementation's ordered worker/interpreter cleanup.
void Coral::submit(const Frame &frame) { // Ignore invalid/duplicate frames and replace the single waiting frame. Shared immutable ownership keeps its pixels alive until inference finishes.
    if (!frame) // Ignore missing or incorrectly sized pixel storage before attempting submission.
        return;
    {
        std::lock_guard<std::mutex> lock(impl_->mutex); // Protect service state against simultaneous worker publication and frame submission.
        if (!impl_->state.ready || frame.sequence == impl_->submitted) // Ignore frames after worker failure and suppress duplicate camera sequences.
            return;
        impl_->submitted = frame.sequence; // Remember this accepted submission so the main loop cannot enqueue it repeatedly.
        impl_->pending = frame; // Replace only the waiting image; an in-flight inference retains its own shared pixels.
    }
    impl_->wake.notify_one(); // Notify after releasing the lock so the worker can immediately acquire the new frame.
}
CoralState Coral::status(double now) { // Return a snapshot with health recomputed at read time. An unready or stale result cannot authorize motion, even if the last detection list was empty.
    std::lock_guard<std::mutex> lock(impl_->mutex); // Protect service state against simultaneous worker publication and frame submission.
    CoralState s = impl_->state; // Copy a coherent result before computing read-time freshness.
    s.healthy = coral_fresh(s, std::max(now, clock_now())); // A result may have arrived after the main loop sampled its clock. Check age after taking the lock to avoid a false one-frame TPU hold. Use a clock sampled after locking to avoid falsely rejecting a just-published future-to- caller result.
    if (!s.healthy) // Stale detections cannot be treated as a new positive semantic observation.
        s.stop_sign = false; // Clear this snapshot's observation; the core's persistent stop latch is independent.
    return s; // Return an owned status snapshot that can be rendered or passed to the core without holding the mutex.
}
} // namespace tabletop
