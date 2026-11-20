/**
 * @file
 * Linux camera and LiDAR acquisition with deterministic resource cleanup.
 * Camera callbacks and a LiDAR worker publish data under one mutex. Pixel data
 * is copied out of mapped DMA buffers before libcamera requests are reused.
 */

#include "app.h"
#include "camera_controls.h"
#include <libcamera/libcamera.h>
#include <sl_lidar.h>
#include <sl_lidar_driver.h>
#include <atomic> // Atomic flags and values used for communication without unsynchronized shared writes.
#include <cerrno> // Linux errno values used to interpret DMA-buffer synchronization failures.
#include <cstring> // Byte copying or C string comparison at native library and image-buffer boundaries.
#include <iostream> // Console progress, usage information, and actionable error reporting.
#include <map> // Keyed lookup of native camera buffers and their mapped-memory metadata.
#include <mutex> // Scope-bound locks protecting snapshots exchanged between acquisition/inference and the main loop.
#include <thread> // Background acquisition/inference workers or main-loop pacing.
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <linux/dma-buf.h>

namespace tabletop {
struct Sensors::Impl { // Own camera mappings/requests and SDK resources. The mutex protects Inputs; stop flags and startup-state flags support cleanup after partial initialization.
    std::mutex mutex; // Protect the published Inputs snapshot and its latched error across producer and main threads.
    Inputs inputs; // Shared latest scan, bounded camera history, and first acquisition error.
    std::atomic<bool> stopping{false}; // Cross-thread shutdown request checked before requeueing or publishing more data.
    std::thread lidar_thread; // Blocking serial scan acquisition runs independently of camera callbacks and rendering.
    std::unique_ptr<sl::ILidarDriver> driver; // Own the SDK driver separately from its serial transport channel.
    std::unique_ptr<sl::IChannel> channel; // Own the serial transport for as long as the connected SDK driver needs it.
    libcamera::CameraManager manager; // Enumerates cameras and manages libcamera's event infrastructure.
    std::shared_ptr<libcamera::Camera> camera; // Hold the acquired camera while requests, buffers, and callbacks depend on it.
    std::unique_ptr<libcamera::CameraConfiguration> configuration; // Retain the validated stream configuration and its stride/layout metadata.
    std::unique_ptr<libcamera::FrameBufferAllocator> allocator; // Own DMA-backed camera buffers until requests and mappings are released.
    std::vector<std::unique_ptr<libcamera::Request>> requests; // Keep request objects alive across queue/reuse cycles.
    struct Mapping { // Information required to read and later unmap one allocated camera buffer.
        void *address; // Base address returned by mmap, before adding the image plane's byte offset.
        std::size_t size, offset; // Total mapping length and image-plane offset in bytes.
        int fd; // Borrowed DMA-buffer descriptor used for cache synchronization; allocator owns its lifetime.
    };
    std::map<libcamera::FrameBuffer *, Mapping> mappings; // Look up mapped CPU storage by the buffer pointer returned in a completed request.
    libcamera::Stream *stream = nullptr; // Borrow the configured stream object for request lookup and buffer allocation.
    bool manager_started = false, acquired = false, camera_started = false, lidar_connected = false; // Track successful acquisition stages so partial-startup cleanup touches only acquired resources.
    unsigned stride = 0; // Actual source bytes per image row, which may include padding beyond packed BGR pixels.
    std::uint64_t camera_sequence = 0, last_sensor_ns = 0; // Application frame counter and last sensor timestamp for duplicate suppression.

    void error(const std::string &message) { // Latch the first acquisition error under lock so the main loop can report the root cause without throwing across a callback or worker-thread boundary.
        std::lock_guard<std::mutex> lock(mutex); // Hold the publication mutex for this scope; automatically release it on return or exception.
        if (inputs.error.empty()) // Preserve the first failure so later shutdown symptoms do not hide its cause.
            inputs.error = message; // Publish the error for the main thread instead of throwing across a callback boundary.
    }
    void camera_done(libcamera::Request *request) { // Completed request callback: timestamp, synchronize DMA access, copy valid rows into shared owned storage, publish bounded history, then reuse the request.
        if (stopping || request->status() == libcamera::Request::RequestCancelled) // Ignore cancellation and shutdown callbacks that should not access or requeue buffers.
            return;
        try {
            const double received = clock_now(); // Timestamp callback receipt with the same monotonic clock used by LiDAR and the main loop.
            auto *buffer = request->findBuffer(stream); // Locate the image buffer attached to this request's configured stream.
            if (!buffer) // Reject an inconsistent completed request before dereferencing its buffer.
                throw std::runtime_error("Camera returned no RGB buffer");
            if (buffer->metadata().status == libcamera::FrameMetadata::FrameSuccess) { // Process pixels only when libcamera marks this frame successful.
                auto stamp = request->metadata().get(libcamera::controls::SensorTimestamp); // Prefer sensor timestamp metadata for duplicate detection, not host-clock pairing.
                const auto ns = stamp ? std::uint64_t(*stamp) : buffer->metadata().timestamp; // Fall back to buffer metadata when the optional sensor timestamp control is absent.
                if (ns != last_sensor_ns || ns == 0) { // Suppress repeated known timestamps; zero means unavailable and cannot identify duplicates.
                    const auto &m = mappings.at(buffer); // Retrieve the previously validated mapping for this completed buffer.
                    auto pixels = std::make_shared<std::vector<unsigned char>>(640 * 480 * 3); // Allocate owned tightly packed BGR storage before the camera reuses its DMA buffer.
                    dma_buf_sync sync{}; // Zero-initialize the Linux DMA synchronization request.
                    sync.flags = DMA_BUF_SYNC_START | DMA_BUF_SYNC_READ; // Begin CPU read access so the DMA buffer's contents are coherent.
                    if (ioctl(m.fd, DMA_BUF_IOCTL_SYNC, &sync) < 0 && errno != ENOTTY) // Treat unsupported synchronization (ENOTTY) as acceptable, but report other ioctl errors.
                        throw std::runtime_error("Camera DMA read synchronization failed");
                    const auto *source = static_cast<const unsigned char *>(m.address) + m.offset; // Offset into the mapped plane's first image byte rather than assuming offset zero.
                    for (int y = 0; y < 480; ++y) // Copy each calibrated image row independently to discard source stride padding.
                        std::memcpy(pixels->data() + y * 640 * 3, source + y * stride, 640 * 3); // Copy exactly 640 BGR pixels into the owned packed row.
                    sync.flags = DMA_BUF_SYNC_END | DMA_BUF_SYNC_READ; // End the CPU read access before the buffer returns to camera ownership.
                    if (ioctl(m.fd, DMA_BUF_IOCTL_SYNC, &sync) < 0 && errno != ENOTTY) // Treat unsupported synchronization (ENOTTY) as acceptable, but report other ioctl errors.
                        throw std::runtime_error("Camera DMA read completion failed");
                    Frame frame{pixels, received, ++camera_sequence, ns}; // Attach host receive time, a unique application sequence, and optional device metadata to owned pixels.
                    last_sensor_ns = ns; // Remember the accepted device timestamp for duplicate suppression on later callbacks.
                    std::lock_guard<std::mutex> lock(mutex); // Hold the publication mutex for this scope; automatically release it on return or exception.
                    inputs.frames.push_back(std::move(frame)); // Publish the new immutable frame while holding the shared snapshot lock.
                    if (inputs.frames.size() > 24) // Cap history at 24 frames to bound memory and pairing work.
                        inputs.frames.erase(inputs.frames.begin()); // Drop the oldest frame once the history capacity is exceeded.
                }
            }
            request->reuse(libcamera::Request::ReuseBuffers); // Retain attached buffers while resetting the request for another camera capture.
            if (!stopping && camera->queueRequest(request) < 0) // Requeue only during live acquisition and report a driver rejection as a latched error.
                error("Could not requeue camera buffer"); // Notify the main loop that this camera buffer could not reenter the capture queue.
        } catch (const std::exception &e) {
            error(std::string("Camera: ") + e.what()); // Preserve a callback failure as an acquisition error rather than letting it escape the callback.
        }
    }
    void start_camera(int index) { // Acquire a 640x480 packed stream, validate layout, map each buffer, and queue four requests. Crop and frame duration must remain consistent with calibration.
        if (manager.start() < 0) // Start libcamera device discovery and event handling before enumerating cameras.
            throw std::runtime_error("libcamera manager could not start");
        manager_started = true; // Record successful manager startup for exception-safe shutdown.
        const auto cameras = manager.cameras(); // Copy the enumerated camera handles before selecting the requested index.
        if (index < 0 || std::size_t(index) >= cameras.size()) // Check both signed and unsigned bounds before indexing the camera list.
            throw std::runtime_error(
                "Requested camera unavailable; check rpicam-hello --list-cameras");
        camera = cameras.at(index); // Retain the selected camera handle through configuration and capture.
        if (camera->acquire() < 0) // Request exclusive use; another camera application can cause this operation to fail.
            throw std::runtime_error("Camera busy; close other applications using this camera");
        acquired = true; // Record ownership so cleanup releases the camera even if later configuration fails.
        configuration = camera->generateConfiguration({libcamera::StreamRole::VideoRecording}); // Ask libcamera for a video-oriented configuration that can be adjusted to the calibrated stream.
        if (!configuration || configuration->empty()) // Reject missing or empty configuration before accessing stream zero.
            throw std::runtime_error("No libcamera video configuration");
        auto &cfg = configuration->at(0); // Modify the first requested video stream's settings in place.
        cfg.pixelFormat = libcamera::formats::RGB888; // libcamera RGB888 is B,G,R byte order on the little-endian Pi.
        cfg.size = {640, 480}; // Preserve the pixel coordinate system used by marker ROI and bearing calibration.
        cfg.bufferCount = 4; // Request four reusable buffers so capture can continue while completed frames are processed.
        cfg.colorSpace = libcamera::ColorSpace::Sycc; // Request the sYCC color interpretation used by the RGB video pipeline.
        if (configuration->validate() == libcamera::CameraConfiguration::Invalid ||
            cfg.pixelFormat != libcamera::formats::RGB888 || cfg.size.width != 640 ||
            cfg.size.height != 480) // Reject unsupported adjustments that change the required pixel format or image dimensions.
            throw std::runtime_error("Camera cannot supply calibrated 640x480 RGB888 stream");
        if (camera->configure(configuration.get()) < 0) // Apply the validated stream configuration to the acquired camera.
            throw std::runtime_error("Camera configuration failed");
        stride = cfg.stride; // Retain actual bytes per row for correct copying from potentially padded buffers.
        stream = cfg.stream(); // Retain the configured stream pointer used to attach and retrieve buffers.
        if (stride < 640 * 3) // Reject a row layout too short to contain 640 three-byte pixels.
            throw std::runtime_error("Invalid RGB camera stride");
        std::cout << "Camera " << camera->id() << " | " << cfg.toString() << '\n'; // Report selected hardware/layout to the terminal for reproducible setup diagnostics.
        allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera); // Create an allocator tied to the acquired camera's lifetime.
        if (allocator->allocate(stream) < 0) // Allocate DMA buffers for the configured stream and stop on allocation failure.
            throw std::runtime_error("Camera buffer allocation failed");
        for (const auto &buffer : allocator->buffers(stream)) { // Map every allocated buffer and create one reusable capture request for each.
            if (buffer->planes().size() != 1) // The copy path expects one packed plane rather than a multiplanar image format.
                throw std::runtime_error("Expected packed RGB camera buffer");
            const auto &plane = buffer->planes()[0]; // Read the sole plane's descriptor, offset, and byte length.
            if (plane.offset == libcamera::FrameBuffer::Plane::kInvalidOffset ||
                plane.length < stride * 480) // Validate offset availability and enough row storage for all 480 image rows.
                throw std::runtime_error("Invalid camera buffer layout");
            const std::size_t length = std::size_t(plane.offset) + plane.length; // Include the plane offset when mapping from descriptor byte offset zero.
            void *ptr = mmap(nullptr, length, PROT_READ, MAP_SHARED, plane.fd.get(), 0); // Map the camera-owned DMA buffer read-only into this process's address space.
            if (ptr == MAP_FAILED) // Stop initialization if no CPU-readable buffer mapping was created.
                throw std::runtime_error("mmap camera buffer failed");
            mappings.emplace(buffer.get(), Mapping{ptr, length, plane.offset, plane.fd.get()}); // Register cleanup and read metadata immediately after successful mmap.
            auto request = camera->createRequest(); // Allocate the libcamera request that will carry this mapped buffer through capture cycles.
            if (!request || request->addBuffer(stream, buffer.get()) < 0) // Check request allocation and buffer attachment before storing a reusable request.
                throw std::runtime_error("Camera request creation failed");
            requests.push_back(std::move(request)); // Transfer exclusive request ownership into the sensor object's lifetime.
        }
        camera->requestCompleted.connect(this, &Impl::camera_done); // Connect completed capture requests to the callback that copies and publishes frames.
        libcamera::ControlList controls(camera->controls()); // Create a control list compatible with the selected camera's supported controls.
        set_camera_frame_duration(controls); // Set int64 frame-duration limits for an approximately 15 fps stream.
        if (auto crop = camera->properties().get(libcamera::properties::ScalerCropMaximum)) { // Use the camera's maximum crop when that property is available.
            controls.set(libcamera::controls::ScalerCrop, *crop); // Apply the selected sensor crop; physical calibration must match the resulting field of view.
            std::cout << "Camera maximum crop " << crop->toString() // Report selected hardware/layout to the terminal for reproducible setup diagnostics.
                      << "; confirm marker alignment before Space.\n";
        }
        if (camera->start(&controls) < 0) // Start capture using the validated frame-duration and optional crop controls.
            throw std::runtime_error("Camera start failed");
        camera_started = true; // Ensure cleanup stops capture before disconnecting callbacks or unmapping buffers.
        for (auto &request : requests) // Seed the capture pipeline with every allocated reusable request.
            if (camera->queueRequest(request.get()) < 0) // Treat rejection of an initial request as startup failure.
                throw std::runtime_error("Camera initial queue failed");
    }
    void start_lidar(std::string port) { // Select one serial device, check health, start scanning, and publish complete scans on a worker. Multiple detected adapters require explicit port selection.
        if (port.empty()) { // Discover a serial device only when the caller did not explicitly select one.
            std::vector<std::string> ports; // Collect matching USB serial adapters before deciding whether selection is unambiguous.
            const std::filesystem::path directory("/dev/serial/by-id"); // Stable by-id links identify adapters independently of ttyUSB enumeration order.
            if (std::filesystem::exists(directory)) // Skip by-id enumeration when the directory is absent.
                for (const auto &entry : std::filesystem::directory_iterator(directory)) // Inspect each stable serial-device link without modifying it.
                    if (entry.path().filename().string().find("CP2102N") != std::string::npos) // Match the CP2102N adapter used by the configured LiDAR connection.
                        ports.push_back(entry.path().string()); // Save the matching stable serial path as a candidate.
            if (ports.size() > 1) // Require an explicit port when multiple matching adapters would make automatic selection ambiguous.
                throw std::runtime_error("Several CP2102N devices found; choose --port explicitly");
            port = ports.empty() ? "/dev/ttyUSB0" : ports.front(); // Prefer the unique stable link; fall back to ttyUSB0 only when no matching by-id link exists.
        }
        driver.reset(*sl::createLidarDriver()); // Take ownership of the SDK driver returned by its factory.
        channel.reset(*sl::createSerialPortChannel(port.c_str(), 460800)); // Create the C1 serial transport at its configured 460800 baud rate.
        if (!driver || !channel || SL_IS_FAIL(driver->connect(channel.get()))) // Require valid driver/channel handles and a successful serial connection before continuing.
            throw std::runtime_error("Cannot open LiDAR " + port +
                                     " at 460800 baud; check serial permissions");
        lidar_connected = true; // Record successful connection so cleanup stops and disconnects the device.
        sl_lidar_response_device_health_t health{}; // Zero-initialize the device health response before requesting it from the SDK.
        if (SL_IS_FAIL(driver->getHealth(health)) || health.status == SL_LIDAR_STATUS_ERROR) // Reject a failed health query or a reported device error status.
            throw std::runtime_error("LiDAR health check failed");
        if (SL_IS_FAIL(driver->setMotorSpeed()) || SL_IS_FAIL(driver->startScan(false, true))) // Start the motor and scanning; either failure aborts startup.
            throw std::runtime_error("LiDAR scan start failed");
        std::cout << "LiDAR " << port << " | native SLAMTEC SDK, 460800 baud\n"; // Report selected hardware/layout to the terminal for reproducible setup diagnostics.
        lidar_thread = std::thread([this] { // Launch the blocking scan reader after serial and device checks succeed.
            try {
                std::vector<sl_lidar_response_measurement_node_hq_t> nodes(8192); // Allocate reusable capacity for up to 8192 HQ measurements per received scan.
                int sequence = 0, failures = 0; // Start unique scan numbering and the consecutive failed-read counter.
                while (!stopping) { // Continue collecting complete scans until shutdown is requested.
                    std::size_t count = nodes.size(); // Reset the SDK's in/out count to the available buffer capacity before each read.
                    auto code = driver->grabScanDataHq(nodes.data(), count, 500); // Wait up to 500 ms for a complete HQ scan rather than spinning on partial measurements.
                    if (stopping) // Exit promptly after a blocking SDK read notices the shutdown request.
                        break;
                    if (SL_IS_FAIL(code)) { // Count failed scan reads instead of publishing incomplete data as a fresh scan.
                        if (++failures >= 10) // Ten consecutive timeout-length failures indicate an approximately five- second outage.
                            throw std::runtime_error("No complete LiDAR scan for five seconds");
                        continue;
                    }
                    failures = 0; // Any successful complete scan resets the consecutive failure counter.
                    Scan scan; // Build the next observation privately before publishing it under the shared lock.
                    scan.received = clock_now(); // Stamp completion on the common monotonic host clock.
                    scan.sequence = sequence++; // Assign a new application scan number, starting at zero.
                    scan.points.reserve(count); // Reserve only the received measurement count to avoid repeated reallocations during conversion.
                    for (std::size_t i = 0; i < count; ++i) { // SDK HQ nodes encode angle in Q14 and range in quarter millimeters. Convert to degrees/meters here; retain sensor-relative angles for the calibrated core. Decode each measurement returned in this complete scan.
                        const auto &n = nodes[i]; // Read one HQ wire-format node without copying its packed data.
                        if (n.dist_mm_q2) // Exclude zero-distance measurements that carry no usable range.
                            scan.points.push_back(
                                {n.angle_z_q14 * 90.0 / 16384.0, n.dist_mm_q2 / 4000.0,
                                 n.quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT}); // Append decoded angle (degrees), range (meters), and shifted quality to the owned scan.
                    }
                    std::lock_guard<std::mutex> lock(mutex); // Core sorts its own ROI; SDK wire angles remain unmodified here. Hold the publication mutex for this scope; automatically release it on return or exception.
                    inputs.scan = std::move(scan); // Atomically publish the completed scan by moving it into the shared latest observation.
                }
            } catch (const std::exception &e) {
                error(std::string("LiDAR: ") + e.what()); // Latch worker failures for the main loop rather than allowing an uncaught thread exception.
            }
        });
    }
    ~Impl() { // Stop publication first, join the scan worker, stop the LiDAR motor, then stop camera callbacks before unmapping buffers and releasing the manager.
        stopping = true; // Ask callbacks and the scan worker to stop producing and requeueing data.
        if (lidar_thread.joinable()) // Join only a worker that was successfully started.
            lidar_thread.join(); // Wait for the scan reader to finish before destroying its driver or channel.
        if (lidar_connected) { // Perform device shutdown only when a serial connection was established.
            driver->stop(); // Stop measurement streaming before stopping the motor and transport.
            driver->setMotorSpeed(0); // Request motor stop as part of normal sensor cleanup.
            driver->disconnect(); // Release the SDK's connection to the serial channel.
        }
        driver.reset(); // Destroy the driver after all reads and device shutdown operations finish.
        channel.reset(); // Release the serial transport after its driver no longer needs it.
        if (camera_started) // Stop capture only if camera startup completed successfully.
            camera->stop(); // Stop capture/callback production before releasing request or buffer storage.
        if (camera) // A selected camera may exist even after partial configuration failure.
            camera->requestCompleted.disconnect(this); // Detach this object's callback before its state is destroyed.
        requests.clear(); // Destroy capture requests while their buffers and camera are still alive.
        for (auto &[buffer, m] : mappings) { // Release every CPU mapping recorded during buffer setup.
            (void)buffer; // The map key is unnecessary during cleanup; explicitly mark it unused.
            munmap(m.address, m.size); // Release the exact mapping address and length recorded at allocation time.
        }
        mappings.clear(); // Discard stale buffer-to-mapping metadata after all mappings are released.
        allocator.reset(); // Free DMA buffers after requests and CPU mappings no longer refer to them.
        if (acquired) // Return exclusive camera ownership only if acquisition succeeded.
            camera->release(); // Make the camera available to other applications again.
        camera.reset(); // Release the retained camera handle after capture, buffers, and ownership are stopped.
        if (manager_started) // Stop the event/device manager only if it was started successfully.
            manager.stop(); // Shut down libcamera's manager after all dependent resources are released.
    }
};
Sensors::Sensors(const std::string &port, int camera_index) : impl_(std::make_unique<Impl>()) { // Construct the private owner before starting devices so failures unwind acquired resources.
    impl_->start_camera(camera_index); // Initialize camera capture before starting the LiDAR reader.
    impl_->start_lidar(port); // Open the serial scanner and its background acquisition worker.
}
Sensors::~Sensors() = default; // Default destruction delegates ordered hardware cleanup to the private implementation.
Inputs Sensors::snapshot() { // Copy a coherent observation snapshot under lock. Pixel vectors are shared immutably, so this does not duplicate the entire camera history.
    std::lock_guard<std::mutex> lock(impl_->mutex); // Lock publication while copying a coherent frame-history and latest-scan snapshot.
    return impl_->inputs; // Copy metadata and scan storage; shared immutable frame pixels retain their owned lifetime.
}
} // namespace tabletop
