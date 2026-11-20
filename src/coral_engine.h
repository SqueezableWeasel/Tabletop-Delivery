/**
 * @file
 * USB delegate/interpreter lifecycle and SSD inference implementation.
 * The private runtime must match this application's TensorFlow Lite layouts.
 * Model bytes, interpreter, delegate, and loaded library have nested lifetimes.
 */

#pragma once // Include this header only once per translation unit.
#include "app.h"
#include "model_check.h"
#include "coral_abi.h"
#include <tensorflow/lite/c/c_api.h>
#include <tensorflow/lite/c/c_api_experimental.h>
#include <opencv2/imgproc.hpp>
#include <dlfcn.h>
#include <cstring> // Byte copying or C string comparison at native library and image-buffer boundaries.
#include <cstdio> // Formatted C-style diagnostics and explicit flushing at native runtime stages.
#include <iostream> // Console progress, usage information, and actionable error reporting.
namespace tabletop {
namespace {
enum DeviceType { Pci = 0,
                  Usb = 1 }; // Public libedgetpu C-delegate ABI. Opaque TensorFlow Lite C handles avoid distro C++ schema-header/FlatBuffers mismatches; all calling code is C++. Match the external runtime's device-type enum and explicitly select USB for this project.
struct Device { // Mirror the C enumeration record; the runtime owns the returned list and its path strings.
    DeviceType type; // Runtime-reported accelerator connection type.
    const char *path; // Borrowed device locator, valid only until the enumeration allocation is released.
};
struct Option { // Mirror the optional name/value configuration pair accepted by the runtime's delegate factory.
    const char *name; // Null-terminated option key expected by the native delegate API.
    const char *value; // Null-terminated option value; this application supplies no custom options.
};
class Engine { // Encapsulate model memory, native runtime, delegate, and interpreter under one lifetime.
    void *library_ = nullptr; // dlopen handle must remain alive while delegate functions or objects can be used.
    TfLiteDelegate *delegate_ = nullptr; // USB delegate installed into the interpreter; freed by the runtime's own deleter.
    void (*free_delegate_)(TfLiteDelegate *) = nullptr; // Retain the matching library function needed to destroy the delegate safely.
    TfLiteModel *model_ = nullptr; // TFLite model object borrows model_data_ bytes, which remain alive through cleanup.
    TfLiteInterpreterOptions *options_ = nullptr; // Store delegate and threading options until interpreter cleanup is complete.
    TfLiteInterpreter *interpreter_ = nullptr; // Own the executable graph and tensors; destroy before its USB delegate.
    std::vector<unsigned char> model_data_; // Keep the validated detector bytes alive as long as the model/interpreter can reference them.
    static void stage(const char *message) { // Emit a flushed startup/cleanup checkpoint to localize native-library failures.
        std::fprintf(stderr, "[native Coral] %s\n", message); // Write native runtime diagnostics directly to stderr, including library identity where applicable.
        std::fflush(stderr); // Flush immediately so the last completed checkpoint survives a native process failure.
    }
    static void library_path(const char *label, const void *symbol_address) { // Resolve a loaded function address to its actual shared-library file for diagnostics.
        Dl_info info{}; // Zero-initialize dladdr's output before querying a symbol's owning shared object.
        if (dladdr(symbol_address, &info) && info.dli_fname) // Print a library path only when the dynamic loader successfully resolves it.
            std::fprintf(stderr, "[native Coral] %s library: %s\n", label, info.dli_fname); // Write native runtime diagnostics directly to stderr, including library identity where applicable.
        std::fflush(stderr); // Flush immediately so the last completed checkpoint survives a native process failure.
    }
    template <class T>
    T symbol(const char *name) { // Resolve a required runtime symbol and cast it to the exact external C function-pointer type.
        void *p = dlsym(library_, name); // Look up the named entry point in the explicitly loaded delegate library.
        if (!p) // Reject an incomplete runtime before invoking a missing function pointer.
            throw std::runtime_error(std::string("Missing libedgetpu C API: ") + name);
        return reinterpret_cast<T>(p); // Return the resolved entry point with the caller-specified native signature.
    }
    void cleanup() { // Destroy in dependency order: interpreter, options/model, delegate, library. Keep the shared library loaded until every object that calls into it is gone.
        if (interpreter_) { // Destroy an interpreter only after successful creation; partial startup may leave it null.
            stage("Destroying interpreter");
            TfLiteInterpreterDelete(interpreter_); // Release the graph and tensor resources while the delegate and model are still valid.
        }
        interpreter_ = nullptr; // Make repeated cleanup harmless after releasing the interpreter.
        if (options_) // Release options only if creation succeeded.
            TfLiteInterpreterOptionsDelete(options_); // Free the option container after the interpreter has stopped using its settings.
        options_ = nullptr; // Clear the released options handle to prevent duplicate destruction.
        if (model_) // Release the model wrapper only if it was successfully created.
            TfLiteModelDelete(model_); // Free the model wrapper before its backing byte vector is destroyed.
        model_ = nullptr; // Mark model cleanup complete for exception and destructor paths.
        if (delegate_ && free_delegate_) { // Delegate destruction requires both a created object and its matching runtime deleter.
            stage("Releasing USB delegate");
            free_delegate_(delegate_); // Release the USB delegate through the same library that allocated it.
        }
        delegate_ = nullptr; // Clear the released delegate handle before unloading the library.
        if (library_) { // Unload only a successfully opened runtime handle.
            stage("Unloading libedgetpu");
            dlclose(library_); // Drop the shared-object reference after all objects with code in that library are gone.
        }
        library_ = nullptr; // Record completed library cleanup so destructor/exception cleanup cannot repeat it.
    }
    const TfLiteTensor *output(int i) const { // Access a borrowed output tensor by the validated SSD output index.
        return TfLiteInterpreterGetOutputTensor(interpreter_, i); // Tensor storage remains owned by the interpreter; callers must not free it.
    }

  public:
    std::string version; // Owned copy of the runtime's reported version for diagnostics.
    Engine(const std::string &path, const std::string &library) { // Startup pipeline: inspect model, load/check runtime, select USB, attach the delegate, validate tensor layouts, and invoke a zero-image warm-up. Any failure unwinds owned resources and prevents the application from claiming readiness.
        try {
            stage("Reading and validating detector model");
            model_data_ = read_detector_model(path); // Read bounded owned bytes and require the Edge TPU custom operator before runtime setup.
            stage("Reading TensorFlow Lite runtime version");
            library_path("TensorFlow Lite", reinterpret_cast<const void *>(&TfLiteVersion)); // Report the actual linked TFLite library, not merely the installed package name.
            std::fprintf(stderr, "[native Coral] TensorFlow Lite version: %s\n", TfLiteVersion()); // Write native runtime diagnostics directly to stderr, including library identity where applicable.
            std::fflush(stderr); // Flush immediately so the last completed checkpoint survives a native process failure.
            stage("Loading libedgetpu");
            library_ = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL); // Resolve symbols immediately and keep them local to this loaded runtime handle.
            if (!library_) // Report loader errors before accessing any delegate API.
                throw std::runtime_error(std::string("Cannot load ") + library + ": " + dlerror() +
                                         ". Build the matching project runtime with bash "
                                         "scripts/build_coral_runtime.sh.");
            stage("Checking delegate/interpreter ABI before opening the accelerator");
            auto abi =
                reinterpret_cast<std::uint64_t (*)()>(dlsym(library_, "edgetpu_tabletop_abi_v1")); // Compare both structure layout and linked TensorFlow Lite version before calling the delegate. Matching a symbol name alone does not establish ABI safety. Look up the private runtime's exported structure-layout fingerprint function.
            auto runtime_version = reinterpret_cast<const char *(*)()>(
                dlsym(library_, "edgetpu_tabletop_tflite_version")); // Look up the runtime's linked TensorFlow Lite version function for a second compatibility check.
            if (!abi || !runtime_version) // Require project ABI metadata before opening the accelerator or applying the delegate.
                throw std::runtime_error(
                    "Coral library lacks this project's ABI metadata. Use "
                    "native_runtime/libedgetpu.so.1 built with scripts/build_coral_runtime.sh; the "
                    "system library is left unchanged.");
            if (abi() != coral_abi_fingerprint()) // Reject mismatched C structure sizes/offsets that could corrupt delegate/interpreter calls.
                throw std::runtime_error(
                    "Coral delegate and application use different TensorFlow Lite layouts. Rebuild "
                    "the private runtime AND application against the same installed headers.");
            const char *linked_version = runtime_version(); // Read the TFLite version actually resolved by the delegate build.
            if (!linked_version || std::strcmp(linked_version, TfLiteVersion()) != 0) // Require the delegate and application to report the same linked TFLite version.
                throw std::runtime_error(
                    "Coral runtime and application resolve different TensorFlow Lite versions");
            stage("Delegate/interpreter ABI matches");
            auto list = symbol<Device *(*)(std::size_t *)>("edgetpu_list_devices"); // Resolve accelerator enumeration using the runtime's count-and-array C contract.
            auto free_devices = symbol<void (*)(Device *)>("edgetpu_free_devices"); // Resolve the runtime-specific deleter for its enumeration allocation.
            auto create =
                symbol<TfLiteDelegate *(*)(DeviceType, const char *, const Option *, std::size_t)>(
                    "edgetpu_create_delegate"); // Resolve delegate construction with an explicit device type, copied path, and optional settings array.
            free_delegate_ = symbol<void (*)(TfLiteDelegate *)>("edgetpu_free_delegate"); // Retain the delegate deleter before creating any object that will need it.
            auto get_version = symbol<const char *(*)()>("edgetpu_version"); // Resolve the runtime's own version-reporting function.
            library_path("Edge TPU", reinterpret_cast<const void *>(get_version)); // Report the actual runtime file supplying the version entry point.
            stage("Reading Edge TPU runtime version");
            const char *reported_version = get_version(); // Read the runtime-owned version string while its library is loaded.
            version = reported_version ? reported_version : "unknown"; // Copy version text into owned storage, allowing an absent string to remain diagnostic only.
            std::fprintf(stderr, "[native Coral] Edge TPU version: %s\n", version.c_str()); // Write native runtime diagnostics directly to stderr, including library identity where applicable.
            std::fflush(stderr); // Flush immediately so the last completed checkpoint survives a native process failure.
            std::size_t count = 0; // Pick a USB device explicitly. The enumeration allocation is freed through its library-provided deleter; copy the chosen path before releasing the list. Enumeration writes the number of returned device records into this variable.
            stage("Enumerating USB accelerators");
            std::unique_ptr<Device, decltype(free_devices)> devices(list(&count), free_devices); // Free the runtime-allocated device list automatically, including on startup exceptions.
            std::string usb_path; // Own the selected device locator independently of the temporary enumeration list.
            bool found = false; // Track explicit USB selection rather than assuming the first accelerator is suitable.
            if (devices) // Enumerate entries only when the runtime returned an allocation.
                for (std::size_t i = 0; i < count; ++i) // Inspect the complete enumeration until a usable USB accelerator is found.
                    if (devices.get()[i].type == Usb) { // Ignore PCI devices and choose the USB Accelerator requested by this backend.
                        if (!devices.get()[i].path) // Reject an invalid runtime record before copying its borrowed string.
                            throw std::runtime_error("USB accelerator has a null device path");
                        usb_path = devices.get()[i].path; // Copy the selected path while the enumeration allocation still owns the source bytes.
                        found = true; // Mark successful USB selection before leaving the enumeration loop.
                        break;
                    }
            if (!found) // Fail startup explicitly when no USB Accelerator is visible.
                throw std::runtime_error("No Coral USB Accelerator visible to libedgetpu");
            stage("Creating USB delegate");
            delegate_ = create(Usb, usb_path.c_str(), nullptr, 0); // Create the USB delegate for the copied device path with no extra option pairs.
            if (!delegate_) // A null delegate means device setup failed; do not attempt interpreter creation.
                throw std::runtime_error(
                    "Coral USB delegate creation failed; check USB permissions/power");
            stage("Creating TensorFlow Lite model");
            model_ = TfLiteModelCreate(model_data_.data(), model_data_.size()); // Create a TFLite model wrapper over the owned detector bytes.
            stage("Creating interpreter options");
            options_ = TfLiteInterpreterOptionsCreate(); // Allocate the interpreter configuration container.
            if (!model_ || !options_) // Require both wrappers before setting options or constructing the interpreter.
                throw std::runtime_error("TensorFlow Lite model/options creation failed");
            TfLiteInterpreterOptionsSetNumThreads(options_, 1); // Use one CPU worker for nondelegated interpreter work to limit Pi CPU contention.
            TfLiteInterpreterOptionsSetEnableDelegateFallback(options_, false); // Refuse automatic CPU fallback if delegation fails.
            TfLiteInterpreterOptionsAddDelegate(options_, delegate_); // Attach the USB delegate so the model's accelerator operation can execute.
            stage("Creating interpreter and applying USB delegate");
            interpreter_ = TfLiteInterpreterCreate(model_, options_); // Construct the graph and apply the checked delegate using the validated model/options.
            if (!interpreter_) // Stop when graph construction or delegate application cannot produce an interpreter.
                throw std::runtime_error("Coral/TensorFlow Lite interpreter creation failed");
            stage("Allocating tensors");
            if (TfLiteInterpreterAllocateTensors(interpreter_) != kTfLiteOk) // Allocate graph tensor storage and check the API status before touching any tensor.
                throw std::runtime_error(
                    "Coral tensor allocation failed; check native runtime compatibility");
            stage("Validating detector tensor layouts");
            if (TfLiteInterpreterGetInputTensorCount(interpreter_) != 1 ||
                TfLiteInterpreterGetOutputTensorCount(interpreter_) != 4) // Require exactly one image input and the four postprocessed SSD outputs.
                throw std::runtime_error(
                    "Use the supplied SSD MobileNet V2 COCO postprocess model");
            auto *input = TfLiteInterpreterGetInputTensor(interpreter_, 0); // Require the detector contract exactly: uint8 RGB input and four float SSD outputs. Bound result capacity before interpreting tensor memory. Borrow the sole input tensor for exact shape/type/byte-count validation and warm-up copying.
            if (TfLiteTensorType(input) != kTfLiteUInt8 || TfLiteTensorNumDims(input) != 4 ||
                TfLiteTensorDim(input, 0) != 1 || TfLiteTensorDim(input, 1) != 300 ||
                TfLiteTensorDim(input, 2) != 300 || TfLiteTensorDim(input, 3) != 3 ||
                TfLiteTensorByteSize(input) != 300 * 300 * 3) // Require uint8 batch-one 300x300 RGB input with exactly 270000 bytes.
                throw std::runtime_error("Expected uint8 [1,300,300,3] RGB input");
            for (int i = 0; i < 4; ++i) // Check each required SSD output before treating tensor memory as float storage.
                if (TfLiteTensorType(output(i)) != kTfLiteFloat32) // Reject quantized or otherwise differently typed output contracts.
                    throw std::runtime_error("Expected float SSD postprocess outputs");
            auto *boxes = output(0); // Output zero must contain the batch-one array of four-coordinate detection boxes.
            if (TfLiteTensorNumDims(boxes) != 3 || TfLiteTensorDim(boxes, 0) != 1 ||
                TfLiteTensorDim(boxes, 2) != 4) // Require [1, N, 4] boxes so later indexing by four floats is valid.
                throw std::runtime_error(
                    "Unexpected SSD output order (boxes, classes, scores, count required)");
            const int capacity = TfLiteTensorDim(boxes, 1); // Read the maximum number of detections from the validated boxes tensor layout.
            if (capacity < 1 || capacity > 1000 ||
                TfLiteTensorByteSize(boxes) != std::size_t(capacity) * 4 * sizeof(float) ||
                TfLiteTensorByteSize(output(1)) != std::size_t(capacity) * sizeof(float) ||
                TfLiteTensorByteSize(output(2)) != std::size_t(capacity) * sizeof(float) ||
                TfLiteTensorByteSize(output(3)) != sizeof(float)) // Bound capacity and require consistent box/class/score/count byte sizes before inference.
                throw std::runtime_error("Invalid SSD output dimensions");
            std::vector<unsigned char> zero(300 * 300 * 3, 0); // Supply a valid all-black RGB tensor for a real startup invocation.
            stage("Copying warmup input");
            if (TfLiteTensorCopyFromBuffer(input, zero.data(), zero.size()) != kTfLiteOk) // Copy the complete warm-up image into interpreter-owned input memory.
                throw std::runtime_error("Native Coral warmup input copy failed");
            stage("Invoking detector on USB accelerator");
            if (TfLiteInterpreterInvoke(interpreter_) != kTfLiteOk) // Invoke the graph and fail explicitly if runtime execution returns an error.
                throw std::runtime_error("Native Coral warmup inference failed");
            std::cout << "TensorFlow Lite " << TfLiteVersion() << " | libedgetpu " << version
                      << " | USB delegate active | native warmup passed\n"; // The stock resolver cannot execute edgetpu-custom-op on the CPU. Successful execution of this model with fallback disabled proves the required custom operation was handled by the USB delegate.
        } catch (...) {
            cleanup(); // Release any successfully acquired resources in dependency order.
            throw; // Rethrow the original startup exception after cleanup so the caller sees its cause.
        }
    }
    ~Engine() { // Release interpreter, model, delegate, and library when the owning service shuts down.
        cleanup(); // Release any successfully acquired resources in dependency order.
    }
    std::vector<Detection> infer(const Frame &frame, double &milliseconds) { // CPU preparation resizes BGR to RGB. The measured interval covers interpreter Invoke, including USB/runtime and CPU postprocessing inside invocation; it excludes resize/copy work and is not end-to-end sensor latency.
        cv::Mat bgr(480, 640, CV_8UC3, const_cast<unsigned char *>(frame.pixels->data())); // Wrap immutable source pixels for read-only OpenCV preprocessing; no copy is made here.
        cv::Mat resized, rgb; // Own CPU intermediate images for resizing and channel-order conversion.
        cv::resize(bgr, resized, {300, 300}); // Scale the paired camera format to the detector's required 300x300 input dimensions.
        cv::cvtColor(resized, rgb, cv::COLOR_BGR2RGB); // Convert camera BGR bytes to the RGB channel order required by the model.
        if (TfLiteTensorCopyFromBuffer(TfLiteInterpreterGetInputTensor(interpreter_, 0), rgb.data,
                                       rgb.total() * rgb.elemSize()) != kTfLiteOk) // Copy the prepared RGB pixels into the validated interpreter input buffer.
            throw std::runtime_error("Coral input copy failed");
        const double start = clock_now(); // Start the Invoke-only timer after CPU resizing, color conversion, and input copying.
        if (TfLiteInterpreterInvoke(interpreter_) != kTfLiteOk) // Invoke the graph and fail explicitly if runtime execution returns an error.
            throw std::runtime_error("Native Coral invocation failed");
        milliseconds = (clock_now() - start) * 1000; // Measure invocation duration in milliseconds; this includes runtime/USB and graph CPU work.
        const auto *boxes = static_cast<const float *>(TfLiteTensorData(output(0))); // Borrow the postprocessed box array from output zero after successful invocation.
        const auto *classes = static_cast<const float *>(TfLiteTensorData(output(1))); // Borrow zero-based COCO class values from output one.
        const auto *scores = static_cast<const float *>(TfLiteTensorData(output(2))); // Borrow confidence values from output two.
        const float count = *static_cast<const float *>(TfLiteTensorData(output(3))); // Read output three's reported detection count before interpreting any result entries.
        const int capacity = TfLiteTensorDim(output(0), 1); // Recover the previously validated maximum detection count for bounds checking.
        if (!std::isfinite(count) || count < 0 || count > capacity || count != std::floor(count)) // Reject nonfinite, fractional, negative, or over-capacity counts before looping.
            throw std::runtime_error("Invalid SSD detection count");
        std::vector<Detection> result; // Reject nonfinite/out-of-contract outputs before using them as a semantic observation. Clip boxes to normalized image bounds and omit degenerate boxes. Collect validated detections above the display/semantic candidate confidence floor.
        for (int i = 0; i < int(count); ++i) { // Read only the number of detections reported by the validated result count.
            if (!std::isfinite(scores[i]) || !std::isfinite(classes[i])) // Reject NaN/infinite confidence or class values before comparisons and integer conversion.
                throw std::runtime_error("Nonfinite SSD result");
            if (scores[i] < .45f) // Drop low-confidence candidates before exposing them to the application.
                continue;
            if (scores[i] > 1 || classes[i] < 0 || classes[i] > 89 ||
                classes[i] != std::floor(classes[i])) // Require unit-interval confidence and integral zero-based class IDs within the model's 90-slot range.
                throw std::runtime_error("Invalid SSD class/score");
            for (int j = 0; j < 4; ++j) // Validate all four coordinates of this detection box before clipping them.
                if (!std::isfinite(boxes[4 * i + j])) // A nonfinite box cannot represent a usable image region.
                    throw std::runtime_error("Nonfinite SSD box");
            float y0 = std::clamp(boxes[4 * i], 0.f, 1.f),
                  x0 = std::clamp(boxes[4 * i + 1], 0.f, 1.f); // Clamp top/left normalized coordinates to the image, converting SSD's YXYX order.
            float y1 = std::clamp(boxes[4 * i + 2], 0.f, 1.f),
                  x1 = std::clamp(boxes[4 * i + 3], 0.f, 1.f); // Clamp bottom/right normalized coordinates while preserving their semantic order.
            if (x1 > x0 && y1 > y0) // Discard boxes with zero or negative area after clipping.
                result.push_back({int(classes[i]), scores[i], x0, y0, x1, y1}); // Store accepted class, confidence, and normalized XYXY coordinates in the application contract.
        }
        return result; // Return an owned list; future interpreter invocations may overwrite the underlying output tensors.
    }
};
} // namespace
} // namespace tabletop
