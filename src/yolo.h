/**
 * @file
 * Portable YOLOv8 detection decoding. No TPU, OpenCV, or Python is needed here.
 * The selected contract is COCO-80 or figure/stop_sign, batch one, decoded XYWH plus class
 * probabilities, with NMS outside the graph. Coordinates may be normalized or
 * input pixels; the caller must declare the export's convention explicitly.
 */
#pragma once
#include "runtime.h"
#include <cstring> // Read float tensors without assuming their alignment.
#include <limits> // Bound quantized conversions before integer casts.
#include <stdexcept> // Reject unsupported or malformed model contracts.

namespace tabletop {
enum class DetectorKind { Ssd, YoloV8 }; // Keep the proven SSD backend selectable.
enum class YoloProfile { Coco, Figure }; // Explicit class semantics: shape alone cannot identify class names.
struct DetectorOptions {
    DetectorKind kind = DetectorKind::Ssd; // Existing runs keep the SSD model.
    float person_confidence = .35f; // Evaluation threshold, independent of stop-sign confidence.
    bool normalized_boxes = true; // Ultralytics TFLite exports normally normalize XYWH.
    float max_person_width = .30f, max_person_height = .65f; // Tabletop target-size gates reject broad arm/person boxes; fractions of the original camera frame.
    YoloProfile profile = YoloProfile::Coco; // Custom figure weights must be selected deliberately.
};
inline int yolo_classes(const DetectorOptions &o) { return o.profile == YoloProfile::Figure ? 2 : 80; } // XYWH precedes these class scores.
inline int stop_class(const DetectorOptions &o) { return o.kind == DetectorKind::Ssd ? 12 : (o.profile == YoloProfile::Figure ? 1 : 11); } // Preserve each model's actual label mapping.
inline bool detected_stop(const Detection &d, const DetectorOptions &o) { return d.id == stop_class(o) && d.score >= .60f; } // Leave the existing consecutive-detection hold policy in the core.
enum class TensorScalar { Float32, UInt8, Int8 }; // Supported public output tensor types.
struct YoloTensor {
    const void *data = nullptr; // Borrow only for the current inference invocation.
    std::size_t bytes = 0; // Validate the entire tensor before indexing it.
    int rows = 0, columns = 0; // Batch dimension was validated separately as one.
    TensorScalar type = TensorScalar::Float32;
    float scale = 1; // Per-tensor quantization scale from the model.
    int zero = 0; // Per-tensor zero point, including signed int8 zero points.
};
struct Letterbox {
    int width = 0, height = 0, resized_width = 0, resized_height = 0, left = 0, top = 0;
    float scale = 0; // Resize scale from the fixed 640x480 camera frame.
};
inline Letterbox letterbox(int width, int height) {
    if (width < 32 || height < 32 || width > 1280 || height > 1280 || width % 32 || height % 32)
        throw std::runtime_error("YOLO input must be static NHWC RGB, 32..1280 and divisible by 32");
    const float scale = std::min(width / 640.f, height / 480.f); // Preserve the original aspect ratio.
    const int w = int(std::round(640 * scale)), h = int(std::round(480 * scale));
    return {width, height, w, h, (width - w) / 2, (height - h) / 2, scale}; // Extra odd padding goes right/bottom.
}
inline std::size_t yolo_elements(const YoloTensor &tensor, int classes = 80) {
    if (classes != 2 && classes != 80) throw std::runtime_error("Unsupported YOLO class profile"); // Do not guess semantics for arbitrary custom models.
    const int channels = 4 + classes; // Four decoded coordinates followed by one probability per class.
    if (!tensor.data || tensor.rows < 1 || tensor.columns < 1 ||
        (tensor.rows != channels && tensor.columns != channels) || (tensor.rows == channels && tensor.columns == channels))
        throw std::runtime_error("YOLO output disagrees with --yolo-profile: expected " + std::to_string(channels) + " channels (XYWH + class scores), without embedded NMS");
    const int count = tensor.rows == channels ? tensor.columns : tensor.rows;
    if (count > 40000) throw std::runtime_error("YOLO prediction count exceeds evaluation limit");
    const std::size_t n = std::size_t(tensor.rows) * tensor.columns;
    const std::size_t unit = tensor.type == TensorScalar::Float32 ? sizeof(float) : 1;
    if (tensor.bytes != n * unit) throw std::runtime_error("YOLO output byte size does not match its shape");
    if (tensor.type != TensorScalar::Float32 &&
        (!std::isfinite(tensor.scale) || tensor.scale <= 0 ||
         tensor.zero < (tensor.type == TensorScalar::Int8 ? -128 : 0) ||
         tensor.zero > (tensor.type == TensorScalar::Int8 ? 127 : 255)))
        throw std::runtime_error("Invalid YOLO output quantization");
    return n;
}
inline float yolo_value(const YoloTensor &tensor, std::size_t index) {
    if (tensor.type == TensorScalar::Float32) {
        float value; // memcpy handles a possibly unaligned tensor allocation.
        std::memcpy(&value, static_cast<const unsigned char *>(tensor.data) + index * sizeof(float), sizeof(float));
        return value;
    }
    const int value = tensor.type == TensorScalar::Int8
        ? int(static_cast<const std::int8_t *>(tensor.data)[index])
        : int(static_cast<const std::uint8_t *>(tensor.data)[index]);
    return (value - tensor.zero) * tensor.scale; // Decode the model's own scale, never a fixed /255 assumption.
}
inline unsigned char quantized_rgb(unsigned char pixel, float scale, int zero, bool signed_input) {
    if (!std::isfinite(scale) || scale <= 0 || zero < (signed_input ? -128 : 0) || zero > (signed_input ? 127 : 255))
        throw std::runtime_error("Invalid YOLO input quantization");
    const double q = std::round((pixel / 255.) / scale + zero); // YOLO consumes RGB normalized to [0,1].
    const int value = int(std::clamp(q, signed_input ? -128. : 0., signed_input ? 127. : 255.));
    return static_cast<unsigned char>(value); // Preserve the two's-complement bit pattern for int8 input.
}
inline float box_iou(const Detection &a, const Detection &b) {
    const float area = std::max(0.f, std::min(a.x1, b.x1) - std::max(a.x0, b.x0)) *
                       std::max(0.f, std::min(a.y1, b.y1) - std::max(a.y0, b.y0));
    const float total = (a.x1-a.x0)*(a.y1-a.y0) + (b.x1-b.x0)*(b.y1-b.y0) - area;
    return total > 0 ? area / total : 0;
}
inline std::vector<Detection> decode_yolov8(const YoloTensor &tensor, const Letterbox &box,
                                          const DetectorOptions &options) {
    const int classes = yolo_classes(options), channels = classes + 4; // The explicit profile determines indexing and semantic class IDs.
    yolo_elements(tensor, classes); // Validate shape, byte count, and quantization before reading values.
    if (!(options.person_confidence > 0 && options.person_confidence <= 1) || box.scale <= 0)
        throw std::runtime_error("Invalid YOLO confidence or letterbox geometry");
    const bool channels_first = tensor.rows == channels;
    const int n = channels_first ? tensor.columns : tensor.rows;
    const auto at = [&](int anchor, int channel) {
        return yolo_value(tensor, channels_first ? std::size_t(channel)*n+anchor : std::size_t(anchor)*channels+channel);
    };
    std::vector<Detection> candidates;
    for (int i = 0; i < n; ++i) {
        int category = 0;
        float score = -1;
        for (int c = 0; c < classes; ++c) {
            const float value = at(i, c + 4); // YOLOv8 has no separate objectness channel.
            if (!std::isfinite(value) || value < -.02f || value > 1.02f)
                throw std::runtime_error("Invalid YOLO class probability; check export contract");
            if (value > score) { score = value; category = c; }
        }
        const float threshold = category == 0 ? options.person_confidence : .45f;
        if (score < threshold) continue; // Retain person candidates and other semantic boxes for diagnostics.
        float cx = at(i,0), cy = at(i,1), w = at(i,2), h = at(i,3);
        if (!std::isfinite(cx) || !std::isfinite(cy) || !std::isfinite(w) || !std::isfinite(h))
            throw std::runtime_error("Nonfinite YOLO box");
        if (w <= 0 || h <= 0) continue;
        if (options.normalized_boxes) { cx *= box.width; w *= box.width; cy *= box.height; h *= box.height; }
        if (std::abs(cx)>1e6f || std::abs(cy)>1e6f || w>1e6f || h>1e6f)
            throw std::runtime_error("YOLO box exceeds plausible image bounds");
        Detection d{category, std::clamp(score,0.f,1.f),
            (cx-w/2-box.left)/(box.scale*640), (cy-h/2-box.top)/(box.scale*480),
            (cx+w/2-box.left)/(box.scale*640), (cy+h/2-box.top)/(box.scale*480)};
        if (d.x1 > d.x0 && d.y1 > d.y0) candidates.push_back(d);
    }
    std::stable_sort(candidates.begin(), candidates.end(), [](const Detection &a, const Detection &b) { return a.score > b.score; });
    std::vector<Detection> accepted;
    for (const auto &d : candidates) {
        bool duplicate = false;
        for (const auto &other : accepted) if (d.id == other.id && box_iou(d,other) > .45f) { duplicate = true; break; }
        if (!duplicate) accepted.push_back(d); // Class-aware NMS preserves overlapping person/stop-sign classes.
        if (accepted.size() == 300) break; // Bound downstream display and association work.
    }
    std::vector<Detection> clipped;
    for(auto d:accepted) { // Match standard YOLO ordering: NMS first, then clip to the original image.
        d.x0=std::clamp(d.x0,0.f,1.f); d.y0=std::clamp(d.y0,0.f,1.f);
        d.x1=std::clamp(d.x1,0.f,1.f); d.y1=std::clamp(d.y1,0.f,1.f);
        if(d.x1>d.x0 && d.y1>d.y0) clipped.push_back(d); // Drop predictions wholly inside letterbox padding.
    }
    return clipped;
}
inline bool eligible_person(const Detection &d, const DetectorOptions &options) {
    return d.id == 0 && d.score >= options.person_confidence &&
           d.x1-d.x0 <= options.max_person_width && d.y1-d.y0 <= options.max_person_height;
}
inline std::vector<Marker> person_observations(const CoralState &state, const DetectorOptions &options) {
    std::vector<Marker> found;
    for (const auto &d : state.detections) {
        if (!eligible_person(d,options)) continue; // Class zero is person in COCO or figure in the custom profile; apply the same size gates.
        if (!std::isfinite(d.x0) || !std::isfinite(d.x1) || !std::isfinite(d.y0) || !std::isfinite(d.y1) ||
            d.x0 < 0 || d.y0 < 0 || d.x1 > 1 || d.y1 > 1 || d.x1 <= d.x0 || d.y1 <= d.y0)
            throw std::runtime_error("Invalid person box");
        const int x = int(std::floor(d.x0*640)), y = int(std::floor(d.y0*480));
        const int w = int(std::ceil(d.x1*640))-x, h = int(std::ceil(d.y1*480))-y;
        found.push_back({(d.x0+d.x1)*320., (d.y0+d.y1)*240., x,y,w,h,w*h}); // Center bearing is an evaluation approximation.
        if (found.size() == 16) break; // Two or more candidates already force an ambiguity hold.
    }
    return found;
}
} // namespace tabletop
