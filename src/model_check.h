/**
 * @file
 * Bounded inspection of a TFLite FlatBuffer operator-code table.
 * The detector must contain the Edge TPU custom operation. This is an early
 * model identity check; TensorFlow Lite performs full graph validation.
 */

#pragma once // Include this header only once per translation unit.
#include <cstdint> // Explicit-width integer types for binary contracts, timestamps, and sequence values.
#include <fstream> // File streams for configuration, detector bytes, and run diagnostics.
#include <string> // Owned text values for paths, diagnostic messages, and configuration names.
#include <vector> // Owned variable-length storage for pixels, measurements, routes, or detections.
#include <stdexcept> // Exceptions used to report invalid input or resource initialization failures.
namespace tabletop {
inline bool has_edge_tpu_operator(const std::vector<unsigned char> &data) { // Read just the FlatBuffer operator-code tables. No generated schema headers, so this check is independent of the distro's FlatBuffers header version. TensorFlow Lite remains responsible for full model validation/execution.
    auto fail = []() { throw std::runtime_error("Malformed TFLite operator-code table"); }; // Use one failure path for malformed FlatBuffer offsets and table bounds.
    auto u16 = [&](std::size_t p) -> std::uint16_t { // All integer/relative-offset readers check remaining buffer length before access. Vtable lookup returns zero for an absent field, never an unchecked pointer. Read an unaligned little-endian 16-bit integer only after checking remaining buffer size.
        if (p > data.size() || data.size() - p < 2) // Check the offset first so subtraction cannot underflow before the two-byte length test.
            fail();
        return data[p] | (std::uint16_t(data[p + 1]) << 8); // Combine two bytes explicitly without assuming host alignment or endianness.
    };
    auto u32 = [&](std::size_t p) -> std::uint32_t { // Provide the equivalent checked little-endian reader for 32-bit lengths and relative offsets.
        if (p > data.size() || data.size() - p < 4) // Require all four bytes before accessing the encoded integer.
            fail();
        return data[p] | (std::uint32_t(data[p + 1]) << 8) | (std::uint32_t(data[p + 2]) << 16) |
               (std::uint32_t(data[p + 3]) << 24); // Assemble the four little-endian bytes using unsigned shifts.
    };
    auto indirect = [&](std::size_t p) { // Resolve a FlatBuffer forward-relative offset without creating an unchecked pointer.
        std::size_t n = u32(p); // Read the relative byte displacement stored at this offset location.
        if (n > data.size() - p) // Prevent the resolved location from extending beyond the owned byte vector.
            fail();
        return p + n; // Return the checked absolute buffer offset for the referenced object.
    };
    auto field = [&](std::size_t table, unsigned index) -> std::size_t { // Locate an optional table field by reading its vtable entry.
        std::int64_t vt = std::int64_t(table) - std::int32_t(u32(table)); // FlatBuffer vtable offsets are signed; promote before subtracting to check either direction.
        if (vt < 0 || std::uint64_t(vt) > data.size()) // Reject a vtable location outside the buffer before converting it to an unsigned index.
            fail();
        std::size_t v = std::size_t(vt), entry = 4 + 2 * index; // Skip the vtable's two 16-bit headers and index its 16-bit field offsets.
        if (u16(v) < entry + 2) // A short vtable means this field is absent from the serialized table.
            return 0;
        auto offset = u16(v + entry); // Read the table-relative field offset from its checked vtable entry.
        if (!offset) // A zero entry explicitly represents an omitted/default field.
            return 0;
        if (offset > data.size() - table) // Check the field's relative location before adding it to the table base.
            fail();
        return table + offset; // Return the absolute location of the present field's encoded value.
    };
    if (data.size() < 8 || std::string(data.begin() + 4, data.begin() + 8) != "TFL3") // Confirm the TFL3 file identifier, locate the root operator vector, then scan custom-op names. Both scalar encodings in the TFLite schema are supported. Require enough bytes for root offset and the TFLite file identifier TFL3.
        return false;
    auto op_field = field(u32(0), 1); // Locate schema field 1, the root model's operator-code vector.
    if (!op_field) // A model with no operator-code vector cannot contain the required accelerator custom operation.
        return false;
    auto vec = indirect(op_field); // Resolve the operator vector's relative offset to its length header.
    std::size_t count = u32(vec); // Read the number of operator-table offsets stored in the vector.
    if (count > (data.size() - vec - 4) / 4) // Bound the count by the remaining bytes, with four bytes per table offset.
        fail();
    for (std::size_t i = 0; i < count; ++i) { // Inspect each declared operator code without reading beyond the checked vector.
        auto table = indirect(vec + 4 + i * 4), name_field = field(table, 1); // Resolve this operator table and locate its optional custom-name string field.
        auto code_field = field(table, 3), byte_code_field = field(table, 0); // Support both the wider builtin-code field and its compact byte encoding.
        int code = code_field ? int(u32(code_field)) // Prefer the wider field when present; otherwise read the byte code or its default zero.
                              : (byte_code_field ? int(data.at(byte_code_field)) : 0);
        if (code != 32 || !name_field) // Examine names only for BuiltinOperator_CUSTOM entries with an actual custom-name field.
            continue; // BuiltinOperator_CUSTOM
        auto name = indirect(name_field); // Resolve the custom operator's length-prefixed string object.
        std::size_t length = u32(name); // Read the string byte count before constructing a bounded C++ string.
        if (length > data.size() - name - 4) // Ensure all declared string bytes lie inside the model buffer.
            fail();
        if (std::string(data.begin() + name + 4, data.begin() + name + 4 + length) ==
            "edgetpu-custom-op") // Match the exact Edge TPU operation name rather than merely accepting any custom operator.
            return true;
    }
    return false;
}
inline std::vector<unsigned char> read_detector_model(const std::string &path) { // Bound file size before allocation and require the accelerator operation before creating an interpreter. Return owned bytes that outlive the TFLite model.
    std::ifstream f(path, std::ios::binary | std::ios::ate); // Open in binary mode at end-of-file to validate size before allocation.
    if (!f) // Report a missing/unreadable detector path before attempting to inspect bytes.
        throw std::runtime_error("Cannot open detector model: " + path);
    auto size = f.tellg(); // Obtain the model's byte length from the end-positioned stream.
    if (size < 8 || size > 20 * 1024 * 1024) // Reject undersized files and files over 20 MiB before allocating a buffer.
        throw std::runtime_error("Unexpected detector model size");
    std::vector<unsigned char> bytes(static_cast<std::size_t>(size)); // Allocate exactly the bounded model length in owned storage.
    f.seekg(0); // Rewind to the first byte after the size check.
    if (!f.read(reinterpret_cast<char *>(bytes.data()), size)) // Require a complete binary read; partial reads cannot be used as a model.
        throw std::runtime_error("Could not read detector model");
    if (!has_edge_tpu_operator(bytes)) // Require a compiled accelerator graph instead of substituting a CPU-only model.
        throw std::runtime_error("No Edge TPU custom operator in model; refusing CPU substitution");
    return bytes; // Transfer owned model bytes to the engine so their lifetime exceeds the TFLite model wrapper's.
}
} // namespace tabletop
