/**
 * @file
 * TensorFlow Lite layout fingerprint shared by the application and delegate.
 * Hash sizes and offsets that cross the C boundary. This catches structural
 * mismatches before a delegate callback can interpret an incompatible object.
 */

#pragma once // Include this header only once per translation unit.
#include <tensorflow/lite/c/common.h>
#include <array> // Fixed-size standard arrays used where element count is part of the data contract.
#include <cstddef> // Byte sizes and member offsets used to compare native structure layouts.
#include <cstdint> // Explicit-width integer types for binary contracts, timestamps, and sequence values.
namespace tabletop {
inline std::uint64_t coral_abi_fingerprint() { // Fingerprint the C layouts which cross the delegate/interpreter boundary. In particular, TfLiteRegistration is passed BY VALUE during replacement. A C API call does not make differently compiled delegate layouts compatible.
    const std::size_t fields[]{sizeof(void *),
                               sizeof(TfLiteRegistration),
                               sizeof(TfLiteDelegate),
                               sizeof(TfLiteContext),
                               sizeof(TfLiteNode),
                               sizeof(TfLiteTensor),
                               sizeof(TfLiteDelegateParams),
                               offsetof(TfLiteRegistration, init),
                               offsetof(TfLiteRegistration, prepare),
                               offsetof(TfLiteRegistration, invoke),
                               offsetof(TfLiteRegistration, custom_name),
                               offsetof(TfLiteDelegate, data_),
                               offsetof(TfLiteDelegate, Prepare),
                               offsetof(TfLiteDelegate, flags),
                               offsetof(TfLiteContext, GetNodeAndRegistration),
                               offsetof(TfLiteContext, ReplaceNodeSubsetsWithDelegateKernels),
                               offsetof(TfLiteContext, GetExecutionPlan),
                               offsetof(TfLiteContext, SetExternalContext),
                               offsetof(TfLiteContext, GetExternalContext),
                               offsetof(TfLiteNode, user_data),
                               offsetof(TfLiteNode, custom_initial_data),
                               offsetof(TfLiteTensor, data),
                               offsetof(TfLiteTensor, dims)}; // Record pointer width plus critical structure sizes and member offsets from these installed headers.
    std::uint64_t hash = 14695981039346656037ull; // Initialize the standard 64-bit FNV-1a offset basis for a reproducible layout fingerprint.
    for (std::size_t value : fields) // Fold each measured layout fact into the fingerprint in a fixed order.
        for (unsigned shift = 0; shift < 64; shift += 8) { // Serialize each value as eight low-to-high bytes independent of host memory byte order.
            hash ^= (std::uint64_t(value) >> shift) & 255; // XOR the next layout byte into the current hash accumulator.
            hash *= 1099511628211ull; // Apply the FNV-1a 64-bit prime; unsigned overflow is the intended modular arithmetic.
        }
    return hash; // Return a structural mismatch detector, not a complete proof of arbitrary binary compatibility.
}
} // namespace tabletop
