/**
 * @file
 * Identity exports resolved by the application before delegate creation.
 * Compile this wrapper alongside libedgetpu with the same TensorFlow Lite
 * headers/library as the app. Exported symbol names form the runtime contract.
 */

#include "../../src/coral_abi.h"
#include <tensorflow/lite/c/c_api.h>
extern "C" __attribute__((visibility("default"))) std::uint64_t edgetpu_tabletop_abi_v1() { // Export the exact layout hash produced by the runtime compilation environment.
    return tabletop::coral_abi_fingerprint(); // Expose the layout fingerprint computed with the runtime's own compilation headers.
}
extern "C" __attribute__((visibility("default"))) const char *edgetpu_tabletop_tflite_version() { // Return the version of the TensorFlow Lite library actually linked by this runtime.
    return TfLiteVersion(); // Report the actual TensorFlow Lite library version linked by this shared object.
}
