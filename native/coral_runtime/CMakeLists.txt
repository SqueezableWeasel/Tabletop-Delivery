cmake_minimum_required(VERSION 3.18) # Require CMake 3.18 features and select the corresponding policy baseline.
project(tabletop_coral_runtime LANGUAGES CXX) # Declare this native build's project identity and enable the C++ compiler.
set(CMAKE_CXX_STANDARD 17) # Compile upstream delegate sources using their supported C++17 language level.
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Fail configuration rather than silently compiling with a lower C++ language standard.
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # Generate relocatable machine code suitable for a dynamically loaded shared library.
if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux") # Reject non-Linux targets for code that depends on Linux devices and dynamic loading.
  message(FATAL_ERROR "Build this private runtime on the Linux Raspberry Pi") # Stop configuration with an actionable prerequisite or contract error.
endif()
set(EDGETPU_SOURCE "" CACHE PATH "Pinned upstream libedgetpu source") # Accept the verified pinned upstream checkout as a caller-supplied cache path.
if(NOT EXISTS "${EDGETPU_SOURCE}/makefile_build/Makefile") # Require the expected upstream build manifest before deriving a native source list.
  message(FATAL_ERROR "EDGETPU_SOURCE must point to the downloaded pinned source") # Stop configuration with an actionable prerequisite or contract error.
endif()
find_package(PkgConfig REQUIRED) # Require pkg-config for discovery of installed native development libraries.
find_package(Threads REQUIRED) # Discover the platform's thread library and compiler/linker requirements.
find_package(absl CONFIG REQUIRED) # Load installed Abseil CMake targets required by upstream runtime sources.
pkg_check_modules(USB REQUIRED IMPORTED_TARGET libusb-1.0) # Import installed libusb headers/linker settings for USB Accelerator communication.
find_path(TFLITE_INCLUDE tensorflow/lite/c/c_api.h REQUIRED) # Locate the installed public TensorFlow Lite C headers used for ABI-compatible compilation.
find_library(TFLITE_LIBRARY NAMES tensorflow-lite tensorflowlite REQUIRED) # Locate the installed TensorFlow Lite shared library using supported distro names.
find_path(FLATBUFFERS_INCLUDE flatbuffers/flatbuffers.h REQUIRED) # Find the installed FlatBuffers headers paired with the local schema-generation toolchain.
find_library(FLATBUFFERS_LIBRARY NAMES flatbuffers REQUIRED) # Find the FlatBuffers support library linked by generated/runtime code.
find_program(FLATC flatc REQUIRED) # Require the schema compiler used to generate headers matching installed FlatBuffers.

file(READ "${EDGETPU_SOURCE}/makefile_build/Makefile" makefile) # Use the same official native source list as Google's Makefile, but link common/util functions from the installed TFLite instead of embedding a second copy of those functions from a different TensorFlow checkout. Read the pinned upstream build manifest instead of maintaining a divergent manual source list.
string(REGEX MATCHALL "[$][(]BUILDROOT[)]/[a-zA-Z0-9_/]+[.]cc" source_lines "${makefile}") # Extract BUILDROOT-relative C++ source paths from the official Makefile.
set(sources) # Begin an empty list of source files for the private delegate target.
foreach(line IN LISTS source_lines) # Convert every extracted Makefile path into a path under the verified checkout.
  string(REPLACE "$(BUILDROOT)" "${EDGETPU_SOURCE}" source "${line}") # Substitute the actual pinned source directory for the upstream build-root variable.
  list(APPEND sources "${source}") # Add this resolved native source to the target input list.
endforeach()
list(REMOVE_DUPLICATES sources) # Deduplicate repeated source references in the upstream manifest.
list(LENGTH sources source_count) # Count native sources to detect an unexpectedly changed or unparsed manifest.
if(source_count LESS 70) # Reject an implausibly short list rather than silently compiling an incomplete runtime.
  message(FATAL_ERROR "Unexpected upstream native source list") # Stop configuration with an actionable prerequisite or contract error.
endif()
set(generated "${CMAKE_CURRENT_BINARY_DIR}/generated") # Isolate adapted wrappers, schemas, and firmware headers in this build's generated directory.
file(MAKE_DIRECTORY "${generated}/executable" "${generated}/api" "${generated}/driver/usb") # Create the subdirectories expected by upstream include paths and generated headers.
foreach(name edgetpu_delegate_for_custom_op edgetpu_delegate_for_custom_op_tflite_plugin) # The two delegate wrappers include unused builtin-op definitions, whose transitive headers are missing from the Pi 2.20 development package. util.h also pulls in a mismatched generated FlatBuffers schema merely for a one-element array helper. Use its equivalent public C allocation API. Preserve the upstream files; compile adjusted copies in the build directory. Adapt the two delegate wrappers without modifying the downloaded upstream source files.
  set(original "${EDGETPU_SOURCE}/tflite/${name}.cc") # Locate the current wrapper's original upstream C++ implementation.
  file(READ "${original}" text) # Read the wrapper into a CMake string for narrowly scoped compatibility substitutions.
  string(REPLACE "#include \"tensorflow/lite/builtin_op_data.h\"\n" "" text "${text}") # Remove unused includes that pull unavailable or mismatched transitive development headers.
  string(REPLACE "#include \"tensorflow/lite/util.h\"\n" "" text "${text}") # Remove unused includes that pull unavailable or mismatched transitive development headers.
  string(REPLACE "using tflite::ConvertVectorToTfLiteIntArray;\n" "" text "${text}") # Remove the helper alias after replacing its sole one-element allocation use.
  string(REPLACE "TfLiteIntArray* nodes = ConvertVectorToTfLiteIntArray({node_index});" # Replace the schema-dependent helper with the equivalent public C allocation and null check.
    "TfLiteIntArray* nodes = TfLiteIntArrayCreate(1);\n    TF_LITE_ENSURE(context, nodes != nullptr);\n    nodes->data[0] = node_index;" text "${text}")
  file(WRITE "${generated}/${name}.cc" "${text}") # Write the adapted wrapper into the build tree, preserving upstream source integrity.
  list(REMOVE_ITEM sources "${original}") # Remove the original wrapper from compilation so it is not linked twice.
  list(APPEND sources "${generated}/${name}.cc") # Compile the generated compatibility wrapper in place of its original counterpart.
endforeach()
foreach(schema executable/executable api/driver_options) # Generate both executable-format and driver-options schema headers with the installed flatc.
  get_filename_component(subdir "${schema}" DIRECTORY) # Preserve the schema's relative subdirectory in generated include paths.
  execute_process(COMMAND "${FLATC}" --cpp --gen-object-api --force-empty --gen-mutable # Run flatc during configuration and capture its exit status before continuing the build.
    -o "${generated}/${subdir}" "${EDGETPU_SOURCE}/${schema}.fbs" RESULT_VARIABLE result)
  if(NOT result EQUAL 0) # A failed schema compiler must stop configuration before sources reference missing headers.
    message(FATAL_ERROR "flatc failed for ${schema}") # Stop configuration with an actionable prerequisite or contract error.
  endif()
endforeach()

set(firmware "namespace {\n") # Embed the upstream USB firmware directly from CMake's byte-to-hex conversion. Begin a generated anonymous-namespace header that gives firmware arrays internal linkage.
foreach(name apex_latest_single_ep apex_latest_multi_ep) # Embed both upstream single-endpoint and multi-endpoint USB firmware variants.
  file(READ "${EDGETPU_SOURCE}/driver/usb/${name}.bin" bytes HEX) # Read firmware bytes as hexadecimal text for portable C++ array generation.
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," literals "${bytes}") # Convert each two-digit byte into a comma-separated 0xNN C++ initializer literal.
  string(APPEND firmware "const unsigned char ${name}[] = {${literals}};\n") # Emit the named immutable firmware byte array used by the USB driver.
  string(APPEND firmware "constexpr unsigned int ${name}_len = sizeof(${name});\n") # Emit the matching byte count from sizeof so array and length cannot diverge.
endforeach()
string(APPEND firmware "}\n") # Close the generated anonymous namespace after both firmware variants.
file(WRITE "${generated}/driver/usb/usb_latest_firmware.h" "${firmware}") # Materialize the firmware header at the include path expected by upstream USB sources.

add_library(tabletop_edgetpu SHARED ${sources} runtime_identity.cpp) # Build upstream runtime sources and project identity exports into one shared object.
set_target_properties(tabletop_edgetpu PROPERTIES OUTPUT_NAME edgetpu VERSION 1.0 SOVERSION 1) # Name the library libedgetpu and provide the .so.1 ABI symlink expected by the application.
target_compile_definitions(tabletop_edgetpu PRIVATE DARWINN_PORT_DEFAULT THROTTLE_EDGE_TPU) # Select the upstream default platform port and throttled operating mode.
target_include_directories(tabletop_edgetpu PRIVATE "${EDGETPU_SOURCE}" "${generated}" "${TFLITE_INCLUDE}" "${FLATBUFFERS_INCLUDE}") # Compile against verified upstream, generated, installed TFLite, and FlatBuffers headers.
target_link_libraries(tabletop_edgetpu PRIVATE "${TFLITE_LIBRARY}" "${FLATBUFFERS_LIBRARY}" PkgConfig::USB Threads::Threads ${CMAKE_DL_LIBS} # Link one installed TFLite plus USB, threading, FlatBuffers, loader, and required Abseil components.
  absl::flags absl::flags_internal absl::flags_reflection absl::flags_marshalling
  absl::hash absl::hashtablez_sampler absl::raw_hash_set absl::str_format_internal)
target_link_options(tabletop_edgetpu PRIVATE "-Wl,--no-undefined" "-Wl,--version-script=${EDGETPU_SOURCE}/tflite/public/libedgetpu.lds") # Fail unresolved symbols and preserve upstream's public symbol-version/export policy.
install(TARGETS tabletop_edgetpu LIBRARY DESTINATION .) # Install the shared library under the caller's project-local prefix rather than into system directories.
message(STATUS "Private Coral runtime: ${source_count} upstream C++ sources; TFLite ${TFLITE_LIBRARY}") # Report source count and actual TFLite library selected for this private runtime build.
