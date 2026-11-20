cmake_minimum_required(VERSION 3.18) # Require CMake 3.18 features and select the corresponding policy baseline.
project(tabletop_delivery LANGUAGES CXX) # Declare this native build's project identity and enable the C++ compiler.
set(CMAKE_CXX_STANDARD 20) # The application uses C++20 to match libcamera's public control interfaces. Hardware dependencies are optional so the same core/UI can run on a desktop. Use C++20, including the typed std::span contract required by installed libcamera controls.
set(CMAKE_CXX_STANDARD_REQUIRED ON) # Fail configuration rather than silently compiling with a lower C++ language standard.
set(CMAKE_CXX_EXTENSIONS OFF) # Disable compiler-specific language extensions for portable core and test builds.
option(TABLETOP_BUILD_APP "Build OpenCV application" ON) # Allow core-only testing when OpenCV presentation/configuration is not needed.
option(TABLETOP_HARDWARE "Build native libcamera, SLAMTEC and Coral backends" ON) # Select real Linux backends or explicit rejecting desktop stubs at build time.
set(RPLIDAR_SDK "${CMAKE_CURRENT_SOURCE_DIR}/third_party/rplidar_sdk" CACHE PATH "SLAMTEC SDK checkout") # Cache the caller-overridable SDK checkout, defaulting to a repository-local dependency directory.
add_library(tabletop_core STATIC native/core.cpp src/runtime.cpp) # Portable layer: no OpenCV, sensor SDK, or inference libraries are required. Build the portable image/geometry/state-machine code plus synthetic observation helpers.
target_include_directories(tabletop_core PUBLIC native src) # Export core and runtime headers to targets that link the portable library.
if(MSVC) # Select warning and exception flags appropriate to the compiler family.
  target_compile_options(tabletop_core PRIVATE /W4 /EHsc) # Enable strong MSVC warnings and standard C++ exception unwinding.
else()
  target_compile_options(tabletop_core PRIVATE -Wall -Wextra -Wpedantic) # Enable common, extra, and pedantic diagnostics on GCC/Clang.
endif()
enable_testing() # Register this directory's tests with CTest.
add_executable(yolo_test tests/yolo_test.cpp) # Exercise quantization, NMS, letterbox inversion, and independent temporal evidence without a TPU.
target_link_libraries(yolo_test PRIVATE tabletop_core)
add_test(NAME yolo COMMAND yolo_test)
add_executable(planner_test tests/planner_test.cpp) # Always build geometry, behavioral, and model-table tests. They need no devices. Build independent continuous-geometry and obstacle-sweep checks.
target_link_libraries(planner_test PRIVATE tabletop_core) # Link only the portable core into the geometry test.
add_test(NAME planner COMMAND planner_test) # Register the geometry executable as the planner CTest case.
add_executable(runtime_test tests/runtime_test.cpp) # Build behavioral scenarios for tracking, movement, holds, and live boundary changes.
target_link_libraries(runtime_test PRIVATE tabletop_core) # Link the core and deterministic observation generator without physical hardware.
add_test(NAME runtime COMMAND runtime_test) # Register the deterministic state-machine behavior suite.
add_executable(model_test tests/model_test.cpp) # Build the standalone bounded model-table parser checks without TFLite or a TPU.
add_test(NAME model COMMAND model_test) # Register the parser's malformed/model-identity fixtures.
if(TABLETOP_BUILD_APP) # Add OpenCV application/configuration/presentation targets only when requested.
  find_package(OpenCV 4 REQUIRED COMPONENTS core imgproc imgcodecs highgui) # Presentation/configuration layer: OpenCV is shared by the app and its tests. Require OpenCV's matrix, image-processing, image-codec, and GUI components.
  find_package(Threads REQUIRED) # Discover the platform's thread library and compiler/linker requirements.
  add_library(tabletop_ui STATIC src/config.cpp src/view.cpp src/calibration_math.cpp) # Share configuration, measured calibration fitting, and presentation helpers with headless tests.
  target_include_directories(tabletop_ui PUBLIC ${OpenCV_INCLUDE_DIRS}) # Export OpenCV include paths required by the public application interfaces.
  target_link_libraries(tabletop_ui PUBLIC tabletop_core ${OpenCV_LIBS}) # Propagate core and OpenCV libraries to application/test consumers.
  add_executable(evaluation_ui_test tests/evaluation_ui_test.cpp) # Verify evaluation CSV/JSON and render both modes using explicitly synthetic detections.
  target_link_libraries(evaluation_ui_test PRIVATE tabletop_ui)
  add_test(NAME evaluation_ui COMMAND evaluation_ui_test "${CMAKE_CURRENT_BINARY_DIR}/evaluation_fixture")
  add_executable(tabletop_delivery src/main.cpp src/calibration.cpp) # Build delivery coordination and interactive calibration with the selected hardware backend.
  target_link_libraries(tabletop_delivery PRIVATE tabletop_ui Threads::Threads) # Link presentation, core, and platform thread support into the application.
  if(TABLETOP_HARDWARE) # Select live camera/LiDAR/inference implementation and its platform dependencies.
    if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux") # Live backends are Linux-only. TensorFlow Lite can be discovered through pkg-config or explicit header/library locations supplied to CMake. Reject non-Linux targets for code that depends on Linux devices and dynamic loading.
      message(FATAL_ERROR "Live hardware requires Linux. For desktop simulation use -DTABLETOP_HARDWARE=OFF") # Stop configuration with an actionable prerequisite or contract error.
    endif()
    find_package(PkgConfig REQUIRED) # Require pkg-config for discovery of installed native development libraries.
    pkg_check_modules(CAMERA REQUIRED IMPORTED_TARGET libcamera) # Discover installed libcamera headers and libraries as an imported CMake target.
    add_executable(camera_controls_test tests/camera_controls_test.cpp) # Build a device-free check of the installed library's typed control serialization.
    target_link_libraries(camera_controls_test PRIVATE PkgConfig::CAMERA) # Test the real installed libcamera implementation rather than a mock control container.
    add_test(NAME camera_controls COMMAND camera_controls_test) # Register the hardware-build-only control type/size/roundtrip check.
    pkg_check_modules(TFLITE QUIET IMPORTED_TARGET tensorflow-lite) # Prefer TensorFlow Lite's pkg-config metadata when the distro provides it.
    if(TFLITE_FOUND) # Use discovered transitive dependency metadata when available.
      set(TFLITE_TARGET PkgConfig::TFLITE) # Select the pkg-config imported target for subsequent application linking.
    else()
      find_path(TFLITE_INCLUDE tensorflow/lite/c/c_api.h REQUIRED) # Locate the installed public TensorFlow Lite C headers used for ABI-compatible compilation.
      find_library(TFLITE_LIBRARY NAMES tensorflow-lite tensorflowlite REQUIRED) # Locate the installed TensorFlow Lite shared library using supported distro names.
      add_library(tflite_system UNKNOWN IMPORTED) # Represent an explicitly discovered shared library as an imported target.
      set_target_properties(tflite_system PROPERTIES IMPORTED_LOCATION "${TFLITE_LIBRARY}" INTERFACE_INCLUDE_DIRECTORIES "${TFLITE_INCLUDE}") # Attach the fallback library location and matching headers to one coherent dependency target.
      set(TFLITE_TARGET tflite_system) # Use the fallback dependency target when pkg-config metadata is unavailable.
    endif()
    find_path(SLAMTEC_INCLUDE sl_lidar_driver.h HINTS "${RPLIDAR_SDK}/sdk/include" REQUIRED NO_DEFAULT_PATH) # Require the selected checkout's SDK headers rather than silently finding another system copy.
    find_library(SLAMTEC_LIBRARY NAMES sl_lidar_sdk rplidar_sdk HINTS "${RPLIDAR_SDK}/output/Linux/Release" REQUIRED NO_DEFAULT_PATH) # Require the library built in that checkout's Linux Release output directory.
    target_sources(tabletop_delivery PRIVATE src/sensors.cpp src/coral.cpp) # Compile native camera, serial LiDAR, and asynchronous USB inference backends.
    target_include_directories(tabletop_delivery PRIVATE "${SLAMTEC_INCLUDE}") # Add the selected SDK's public header directory to the live application.
    target_link_libraries(tabletop_delivery PRIVATE PkgConfig::CAMERA ${TFLITE_TARGET} "${SLAMTEC_LIBRARY}" ${CMAKE_DL_LIBS}) # Link camera, TFLite, SDK, and dynamic-loader support for live hardware operation.
  else()
    target_sources(tabletop_delivery PRIVATE src/no_hardware.cpp) # Explicit --demo is required when these rejecting hardware stubs are linked. Link rejecting hardware stubs; simulation is entered only through explicit --demo.
  endif()
  add_executable(config_test tests/config_test.cpp) # Build real JSON roundtrip, validation, boundary persistence, and headless rendering checks.
  target_link_libraries(config_test PRIVATE tabletop_ui) # Reuse the production configuration and rendering code in the test executable.
  add_test(NAME config COMMAND config_test "${CMAKE_CURRENT_SOURCE_DIR}/delivery_config.json") # Test the repository's supplied configuration using an explicit source path.
  add_executable(calibration_test tests/calibration_test.cpp src/calibration.cpp src/no_hardware.cpp) # Test measured fitting and rendering with rejecting device stubs; no physical calibration is claimed by this test.
  target_link_libraries(calibration_test PRIVATE tabletop_ui) # Exercise production fitting, persistence, and the capture display in memory.
  add_test(NAME calibration COMMAND calibration_test) # Check sensor-origin separation, stable poses, invalid capture rejection, and configuration roundtrip.
  add_test(NAME app_help COMMAND tabletop_delivery --help) # Verify that usage exits successfully without opening hardware or requiring a configuration file.
  add_test(NAME evaluation_cli COMMAND "${CMAKE_COMMAND}" "-DAPP=$<TARGET_FILE:tabletop_delivery>" -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/evaluation_cli_test.cmake")
  add_test(NAME app_demo COMMAND tabletop_delivery --demo --headless --seconds 1 --config "${CMAKE_CURRENT_SOURCE_DIR}/delivery_config.json" --output "${CMAKE_CURRENT_BINARY_DIR}/test_runs") # Exercise one second of bounded headless simulation with explicit configuration and output paths.
  add_test(NAME app_resources COMMAND "${CMAKE_COMMAND}" # Exercise local-resource startup independently from this source directory. Run the standalone-resource and missing-config checks through a separate CMake test script.
    "-DAPP=$<TARGET_FILE:tabletop_delivery>"
    "-DCONFIG=${CMAKE_CURRENT_SOURCE_DIR}/delivery_config.json"
    "-DRUN_ROOT=${CMAKE_CURRENT_BINARY_DIR}/resource_test"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/tests/resource_paths_test.cmake")
endif()
