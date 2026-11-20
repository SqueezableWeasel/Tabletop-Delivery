#!/usr/bin/env bash
set -u # Read-only inventory of packages, devices, and loaded-library dependencies. Missing optional diagnostic utilities do not prevent the remaining checks. Reject accidental unset variables while allowing optional diagnostic commands to fail individually.
cd -- "$(dirname -- "$0")/.." # Resolve executable and runtime paths from the repository containing this script.
printf 'Native build and hardware diagnostics\n' # Label each group of read-only diagnostic output for a human reviewing the report.
uname -a # Report kernel and machine architecture to identify the native build environment.
dpkg-query -W -f='${Package} ${Version}\n' 'libopencv*' 'libtensorflow-lite*' 'libcamera*' 'libedgetpu*' 2>/dev/null || true # Inventory installed OpenCV, TensorFlow Lite, libcamera, and delegate package versions; tolerate absent matches.
pkg-config --modversion opencv4 libcamera tensorflow-lite 2>/dev/null || true # Report development-package versions when their metadata is installed.
printf '\nCoral libraries in loader cache\n' # Label each group of read-only diagnostic output for a human reviewing the report.
ldconfig -p 2>/dev/null | grep -E 'edgetpu|tensorflow' || true # Read the dynamic loader cache and filter relevant runtime libraries without modifying the cache.
printf '\nUSB inventory\n' # Label each group of read-only diagnostic output for a human reviewing the report.
lsusb || true # Show connected USB devices to check whether an Accelerator or serial adapter is enumerated.
printf '\nSerial links\n' # Label each group of read-only diagnostic output for a human reviewing the report.
ls -l /dev/serial/by-id/ 2>/dev/null || true # Show stable serial-device links and their tty targets when the by-id directory exists.
printf '\nNative executable library dependencies\n' # Label each group of read-only diagnostic output for a human reviewing the report.
ldd build/tabletop_delivery 2>/dev/null || true # List the compiled application's resolved shared-library dependencies when the executable exists.
printf '\nProject-private Coral runtime and dependencies\n' # Label each group of read-only diagnostic output for a human reviewing the report.
if [[ -f native_runtime/libedgetpu.so.1 ]]; then # Inspect the private delegate only when it has already been built.
  readlink -f native_runtime/libedgetpu.so.1 # Resolve the private library symlink to its actual file.
  ldd native_runtime/libedgetpu.so.1 # Show which TensorFlow Lite and support libraries the private delegate will load.
else
  printf 'Missing private runtime. Run bash scripts/build_coral_runtime.sh\n' # Label each group of read-only diagnostic output for a human reviewing the report.
fi
