#!/usr/bin/env bash
set -euo pipefail # Build the SLAMTEC SDK, configure hardware dependencies, compile, and run CTest. RPLIDAR_SDK overrides the repository-local checkout; JOBS limits compiler memory. Stop on failed commands, unset variables, or failed pipeline stages so a partial build is not reported as success.
cd -- "$(dirname -- "$0")" # Resolve all project-relative paths from this script's own location, even when launched elsewhere.
sdk="${RPLIDAR_SDK:-$PWD/third_party/rplidar_sdk}" # Prefer an explicit SDK checkout; otherwise use the repository-local third_party/rplidar_sdk directory.
jobs="${JOBS:-2}" # Default to two compiler jobs to limit memory pressure on a Raspberry Pi 4.
if [[ ! -f "$sdk/sdk/include/sl_lidar_driver.h" ]]; then # Validate the SDK checkout root by checking its public driver header.
  printf 'Cannot find the SLAMTEC SDK at %s\nSet RPLIDAR_SDK to its checkout directory.\n' "$sdk" >&2 # Explain the missing checkout and the environment variable that selects an existing one.
  exit 1 # Return a nonzero status so callers stop instead of continuing with missing prerequisites.
fi
for dependency in cmake make g++ pkg-config; do # Check the compiler, build generators, and dependency-discovery tool before compiling anything.
  command -v "$dependency" >/dev/null || { printf 'Missing %s. Install the packages listed in README.md.\n' "$dependency" >&2; exit 1; } # Require the named program before starting downloads or builds that depend on it.
done
pkg-config --exists libcamera || { printf 'Missing libcamera development files. Run sudo apt install libcamera-dev\n' >&2; exit 1; } # Require installed libcamera development metadata and show its package name if missing.
make -C "$sdk" -j"$jobs" # Build the native SLAMTEC library within the selected SDK checkout using the requested job limit.
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DTABLETOP_HARDWARE=ON -DRPLIDAR_SDK="$sdk" # Configure an optimized live-hardware application against the explicit SDK location.
cmake --build build --parallel "$jobs" # Compile the configured application and tests with bounded parallelism.
ctest --test-dir build --output-on-failure # Run registered tests and show full diagnostics for any failures before reporting a ready executable.
printf '\nNative executable ready: ./build/tabletop_delivery\n' # Report the executable location only after compilation and CTest succeed.
if [[ ! -f native_runtime/libedgetpu.so.1 ]]; then # Check whether the separately built matching delegate is already present in this repository.
  printf 'Build the matching Coral library: bash scripts/build_coral_runtime.sh\n' # Provide the private runtime build command when its library is missing.
fi
printf 'Then: ./build/tabletop_delivery --check-coral\n' # Suggest the real USB warm-up check before attempting full live acquisition.
