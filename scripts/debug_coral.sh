#!/usr/bin/env bash
set -euo pipefail # Build debug symbols and collect a bounded thread backtrace for Coral startup. Stop on failed commands, unset variables, or failed pipeline stages so a partial build is not reported as success.
cd -- "$(dirname -- "$0")/.." # Resolve all project-relative paths from this script's own location, even when launched elsewhere.
if ! command -v gdb >/dev/null; then # Require the debugger before changing build configuration or starting a diagnostic run.
  printf 'gdb is missing. Run sudo apt install -y gdb, then rerun this script.\n' >&2 # Show the package installation command needed for this optional diagnostic helper.
  exit 1 # Return a nonzero status so callers stop instead of continuing with missing prerequisites.
fi
if [[ ! -f build/CMakeCache.txt ]]; then # Require a configured build so debugging retains the selected hardware dependency paths.
  printf 'Build configuration missing. Run bash build_pi.sh first.\n' >&2 # Point to the normal build entry point when no reusable CMake configuration exists.
  exit 1 # Return a nonzero status so callers stop instead of continuing with missing prerequisites.
fi
cmake -S . -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo # Retain optimization while adding debug information for meaningful native stack traces.
cmake --build build --target tabletop_delivery --parallel "${JOBS:-2}" # Rebuild only the application target with the chosen job limit.
report="coral_debug_$(date -u +%Y%m%dT%H%M%SZ).txt" # Give the report a sortable UTC timestamp so the diagnostic can be shared as one text file.
{
  printf 'Native Coral crash diagnostic (does not open camera or LiDAR)\n' # Label this diagnostic as accelerator-only; camera and LiDAR are not started.
  bash scripts/diagnose.sh # Include package/device/library inventory before collecting the debugger trace.
  printf '\nDebugger trace\n' # Separate environment inventory from the native stack-trace portion of the report.
  gdb -q -batch \
    -ex 'set pagination off' \
    -ex 'set print thread-events off' \
    -ex 'set debuginfod enabled off' \
    -ex run \
    -ex 'info sharedlibrary' \
    -ex 'thread apply all bt 20' \
    --args ./build/tabletop_delivery --check-coral "$@" # Run without interactive prompts, disable pagination/downloads, then report libraries and up to 20 frames per thread.
} 2>&1 | tee "$report" # Capture both standard streams in one report while also showing the diagnostic output in the terminal.
printf '\nSaved diagnostic report: %s/%s\n' "$PWD" "$report" # Print the report's resolved path after collection completes.
