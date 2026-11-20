#!/usr/bin/env bash
set -euo pipefail # Fetch SHA-verified upstream source and build the project-local USB runtime. Link against installed TensorFlow Lite and preserve the upstream license notice. Stop on failed commands, unset variables, or failed pipeline stages so a partial build is not reported as success.
cd -- "$(dirname -- "$0")/.." # Resolve all project-relative paths from this script's own location, even when launched elsewhere.
project_root="$PWD" # Capture the resolved repository root for paths used throughout this build.
runtime_dir="$project_root/native_runtime" # Keep downloaded source, build products, and installed delegate isolated inside this repository.
commit=e35aed18fea2e2d25d98352e5a5bd357c170bd4d # Pin the exact upstream native runtime source revision for reproducible builds.
expected=4e0a49c767104438e5eebd638fe814c9c5776f5173a27684ed4ecb401173872d # Require this SHA-256 digest for the pinned upstream source archive before extraction.
archive="$runtime_dir/libedgetpu-$commit.tar.gz" # Name the cached source archive using its exact pinned revision.
source_dir="$runtime_dir/libedgetpu-$commit" # Predict the source directory produced by the official revision archive.
mkdir -p "$runtime_dir" # Ensure the local runtime directory exists without changing system library locations.
for program in cmake curl sha256sum tar flatc pkg-config; do # Verify download, checksum, archive, schema compiler, and CMake prerequisites first.
  command -v "$program" >/dev/null || { printf 'Missing %s. Install dependencies listed in README.md.\n' "$program" >&2; exit 1; } # Require the named program before starting downloads or builds that depend on it.
done
if [[ ! -f "$archive" ]]; then # Reuse a cached archive only when it exists; it is still verified below on every run.
  curl --fail --location --retry 3 --connect-timeout 20 --max-time 180 \
    "https://github.com/google-coral/libedgetpu/archive/$commit.tar.gz" -o "$archive.part" # Follow redirects, fail on HTTP errors, retry transient failures, and bound connection/total download time.
  printf '%s  %s\n' "$expected" "$archive.part" | sha256sum --check --status # Verify the temporary download before promoting it to the reusable archive path.
  mv -- "$archive.part" "$archive" # Rename only the successfully verified partial archive into its final cached name.
fi
printf '%s  %s\n' "$expected" "$archive" | sha256sum --check # Recheck cached source integrity before extraction or compilation.
if [[ ! -f "$source_dir/README.md" ]]; then tar -xzf "$archive" -C "$runtime_dir"; fi # Extract the verified revision only when its source tree is not already present.
cmake -S native/coral_runtime -B "$runtime_dir/build" -DCMAKE_BUILD_TYPE=Release -DEDGETPU_SOURCE="$source_dir" # Configure the private C++ runtime against this Pi's installed TensorFlow Lite and dependency headers.
cmake --build "$runtime_dir/build" --parallel "${JOBS:-2}" # Compile the matching delegate with a default two-job limit to reduce Pi memory pressure.
cmake --install "$runtime_dir/build" --prefix "$runtime_dir" # Install the shared library into native_runtime, leaving system-wide delegate libraries untouched.
cp -- "$source_dir/LICENSE" "$runtime_dir/LICENSE.libedgetpu" # Preserve the upstream license alongside the locally installed runtime.
printf '\nPrivate Coral runtime built against installed TensorFlow Lite:\n%s/libedgetpu.so.1\n' "$runtime_dir" # Report the exact private shared-library location after a successful build and install.
printf 'Runtime installed within this repository.\n' # Clarify that the generated library belongs to this repository's runtime directory.
