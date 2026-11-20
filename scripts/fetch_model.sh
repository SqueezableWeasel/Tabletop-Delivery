#!/usr/bin/env bash
set -euo pipefail # Fetch the supported compiled detector into models/ and verify its SHA-256. A partial download is accepted only after its checksum matches the expected model. Stop on failed commands, unset variables, or failed pipeline stages so a partial build is not reported as success.
cd -- "$(dirname -- "$0")/.." # Resolve all project-relative paths from this script's own location, even when launched elsewhere.
mkdir -p models # Create the repository's model directory if needed.
name=ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite # Select the supported 300x300 SSD MobileNet V2 COCO graph compiled for the Edge TPU.
expected=b94e2d58222c32f31062c7604e10488e2aba9259ab77462039476a3ba4597fef # Pin the supported detector bytes by SHA-256 rather than trusting a filename alone.
target="models/$name" # Resolve the model destination relative to the repository root.
if [[ ! -f "$target" ]]; then # Download only when absent; existing files still undergo checksum verification below.
  curl --fail --location --retry 3 --connect-timeout 20 --max-time 180 \
    "https://raw.githubusercontent.com/google-coral/test_data/master/$name" -o "$target.part" # Fetch from the official model source with HTTP failure detection, retries, and bounded timeouts.
  printf '%s  %s\n' "$expected" "$target.part" | sha256sum --check --status || { # Verify the partial download before accepting it as the operational detector.
    printf 'Model checksum mismatch; original model left unchanged.\n' >&2; exit 1; # Report integrity failure and stop without promoting unverified bytes to the final path.
  }
  mv -- "$target.part" "$target" # Promote the verified temporary model into its stable application path.
fi
printf '%s  %s\n' "$expected" "$target" | sha256sum --check # Verify both new and preexisting detector files against the expected SHA-256 digest.
printf 'SSD model ready. Network inference runs on Coral; SSD postprocessing runs on the CPU.\n' # State the execution split accurately: neural network on TPU, SSD postprocessing on CPU.
