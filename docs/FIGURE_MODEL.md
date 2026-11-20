# Custom figure model on Coral

The custom profile supports **figure = 0** and **stop_sign = 1**. Select it with
`--yolo-profile figure`. The stock YOLO profile remains COCO-80, with person = 0
and stop sign = 11. SSD remains selectable. The program checks the selected
profile's tensor shape instead of guessing labels from the file name.

The live pipeline is C++: camera acquisition, RGB letterboxing, quantization,
TensorFlow Lite/USB delegation, tensor decoding, class-aware NMS, LiDAR fusion,
motion decisions, display, and logging. Python is used only for offline training
and conversion. A `best.pt` checkpoint cannot be opened by the C++ runtime.

The compiled model, labels, checksum, export manifest, and validation evidence
are included in this repository. Use the root README to build and run it.
The conversion workflow below is for reproducing or replacing that model.

## Convert the trained checkpoint

Open [export_figure_colab.ipynb](../scripts/export_figure_colab.ipynb) in Colab
with Linux x86-64 and Python 3.12. CPU is sufficient. Run its cells in order.
It installs the conversion dependencies in a separate virtual environment and
invokes them in fresh processes; it does not replace the running notebook's
NumPy modules. The setup uses pip's `--python` option because Colab can lack
the `ensurepip` module needed by ordinary `venv` bootstrapping.

Supply both `figure_training_result.zip` and the matching `tabletop_dataset.zip`.
The notebook can reuse these files when they already exist in `/content`.
The export script verifies the checkpoint SHA-256, class mapping, dataset
manifest, split assignments, image hashes, and annotations before conversion.

The conversion steps are:

1. Load the saved **custom best.pt**, preserving its learned weights.
2. Export 320x320, batch-one TFLite with INT8 quantization and no embedded NMS.
3. Select `*_full_integer_quant.tflite`, with integer input and output.
4. Compile with the pinned compiler 16 bundle and minimum runtime version 14.
5. Inspect `[1,320,320,3]` input and `[1,6,2100]` or `[1,2100,6]` output,
   quantization parameters, and presence of the Edge TPU operation.
6. Validate PyTorch and uncompiled INT8 on the same held-out images, then
   package `figure_coral_model.zip` with weights, checksums, labels, and evidence.

Quantization uses **only training images**. The pinned exporter normally uses
the dataset's `val` entry for calibration, so a dedicated calibration YAML maps
both `train` and `val` to `images/train`. A separate validation YAML keeps the
actual held-out split. The 60/18 experiment supplies only 60 calibration images,
fewer than Ultralytics recommends; review the measured INT8 accuracy rather than
adding validation images to the calibration pool.

Compare the generated `quantization_validation.json` metrics and plots. A major
drop in recall or mAP needs investigation before deployment. The compiler report
records operations assigned to CPU/TPU; compilation alone does not guarantee that
every operation is accelerated. Colab CPU/GPU timings are not Pi benchmarks.

The downloaded model ZIP contains:

```text
models/
  figure_yolov8n_320_edgetpu.tflite
  figure_yolov8n_320_edgetpu.sha256
  figure_yolov8n_320_manifest.json
  figure_labels.txt
evidence/
  compiler_stdout.txt
  quantization_validation.json
  ...compiler log and validation plots...
```

Download the ZIP from the final cell or Colab's Files panel. The compiled model
needs a real invocation and live evaluation on the Pi before performance can be
claimed. This dataset's held-out images come from the same recording; they are
not an independent test of new lighting, distances, or marker-free operation.

## Model validation and installation

### Verified export from the 60/18 experiment

Conversion and compilation completed in Colab on November 20, 2026, using
Ultralytics 8.3.228, TensorFlow 2.19.0, NumPy 2.1.3, and compiler 16.0.384591198.
The compiled model is 3.35 MiB, with **233 operations on Edge TPU and 22 on CPU**.
It has INT8 input `[1,320,320,3]` (scale 0.003921568859368563, zero point -128)
and INT8 output `[1,6,2100]` (scale 0.006824628449976444, zero point -117).
The C++ implementation reads these quantization parameters from the tensors.

| Validation model | Precision | Recall | mAP50 | mAP50-95 |
| --- | ---: | ---: | ---: | ---: |
| PyTorch checkpoint | 98.40% | 100% | 99.5% | 86.71% |
| Full INT8, CPU interpreter | 98.59% | 100% | 99.5% | 81.65% |

Both runs used the same 18 held-out images at 320x320 and batch one. The stricter
localization metric decreased, mostly for stop-sign boxes. Review the included
plots and test the actual Pi before adopting the model for the live demo.
These results are not measurements of the compiled model on Coral hardware.

Checkpoint SHA-256:
`c0edd4275ef6d42deb5aaf8a792de4fe15939edcf7f9cd08693d931997a0f39b`.
Compiled model SHA-256:
`b01bb93d0dc884450b42fa54117fec06880a6204d0c59f1aa41b834900931086`.

The updated C++ project passed all 11 desktop CTest cases, including custom
six-channel quantized/transposed decoding, profile mismatch rejection, and
figure/stop-sign class separation. Hardware-facing source syntax was checked
against ARM64 headers during detector integration. Subsequent supplied recordings show the custom model running on the Pi with
Coral, including nominal/narrow-lane deliveries and stop-sign holds. See the
[live demonstrations](../README.md#live-demonstrations). These qualitative
recordings are separate from the export-time validation metrics.

### Install and build

The compiled model is already under `models/`. From the repository root, after
installing the dependencies and SDK described in the [README](../README.md):

```bash
bash scripts/build_coral_runtime.sh
bash build_pi.sh
(cd models && sha256sum -c figure_yolov8n_320_edgetpu.sha256)
./build/tabletop_delivery --yolo-profile figure --check-coral
```

The check must end with `PASS: USB delegate attached and model invoked in C++.`
Rebuild the private runtime and app after changing the system TensorFlow Lite
version. Calibration is saved and activated by `./build/tabletop_delivery --calibrate`.
When updating an existing installation, retain its measured calibration and
boundary sidecar; the package does not include machine-specific calibration.

## Compare first, then use detector control

On the Pi desktop/VNC, retain the blue marker for the initial comparison:

```bash
./build/tabletop_delivery --yolo-profile figure --tracking compare
```

Marker fusion controls the robot while the independent custom detector is
displayed alongside it. Confirm boxes stay on the figure near/far and across
the lane, and that the matched LiDAR cluster agrees with the physical position.
The default figure confidence is 0.25; override with `--target-confidence 0.30`
when evaluating a different threshold. Existing `--person-confidence` and size
gate options remain compatible. The size gates reject boxes wider than 30% or
taller than 65% of the camera image by default, including an extremely close prop.

After that comparison:

```bash
./build/tabletop_delivery --yolo-profile figure --tracking yolo
```

This mode uses the figure detector, not a marker fallback. Its boxes supply the
camera bearing; the corresponding time-paired LiDAR scan supplies metric range.
The camera heading reads `CAMERA + YOLO FIGURE | EVALUATION`. Each detection
has one purple box labeled `figure` or `stop`, followed by its confidence.
Green `BLUE MARKER` boxes appear in marker-controlled comparison mode only,
where they represent a separate color observation. The tracker reuses the
detector's box internally; that is not another object or another detector.
Missing, stale, multiple, or unmatched detections hold motion. Calibration still
uses the blue marker and the established physical placement procedure. Removing
the blue square is an evaluation change if the training images contained it;
test carefully and collect marker-free examples when needed.

The sign response uses class 1 at the existing 0.60 confidence threshold and
existing confirmation/release timing. A detected sign can hold the robot even
when figure tracking is healthy. A sign visible on the background ledge also
counts: remove it during ordinary movement trials and introduce it deliberately
for the stop-sign demonstration. The figure boundary and route-planning logic
are unchanged.

Recorded CSV column names containing `person` retain their format for existing
analysis tools; under this profile they mean the custom figure. The profile,
class count, and stop-sign ID are recorded in `evaluation_settings.txt`.

## References

- [Pinned Ultralytics exporter and calibration split behavior](https://github.com/ultralytics/ultralytics/blob/v8.3.228/ultralytics/engine/exporter.py)
- [Coral compiler requirements and supported architecture](https://coral.ai/docs/edgetpu/compiler/)
- [Managing a separate Python environment with pip](https://pip.pypa.io/en/stable/topics/python-option/)
