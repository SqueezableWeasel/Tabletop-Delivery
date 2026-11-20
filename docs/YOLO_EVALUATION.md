# Detector modes and comparison

The full C++ application supports a custom two-class figure model, stock COCO
YOLOv8n, and SSD MobileNet V2. The [custom model guide](FIGURE_MODEL.md) describes
the included model used in the README recordings. Calibration and marker
tracking remain available alongside detector control.

## Compare visual observations

Attach the blue/cyan square and run:

```bash
./build/tabletop_delivery --yolo-profile figure --tracking compare
```

The marker controls the robot while an independent YOLO track consumes the same
detector source image and time-paired LiDAR scan. Green boxes identify the
marker; purple boxes identify detector observations. Position disagreement is
shown only while both tracks are valid. Waiting for inference frames makes this
an observation-quality comparison rather than a full-rate marker benchmark.

```bash
./build/tabletop_delivery --yolo-profile figure --tracking yolo
```

Detector control uses the figure box's horizontal center as a calibrated bearing
and associates it with LiDAR range. The marker tracker is diagnostic only;
there is no marker fallback. Each detector box is drawn once on its own source
frame. Missing, stale, multiple, or unmatched observations hold virtual motion.

## Thresholds and profiles

| Setting | Meaning |
| --- | --- |
| `--yolo-profile figure` | Two classes: figure 0 and stop_sign 1; six output channels. |
| `--yolo-profile coco` | 80 classes: person 0 and stop sign 11; 84 output channels. |
| `--target-confidence` | Target score threshold; 0.25 for figure, 0.35 for COCO by default. |
| `--person-confidence` | Alias for the target threshold. |
| `--person-max-width 0.30` | Maximum target width as a fraction of the original camera image. |
| `--person-max-height 0.65` | Maximum target height as a fraction of the original camera image. |
| `--yolo-coordinates normalized` | Exported coordinate convention; `pixels` is also supported explicitly. |

Internal function names and CSV columns containing `person` refer to class zero
of the selected profile. They do not indicate a second person detector when the
custom figure model is selected. A successful camera detection still needs a
valid LiDAR association.

## Export the optional COCO model

The stock model is not included. The [stock export notebook](../scripts/export_yolo_colab.ipynb)
targets Linux x86-64 with Python 3.12 and the pinned conversion dependencies.
Run its installation, script-writing, export, and download cells in order.
The custom model uses the separate [custom export notebook](../scripts/export_figure_colab.ipynb).

Outside Colab, in a compatible Linux environment:

```bash
python3.12 -m venv .venv-yolo
source .venv-yolo/bin/activate
python -m pip install -r scripts/requirements-export.txt
python scripts/export_yolo.py --output models
```

Copy the resulting `yolov8n_320_edgetpu.tflite`, checksum, and manifest to the
Pi's `models/` directory, then run:

```bash
(cd models && sha256sum --check yolov8n_320_edgetpu.sha256)
./build/tabletop_delivery --yolo-profile coco --check-coral
./build/tabletop_delivery --yolo-profile coco --tracking compare
```

The stock model was unreliable on this miniature figure; see the
[stock-model recording study](YOLO_PREFLIGHT.md). Custom training addresses this
specific prop rather than establishing human-detection performance.

## Diagnostics

Detector modes write `evaluation.csv`, `evaluation_summary.json`, and
`evaluation_settings.txt` in the run directory. They distinguish inference time,
pre/postprocessing time, total worker time, source age, track validity, and
marker/YOLO disagreement. Repeated UI refreshes do not count as new inferences.

Temporal pairing uses host receive timestamps. It is not hardware synchronization.
The quantized model's decoder, profile checks, timing/association behavior, CLI,
and renderer are covered by the [tests](TESTING.md).
