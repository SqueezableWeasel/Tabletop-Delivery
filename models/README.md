# Detector models

## Included custom figure model

`figure_yolov8n_320_edgetpu.tflite` is the fine-tuned YOLOv8n model used by the
demonstrations. It has two classes: `0 = figure` and `1 = stop_sign`. The labels,
checksum, and export manifest are included alongside it. These four files are
deliberately tracked in Git; other generated weights remain ignored.

```bash
(cd models && sha256sum --check figure_yolov8n_320_edgetpu.sha256)
./build/tabletop_delivery --yolo-profile figure --check-coral
./build/tabletop_delivery --yolo-profile figure --tracking yolo
```

SHA-256: `b01bb93d0dc884450b42fa54117fec06880a6204d0c59f1aa41b834900931086`.

The input is INT8 RGB `[1,320,320,3]`; the output is INT8 `[1,6,2100]`,
containing four box channels and two class scores. The C++ code reads tensor
quantization parameters, reverses letterboxing, and performs class-aware NMS.
The compiled network has 233 operations mapped to the TPU and 22 on the CPU.

See [the model guide](../docs/FIGURE_MODEL.md) for training/export details and
limitations. [Compiler and validation evidence](../evidence/README.md) records
the conversion. The manifest's `pi_hardware_tested: false` describes the export
environment, which had no Coral attached; subsequent live recordings are linked
in the root README. The original export record is retained unchanged.

## Optional SSD model

The no-flag CLI uses marker tracking with SSD MobileNet V2 COCO. Download that
separate model if you want this mode:

```bash
bash scripts/fetch_model.sh
./build/tabletop_delivery --check-coral
./build/tabletop_delivery
```

The helper downloads `ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite`
from [Google Coral test_data](https://github.com/google-coral/test_data) and
verifies SHA-256 `b94e2d58222c32f31062c7604e10488e2aba9259ab77462039476a3ba4597fef`.
It uses uint8 RGB `[1,300,300,3]` input and float boxes/classes/scores/count
outputs. SSD's stop-sign class is 12.

## Optional stock YOLO model

`--yolo-profile coco` uses the separate COCO-80 contract: person 0, stop sign 11,
and 84 output channels. See [the comparison guide](../docs/YOLO_EVALUATION.md).
The stock model is not included and is not trained specifically for the toy.

Every live model must contain the Edge TPU operation and pass a real USB warm-up.
`--model FILE` changes the path, not the selected class/tensor contract. Upstream
model/tool notices are listed in [THIRD_PARTY.md](../THIRD_PARTY.md).
