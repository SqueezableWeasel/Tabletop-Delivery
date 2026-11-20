# Detector model

From the repository root, run:

```bash
bash scripts/fetch_model.sh
```

The helper downloads `ssd_mobilenet_v2_coco_quant_postprocess_edgetpu.tflite` from [Google Coral test_data](https://github.com/google-coral/test_data) and verifies:

```text
b94e2d58222c32f31062c7604e10488e2aba9259ab77462039476a3ba4597fef
```

Input is uint8 RGB `[1,300,300,3]`. Outputs are float boxes, classes, scores, and count. The file must contain the Edge TPU custom operation, and the native USB warm-up must succeed before readiness is reported.

The model is a generated download excluded from Git. Its upstream licensing and notices apply. Pass `--model PATH` only for a model satisfying the same detector contract; this application does not provide a generic model adapter.
