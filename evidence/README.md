# Custom detector evidence

These files accompany the included compiled figure model:

- [Compiler output](compiler_stdout.txt): operation placement and compilation result.
- [Quantization validation](quantization_validation.json): PyTorch and uncompiled INT8 CPU metrics.
- `validation/pytorch/` and `validation/int8_cpu/`: confusion matrices and precision/recall curves.
- [Export manifest](../models/figure_yolov8n_320_manifest.json): source hashes, tensor contract, and tool versions.

The experiment used 60 training images and 18 held-out images from the same
recording. Only training images calibrated INT8 quantization. The holdout is
scene-dependent and is not an independent generalization test.

Export-time evidence was generated without a Pi/Coral hardware test. The
subsequent [live demonstrations](../README.md#live-demonstrations) show custom
figure tracking, nominal/narrow-lane delivery, and the stop-sign hold on the
physical system. They do not supply a measured CPU/TPU speedup or a full latency
distribution.
