# Detector validation scope

See [Testing](TESTING.md) for the merged package's automated checks,
[the custom model guide](FIGURE_MODEL.md) for accuracy/conversion results,
and [the live recordings](../README.md#live-demonstrations) for the physical demo.

The custom detector's 60/18 split is an exploratory holdout from one recording.
The conversion evidence and manifest are retained unchanged under `evidence/`
and `models/`. The Pi recordings were captured after export and show the model
without the blue calibration square, with corrected single-box overlays.

The stock COCO [preflight study](YOLO_PREFLIGHT.md) is separate evidence about
the stock person detector and does not measure the custom figure model.
