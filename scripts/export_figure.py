#!/usr/bin/env python3
"""Convert the reviewed figure checkpoint to INT8 and compile for Coral on Linux x86-64.

The C++ application performs all live inference, decoding, NMS, and fusion.
This offline tool verifies the checkpoint/dataset, calibrates using TRAIN only,
compares PyTorch and uncompiled INT8 on VAL, and records the real compiler report.
It does not train the model or claim a hardware benchmark.
"""
from pathlib import Path, PurePosixPath
import argparse
import hashlib
import json
import os
import platform
import shutil
import subprocess
import zipfile

from export_yolo import ensure_compiler, COMPILER_SHA256

NAMES = {0: "figure", 1: "stop_sign"}
STEM = "figure_yolov8n_320"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def extract(archive, destination):
    """Validate every ZIP member before extracting into a new, dedicated directory."""
    destination = destination.resolve()
    if destination.exists():
        raise ValueError(f"Extraction directory already exists: {destination}; use a fresh output folder")
    with zipfile.ZipFile(archive) as source:
        members = source.infolist()
        if sum(item.file_size for item in members) > 2_000_000_000:
            raise ValueError("ZIP exceeds the 2 GB extraction limit")
        seen = set()
        for item in members:
            name = PurePosixPath(item.filename)
            if name.is_absolute() or ".." in name.parts or "\\" in item.filename or ":" in item.filename:
                raise ValueError("Unsafe ZIP member path")
            if (item.external_attr >> 16) & 0o170000 == 0o120000:
                raise ValueError("ZIP symlinks are not supported")
            if name in seen or not (destination / str(name)).resolve().is_relative_to(destination):
                raise ValueError("Duplicate or escaping ZIP member")
            seen.add(name)
        source.extractall(destination)


def inspect_model(path, compiled=False):
    """Inspect the FlatBuffer directly: the Colab machine has no USB TPU attached."""
    from tensorflow.lite.python import schema_py_generated as schema
    data = path.read_bytes()
    if data[4:8] != b"TFL3":
        raise ValueError("Not a TFLite FlatBuffer")
    model = schema.Model.GetRootAsModel(data, 0)
    graph = model.Subgraphs(0)
    if graph.InputsLength() != 1 or graph.OutputsLength() != 1:
        raise ValueError("Expected one input/output; export a detection model without embedded NMS")
    tensors = [graph.Tensors(graph.Inputs(0)), graph.Tensors(graph.Outputs(0))]
    shapes = [[t.Shape(i) for i in range(t.ShapeLength())] for t in tensors]
    if shapes[0] != [1, 320, 320, 3] or shapes[1] not in ([1, 6, 2100], [1, 2100, 6]):
        raise ValueError(f"Unexpected custom model shapes: {shapes}")
    descriptions = []
    for tensor, shape in zip(tensors, shapes):
        if tensor.Type() not in (schema.TensorType.INT8, schema.TensorType.UINT8):
            raise ValueError("Use full_integer_quant.tflite, with integer input AND output")
        q = tensor.Quantization()
        if q is None or q.ScaleLength() != 1 or q.ZeroPointLength() != 1 or not q.Scale(0) > 0:
            raise ValueError("Expected per-tensor input/output quantization")
        descriptions.append({"shape": shape, "type": "int8" if tensor.Type() == schema.TensorType.INT8 else "uint8",
                             "scale": float(q.Scale(0)), "zero_point": int(q.ZeroPoint(0))})
    has_tpu = any(model.OperatorCodes(i).CustomCode() == b"edgetpu-custom-op" for i in range(model.OperatorCodesLength()))
    if has_tpu != compiled:
        raise ValueError("Unexpected presence/absence of the compiled Edge TPU operation")
    return {"input": descriptions[0], "output": descriptions[1], "edge_tpu_compiled": has_tpu}


def prepare_inputs(training_zip, dataset_zip, output):
    """Tie the supplied dataset to the model's own manifest; never calibrate on VAL."""
    import yaml
    from dataset_tools import validate_dataset
    extract(training_zip, output / "training_result")
    extract(dataset_zip, output / "dataset")
    training = output / "training_result"
    contract = json.loads((training / "model_contract.json").read_text())
    if {int(k): v for k, v in contract["classes"].items()} != NAMES or contract["training_imgsz"] != 320:
        raise ValueError("Checkpoint contract must be figure=0, stop_sign=1, trained at 320")
    weights = training / "weights/best.pt"
    if sha(weights) != contract["best_pt_sha256"]:
        raise ValueError("Checkpoint SHA-256 does not match its training contract")
    roots = [p.parent for p in (output / "dataset").rglob("data.yaml") if (p.parent / "manifest.json").is_file()]
    if len(roots) != 1:
        raise ValueError("Expected one prepared dataset with manifest.json")
    dataset = roots[0]
    summary = validate_dataset(dataset)
    manifest = json.loads((dataset / "manifest.json").read_text())
    trained_manifest = json.loads((training / "dataset_manifest.json").read_text())
    if manifest != trained_manifest:
        raise ValueError("The supplied dataset is not the dataset recorded by this training run")
    for record in manifest["images"]:
        path = (dataset / record["image"]).resolve()
        if not path.is_relative_to(dataset.resolve()) or sha(path) != record["sha256"]:
            raise ValueError("Dataset image differs from the training manifest")
    # Ultralytics 8.3.228 defaults to val when collecting INT8 examples. This
    # separate YAML intentionally maps BOTH entries to TRAIN, so VAL never leaks.
    calibration = {"path": str(dataset), "train": "images/train", "val": "images/train", "names": NAMES}
    (output / "calibration_train_only.yaml").write_text(yaml.safe_dump(calibration, sort_keys=False))
    validation = {**calibration, "val": "images/val"}
    (output / "validation.yaml").write_text(yaml.safe_dump(validation, sort_keys=False))
    shutil.copyfile(weights, output / (STEM + ".pt"))
    print(f"Verified model {sha(weights)}; calibrating on {summary['train']['images']} TRAIN images only.", flush=True)
    return {"weights_sha256": sha(weights), "dataset_zip_sha256": sha(dataset_zip),
            "training_zip_sha256": sha(training_zip), "calibration_images": summary["train"]["images"],
            "validation_images": summary["val"]["images"], "evaluation_note": manifest.get("evaluation_note", "")}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--training-zip", type=Path, required=True)
    parser.add_argument("--dataset-zip", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--compile-only", action="store_true", help="Resume a verified completed INT8 conversion after compiler failure")
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        parser.error("Use x86-64 Linux/Colab for conversion; run the compiled result on the Pi.")
    os.environ["YOLO_AUTOINSTALL"] = "false"
    from ultralytics import YOLO, __version__
    if __version__ != "8.3.228":
        raise ValueError("Use the included pinned export requirements")
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    record = output / "conversion_record.json"
    source = output / (STEM + "_saved_model") / (STEM + "_full_integer_quant.tflite")
    if args.compile_only:
        provenance = json.loads(record.read_text())
        if sha(args.training_zip) != provenance["training_zip_sha256"] or sha(args.dataset_zip) != provenance["dataset_zip_sha256"] or sha(source) != provenance["int8_sha256"]:
            raise ValueError("Resume inputs differ from the completed INT8 conversion")
    else:
        provenance = prepare_inputs(args.training_zip, args.dataset_zip, output)
        model = YOLO(str(output / (STEM + ".pt")))
        if model.task != "detect" or model.names != NAMES:
            raise ValueError(f"Unexpected trained classes/task: {model.names}, {model.task}")
        # TFLite export accepts int8=True; the edgetpu export entry point does not
        # accept it in 8.3.228. Select the FULL integer artifact explicitly below.
        model.export(format="tflite", int8=True, imgsz=320, batch=1, nms=False,
                     data=str(output / "calibration_train_only.yaml"), device="cpu")
        inspect_model(source)
        provenance["int8_sha256"] = sha(source)
        record.write_text(json.dumps(provenance, indent=2))
    compiler, compiler_version = ensure_compiler(output)
    result = subprocess.run([str(compiler), "--out_dir", str(source.parent), "--show_operations",
                             "--search_delegate", "--delegate_search_step", "30", "--timeout_sec", "180",
                             "--min_runtime_version", "14", str(source)], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    print(result.stdout, flush=True)
    (output / "compiler_stdout.txt").write_text(result.stdout)
    result.check_returncode()
    compiled = source.with_name(source.stem + "_edgetpu.tflite")
    tensors = inspect_model(compiled, compiled=True)
    # Evaluate uncompiled full-INT8 on CPU and PyTorch at identical image/batch
    # settings. The compiled model still needs a real --check-coral/live Pi run.
    report = {}
    for label, path in (("pytorch", output / (STEM + ".pt")), ("int8_cpu", source)):
        metrics = YOLO(str(path), task="detect").val(data=str(output / "validation.yaml"), split="val",
                    imgsz=320, rect=False, batch=1, device="cpu", workers=0, plots=True,
                    project=str(output / "validation"), name=label)
        report[label] = {k: float(v) for k, v in metrics.results_dict.items()}
    (output / "quantization_validation.json").write_text(json.dumps(report, indent=2))
    model_dir = output / "pi_models"
    model_dir.mkdir(exist_ok=True)
    destination = model_dir / (STEM + "_edgetpu.tflite")
    shutil.copyfile(compiled, destination)
    manifest = {**provenance, **tensors, "model": destination.name, "sha256": sha(destination),
                "class_count": 2, "classes": NAMES, "target_class": 0, "stop_sign_class": 1,
                "box_coordinates": "normalized", "nms_in_graph": False, "ultralytics": __version__,
                "compiler_version": compiler_version, "compiler_archive_sha256": COMPILER_SHA256,
                "min_runtime_version": 14, "pi_hardware_tested": False, "validation": report}
    (model_dir / (STEM + "_manifest.json")).write_text(json.dumps(manifest, indent=2))
    (model_dir / (STEM + "_edgetpu.sha256")).write_text(f"{sha(destination)}  {destination.name}\n")
    (model_dir / "figure_labels.txt").write_text("0 figure\n1 stop_sign\n")
    bundle = output / "figure_coral_model.zip"
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED) as archive:
        for path in sorted(model_dir.iterdir()):
            archive.write(path, "models/" + path.name)
        archive.write(output / "compiler_stdout.txt", "evidence/compiler_stdout.txt")
        archive.write(output / "quantization_validation.json", "evidence/quantization_validation.json")
        for path in source.parent.glob("*edgetpu.log"):
            archive.write(path, "evidence/" + path.name)
        for path in (output / "validation").rglob("*.png"):
            archive.write(path, "evidence/validation/" + path.relative_to(output / "validation").as_posix())
    with zipfile.ZipFile(bundle) as archive:
        assert archive.testzip() is None
    print("READY:", bundle, flush=True)
    print("Next: compare PyTorch/INT8 metrics, transfer models/, then run --yolo-profile figure --check-coral on the Pi.")


if __name__ == "__main__":
    main()
