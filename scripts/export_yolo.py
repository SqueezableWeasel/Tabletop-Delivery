#!/usr/bin/env python3
"""Export the pinned stock YOLOv8n COCO detector on x86_64 Linux, not on the Pi.

Python is only a model-preparation dependency. The live application, inference,
letterboxing, decoding, NMS, tracking, planning, display, and logs are C++.
"""
from pathlib import Path
import argparse
import hashlib
import io
import json
import os
import platform
import shutil
import subprocess
import tarfile
import urllib.request

WEIGHTS_URL = "https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt"
WEIGHTS_SHA256 = "f59b3d833e2ff32e194b5bb8e08d211dc7c5bdf144b90d2c8412c47ccfc83b36"
COMPILER_URL = "https://github.com/ultralytics/assets/releases/download/v0.0.0/edgetpu-compiler_16.0_amd64.tar.gz"
COMPILER_SHA256 = "f9a97adbf2ff621d24a91721191264f2abc3c1db31e63d9a266d988f907269ce"


def ensure_compiler(output):
    """Install a verified private compiler; the Coral apt repository is unavailable.

    Ultralytics distributes the original compiler 16 package files. Keep its
    bundled loader/libraries together, verify the archive before extraction,
    and prepend a quoted launcher to PATH for the pinned Ultralytics exporter.
    No sudo, apt source changes, GPG prompts, or system installation are needed.
    """
    root = (output / ".compiler16").resolve()
    root.mkdir(parents=True, exist_ok=True)
    archive_path = root / "compiler.tar.gz"
    if not archive_path.is_file():
        temporary = archive_path.with_suffix(".gz.part")
        urllib.request.urlretrieve(COMPILER_URL, temporary)
        if hashlib.sha256(temporary.read_bytes()).hexdigest() != COMPILER_SHA256:
            raise RuntimeError("Downloaded compiler failed SHA-256 validation")
        temporary.replace(archive_path)
    data = archive_path.read_bytes()
    if hashlib.sha256(data).hexdigest() != COMPILER_SHA256:
        raise RuntimeError("Cached compiler archive failed SHA-256 validation")
    with tarfile.open(fileobj=io.BytesIO(data), mode="r:gz") as archive:
        for member in archive.getmembers():
            relative = Path(member.name)
            if relative.is_absolute() or ".." in relative.parts:
                raise RuntimeError("Unsafe compiler archive path")
            target = root / relative
            if not target.resolve().is_relative_to(root):
                raise RuntimeError("Compiler extraction would leave its directory")
            if member.isdir():
                target.mkdir(parents=True, exist_ok=True)
            elif member.isfile():
                target.parent.mkdir(parents=True, exist_ok=True)
                target.write_bytes(archive.extractfile(member).read())
                target.chmod(0o755 if "bin" in relative.parts else 0o644)
            else:
                raise RuntimeError("Unexpected compiler archive member")
    launcher = root / "edgetpu_compiler"
    launcher.write_text(
        '#!/bin/sh\n'
        'bundle=$(CDPATH= cd -- "$(dirname -- "$0")/usr/bin/edgetpu_compiler_bin" && pwd)\n'
        'exec "$bundle/ld-linux-x86-64.so.2" --library-path "$bundle" "$bundle/edgetpu_compiler" "$@"\n',
        encoding="utf-8",
    )
    launcher.chmod(0o755)
    version = subprocess.check_output([str(launcher), "--version"], text=True).strip()
    if "16.0.384591198" not in version:
        raise RuntimeError(f"Unexpected compiler version: {version}")
    os.environ["PATH"] = str(root) + os.pathsep + os.environ.get("PATH", "")
    print(version, flush=True)
    return launcher, version

def inspect_compiled_model(path):
    """Read the FlatBuffer contract without trying to execute an Edge TPU on this computer."""
    from tensorflow.lite.python import schema_py_generated as schema
    data = path.read_bytes()
    if data[4:8] != b"TFL3":
        raise RuntimeError("Exporter did not produce a TFLite FlatBuffer")
    model = schema.Model.GetRootAsModel(data, 0)
    graph = model.Subgraphs(0)
    if graph.InputsLength() != 1 or graph.OutputsLength() != 1:
        raise RuntimeError("Expected one input and one output; use COCO detect with nms=False")
    input_tensor = graph.Tensors(graph.Inputs(0))
    output_tensor = graph.Tensors(graph.Outputs(0))
    input_shape = [input_tensor.Shape(i) for i in range(input_tensor.ShapeLength())]
    output_shape = [output_tensor.Shape(i) for i in range(output_tensor.ShapeLength())]
    if input_shape != [1, 320, 320, 3] or output_shape not in ([1, 84, 2100], [1, 2100, 84]):
        raise RuntimeError(f"Unexpected model shapes: {input_shape}, {output_shape}")
    quantized = (schema.TensorType.INT8, schema.TensorType.UINT8)
    if input_tensor.Type() not in quantized:
        raise RuntimeError("Expected quantized int8/uint8 input")
    if not any(model.OperatorCodes(i).CustomCode() == b"edgetpu-custom-op" for i in range(model.OperatorCodesLength())):
        raise RuntimeError("Compilation did not produce an Edge TPU operation")
    return {"input_shape": input_shape, "output_shape": output_shape,
            "input_tensor_type": int(input_tensor.Type()), "output_tensor_type": int(output_tensor.Type())}

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("models"))
    parser.add_argument("--compile-only", action="store_true",
                        help="Reuse this export's existing full_integer_quant.tflite after a compiler failure")
    args = parser.parse_args()
    if platform.system() != "Linux" or platform.machine() not in ("x86_64", "AMD64"):
        parser.error("Export requires x86_64 Linux or Google Colab. Run the compiled model on the Pi afterward.")
    from ultralytics import YOLO, __version__
    if __version__ != "8.3.228":
        raise RuntimeError("Use ultralytics==8.3.228 for this evaluation export contract")
    args.output.mkdir(parents=True, exist_ok=True)
    weights = args.output / "yolov8n.pt"
    if not weights.exists():
        temporary = weights.with_suffix(".pt.part")
        urllib.request.urlretrieve(WEIGHTS_URL, temporary)
        if hashlib.sha256(temporary.read_bytes()).hexdigest() != WEIGHTS_SHA256:
            raise RuntimeError("Downloaded YOLO weights failed SHA-256 validation")
        temporary.replace(weights)
    if hashlib.sha256(weights.read_bytes()).hexdigest() != WEIGHTS_SHA256:
        raise RuntimeError("Existing weights do not match the pinned stock COCO model")
    model = YOLO(str(weights))
    if len(model.names) != 80 or model.names[0] != "person" or model.names[11] != "stop sign":
        raise RuntimeError("Expected COCO-80 class mapping")
    compiler, compiler_version = ensure_compiler(args.output)
    if args.compile_only:
        # Reuse only the known output of this recipe, not an arbitrary model.
        source = args.output / "yolov8n_saved_model/yolov8n_full_integer_quant.tflite"
        if not source.is_file():
            raise RuntimeError("No completed INT8 conversion found. Run without --compile-only first.")
        subprocess.run([str(compiler), "--out_dir", str(source.parent), "--show_operations",
                        "--search_delegate", "--delegate_search_step", "30", "--timeout_sec", "180",
                        "--min_runtime_version", "14", str(source)], check=True)
        exported = source.with_name(source.stem + "_edgetpu.tflite")
    else:
        # Edge TPU export enables INT8 internally; 8.3.228 rejects an explicit int8=True.
        exported = Path(model.export(format="edgetpu", imgsz=320, batch=1,
                                     nms=False, data="coco128.yaml", device="cpu"))
    contract = inspect_compiled_model(exported)
    destination = args.output / "yolov8n_320_edgetpu.tflite"
    shutil.copyfile(exported, destination)
    digest = hashlib.sha256(destination.read_bytes()).hexdigest()
    manifest = {"model": destination.name, "sha256": digest, "weights_sha256": WEIGHTS_SHA256,
                "ultralytics": __version__, "compiler_version": compiler_version,
                "compiler_archive_sha256": COMPILER_SHA256, "min_runtime_version": 14,
                "class_count": 80, "person_class": 0, "stop_sign_class": 11,
                "box_coordinates": "normalized", "nms_in_graph": False, **contract}
    (args.output / "yolov8n_320_manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    (args.output / "yolov8n_320_edgetpu.sha256").write_text(f"{digest}  yolov8n_320_edgetpu.tflite\n")
    for log in exported.parent.glob("*edgetpu.log"):
        shutil.copyfile(log, args.output / "yolov8n_320_compile.log")
    print(f"Ready: {destination}. Inspect the compiler log for CPU operations; this is not a Pi benchmark.")

if __name__ == "__main__":
    main()
