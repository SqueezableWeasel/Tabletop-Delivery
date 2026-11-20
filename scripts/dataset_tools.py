"""Prepare camera recordings and reviewed CVAT labels for this two-class experiment.

Run --help for commands. Python is used offline for dataset preparation; this
does not change the Raspberry Pi's native C++ application.
"""
from pathlib import Path
import argparse
import collections
import hashlib
import json
import math
import re
import shutil
import stat
import sys
import zipfile

NAMES = {0: "figure", 1: "stop_sign"}
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}


def recording(name):
    """Keep every frame from one recording in its preselected split."""
    match = re.match(r"^((train|val|test)\d+)_", name)
    if not match:
        raise ValueError(f"Image name must start train01_, val01_, test01_, etc.: {name}")
    return match.group(1), match.group(2)


def names_from_yaml(path):
    import yaml
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = data.get("names", {})
    if isinstance(names, list):
        names = dict(enumerate(names))
    names = {int(key): value for key, value in names.items()}
    if len(names) != 2 or set(names.values()) != set(NAMES.values()):
        raise ValueError(f"Expected only figure and stop_sign classes in {path}; got {names}")
    return names


def read_labels(path, mapping=None):
    """Reject malformed, out-of-frame, or non-detection annotations."""
    rows = []
    for line_number, line in enumerate(path.read_text(encoding="utf-8-sig").splitlines(), 1):
        if not line.strip():
            continue
        values = line.split()
        if len(values) != 5:
            raise ValueError(f"{path}:{line_number}: need class cx cy width height (5 columns)")
        category = int(values[0])
        if mapping is not None:
            if category not in mapping:
                raise ValueError(f"Unknown class {category} in {path}")
            category = mapping[category]
        if category not in NAMES:
            raise ValueError(f"Unsupported class {category} in {path}")
        x, y, width, height = map(float, values[1:])
        if not all(math.isfinite(v) for v in (x, y, width, height)):
            raise ValueError(f"Nonfinite label in {path}")
        if not (0 <= x <= 1 and 0 <= y <= 1 and 0 < width <= 1 and 0 < height <= 1):
            raise ValueError(f"Coordinates must be normalized to [0,1] in {path}")
        if x-width/2 < -1e-5 or y-height/2 < -1e-5 or x+width/2 > 1+1e-5 or y+height/2 > 1+1e-5:
            raise ValueError(f"Box extends outside the image in {path}")
        rows.append((category, x, y, width, height))
    return rows


def validate_dataset(root):
    """Validate images and labels; an explicit manifest permits a within-recording holdout."""
    from PIL import Image
    import yaml
    root = Path(root)
    if names_from_yaml(root/"data.yaml") != NAMES:
        raise ValueError("Prepared class order must be 0=figure, 1=stop_sign")
    config = yaml.safe_load((root/"data.yaml").read_text(encoding="utf-8"))
    manifest_path = root/"manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8")) if manifest_path.is_file() else {}
    policy = manifest.get("split_policy", "recording_prefix")
    if policy not in ("recording_prefix", "explicit_holdout"):
        raise ValueError(f"Unknown split policy: {policy}")
    entries = manifest.get("images", [])
    expected = {item["image"]: item for item in entries}
    if policy == "explicit_holdout" and (not expected or len(expected) != len(entries)):
        raise ValueError("An explicit holdout requires a complete, unique image manifest")
    seen_pixels, summary, seen_paths, seen_stems = {}, {}, set(), set()
    for split in ("train", "val", "test"):
        images = sorted(p for p in (root/"images"/split).glob("*") if p.is_file() and p.suffix.lower() in IMAGE_SUFFIXES)
        if split == "test" and not images and not config.get("test"):
            continue  # A test set is optional; never substitute validation for it.
        if not images:
            raise ValueError(f"No images in required {split} split")
        counts = collections.Counter()
        groups, negatives = set(), 0
        for image in images:
            group, declared_split = recording(image.name)
            relative = image.relative_to(root).as_posix()
            seen_paths.add(relative)
            if image.stem in seen_stems:
                raise ValueError(f"An image filename is reused across splits: {image.name}")
            seen_stems.add(image.stem)
            if policy == "explicit_holdout":
                item = expected.get(relative)
                if not item or item.get("split") != split or item.get("recording") != group:
                    raise ValueError(f"Image does not match the explicit split manifest: {image}")
                if item.get("sha256") != hashlib.sha256(image.read_bytes()).hexdigest():
                    raise ValueError(f"Image content differs from split manifest: {image}")
            elif declared_split != split:
                raise ValueError(f"Recording moved across splits: {image}")
            groups.add(group)
            with Image.open(image) as opened:
                if opened.size != (640, 480):
                    raise ValueError(f"Expected the 640x480 camera image, not a screen recording: {image}")
                digest = hashlib.sha256(opened.convert("RGB").tobytes()).hexdigest()
            if digest in seen_pixels and seen_pixels[digest][0] != split:
                raise ValueError(f"Identical pixels cross splits: {image} and {seen_pixels[digest][1]}")
            seen_pixels[digest] = (split, str(image))
            label = root/"labels"/split/(image.stem+".txt")
            if not label.is_file():
                raise ValueError(f"Missing label {label}; reviewed background images need an empty label file")
            rows = read_labels(label)
            negatives += not rows
            counts.update(row[0] for row in rows)
        if any(counts[category] == 0 for category in NAMES):
            raise ValueError(f"Both classes need examples in {split}: {counts}")
        summary[split] = {"images": len(images), "backgrounds": negatives,
                          "boxes": {NAMES[k]: counts[k] for k in NAMES}, "recordings": sorted(groups)}
    if policy == "explicit_holdout" and seen_paths != set(expected):
        raise ValueError("Split manifest contains missing or extra image entries")
    return summary


def safe_extract(archive_path, destination):
    """Extract a supplied dataset into a new folder without path traversal or overwrites."""
    destination = Path(destination).resolve()
    destination.mkdir(parents=True, exist_ok=False)
    with zipfile.ZipFile(archive_path) as archive:
        if sum(item.file_size for item in archive.infolist()) > 8_000_000_000:
            raise ValueError("Dataset expands beyond this notebook's 8 GB limit")
        for item in archive.infolist():
            name = item.filename.replace("\\", "/")
            relative = Path(name)
            if relative.is_absolute() or ".." in relative.parts or ":" in name or stat.S_ISLNK(item.external_attr >> 16):
                raise ValueError(f"Unsafe ZIP entry: {name}")
            target = destination/relative
            if not target.resolve().is_relative_to(destination):
                raise ValueError(f"ZIP entry escapes destination: {name}")
            if item.is_dir():
                target.mkdir(parents=True, exist_ok=True)
            else:
                target.parent.mkdir(parents=True, exist_ok=True)
                with archive.open(item) as source, target.open("xb") as output:
                    shutil.copyfileobj(source, output)


def extract(args):
    """Decode at capture rate and keep one frame per interval; skip exposure startup."""
    import cv2
    if args.fps <= 0 or args.every <= 0:
        raise ValueError("fps and every must be positive")
    videos = sorted(args.videos.glob("*.mjpeg"))
    if not videos:
        raise ValueError("No .mjpeg recordings found")
    args.out.mkdir(parents=True, exist_ok=True)
    for video in videos:
        if not re.fullmatch(r"(train|val|test)\d+", video.stem):
            raise ValueError(f"Name recordings train01.mjpeg, val01.mjpeg, test01.mjpeg, etc.: {video}")
        if any(args.out.glob(video.stem+"_*")):
            raise ValueError(f"Frames for {video.stem} already exist; use a new output folder or recording name")
        capture = cv2.VideoCapture(str(video))
        if not capture.isOpened():
            raise ValueError(f"Cannot open {video}")
        frame_number, saved = 0, 0
        stride = max(1, round(args.fps*args.every))
        skip = round(args.fps*2)  # Let camera exposure settle for two seconds.
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            if frame.shape != (480, 640, 3):
                raise ValueError(f"Expected 640x480 camera video: {video}")
            if frame_number >= skip and (frame_number-skip) % stride == 0:
                target = args.out/f"{video.stem}_f{frame_number:06d}.jpg"
                if not cv2.imwrite(str(target), frame, [cv2.IMWRITE_JPEG_QUALITY, 95]):
                    raise IOError(f"Cannot write {target}")
                saved += 1
            frame_number += 1
        capture.release()
        if not saved:
            raise ValueError(f"No usable frames from {video}")
        print(f"{video.name}: {saved} images; keep/review a varied subset before labeling")


def prepare(args):
    """Normalize CVAT labels using recording prefixes or an explicit reviewed split plan."""
    import yaml
    candidates = [p for p in args.source.rglob("data.yaml") if (p.parent/"images").is_dir()]
    if len(candidates) != 1:
        raise ValueError("Expected one exported data.yaml alongside images/ and labels/")
    origin = candidates[0].parent
    source_names = names_from_yaml(candidates[0])
    mapping = {source_id: next(k for k,v in NAMES.items() if v==name) for source_id,name in source_names.items()}
    reviewed = set()
    if args.reviewed_backgrounds:
        reviewed = {line.strip() for line in args.reviewed_backgrounds.read_text().splitlines() if line.strip()}
    images = sorted(p for p in (origin/"images").rglob("*") if p.suffix.lower() in IMAGE_SUFFIXES)
    planned, seen_names = [], set()
    for image in images:
        group, split = recording(image.name)
        if image.stem in seen_names:
            raise ValueError(f"Duplicate image stem: {image.stem}")
        seen_names.add(image.stem)
        label = origin/"labels"/image.relative_to(origin/"images").with_suffix(".txt")
        if label.is_file():
            rows = read_labels(label, mapping)
        elif image.name in reviewed:
            rows = []  # The user explicitly reviewed this frame as containing neither class.
        else:
            raise ValueError(f"Missing label for {image.name}. Label it, or list this exact filename in reviewed_backgrounds.txt if BOTH target classes are absent.")
        planned.append((image, group, split, rows))
    if not planned:
        raise ValueError("No exported images; export the CVAT dataset WITH images")
    split_policy, split_note = "recording_prefix", "Recordings assigned by their original filename prefixes."
    split_plan_path = getattr(args, "split_plan", None)
    if split_plan_path:
        plan = json.loads(split_plan_path.read_text(encoding="utf-8"))
        assignments = plan.get("assignments", {})
        if set(assignments) != {image.name for image, _, _, _ in planned}:
            raise ValueError("Split plan must assign every source image exactly once by its original filename")
        if any(split not in ("train", "val", "test") for split in assignments.values()):
            raise ValueError("Split assignments must be train, val, or test")
        planned = [(image, group, assignments[image.name], rows) for image, group, _, rows in planned]
        split_policy = "explicit_holdout"
        split_note = plan.get("evaluation_note", "Exploratory split; source recordings may span training and validation.")
        intended_counts = plan.get("target_counts")
        actual_counts = dict(collections.Counter(split for _, _, split, _ in planned))
        if intended_counts is not None and intended_counts != actual_counts:
            raise ValueError(f"Split plan counts differ: expected {intended_counts}, found {actual_counts}")
    # Check coverage before creating output, so missing recordings leave no partial dataset.
    split_sizes = collections.Counter(split for _, _, split, _ in planned)
    required_splits = ("train", "val") + (("test",) if split_sizes["test"] else ())
    missing_splits = [split for split in required_splits if not split_sizes[split]]
    if missing_splits:
        counts = ", ".join(f"{split}={split_sizes[split]}" for split in required_splits)
        recordings = ", ".join(f"{split}01.mjpeg" for split in missing_splits)
        raise ValueError(
            f"Dataset is incomplete ({counts}). Missing splits: {', '.join(missing_splits)}. "
            f"Add labeled sessions named {recordings}, or supply a reviewed --split-plan "
            "to reserve existing images for validation. Test images are optional. No output was written."
        )
    for split in required_splits:
        classes = {row[0] for _, _, selected_split, rows in planned if selected_split == split for row in rows}
        missing_classes = [NAMES[category] for category in NAMES if category not in classes]
        if missing_classes:
            raise ValueError(
                f"The {split} split has no labeled examples of: {', '.join(missing_classes)}. "
                "Record and label these objects in this split before preparation. No output was written."
            )
    if args.out.exists() or args.out.with_suffix(".zip").exists():
        raise ValueError("Output dataset/ZIP already exists; choose a new --out name")
    manifest = []
    for image, group, split, rows in planned:
        dest = args.out/"images"/split/image.name
        label = args.out/"labels"/split/(image.stem+".txt")
        dest.parent.mkdir(parents=True, exist_ok=True)
        label.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(image, dest)
        label.write_text("".join(str(row[0])+" "+" ".join(f"{v:.8f}" for v in row[1:])+"\n" for row in rows))
        manifest.append({"image": dest.relative_to(args.out).as_posix(), "recording": group,
                         "split": split, "sha256": hashlib.sha256(dest.read_bytes()).hexdigest()})
    data = {"path": ".", "train": "images/train", "val": "images/val", "names": NAMES}
    if split_sizes["test"]:
        data["test"] = "images/test"
    (args.out/"data.yaml").write_text(yaml.safe_dump(data, sort_keys=False))
    metadata = {"images": manifest, "split_policy": split_policy, "evaluation_note": split_note}
    if split_plan_path:
        metadata["split_plan"] = plan
    # The validator checks this assignment manifest without changing original image names.
    (args.out/"manifest.json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    summary = validate_dataset(args.out)
    metadata["summary"] = summary
    (args.out/"manifest.json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    with zipfile.ZipFile(args.out.with_suffix(".zip"), "w", zipfile.ZIP_DEFLATED) as archive:
        for file in sorted(args.out.rglob("*")):
            if file.is_file():
                archive.write(file, Path(args.out.name)/file.relative_to(args.out))
    print(json.dumps(summary, indent=2))
    print(f"Ready: {args.out.with_suffix('.zip')}")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    extract_parser = sub.add_parser("extract", help="Sample camera MJPEG recordings for annotation")
    extract_parser.add_argument("--videos", type=Path, required=True)
    extract_parser.add_argument("--out", type=Path, default=Path("images_to_label"))
    extract_parser.add_argument("--fps", type=float, default=15, help="Capture frame rate; raw MJPEG has no reliable timing metadata")
    extract_parser.add_argument("--every", type=float, default=1, help="Seconds between sampled frames")
    prepare_parser = sub.add_parser("prepare", help="Normalize a reviewed CVAT dataset and create its ZIP")
    prepare_parser.add_argument("--source", type=Path, required=True)
    prepare_parser.add_argument("--out", type=Path, default=Path("tabletop_dataset"))
    prepare_parser.add_argument("--reviewed-backgrounds", type=Path)
    prepare_parser.add_argument("--split-plan", type=Path, help="JSON assignments for an explicitly chosen train/val holdout; filenames stay unchanged")
    validate_parser = sub.add_parser("validate", help="Check an already prepared dataset")
    validate_parser.add_argument("root", type=Path)
    args = parser.parse_args()
    if args.command == "extract":
        extract(args)
    elif args.command == "prepare":
        prepare(args)
    else:
        print(json.dumps(validate_dataset(args.root), indent=2))


if __name__ == "__main__":
    try:
        main()
    except (ValueError, OSError) as error:
        print(f"ERROR: {error}", file=sys.stderr)
        raise SystemExit(1)
