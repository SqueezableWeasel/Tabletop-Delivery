# Configuration and calibration

Run from the repository root or provide explicit resource paths. An explicit `--config FILE` takes precedence. Otherwise startup selects the file named in `active_calibration.json`, or `calibration_16in.json` when no activation record exists, or `delivery_config.json` when neither saved calibration is present. It then applies an explicitly requested `--tracking-config` overlay and the selected configuration's boundary sidecar. Missing or invalid selected files fail startup without silently choosing a different alignment.

Settings are validated before starting devices. Numeric values must be finite, integer fields cannot contain fractions, and arrays must have the specified lengths. The schema identifier describes the file format, not an application release.

## Coordinates and units

- X is right and Y is forward from the LiDAR center.
- Positions, radii, gaps, and widths are meters; speeds are meters/second.
- Raw LiDAR angles and calibration bearings are degrees. Display heading is radians.
- Tracking durations are seconds; internal host timestamps use a monotonic clock.
- Image coordinates are pixels in the 640 × 480 captured frame.
- HSV uses OpenCV's H = 0–179 and S/V = 0–255 convention.

## Calibration fields

| Key | Default | Interpretation |
| --- | --- | --- |
| `schema_version` | `1` | Required configuration format identifier. |
| `lidar_forward_deg` | `269.065` | Raw LiDAR azimuth corresponding to camera/table forward. |
| `camera_bearing_anchors` | Three `[u, bearing]` pairs | Piecewise-linear mapping from marker horizontal centroid to relative LiDAR bearing. |

Seed anchor pairs (replace through a measured calibration of the installed mount):

```json
[
  [155.326484, -12.678333],
  [333.378556, 0.0],
  [517.592068, 13.3075]
]
```

These values provide a starting map for the supplied mount. They are not a measured calibration at the 16-inch camera reference. Both pixel coordinates and bearings must increase across the three anchors.

### Measuring your mount

1. Secure camera and LiDAR on one rigid plate. Keep the useful scan sector clear of cables and supports.
2. Set the camera lens center **28 mm above the table**. Measure **16 inches (0.4064 m) horizontally forward from the point below the lens** to the figure's center projected onto the tabletop. Keep the body intersecting the horizontal scan; do not measure to the front of the figure.
3. Run `./build/tabletop_delivery --calibrate`. Capture center, then about 10 cm left, then about 10 cm right of center along the same forward row. For each stationary placement, click the figure's LiDAR cluster, wait for 20 stable paired samples, and press Space.
4. Accepting the third pose automatically saves `calibration_16in.json`, its `.measurements.json` report, and the active-file preference. Use `--calibration-output another_name.json` to select another destination; it also becomes the automatic default. Repeated saves retain uniquely named backups. If saving fails, S retries without discarding the captures.
5. Run `./build/tabletop_delivery`, verify intermediate positions, and only then start virtual motion. The saved calibration is loaded automatically. Recalibrate when the camera moves, focus changes, or capture crop changes. See [the capture walkthrough](CALIBRATION.md).

This is a local bearing map, not full intrinsic/extrinsic calibration. Camera/LiDAR parallax limits accuracy across substantially different depths. Do not subtract an arbitrary range offset to force a surface return to equal the base-center measurement.

## `marker`: image segmentation

| Key | Default | Purpose |
| --- | --- | --- |
| `hsv_low` | `[75,100,120]` | Inclusive lower threshold for blue/cyan pixels. |
| `hsv_high` | `[115,255,255]` | Inclusive upper threshold. |
| `roi_xyxy_px` | `[0,170,640,450]` | Processing rectangle; upper X/Y limits are exclusive. |
| `min_area_px` | `80` | Minimum connected-component area. |
| `max_area_px` | `4000` | Maximum area; rejects large unrelated regions. |
| `min_aspect` | `0.45` | Minimum bounding-box width/height. |
| `max_aspect` | `2.2` | Maximum bounding-box width/height. |
| `anchor_extrapolation_px` | `65` | Allowed horizontal extension beyond the outer calibration anchors. |

Multiple accepted components cause a hold. Increasing area/shape tolerance may admit unrelated objects and make association ambiguous.

## `lidar`: range clustering and association

| Key | Default | Purpose |
| --- | --- | --- |
| `roi_x_m` | `[-0.25,0.25]` | Accepted lateral range. |
| `roi_y_m` | `[0.4,0.8]` | Accepted forward range. |
| `min_quality` | `1` | Minimum decoded return quality; zero-quality returns are excluded. |
| `cluster_gap_m` | `0.045` | Maximum Euclidean gap between adjacent cluster returns. |
| `cluster_angle_gap_deg` | `2.5` | Maximum angular gap between adjacent returns. |
| `min_cluster_points` | `3` | Required points for a candidate cluster. |
| `max_cluster_width_m` | `0.1` | Maximum diagonal extent of a cluster's XY bounding box. |
| `bearing_gate_deg` | `3.5` | Maximum camera-to-cluster bearing difference. |

Exactly one candidate must pass the bearing gate. A wider gate can increase ambiguity; it does not improve physical calibration. The configured region restricts processing and is not a sensed table-edge boundary.

## `tracking`: temporal validation

| Key | Default | Purpose |
| --- | --- | --- |
| `freshness_s` | `0.5` | Maximum age of camera and scan observations; equality is stale. |
| `max_pair_delta_s` | `0.25` | Maximum absolute receive-time difference for a paired observation. |
| `confirmation_scans` | `2` | Consistent new scans required to establish a track. |
| `reacquire_after_s` | `0.7` | Gap after which position confirmation starts again. |
| `max_speed_m_s` | `0.4` | Figure-motion bound used to reject implausible jumps. |
| `jump_allowance_m` | `0.035` | Fixed tolerance added to the motion bound. |
| `smoothing_time_s` | `0.12` | Time constant for `alpha = 1 - exp(-dt/tau)`. |
| `trail_s` | `5.0` | Display history duration, also capped by a 256-point array. |

Position smoothing affects the displayed/filtered track. Raw positions still constrain every robot movement, so filtering does not enlarge free space.

## `robot`: geometry and motion

| Key | Default | Purpose |
| --- | --- | --- |
| `lane_width` | `0.48` | Wide lane width. |
| `narrow_width` | `0.16` | Narrow lane width selected by N. |
| `lane_ymin` / `lane_ymax` | `0.4` / `0.8` | Forward lane interval. |
| `robot_radius` | `0.022` | Circular virtual footprint; also insets the lane walls. |
| `figure_radius` | `0.045` | Modeled figure size, not automatically estimated from image width. |
| `margin` | `0.005` | Additional clearance beyond figure and robot radii. |
| `boundary_radius` | `0` | Optional total exclusion radius. Zero or omission uses the nominal figure + robot + margin sum; an explicit value must be 0.03–0.10 m. |
| `speed` | `0.028` | Nominal virtual travel speed in m/s, before pacing. |
| `speed_scale` | `0.8` | Optional pacing fraction, greater than 0 and at most 1. Omission selects 80% pacing. |
| `yield_s` | `1.0` | Pause before detour planning when the direct passage is obstructed. |
| `grid_m` | `0.01` | A* grid spacing. |
| `start` | `[0,0.435]` | Depot center. |
| `goal` | `[0,0.765]` | Delivery center. |

Both lanes must fit in the tracking region; endpoints must fit the inset narrow lane. The nominal starting radius is `figure_radius + robot_radius + margin` (7.2 cm by default). An explicit `boundary_radius`, live adjustment, or saved preference selects the total exclusion radius directly. The effective radius must always be 3–10 cm. It is the exclusion distance from the tracked figure to the virtual robot center, not a physical measurement of the prop. The selected total is not inflated again by the body radii; `robot_radius` still insets the lane walls.

Actual travel speed is `speed × speed_scale`: the defaults give **0.0224 m/s (2.24 cm/s)**. The fraction applies to motion along both direct routes and detours. Sensor processing, stop checks, and yield durations retain their normal timing. Saved calibrations that omit `speed_scale` automatically use `0.8`; calibration and session saves store nominal speed and pacing separately, so repeated saves do not compound the slowdown. To adjust pacing, edit `robot.speed_scale` in the configuration printed at startup. A value of `1.0` uses the full nominal speed.

## Boundary persistence

The slider, −/+ buttons, and `[`/`]` keys select **3.0–10.0 cm**, inclusive, in integer millimeters (0.1 cm steps). These limits are fixed and shared with configuration validation and the engine API. Selecting a radius changes `boundary_radius` without modifying the nominal body sizes or margin; a 3 cm selection is therefore usable even when the nominal sizes sum to a larger value.

Save or B writes `<active-config>.boundary.json`:

```json
{
  "schema_version": 1,
  "radius_m": 0.072
}
```

The saved radius is restored directly into `boundary_radius`. It must be finite and between 0.03 and 0.10 m, inclusive. Invalid preferences, including saved values above 10 cm, produce a warning and leave the validated main configuration intact. Removing the sidecar restores the main configuration's radius. Sidecars are local preferences excluded from Git.

## Optional tracking overlay

`--tracking-config PATH` selects a JSON file containing `schema_version`, `lidar_forward_deg`, `camera_bearing_anchors`, `marker`, `lidar`, and `tracking`. It replaces those fields while retaining `robot` geometry from the selected main config. The resulting combination is validated, then its boundary sidecar is applied. There is no automatic overlay search.

## Reference metadata and output

The `reference` object records `camera_to_figure_center_m` (0.4064), `camera_center_height_m` (0.028), `distance_origin`, scanning-window height when supplied, and capture notes. The template's `calibration_status` states that new physical captures are required. These descriptive values never alter LiDAR ranges or robot paths. A newly captured configuration records the camera height, horizontal reference, and measured-anchor provenance; its companion `.measurements.json` contains the same setup geometry plus three actual pixel/bearing/range averages and sample counts. `effective_config.json` records operational settings without copying descriptive notes. Live boundary changes appear in CSV and snapshot state rather than rewriting that startup record.
