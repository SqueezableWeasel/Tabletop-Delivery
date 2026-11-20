# Configuration and calibration

Run from the repository root or provide explicit resource paths. Startup reads `delivery_config.json` by default, applies `--tracking-config` only when requested, then applies the selected configuration's boundary sidecar. Missing required configuration files fail startup.

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

Reference anchor pairs:

```json
[
  [155.326484, -12.678333],
  [333.378556, 0.0],
  [517.592068, 13.3075]
]
```

These values describe the supplied mount. Both pixel coordinates and bearings must increase across the three anchors.

### Measuring your mount

1. Secure camera and LiDAR on one rigid plate. Keep the useful scan sector clear of cables and supports.
2. Place the marked figure near the two-foot row, with its body intersecting the horizontal scan. Record distance from the LiDAR center to the figure's base center separately from measured surface ranges.
3. Capture stationary left, center, and right placements near the same distance. For each placement, pair the marker centroid with the corresponding compact LiDAR cluster.
4. Set the forward angle and anchor pairs. Use M to inspect the threshold mask and adjust lighting/HSV limits if needed.
5. Verify intermediate positions before starting virtual motion. Recalibrate when the camera moves, focus changes, or capture crop changes.

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
| `speed` | `0.028` | Virtual travel speed. |
| `yield_s` | `1.0` | Pause before detour planning when the direct passage is obstructed. |
| `grid_m` | `0.01` | A* grid spacing. |
| `start` | `[0,0.435]` | Depot center. |
| `goal` | `[0,0.765]` | Delivery center. |

Both lanes must fit in the tracking region; endpoints must fit the inset narrow lane. The displayed combined radius is `figure_radius + robot_radius + margin` (7.2 cm by default). It is the exclusion distance for the robot center, not a physical measurement of the prop alone.

## Boundary persistence

The slider and −/+ buttons update radius in integer millimeters. `[` and `]` step by 1 mm (0.1 cm). The minimum is figure + robot radius, ensuring nonnegative margin. The standard maximum is 20 cm, extended if a larger configured radius requires it.

Save or B writes `<active-config>.boundary.json`:

```json
{
  "schema_version": 1,
  "radius_m": 0.072
}
```

The saved radius is converted back into margin. It must be finite, at least figure + robot radius, and no more than 2 m. Invalid preferences produce a warning and leave the main configuration intact. Removing the sidecar restores the main configuration's radius. Sidecars are local preferences excluded from Git.

## Optional tracking overlay

`--tracking-config PATH` selects a JSON file containing `schema_version`, `lidar_forward_deg`, `camera_bearing_anchors`, `marker`, `lidar`, and `tracking`. It replaces those fields while retaining `robot` geometry from the selected main config. The resulting combination is validated, then its boundary sidecar is applied. There is no automatic overlay search.

## Reference metadata and output

The `reference` object records the base-center distance, scanning-window height, and measurement notes. These descriptive values are not used to alter measurements or plan paths. `effective_config.json` in a run directory contains resolved operational settings; it does not copy descriptive reference text. Live boundary changes appear in CSV and snapshot state rather than rewriting that startup record.
