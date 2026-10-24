# Two-camera railway clearance measurement

This project estimates the rail-relative height of a component visible in an
undercarriage image and a side-view image. It uses calibrated stereo
triangulation with OpenCV and NumPy.

The repository did not contain an existing image pipeline or detection model.
The current workflow therefore accepts manually selected pixel coordinates.
The two pixels must identify the same physical point; identical pixel
coordinates in different images are not assumed to correspond.

## Setup with Conda

Create the local environment:

```bash
conda create --prefix .conda-env python=3.11 pip -y
conda run --prefix .conda-env python -m pip install -r requirements.txt
```

Activate it when working interactively:

```bash
conda activate ./.conda-env
```

Run the tests:

```bash
conda run --prefix .conda-env python -m pytest -q
```

## Project files

- `camera_calibration.py` - camera calibration and `.npz` persistence.
- `coordinate_transform.py` - camera/world transforms, projection, and
  triangulation.
- `clearance_measurement.py` - rail-relative height and configurable rules.
- `main.py` - command-line measurement workflow.
- `config.json` - calibration paths, rail datum, threshold, and rule.
- `tests/test_geometry.py` - synthetic geometry and measurement tests.

## Run a measurement

Place calibration files at the paths configured in `config.json`, then run:

```bash
conda run --prefix .conda-env python main.py \
  --undercarriage-image data/undercarriage.jpg \
  --side-image data/side.jpg \
  --undercarriage-pixel 520 310 \
  --side-pixel 480 300
```

The command reports:

- the selected pixel in each camera;
- the triangulated world point in metres;
- reprojection error in pixels;
- rail-relative height in metres and centimetres;
- measurement validity and inspection status.

If only one pixel is supplied, the result is
`INSUFFICIENT_GEOMETRY`. A single pixel does not determine a 3D height
without a known plane or component geometry. If either reprojection error is
greater than `max_reprojection_error_px`, the result is flagged
`REVIEW_REPROJECTION_ERROR`.

## Calibration

Calibrate each camera separately with a checkerboard or another known target.
Use `calibrate_camera` in `camera_calibration.py` with:

1. image points detected in each calibration image;
2. checkerboard object points in metres;
3. the image size;
4. the camera ID.

The object points must use one shared railway world frame:

- `X` - along the track;
- `Y` - vertical, positive upward;
- `Z` - lateral across the track.

Use `save_calibration` to write each `CameraCalibration` to an `.npz` file.
The two camera files must use the same world frame.

To create structurally valid placeholders for development or loader testing:

```bash
conda run --prefix .conda-env python calibration/create_placeholder_calibration.py
```

The generated files use an identity camera matrix and zero pose only so the
file format can be tested. They are not physical calibrations and must not be
used for railway measurements or inspection decisions.

A checkerboard square size establishes scale, but does not establish the rail
datum. The target must have a surveyed relationship to the rail or track.
Do not set `rail_reference_y_m` until that relationship is known.

## Configuration

The default [config.json](config.json) is intentionally fail-closed:

```json
{
  "rail_reference_y_m": null,
  "threshold_m": 1.5,
  "rule": null,
  "max_reprojection_error_px": 3.0,
  "cameras": {
    "UNDERCARRIAGE_CAM": "calibration/undercarriage.npz",
    "SIDE_CAM": "calibration/side.npz"
  }
}
```

Supported rules are:

- `maximum_height`: pass when `height_m <= threshold_m`;
- `minimum_clearance`: pass when `height_m >= threshold_m`;
- `absolute_separation`: pass when `abs(height_m) <= threshold_m`.

The height calculation is:

```text
height_m = point_world_y - rail_reference_y_m
```

If the rail reference or rule is missing, the program reports
`RULE_UNCONFIGURED` and does not report PASS or FAIL.

## Limitations

The synthetic tests validate transformations, stereo reconstruction, height
calculation, threshold behavior, and missing-reference handling. They do not
prove field accuracy. Real accuracy depends on camera baseline, calibration
quality, target-to-rail surveying, image synchronization, point selection,
occlusion, and the official railway inspection rule.
