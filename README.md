# Vertical-baseline stereo experiment

This project validates vertical coordinate estimation with one mobile phone.
Capture a stationary object twice: first at `C1`, then after moving the phone
by a measured vertical baseline to `C2`. Keep orientation, X/Z position,
focus, exposure, resolution, and focal length unchanged. Do not duplicate or
warp the first image.

## Mathematical approach

World X is horizontal, world Y is vertical and positive upward, and world Z
points forward from the phone. OpenCV camera Y is positive downward, so the
controlled orientation is:

```text
R = diag(1, -1, 1)
C1 = (0, 0, 0)
C2 = (0, baseline, 0)
t = -R C
P = K [R | t]
```

Each selected pixel defines a viewing ray. `cv2.triangulatePoints(P1, P2, ...)`
estimates the homogeneous 3D point satisfying both rays. With this vertical
baseline, disparity is derived rather than assumed:
`v2 - v1 = +fy * baseline / Z`. Equal pixel coordinates are not expected.

The baseline and checkerboard square size use the same physical unit. Examples
below use centimetres, so XYZ results are centimetres.

## Install and calibrate

```bash
python -m pip install -r requirements.txt
python vertical_stereo.py calibrate \
  --images calibration/*.jpg --output calibration/mobile.npz \
  --board-columns 9 --board-rows 6 --square-size 2.5
```

Calibration uses `findChessboardCorners`, `cornerSubPix`, and
`calibrateCamera`. Failed checkerboard views are skipped, but at least one
successful view is required. Distortion is retained and points are
undistorted before triangulation. Assuming zero distortion is suitable only
for an initial synthetic check; phone lenses can otherwise bias depth near
image edges.

## Select points and measure

```bash
python vertical_stereo.py measure \
  --image1 data/c1.jpg --image2 data/c2.jpg \
  --baseline-cm 8.0 \
  --actual-y-cm 35.0
```

The measurement command always loads the hard-coded calibration file
`calibration/mobile.npz`. Generate that file with the calibration command
above before measuring.

Click the exact same physical feature in each window and press Enter.
For repeatable testing, use `--point1 x y --point2 x y` instead.
The output contains X, Y, Z and, when supplied, absolute Y error.

Repeat with multiple baselines and known object heights. Larger baselines
usually improve sensitivity, but make correspondence and framing harder.
Record point-selection uncertainty with every measurement.

## Tests

```bash
python -m pytest -q
```

The synthetic tests verify pose construction, the disparity equation, and
recovery of a known 3D point. They do not replace a physical experiment.
