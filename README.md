# v2: SBL/SLL height check

This program estimates the 3D position of one point visible in the SBL and SLL images. The configured rule is `DEFECT` at or below 1.50 m above the track datum (`Y = 0 m`).

## Run now with generated images (demo only)

The supplied images do not contain real camera calibration and are not guaranteed to show the same physical point. The demo mode runs the software path with approximate camera geometry from the setup drawing. Its output is always labeled `DEMO_ONLY`; it is not a valid height measurement or railway defect decision.

From the `v2` folder, run these commands in order:

1. `python calibration/create_demo_calibrations.py` — writes assumed SBL/SLL calibration files using the values in `config.json` and the image sizes in `data/`.
2. `python main.py --sbl-image data/sbl.png --sll-image data/sll.png --sbl-pixel 838.9 522.2 --sll-pixel 757.6 901.0` — runs the pipeline using the green marker centers in the current images. The markers are on different wagon parts, so only use this to see the program execute; its 3D point and threshold result are not meaningful.

Demo assumptions currently set in `config.json`:

- SBL camera center: approximately `(X=0, Y=0.322, Z=2.960)` m.
- SLL camera center: approximately `(X=0, Y=2.100, Z=2.850)` m.
- `Z` assumes the drawing's 1.625 m track-center-to-near-side distance plus each camera's wagon-side offset. The drawing does not fully specify camera orientation or lens properties.
- Focal length 1200 px, zero lens distortion, and aim point at Y=1.0 m are chosen placeholders for a runnable demonstration.

## Switch to real camera measurements later

1. Set `mode` to `real` in `config.json`.
2. Put at least 8 checkerboard views for each camera in `calibration/images/SBL/intrinsics/` and `calibration/images/SLL/intrinsics/`. Keep image resolution fixed.
3. Put a checkerboard at a surveyed, fixed position relative to the track datum. Capture it without moving it as `calibration/images/SBL/track_pose.png` and `calibration/images/SLL/track_pose.png`.
4. Fill `calibration/calibration_setup.json` with the checkerboard inner-corner counts, measured square size, surveyed first-corner coordinates, and surveyed row/column step vectors. All distances are in metres. The first corner must be identifiable in both images; the per-camera flip settings tell the script how to order detected corners.
5. Run `python calibration/create_calibrations.py`. Inspect `calibration/debug/` to confirm corner 0 is the surveyed first corner. Validate the results against surveyed points.
6. Only after validation, set `calibration_verified` to `true` and run `main.py` with simultaneous images and pixels for the same physical point.

Do not use the demo calibration files in real mode. The program rejects them.

## Config notes

`config.json` contains a `_comments` object because standard JSON does not allow `//` comments. It describes the purpose of every setting. Camera locations from the drawing are approximate; camera intrinsics and full poses require calibration.

## File guide

- `main.py` — command-line entry point: loads two images and calibrations, triangulates a selected point, and reports height/status.
- `config.json` — threshold, track datum, camera file paths, run mode, and explicitly labeled demo assumptions.
- `camera_calibration.py` — stores, loads, and saves camera matrices and camera-to-world-frame pose data.
- `coordinate_transform.py` — converts between camera/world coordinates, undistorts pixels, triangulates points, and calculates reprojection error.
- `clearance_measurement.py` — converts world Y to track-relative height and applies the configured threshold rule.
- `calibration/create_demo_calibrations.py` — creates approximate calibrations for software demonstration only.
- `calibration/create_calibrations.py` — creates real calibrations from checkerboard images and surveyed track-frame coordinates.
- `calibration/calibration_setup.json` — input measurements for the real calibration script; not needed in demo mode.
- `calibration/create_placeholder_calibration.py` — creates fake files for format checks only; never use its output for measurements.
- `calibration/sbl.npz`, `calibration/sll.npz` — generated calibration data read by `main.py`; their meaning depends on `mode` and the file's demo/placeholder flags.
- `data/sbl.png`, `data/sll.png` — generated demo images; not calibration photos or a verified matching stereo pair.
- `tests/test_geometry.py` — synthetic checks for coordinate transforms and threshold behavior.
- `requirements.txt` — Python packages required by the scripts (NumPy and OpenCV; pytest is for the existing tests).
- `README.md` — run instructions and project notes.
