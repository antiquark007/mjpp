# v2: SBL/SLL height check

The system estimates the 3D position of one point seen by both cameras. It reports `DEFECT` when the point is at or below 1.50 m above the configured track datum (`Y = 0 m`).

## Do these steps in order

### 1. Prepare calibration photos

Keep the cameras at their final fixed positions and use the same image resolution that will be used during inspection.

For **each camera**, take at least 8 checkerboard photos with the board at different distances and angles. Save them here:

- `calibration/images/SBL/intrinsics/`
- `calibration/images/SLL/intrinsics/`

Use a board that is fully visible and in focus. The checkerboard size means the number of inner corner points, not the number of squares.

### 2. Capture the surveyed board pose

Place the checkerboard at a fixed, surveyed position relative to the track. Both cameras must see it. Capture one image from each camera without moving the board:

- `calibration/images/SBL/track_pose.png`
- `calibration/images/SLL/track_pose.png`

Mark the board's chosen first inner corner. Record that corner's track coordinates and the 3D step between neighboring corners in both board directions. Use metres and the shared axes: X along track, Y upward from the track datum, Z lateral from the chosen track plane. Do not use camera-to-wagon offsets as board coordinates.

### 3. Enter the board measurements

Edit `calibration/calibration_setup.json`:

- `inner_corners_cols`, `inner_corners_rows`: checkerboard inner corner counts.
- `square_size_m`: measured side length of one square.
- `world_first_inner_corner_m`: surveyed `[X, Y, Z]` of the marked first inner corner.
- `world_column_step_m`, `world_row_step_m`: surveyed `[dX, dY, dZ]` from one inner corner to the next in each direction.
- `pose_corner_flips`: leave `false` unless the debug image shows that the detected corner order must be flipped to make corner 0 the marked corner and align the rows/columns with the entered step vectors.

These surveyed values are required; the script will stop if they are missing. The approximate SBL/SLL mounting dimensions in `config.json` do not provide them.

### 4. Create the calibration files

From the `v2` folder, run:

```bash
python calibration/create_calibrations.py
```

This detects checkerboard corners, calculates each lens calibration and track-frame camera pose, then writes `calibration/sbl.npz` and `calibration/sll.npz`. It also writes corner-order previews to `calibration/debug/`. Check that the red point labeled `0` is on the marked first corner in both previews. If it is not, adjust that camera's `pose_corner_flips` and run the script again.

The script prints lens and pose reprojection errors. Check the previews and validate the result against surveyed reference points before continuing. A low reprojection error alone does not prove the track coordinates are correct.

### 5. Verify and run one measurement

After reviewing the calibration against known points, set `calibration_verified` to `true` in `config.json`. It is `false` by default so unreviewed files cannot produce a defect decision.

Use simultaneous SBL/SLL images and enter the pixel location of the **same physical point** in both. The pixel coordinates will usually differ:

```bash
python main.py --sbl-image data/sbl.png --sll-image data/sll.png --sbl-pixel 520 310 --sll-pixel 480 300
```

Replace the example paths and coordinates with your images and selected point. The images currently in `data/` are synthetic, and their green markers are on different wagon parts; they cannot be used as a valid matching pair.

## If the CLI says the calibration files are placeholders

The files at the configured paths were created by `calibration/create_placeholder_calibration.py`. That script only makes fake files for software-format checks; it does not calibrate cameras. Run `create_calibrations.py` as described above to replace them. Do not use files under `calibration/placeholders/` for measurement.
