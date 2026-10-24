"""Create real SBL/SLL calibration files from checkerboard image sets.

Run from the v2 directory after preparing calibration/calibration_setup.json
and the image folders described in README.md.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from camera_calibration import CameraCalibration, save_calibration

SETUP_PATH = ROOT / "calibration" / "calibration_setup.json"
CAMERAS = {
    "SBL": (ROOT / "calibration" / "images" / "SBL", ROOT / "calibration" / "sbl.npz"),
    "SLL": (ROOT / "calibration" / "images" / "SLL", ROOT / "calibration" / "sll.npz"),
}
IMAGE_SUFFIXES = {".png", ".jpg", ".jpeg", ".bmp"}


def read_setup() -> dict:
    setup = json.loads(SETUP_PATH.read_text(encoding="utf-8-sig"))
    cols = int(setup.get("inner_corners_cols", 0))
    rows = int(setup.get("inner_corners_rows", 0))
    if cols < 2 or rows < 2:
        raise ValueError("set inner_corners_cols and inner_corners_rows in calibration/calibration_setup.json")
    if setup.get("square_size_m") is None or float(setup["square_size_m"]) <= 0:
        raise ValueError("set the measured checkerboard square_size_m in calibration/calibration_setup.json")
    square = float(setup["square_size_m"])
    for key in ("world_first_inner_corner_m", "world_column_step_m", "world_row_step_m"):
        value = setup.get(key)
        try:
            coordinates = np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            coordinates = np.asarray([])
        if coordinates.shape != (3,) or not np.all(np.isfinite(coordinates)):
            raise ValueError(f"set {key} to its surveyed [X, Y, Z] values in metres")
        setup[key] = coordinates.tolist()
    for key in ("world_column_step_m", "world_row_step_m"):
        step_length = float(np.linalg.norm(setup[key]))
        if not np.isclose(step_length, square, rtol=0.05, atol=1e-6):
            raise ValueError(f"{key} length must match square_size_m (within 5%)")
    if np.linalg.norm(np.cross(setup["world_column_step_m"], setup["world_row_step_m"])) < 1e-8:
        raise ValueError("world_column_step_m and world_row_step_m must point in different directions")
    return setup


def checkerboard_points(cols: int, rows: int, square_size_m: float) -> np.ndarray:
    grid = np.zeros((rows * cols, 3), dtype=np.float32)
    grid[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_size_m
    return grid


def find_corners(image_path: Path, pattern_size: tuple[int, int]) -> tuple[np.ndarray, tuple[int, int]]:
    image = cv2.imread(str(image_path))
    if image is None:
        raise ValueError(f"could not read image: {image_path}")
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size, cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    if not found:
        raise ValueError(f"checkerboard not detected in {image_path}")
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
    )
    return corners, (gray.shape[1], gray.shape[0])


def calibrate_one(camera_id: str, camera_dir: Path, output_path: Path, setup: dict) -> None:
    cols, rows = int(setup["inner_corners_cols"]), int(setup["inner_corners_rows"])
    square = float(setup["square_size_m"])
    pattern_size = (cols, rows)
    local_board = checkerboard_points(cols, rows, square)

    intrinsics_dir = camera_dir / "intrinsics"
    images = sorted(p for p in intrinsics_dir.iterdir() if p.suffix.lower() in IMAGE_SUFFIXES) if intrinsics_dir.exists() else []
    if len(images) < 8:
        raise ValueError(f"{camera_id}: add at least 8 checkerboard views to {intrinsics_dir}")

    object_sets, image_sets = [], []
    image_size = None
    for path in images:
        corners, size = find_corners(path, pattern_size)
        if image_size is not None and size != image_size:
            raise ValueError(f"{camera_id}: all calibration photos must use the same resolution")
        image_size = size
        object_sets.append(local_board.copy())
        image_sets.append(corners)

    rms, matrix, distortion, _, _ = cv2.calibrateCamera(
        object_sets, image_sets, image_size, None, None
    )
    pose_path = camera_dir / "track_pose.png"
    pose_corners, pose_size = find_corners(pose_path, pattern_size)
    if pose_size != image_size:
        raise ValueError(f"{camera_id}: track_pose.png resolution must match the checkerboard photos")

    flips = setup.get("pose_corner_flips", {}).get(camera_id, {})
    pose_grid = pose_corners.reshape(rows, cols, 1, 2)
    if flips.get("rows", False):
        pose_grid = pose_grid[::-1, :, :, :]
    if flips.get("columns", False):
        pose_grid = pose_grid[:, ::-1, :, :]
    pose_image_points = np.ascontiguousarray(pose_grid.reshape(-1, 1, 2))

    debug_image = cv2.imread(str(pose_path))
    for index, corner in enumerate(pose_image_points.reshape(-1, 2)):
        x, y = np.rint(corner).astype(int)
        color = (0, 0, 255) if index == 0 else (0, 200, 0)
        cv2.circle(debug_image, (x, y), 5, color, -1)
        cv2.putText(debug_image, str(index), (x + 5, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    debug_path = ROOT / "calibration" / "debug" / f"{camera_id}_track_pose_corners.png"
    debug_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(debug_path), debug_image)

    origin = np.asarray(setup["world_first_inner_corner_m"], dtype=np.float32)
    col_step = np.asarray(setup["world_column_step_m"], dtype=np.float32)
    row_step = np.asarray(setup["world_row_step_m"], dtype=np.float32)
    world_points = np.asarray([
        origin + col * col_step + row * row_step
        for row in range(rows) for col in range(cols)
    ], dtype=np.float32)
    ok, rvec, tvec = cv2.solvePnP(
        world_points, pose_image_points, matrix, distortion, flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok:
        raise ValueError(f"{camera_id}: solvePnP could not find the camera pose")

    projected, _ = cv2.projectPoints(world_points, rvec, tvec, matrix, distortion)
    pose_rms = float(np.sqrt(np.mean(np.sum((projected.reshape(-1, 2) - pose_image_points.reshape(-1, 2)) ** 2, axis=1))))
    rotation, _ = cv2.Rodrigues(rvec)
    calibration = CameraCalibration(
        camera_id, matrix, distortion, rotation, tvec.reshape(3), is_placeholder=False
    )
    save_calibration(calibration, output_path)
    print(f"{camera_id}: saved {output_path.relative_to(ROOT)} | lens RMS={rms:.3f}px | track-pose RMS={pose_rms:.3f}px")


def main() -> int:
    setup = read_setup()
    for camera_id, (camera_dir, output_path) in CAMERAS.items():
        calibrate_one(camera_id, camera_dir, output_path, setup)
    print("Calibration files created. Review the RMS errors and validate against surveyed points before setting calibration_verified=true.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Calibration stopped: {exc}")
