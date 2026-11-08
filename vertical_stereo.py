"""Classical vertical-baseline stereo experiment.

The module deliberately contains no detector: corresponding image points are
selected manually so that the geometry can be validated independently.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Sequence

import cv2
import numpy as np

DEFAULT_CALIBRATION_PATH = Path("calibration/mobile.npz")


def calibrate_camera(
    image_paths: Sequence[str | Path],
    board_size: tuple[int, int],
    square_size: float,
) -> tuple[np.ndarray, np.ndarray, tuple[int, int], float]:
    """Calibrate a camera from checkerboard images.

    ``board_size`` is the number of inner corners (columns, rows), and
    ``square_size`` is in the desired physical unit, commonly centimetres.
    """
    if square_size <= 0:
        raise ValueError("square_size must be positive")

    columns, rows = board_size
    if columns < 2 or rows < 2:
        raise ValueError("board_size must contain at least two corners per side")

    object_template = np.zeros((columns * rows, 3), np.float32)
    object_template[:, :2] = (
        np.mgrid[0:columns, 0:rows].T.reshape(-1, 2) * square_size
    )
    object_points: list[np.ndarray] = []
    image_points: list[np.ndarray] = []
    image_size: tuple[int, int] | None = None

    for path in image_paths:
        image = cv2.imread(str(path))
        if image is None:
            raise FileNotFoundError(f"could not read calibration image: {path}")
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        current_size = (gray.shape[1], gray.shape[0])
        if image_size is None:
            image_size = current_size
        elif current_size != image_size:
            raise ValueError("all calibration images must have identical resolution")

        found, corners = cv2.findChessboardCorners(gray, board_size, None)
        if not found:
            continue
        refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                30,
                0.001,
            ),
        )
        object_points.append(object_template.copy())
        image_points.append(refined)

    if image_size is None:
        raise ValueError("no calibration images were supplied")
    if not object_points:
        raise ValueError("checkerboard was not detected in any calibration image")

    rms, camera_matrix, distortion, _, _ = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    return camera_matrix, distortion, image_size, float(rms)


def save_calibration(
    path: str | Path,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    image_size: tuple[int, int],
) -> None:
    """Save calibration data in a portable NumPy archive."""
    np.savez(
        path,
        camera_matrix=np.asarray(camera_matrix, dtype=np.float64),
        distortion=np.asarray(distortion, dtype=np.float64),
        image_size=np.asarray(image_size, dtype=np.int32),
    )


def load_calibration(path: str | Path) -> tuple[np.ndarray, np.ndarray, tuple[int, int]]:
    with np.load(path) as data:
        matrix = np.asarray(data["camera_matrix"], dtype=np.float64)
        distortion = np.asarray(data["distortion"], dtype=np.float64)
        image_size = tuple(int(value) for value in data["image_size"])
    if matrix.shape != (3, 3):
        raise ValueError("camera_matrix must have shape (3, 3)")
    return matrix, distortion, (image_size[0], image_size[1])


def vertical_baseline_pose(
    baseline: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Return world-to-camera poses for camera 1 and camera 2.

    World Y is positive upward. OpenCV camera Y is positive down, so the
    orientation is ``diag(1, -1, 1)``. ``baseline`` is the signed upward
    displacement of camera 2 from camera 1. Camera 1 is the world origin.
    """
    rotation = np.diag([1.0, -1.0, 1.0])
    camera1_center = np.zeros(3)
    camera2_center = np.array([0.0, baseline, 0.0])
    translation1 = -rotation @ camera1_center
    translation2 = -rotation @ camera2_center
    return rotation, translation1, rotation.copy(), translation2


def projection_matrix(
    camera_matrix: np.ndarray,
    rotation: np.ndarray,
    translation: np.ndarray,
) -> np.ndarray:
    """Construct P = K [R | t], mapping world points to image points."""
    return np.asarray(camera_matrix) @ np.column_stack((rotation, translation))


def undistort_points(
    points: np.ndarray,
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
) -> np.ndarray:
    """Return pixel coordinates corrected for lens distortion."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.undistortPoints(
        points, camera_matrix, distortion, P=camera_matrix
    ).reshape(-1, 2)


def triangulate(
    point1: Sequence[float],
    point2: Sequence[float],
    camera_matrix: np.ndarray,
    distortion: np.ndarray,
    baseline: float,
) -> np.ndarray:
    """Triangulate one corresponding point and return world XYZ."""
    corrected1 = undistort_points(np.asarray([point1]), camera_matrix, distortion)
    corrected2 = undistort_points(np.asarray([point2]), camera_matrix, distortion)
    r1, t1, r2, t2 = vertical_baseline_pose(baseline)
    p1 = projection_matrix(camera_matrix, r1, t1)
    p2 = projection_matrix(camera_matrix, r2, t2)
    homogeneous = cv2.triangulatePoints(p1, p2, corrected1.T, corrected2.T)
    if abs(float(homogeneous[3, 0])) < np.finfo(float).eps:
        raise ValueError("triangulation produced a point at infinity")
    return (homogeneous[:3, 0] / homogeneous[3, 0]).astype(float)


def select_point(image: np.ndarray, window_name: str) -> tuple[float, float]:
    """Select one point with a mouse click; Enter accepts and Esc cancels."""
    selected: list[tuple[float, float]] = []
    canvas = image.copy()

    def on_mouse(event: int, x: int, y: int, _flags: int, _param: object) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            selected[:] = [(float(x), float(y))]

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse)
    while True:
        display = canvas.copy()
        if selected:
            cv2.drawMarker(
                display, tuple(map(int, selected[0])), (0, 0, 255),
                cv2.MARKER_CROSS, 20, 2
            )
        cv2.imshow(window_name, display)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 32) and selected:
            cv2.destroyWindow(window_name)
            return selected[0]
        if key == 27:
            cv2.destroyWindow(window_name)
            raise KeyboardInterrupt("point selection cancelled")


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    calibration = subparsers.add_parser("calibrate")
    calibration.add_argument("--images", nargs="+", required=True)
    calibration.add_argument("--output", required=True)
    calibration.add_argument("--board-columns", type=int, required=True)
    calibration.add_argument("--board-rows", type=int, required=True)
    calibration.add_argument("--square-size", type=float, required=True)

    measure = subparsers.add_parser("measure")
    measure.add_argument("--image1", required=True)
    measure.add_argument("--image2", required=True)
    measure.add_argument("--baseline-cm", type=float, required=True)
    measure.add_argument("--point1", nargs=2, type=float)
    measure.add_argument("--point2", nargs=2, type=float)
    measure.add_argument("--actual-y-cm", type=float)
    return parser


def main() -> None:
    args = _parser().parse_args()
    if args.command == "calibrate":
        result = calibrate_camera(
            args.images,
            (args.board_columns, args.board_rows),
            args.square_size,
        )
        save_calibration(args.output, *result[:3])
        print(f"saved {args.output}; RMS reprojection error: {result[3]:.4f} px")
        return

    image1 = cv2.imread(args.image1)
    image2 = cv2.imread(args.image2)
    if image1 is None or image2 is None:
        raise FileNotFoundError("both measurement images must be readable")
    if image1.shape[:2] != image2.shape[:2]:
        raise ValueError("measurement images must have identical resolution")
    matrix, distortion, image_size = load_calibration(DEFAULT_CALIBRATION_PATH)
    if (image1.shape[1], image1.shape[0]) != image_size:
        raise ValueError("measurement image resolution differs from calibration")

    point1 = tuple(args.point1) if args.point1 else select_point(image1, "Camera 1")
    point2 = tuple(args.point2) if args.point2 else select_point(image2, "Camera 2")
    point = triangulate(point1, point2, matrix, distortion, args.baseline_cm)
    print(f"point 1: {point1}")
    print(f"point 2: {point2}")
    print(f"XYZ (cm): X={point[0]:.3f}, Y={point[1]:.3f}, Z={point[2]:.3f}")
    if args.actual_y_cm is not None:
        print(f"absolute Y error (cm): {abs(point[1] - args.actual_y_cm):.3f}")


if __name__ == "__main__":
    main()
