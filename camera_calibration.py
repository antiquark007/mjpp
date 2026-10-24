"""OpenCV camera calibration and pose utilities."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    camera_id: str
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    rotation_world_to_camera: np.ndarray
    translation_world_to_camera: np.ndarray

    def __post_init__(self) -> None:
        if self.camera_matrix.shape != (3, 3):
            raise ValueError("camera_matrix must have shape (3, 3)")
        if self.rotation_world_to_camera.shape != (3, 3):
            raise ValueError("rotation_world_to_camera must have shape (3, 3)")
        if self.translation_world_to_camera.shape not in ((3,), (3, 1)):
            raise ValueError("translation_world_to_camera must contain 3 values")

    @property
    def projection_matrix(self) -> np.ndarray:
        return self.camera_matrix @ np.hstack(
            (self.rotation_world_to_camera, self.translation_world_to_camera.reshape(3, 1))
        )


def calibrate_camera(
    image_points: Iterable[np.ndarray],
    object_points: Iterable[np.ndarray],
    image_size: tuple[int, int],
    camera_id: str,
) -> CameraCalibration:
    """Calibrate one camera from multiple views of a known target.

    The returned pose is the target/world pose for the calibration view. For a
    shared railway datum, object_points must be expressed in that datum.
    """
    object_points_list = [np.asarray(points, dtype=np.float32) for points in object_points]
    image_points_list = [np.asarray(points, dtype=np.float32) for points in image_points]
    if not object_points_list or len(object_points_list) != len(image_points_list):
        raise ValueError("matching, non-empty image_points and object_points are required")
    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points_list, image_points_list, image_size, None, None
    )
    if not np.isfinite(rms):
        raise ValueError("camera calibration did not converge")
    return CameraCalibration(
        camera_id,
        matrix,
        distortion,
        cv2.Rodrigues(rotations[0])[0],
        translations[0].reshape(3),
    )


def load_calibration(path: str | Path) -> CameraCalibration:
    data = np.load(path)
    required = {"camera_id", "camera_matrix", "distortion_coefficients",
                "rotation_world_to_camera", "translation_world_to_camera"}
    missing = required - set(data.files)
    if missing:
        raise ValueError(f"calibration is missing fields: {', '.join(sorted(missing))}")
    return CameraCalibration(
        str(data["camera_id"].item()),
        data["camera_matrix"],
        data["distortion_coefficients"],
        data["rotation_world_to_camera"],
        data["translation_world_to_camera"],
    )


def save_calibration(calibration: CameraCalibration, path: str | Path) -> None:
    np.savez(
        path,
        camera_id=calibration.camera_id,
        camera_matrix=calibration.camera_matrix,
        distortion_coefficients=calibration.distortion_coefficients,
        rotation_world_to_camera=calibration.rotation_world_to_camera,
        translation_world_to_camera=calibration.translation_world_to_camera,
    )
