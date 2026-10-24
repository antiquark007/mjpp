"""World/camera transforms and calibrated stereo reconstruction."""

from __future__ import annotations

import cv2
import numpy as np

from camera_calibration import CameraCalibration


def world_to_camera(point_world: np.ndarray, calibration: CameraCalibration) -> np.ndarray:
    point = np.asarray(point_world, dtype=float).reshape(3)
    return calibration.rotation_world_to_camera @ point + calibration.translation_world_to_camera.reshape(3)


def camera_to_world(point_camera: np.ndarray, calibration: CameraCalibration) -> np.ndarray:
    point = np.asarray(point_camera, dtype=float).reshape(3)
    rotation = calibration.rotation_world_to_camera
    return rotation.T @ (point - calibration.translation_world_to_camera.reshape(3))


def project_world_point(point_world: np.ndarray, calibration: CameraCalibration) -> np.ndarray:
    projected, _ = cv2.projectPoints(
        np.asarray(point_world, dtype=float).reshape(1, 3),
        cv2.Rodrigues(calibration.rotation_world_to_camera)[0],
        np.asarray(calibration.translation_world_to_camera, dtype=float).reshape(3, 1),
        np.asarray(calibration.camera_matrix, dtype=float),
        np.asarray(calibration.distortion_coefficients, dtype=float),
    )
    return projected.reshape(2)


def triangulate_world_point(
    pixel_a: np.ndarray,
    camera_a: CameraCalibration,
    pixel_b: np.ndarray,
    camera_b: CameraCalibration,
) -> np.ndarray:
    points = cv2.triangulatePoints(
        camera_a.projection_matrix,
        camera_b.projection_matrix,
        np.asarray(pixel_a, dtype=float).reshape(2, 1),
        np.asarray(pixel_b, dtype=float).reshape(2, 1),
    )
    homogeneous = points[:, 0]
    if abs(homogeneous[3]) < 1e-12:
        raise ValueError("triangulated point is at infinity")
    return homogeneous[:3] / homogeneous[3]


def reprojection_error(
    point_world: np.ndarray,
    pixel: np.ndarray,
    calibration: CameraCalibration,
) -> float:
    return float(np.linalg.norm(project_world_point(point_world, calibration) - np.asarray(pixel)))
