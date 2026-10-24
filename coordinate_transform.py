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
    # Remove lens distortion and express pixels as normalized camera rays.
    normalized_a = cv2.undistortPoints(
        np.asarray(pixel_a, dtype=float).reshape(1, 1, 2),
        camera_a.camera_matrix, camera_a.distortion_coefficients,
    ).reshape(2, 1)
    normalized_b = cv2.undistortPoints(
        np.asarray(pixel_b, dtype=float).reshape(1, 1, 2),
        camera_b.camera_matrix, camera_b.distortion_coefficients,
    ).reshape(2, 1)
    projection_a = np.hstack((camera_a.rotation_world_to_camera,
                              camera_a.translation_world_to_camera.reshape(3, 1)))
    projection_b = np.hstack((camera_b.rotation_world_to_camera,
                              camera_b.translation_world_to_camera.reshape(3, 1)))
    points = cv2.triangulatePoints(projection_a, projection_b, normalized_a, normalized_b)
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
