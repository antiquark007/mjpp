import cv2
import numpy as np
import pytest

from vertical_stereo import (
    projection_matrix,
    triangulate,
    undistort_points,
    vertical_baseline_pose,
)


def test_vertical_baseline_recovers_known_world_point():
    matrix = np.array([[800.0, 0.0, 320.0], [0.0, 800.0, 240.0], [0.0, 0.0, 1.0]])
    distortion = np.zeros(5)
    baseline = 20.0
    expected = np.array([15.0, 35.0, 400.0])
    r1, t1, r2, t2 = vertical_baseline_pose(baseline)
    p1 = projection_matrix(matrix, r1, t1)
    p2 = projection_matrix(matrix, r2, t2)
    point1 = (p1 @ np.r_[expected, 1.0])
    point2 = (p2 @ np.r_[expected, 1.0])
    point1 = point1[:2] / point1[2]
    point2 = point2[:2] / point2[2]

    result = triangulate(point1, point2, matrix, distortion, baseline)

    np.testing.assert_allclose(result, expected, atol=1e-6)
    assert point2[1] - point1[1] == pytest.approx(matrix[1, 1] * baseline / expected[2])


def test_undistort_points_keeps_ideal_pixels_with_zero_distortion():
    matrix = np.array([[500.0, 0.0, 100.0], [0.0, 500.0, 80.0], [0.0, 0.0, 1.0]])
    point = np.array([[123.0, 234.0]])
    np.testing.assert_allclose(undistort_points(point, matrix, np.zeros(5)), point)


def test_projection_matrix_matches_opencv_projection():
    matrix = np.diag([100.0, 100.0, 1.0])
    rotation, translation, _, _ = vertical_baseline_pose(10.0)
    projection = projection_matrix(matrix, rotation, translation)
    world_point = np.array([0.0, 20.0, 100.0, 1.0])
    projected = projection @ world_point
    projected /= projected[2]
    assert projected[:2].tolist() == pytest.approx([0.0, -20.0])
