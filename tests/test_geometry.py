import numpy as np
import pytest

from camera_calibration import CameraCalibration
from clearance_measurement import evaluate_clearance
from coordinate_transform import (
    camera_to_world,
    project_world_point,
    triangulate_world_point,
    world_to_camera,
)


def camera(camera_id: str, translation: tuple[float, float, float]) -> CameraCalibration:
    return CameraCalibration(
        camera_id, np.diag([800.0, 800.0, 1.0]), np.zeros(5),
        np.eye(3), np.asarray(translation),
    )


def test_camera_world_transform_round_trip():
    calibration = camera("A", (1.0, 2.0, 3.0))
    point = np.array([4.0, 5.0, 6.0])
    np.testing.assert_allclose(camera_to_world(world_to_camera(point, calibration), calibration), point)


def test_stereo_reconstructs_known_point():
    a, b = camera("A", (0, 0, 0)), camera("B", (1, 0, 0))
    point = np.array([0.2, 0.1, 4.0])
    pixel_a, pixel_b = project_world_point(point, a), project_world_point(point, b)
    np.testing.assert_allclose(triangulate_world_point(pixel_a, a, pixel_b, b), point, atol=1e-6)


def test_height_and_rules():
    result = evaluate_clearance(0.85, 0.0, threshold_m=1.5, rule="maximum_height")
    assert result.height_m == pytest.approx(0.85)
    assert result.status == "PASS"
    assert evaluate_clearance(0.85, 0.0, threshold_m=1.5, rule=None).status == "RULE_UNCONFIGURED"


def test_missing_rail_reference_is_rejected():
    with pytest.raises(ValueError, match="rail reference"):
        evaluate_clearance(0.85, None, threshold_m=1.5, rule="maximum_height")
