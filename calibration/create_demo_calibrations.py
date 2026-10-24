"""Create approximate demo calibrations from config.json assumptions.

These files exercise the CLI only. They are not real camera calibrations.
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

CONFIG_PATH = ROOT / "config.json"


def make_demo_calibration(camera_id: str, config: dict) -> None:
    demo = config["demo_assumptions"]
    camera_path = ROOT / config["demo_assumptions"]["image_paths"][camera_id]
    image = cv2.imread(str(camera_path))
    if image is None:
        raise ValueError(f"could not read demo image: {camera_path}")
    height, width = image.shape[:2]
    focal = float(demo["focal_length_px"])
    camera_matrix = np.array([
        [focal, 0.0, width / 2.0],
        [0.0, focal, height / 2.0],
        [0.0, 0.0, 1.0],
    ])

    mount = config["camera_mount_approx_m"][camera_id]
    center = np.array([mount["x_m"], mount["y_m"], mount["z_m"]], dtype=float)
    aim = np.array([0.0, float(demo["aim_height_y_m"]), 0.0], dtype=float)
    z_axis = aim - center
    z_axis /= np.linalg.norm(z_axis)
    x_axis = np.array([1.0, 0.0, 0.0])
    y_axis = np.cross(z_axis, x_axis)
    y_axis /= np.linalg.norm(y_axis)
    rotation = np.vstack((x_axis, y_axis, z_axis))
    translation = -rotation @ center

    calibration = CameraCalibration(
        camera_id=camera_id,
        camera_matrix=camera_matrix,
        distortion_coefficients=np.asarray(demo["distortion_coefficients"], dtype=float),
        rotation_world_to_camera=rotation,
        translation_world_to_camera=translation,
        is_placeholder=False,
        is_demo=True,
    )
    output_path = ROOT / config["cameras"][camera_id]
    save_calibration(calibration, output_path)
    print(f"{camera_id}: wrote demo-only calibration to {output_path.relative_to(ROOT)} ({width}x{height})")


def main() -> int:
    config = json.loads(CONFIG_PATH.read_text(encoding="utf-8-sig"))
    if config.get("mode") != "synthetic_demo":
        raise ValueError("set config.json mode to synthetic_demo before creating demo calibrations")
    for camera_id in ("SBL", "SLL"):
        make_demo_calibration(camera_id, config)
    print("Demo calibrations created. Measurements are illustrative only, never valid defect decisions.")
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (OSError, ValueError, KeyError, json.JSONDecodeError) as exc:
        raise SystemExit(f"Demo calibration stopped: {exc}")
