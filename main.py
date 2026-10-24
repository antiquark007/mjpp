"""Command-line workflow for two-camera rail-relative measurement."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from camera_calibration import load_calibration
from clearance_measurement import evaluate_clearance
from coordinate_transform import reprojection_error, triangulate_world_point


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=Path("config.json"))
    parser.add_argument("--undercarriage-image", type=Path, required=True)
    parser.add_argument("--side-image", type=Path, required=True)
    parser.add_argument("--undercarriage-pixel", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--side-pixel", type=float, nargs=2, metavar=("X", "Y"))
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    under_image = cv2.imread(str(args.undercarriage_image))
    side_image = cv2.imread(str(args.side_image))
    if under_image is None or side_image is None:
        raise SystemExit("could not load both input images")
    cameras = config.get("cameras", {})
    under = load_calibration(cameras["UNDERCARRIAGE_CAM"])
    side = load_calibration(cameras["SIDE_CAM"])
    if (args.undercarriage_pixel is None) != (args.side_pixel is None):
        print("World point: unavailable")
        print("Measurement valid: False")
        print("Status: INSUFFICIENT_GEOMETRY (both corresponding pixels are required)")
        return 0
    if args.undercarriage_pixel is None:
        raise SystemExit("provide corresponding pixels for both cameras")
    under_pixel = np.asarray(args.undercarriage_pixel, dtype=float)
    side_pixel = np.asarray(args.side_pixel, dtype=float)
    point = triangulate_world_point(under_pixel, under, side_pixel, side)
    measurement = evaluate_clearance(
        point[1],
        config.get("rail_reference_y_m"),
        threshold_m=config.get("threshold_m"),
        rule=config.get("rule"),
    )
    print(f"Camera: UNDERCARRIAGE_CAM, Pixel: ({under_pixel[0]:.2f}, {under_pixel[1]:.2f})")
    print(f"Camera: SIDE_CAM, Pixel: ({side_pixel[0]:.2f}, {side_pixel[1]:.2f})")
    print(f"World point: ({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}) m")
    print(f"Rail reference height: {measurement.rail_reference_y_m:.4f} m")
    print(f"Measured height: {measurement.height_m:.4f} m ({measurement.height_m * 100:.1f} cm)")
    print(f"Threshold: {measurement.threshold_m * 100:.1f} cm" if measurement.threshold_m is not None else "Threshold: not configured")
    print("Measurement method: calibrated stereo triangulation")
    under_error = reprojection_error(point, under_pixel, under)
    side_error = reprojection_error(point, side_pixel, side)
    print(f"Reprojection error: undercarriage={under_error:.3f}px, side={side_error:.3f}px")
    max_error = float(config.get("max_reprojection_error_px", 3.0))
    valid = measurement.valid and max(under_error, side_error) <= max_error
    print(f"Measurement valid: {valid}")
    print(f"Status: {'REVIEW_REPROJECTION_ERROR' if measurement.valid and not valid else measurement.status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
