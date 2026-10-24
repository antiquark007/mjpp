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
    parser.add_argument("--sbl-image", type=Path, required=True)
    parser.add_argument("--sll-image", type=Path, required=True)
    parser.add_argument("--sbl-pixel", type=float, nargs=2, metavar=("X", "Y"))
    parser.add_argument("--sll-pixel", type=float, nargs=2, metavar=("X", "Y"))
    args = parser.parse_args()

    config = json.loads(args.config.read_text(encoding="utf-8"))
    if not config.get("calibration_verified", False):
        raise SystemExit(
            "calibration is not verified for the track frame; refusing to produce a measurement"
        )
    sbl_image = cv2.imread(str(args.sbl_image))
    sll_image = cv2.imread(str(args.sll_image))
    if sbl_image is None or sll_image is None:
        raise SystemExit("could not load both input images")
    cameras = config.get("cameras", {})
    sbl = load_calibration(cameras["SBL"])
    sll = load_calibration(cameras["SLL"])
    if sbl.camera_id != "SBL" or sll.camera_id != "SLL":
        raise SystemExit("calibration camera IDs must match SBL and SLL")
    if sbl.is_placeholder or sll.is_placeholder:
        raise SystemExit(
            "SBL/SLL calibration files are placeholders; create real track-frame calibrations before measuring"
        )
    if (args.sbl_pixel is None) != (args.sll_pixel is None):
        print("World point: unavailable")
        print("Measurement valid: False")
        print("Status: INSUFFICIENT_GEOMETRY (both corresponding pixels are required)")
        return 0
    if args.sbl_pixel is None:
        raise SystemExit("provide corresponding pixels for both cameras")
    sbl_pixel = np.asarray(args.sbl_pixel, dtype=float)
    sll_pixel = np.asarray(args.sll_pixel, dtype=float)
    point = triangulate_world_point(sbl_pixel, sbl, sll_pixel, sll)
    measurement = evaluate_clearance(
        point[1],
        config.get("rail_reference_y_m"),
        threshold_m=config.get("threshold_m"),
        rule=config.get("rule"),
    )
    print(f"Camera: SBL, Pixel: ({sbl_pixel[0]:.2f}, {sbl_pixel[1]:.2f})")
    print(f"Camera: SLL, Pixel: ({sll_pixel[0]:.2f}, {sll_pixel[1]:.2f})")
    print(f"World point: ({point[0]:.4f}, {point[1]:.4f}, {point[2]:.4f}) m")
    print(f"Rail reference height: {measurement.rail_reference_y_m:.4f} m")
    print(f"Measured height: {measurement.height_m:.4f} m ({measurement.height_m * 100:.1f} cm)")
    print(f"Threshold: {measurement.threshold_m * 100:.1f} cm" if measurement.threshold_m is not None else "Threshold: not configured")
    print("Measurement method: calibrated stereo triangulation")
    sbl_error = reprojection_error(point, sbl_pixel, sbl)
    sll_error = reprojection_error(point, sll_pixel, sll)
    print(f"Reprojection error: SBL={sbl_error:.3f}px, SLL={sll_error:.3f}px")
    max_error = float(config.get("max_reprojection_error_px", 3.0))
    valid = measurement.valid and max(sbl_error, sll_error) <= max_error
    print(f"Measurement valid: {valid}")
    print(f"Status: {'REVIEW_REPROJECTION_ERROR' if measurement.valid and not valid else measurement.status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
