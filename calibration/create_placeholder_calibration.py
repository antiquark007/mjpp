"""Create non-production placeholder files for the SBL and SLL cameras.

Replace these files with results from real checkerboard calibration before
running a measurement. The identity camera matrix is not a physical
calibration and must not be used for inspection decisions.
"""

from pathlib import Path

import numpy as np


def create_placeholder(camera_id: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if output_path.exists():
        raise FileExistsError(f"refusing to overwrite existing file: {output_path}")
    np.savez(
        output_path,
        camera_id=camera_id,
        camera_matrix=np.eye(3, dtype=float),
        distortion_coefficients=np.zeros(5, dtype=float),
        rotation_world_to_camera=np.eye(3, dtype=float),
        translation_world_to_camera=np.zeros(3, dtype=float),
        is_placeholder=True,
    )


if __name__ == "__main__":
    create_placeholder("SBL", Path("calibration/placeholders/sbl.npz"))
    create_placeholder("SLL", Path("calibration/placeholders/sll.npz"))
    print("Created test-only placeholders under calibration/placeholders/.")
