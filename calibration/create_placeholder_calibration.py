"""Create structurally valid, non-production calibration placeholders.

Replace these files with results from real checkerboard calibration before
running a measurement. The identity camera matrix is not a physical
calibration and must not be used for inspection decisions.
"""

from pathlib import Path

import numpy as np


def create_placeholder(camera_id: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        camera_id=camera_id,
        camera_matrix=np.eye(3, dtype=float),
        distortion_coefficients=np.zeros(5, dtype=float),
        rotation_world_to_camera=np.eye(3, dtype=float),
        translation_world_to_camera=np.zeros(3, dtype=float),
    )


if __name__ == "__main__":
    create_placeholder("UNDERCARRIAGE_CAM", Path("calibration/undercarriage.npz"))
    create_placeholder("SIDE_CAM", Path("calibration/side.npz"))
    print("Created placeholder files. Replace them with real calibration results.")
