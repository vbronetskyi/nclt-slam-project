"""I/O utilities for loading configs, data files, and saving results."""

from __future__ import annotations

import json
import logging
from pathlib import Path

import numpy as np
import yaml

logger = logging.getLogger(__name__)


def load_config(config_path: str | Path) -> dict:
    """Load YAML configuration file.

    Args:
        config_path: Path to the YAML config file.

    Returns:
        Parsed configuration dictionary.

    Raises:
        FileNotFoundError: If config file does not exist.
    """
    config_path = Path(config_path)
    if not config_path.exists():
        raise FileNotFoundError(f"Config not found: {config_path}")

    with open(config_path) as f:
        config = yaml.safe_load(f)

    logger.info("Loaded config from %s", config_path)
    return config


def save_config(config: dict, output_path: str | Path) -> None:
    """Save configuration dictionary to YAML file.

    Args:
        config: Configuration dictionary.
        output_path: Path to save the YAML file.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    with open(output_path, "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)

    logger.info("Saved config to %s", output_path)


def load_trajectory(path: str | Path, format: str = "tum") -> np.ndarray:
    """Load trajectory from file.

    Args:
        path: Path to trajectory file.
        format: Format of trajectory file. "tum" for TUM format
            (timestamp tx ty tz qx qy qz qw), "kitti" for KITTI format
            (3x4 flattened row-major per line).

    Returns:
        Array of 4x4 SE3 poses, shape (N, 4, 4).

    Raises:
        ValueError: If format is not recognized.
    """
    path = Path(path)

    if format == "tum":
        return _load_tum_trajectory(path)
    elif format == "kitti":
        return _load_kitti_trajectory(path)
    else:
        raise ValueError(f"Unknown trajectory format: {format}")


def _load_tum_trajectory(path: Path) -> np.ndarray:
    """Load TUM-format trajectory file.

    Format: timestamp tx ty tz qx qy qz qw (one pose per line).
    """
    from scipy.spatial.transform import Rotation

    data = np.loadtxt(path)
    n = len(data)
    poses = np.zeros((n, 4, 4), dtype=np.float64)

    for i in range(n):
        t = data[i, 1:4]
        q = data[i, 4:8]  # qx, qy, qz, qw
        R = Rotation.from_quat(q).as_matrix()
        poses[i, :3, :3] = R
        poses[i, :3, 3] = t
        poses[i, 3, 3] = 1.0

    return poses


def _load_kitti_trajectory(path: Path) -> np.ndarray:
    """Load KITTI-format trajectory file.

    Format: 12 values per line (3x4 matrix, row-major).
    """
    data = np.loadtxt(path)
    n = len(data)
    poses = np.zeros((n, 4, 4), dtype=np.float64)

    for i in range(n):
        poses[i, :3, :] = data[i].reshape(3, 4)
        poses[i, 3, 3] = 1.0

    return poses


def save_trajectory(
    poses: np.ndarray,
    output_path: str | Path,
    timestamps: np.ndarray | None = None,
    format: str = "tum",
) -> None:
    """Save trajectory to file.

    Args:
        poses: Array of 4x4 SE3 poses, shape (N, 4, 4).
        output_path: Path to save trajectory.
        timestamps: Optional timestamps array, shape (N,).
        format: Output format ("tum" or "kitti").
    """
    from scipy.spatial.transform import Rotation

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    n = len(poses)

    if format == "tum":
        if timestamps is None:
            timestamps = np.arange(n, dtype=np.float64)
        lines = []
        for i in range(n):
            t = poses[i, :3, 3]
            q = Rotation.from_matrix(poses[i, :3, :3]).as_quat()  # xyzw
            lines.append(
                f"{timestamps[i]:.6f} {t[0]:.6f} {t[1]:.6f} {t[2]:.6f} "
                f"{q[0]:.6f} {q[1]:.6f} {q[2]:.6f} {q[3]:.6f}"
            )
        output_path.write_text("\n".join(lines) + "\n")

    elif format == "kitti":
        lines = []
        for i in range(n):
            row = poses[i, :3, :].flatten()
            lines.append(" ".join(f"{v:.6e}" for v in row))
        output_path.write_text("\n".join(lines) + "\n")

    logger.info("Saved %d poses to %s (%s format)", n, output_path, format)


def save_results(results: dict, output_path: str | Path) -> None:
    """Save evaluation results to JSON.

    Args:
        results: Dictionary of evaluation metrics.
        output_path: Path to save JSON file.
    """
    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Convert numpy types to native Python
    def _convert(obj):
        if isinstance(obj, np.integer):
            return int(obj)
        if isinstance(obj, np.floating):
            return float(obj)
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return obj

    serializable = {k: _convert(v) for k, v in results.items()}

    with open(output_path, "w") as f:
        json.dump(serializable, f, indent=2)

    logger.info("Saved results to %s", output_path)
