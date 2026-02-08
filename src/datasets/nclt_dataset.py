"""NCLT (North Campus Long-Term) LiDAR dataset loader.

Provides a PyTorch Dataset implementation for the NCLT dataset, supporting
both Kaggle and local data paths. Handles point cloud loading from binary
Velodyne files, pose parsing from track.csv, and optional image loading.

Reference:
    N. Carlevaris-Bianco, A. K. Ushani, and R. M. Eustice,
    "University of Michigan North Campus Long-Term Vision and LiDAR Dataset,"
    International Journal of Robotics Research, 2016.

Typical usage:
    >>> from src.datasets.nclt_dataset import NCLTDataset
    >>> dataset = NCLTDataset("configs/dataset_config.yaml", split="train")
    >>> sample = dataset[0]
    >>> sample["point_cloud"].shape  # (N, 4) tensor
"""

from __future__ import annotations

import logging
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable

import numpy as np
import torch
import yaml
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)

try:
    import cv2

    HAS_CV2 = True
except ImportError:
    HAS_CV2 = False
    logger.debug("OpenCV not installed. Image loading will use PIL or be skipped.")


@dataclass
class SampleRecord:
    """Metadata for a single dataset sample.

    Attributes:
        session: Session date string (e.g. "2012-01-08").
        timestamp: Integer timestamp from track.csv.
        velodyne_path: Absolute path to the .bin point cloud file.
        image_dir: Absolute path to the images_small directory.
        pose: 4x4 SE(3) transformation matrix as float32 numpy array.
    """

    session: str
    timestamp: int
    velodyne_path: Path
    image_dir: Path
    pose: np.ndarray


@dataclass
class PointCloudConfig:
    """Point cloud preprocessing parameters.

    Attributes:
        max_points: Maximum number of points to retain after subsampling.
        voxel_size: Voxel edge length in meters for downsampling.
        remove_ground: Whether to remove ground plane points.
        ground_threshold: Z-value below which points are considered ground.
    """

    max_points: int = 50000
    voxel_size: float = 0.1
    remove_ground: bool = True
    ground_threshold: float = -1.5


@dataclass
class DatasetConfig:
    """Parsed dataset configuration.

    Attributes:
        kaggle_path: Path to dataset on Kaggle.
        local_path: Path to dataset on local filesystem.
        sessions: All available session date strings.
        train_sessions: Sessions assigned to the training split.
        val_sessions: Sessions assigned to the validation split.
        test_sessions: Sessions assigned to the test split.
        positive_threshold: Distance in meters for positive pair matching.
        negative_threshold: Distance in meters for negative pair matching.
        point_cloud: Point cloud preprocessing parameters.
    """

    kaggle_path: str = ""
    local_path: str = ""
    sessions: list[str] = field(default_factory=list)
    train_sessions: list[str] = field(default_factory=list)
    val_sessions: list[str] = field(default_factory=list)
    test_sessions: list[str] = field(default_factory=list)
    positive_threshold: float = 10.0
    negative_threshold: float = 25.0
    point_cloud: PointCloudConfig = field(default_factory=PointCloudConfig)

    @classmethod
    def from_dict(cls, raw: dict[str, Any]) -> DatasetConfig:
        """Create a DatasetConfig from a raw dictionary.

        Args:
            raw: Dictionary parsed from the 'nclt' key in dataset_config.yaml.

        Returns:
            Populated DatasetConfig instance.
        """
        pc_raw = raw.get("point_cloud", {})
        pc_config = PointCloudConfig(
            max_points=pc_raw.get("max_points", 50000),
            voxel_size=pc_raw.get("voxel_size", 0.1),
            remove_ground=pc_raw.get("remove_ground", True),
            ground_threshold=pc_raw.get("ground_threshold", -1.5),
        )
        return cls(
            kaggle_path=raw.get("kaggle_path", ""),
            local_path=raw.get("local_path", ""),
            sessions=raw.get("sessions", []),
            train_sessions=raw.get("train_sessions", []),
            val_sessions=raw.get("val_sessions", []),
            test_sessions=raw.get("test_sessions", []),
            positive_threshold=raw.get("positive_threshold", 10.0),
            negative_threshold=raw.get("negative_threshold", 25.0),
            point_cloud=pc_config,
        )


class NCLTDataset(Dataset):
    """PyTorch Dataset for the NCLT LiDAR place recognition benchmark.

    Loads Velodyne point clouds, ground-truth poses, and optionally camera
    images from the NCLT preprocessed dataset. Automatically resolves the
    data root by checking the Kaggle path first, then falling back to the
    local path specified in the configuration YAML.

    Args:
        config_path: Path to the dataset configuration YAML file.
        split: One of ``"train"``, ``"val"``, or ``"test"``.
        transform: Optional callable applied to the sample dict before
            returning from ``__getitem__``.

    Raises:
        FileNotFoundError: If neither the Kaggle nor local data path exists.
        ValueError: If an invalid split name is provided.

    Example:
        >>> ds = NCLTDataset("configs/dataset_config.yaml", split="train")
        >>> len(ds)
        12345
        >>> sample = ds[0]
        >>> sample["point_cloud"].shape
        torch.Size([N, 4])
    """

    VALID_SPLITS: tuple[str, ...] = ("train", "val", "test")

    def __init__(
        self,
        config_path: str | Path,
        split: str = "train",
        transform: Callable[[dict[str, Any]], dict[str, Any]] | None = None,
    ) -> None:
        super().__init__()

        if split not in self.VALID_SPLITS:
            raise ValueError(
                f"Invalid split '{split}'. Must be one of {self.VALID_SPLITS}."
            )

        self.split = split
        self.transform = transform
        self.config = self._load_config(config_path)
        self.data_root = self._resolve_data_path()
        self.sessions = self._get_split_sessions()
        self.pc_config = self.config.point_cloud
        self.samples: list[SampleRecord] = []

        self._build_index()

        logger.info(
            "NCLTDataset initialized: split=%s, sessions=%d, samples=%d, root=%s",
            self.split,
            len(self.sessions),
            len(self.samples),
            self.data_root,
        )

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        """Return the total number of samples in the dataset."""
        return len(self.samples)

    def __getitem__(self, idx: int) -> dict[str, Any]:
        """Retrieve a single sample by index.

        Args:
            idx: Integer index in ``[0, len(self))``.

        Returns:
            Dictionary with keys:
                - ``point_cloud``: ``(N, 4)`` float32 tensor (x, y, z, intensity).
                - ``pose``: ``(4, 4)`` float32 SE(3) transformation tensor.
                - ``session``: Session date string.
                - ``timestamp``: Integer timestamp.
                - ``image``: ``(H, W, 3)`` uint8 tensor if an image is found,
                  otherwise this key is absent.

        Raises:
            IndexError: If *idx* is out of range.
        """
        if idx < 0 or idx >= len(self.samples):
            raise IndexError(
                f"Index {idx} out of range for dataset of size {len(self.samples)}."
            )

        record = self.samples[idx]

        # Load point cloud --------------------------------------------------
        raw_pc = self.load_point_cloud(record.velodyne_path)
        raw_pc = self._preprocess_point_cloud(raw_pc)
        pc_tensor = torch.from_numpy(raw_pc).float()

        # Build sample dict --------------------------------------------------
        sample: dict[str, Any] = {
            "point_cloud": pc_tensor,
            "pose": torch.from_numpy(record.pose).float(),
            "session": record.session,
            "timestamp": record.timestamp,
        }

        # Optionally load image ----------------------------------------------
        image = self._find_and_load_image(record)
        if image is not None:
            sample["image"] = torch.from_numpy(image)

        if self.transform is not None:
            sample = self.transform(sample)

        return sample

    def get_session_dataset(self, session: str) -> NCLTDataset:
        """Return a shallow-copy dataset filtered to a single session.

        The returned object shares the same configuration and data root but
        contains only samples belonging to the requested session. This avoids
        re-reading configuration and pose files.

        Args:
            session: Session date string (e.g. ``"2012-01-08"``).

        Returns:
            A new :class:`NCLTDataset` instance containing only the specified
            session's samples.

        Raises:
            ValueError: If *session* is not found in the current split.
        """
        if session not in self.sessions:
            raise ValueError(
                f"Session '{session}' not in current split sessions: {self.sessions}"
            )

        subset = object.__new__(NCLTDataset)
        subset.split = self.split
        subset.transform = self.transform
        subset.config = self.config
        subset.data_root = self.data_root
        subset.sessions = [session]
        subset.pc_config = self.pc_config
        subset.samples = [s for s in self.samples if s.session == session]

        logger.info(
            "Session subset created: session=%s, samples=%d",
            session,
            len(subset.samples),
        )
        return subset

    # ------------------------------------------------------------------
    # I/O helpers (public, usable as standalone utilities)
    # ------------------------------------------------------------------

    @staticmethod
    def load_point_cloud(path: Path) -> np.ndarray:
        """Load a Velodyne binary point cloud file.

        The file is expected to contain ``N`` points stored as contiguous
        ``float32`` values in ``(x, y, z, intensity)`` order.

        Args:
            path: Absolute path to the ``.bin`` file.

        Returns:
            ``(N, 4)`` float32 numpy array.

        Raises:
            FileNotFoundError: If the file does not exist.
            ValueError: If the file size is not a multiple of 16 bytes.
        """
        path = Path(path)
        if not path.exists():
            raise FileNotFoundError(f"Point cloud file not found: {path}")

        file_size = path.stat().st_size
        if file_size == 0:
            logger.warning("Empty point cloud file: %s", path)
            return np.empty((0, 4), dtype=np.float32)

        if file_size % 16 != 0:
            raise ValueError(
                f"Point cloud file size ({file_size} bytes) is not a multiple of "
                f"16 (4 floats x 4 bytes). File may be corrupt: {path}"
            )

        points = np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)
        return points

    @staticmethod
    def load_image(path: Path) -> np.ndarray | None:
        """Load an image from disk.

        Args:
            path: Absolute path to the image file.

        Returns:
            ``(H, W, 3)`` uint8 numpy array in RGB order, or ``None`` if the
            file does not exist or cannot be decoded.
        """
        path = Path(path)
        if not path.exists():
            return None

        try:
            if HAS_CV2:
                img = cv2.imread(str(path), cv2.IMREAD_COLOR)
                if img is None:
                    logger.warning("Failed to decode image: %s", path)
                    return None
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                return img
            else:
                from PIL import Image

                img = Image.open(path).convert("RGB")
                return np.asarray(img)
        except Exception:
            logger.warning("Error reading image: %s", path, exc_info=True)
            return None

    @staticmethod
    def load_poses(session_dir: Path) -> np.ndarray:
        """Parse ``track.csv`` for a session directory.

        Each row of the CSV is expected to contain seven comma-separated
        values: ``timestamp, x, y, z, roll, pitch, yaw``.

        Args:
            session_dir: Path to the session directory containing
                ``track.csv``.

        Returns:
            ``(N, 7)`` float64 numpy array with columns
            ``[timestamp, x, y, z, roll, pitch, yaw]``.

        Raises:
            FileNotFoundError: If ``track.csv`` is not found.
        """
        track_path = Path(session_dir) / "track.csv"
        if not track_path.exists():
            raise FileNotFoundError(f"Track file not found: {track_path}")

        try:
            poses = np.loadtxt(str(track_path), delimiter=",", dtype=np.float64)
        except Exception as exc:
            raise RuntimeError(
                f"Failed to parse track file {track_path}: {exc}"
            ) from exc

        if poses.ndim == 1:
            # Single-row file: reshape to (1, 7).
            poses = poses.reshape(1, -1)

        if poses.shape[1] != 7:
            raise ValueError(
                f"Expected 7 columns in track.csv, got {poses.shape[1]}. "
                f"File: {track_path}"
            )

        logger.debug(
            "Loaded %d poses from %s", poses.shape[0], track_path
        )
        return poses

    @staticmethod
    def pose_to_matrix(
        x: float, y: float, z: float,
        roll: float, pitch: float, yaw: float,
    ) -> np.ndarray:
        """Convert an SE(3) pose to a 4x4 homogeneous transformation matrix.

        Rotation order follows the Tait-Bryan convention (ZYX):
        ``R = Rz(yaw) @ Ry(pitch) @ Rx(roll)``.

        Args:
            x: Translation along the X axis in meters.
            y: Translation along the Y axis in meters.
            z: Translation along the Z axis in meters.
            roll: Rotation about the X axis in radians.
            pitch: Rotation about the Y axis in radians.
            yaw: Rotation about the Z axis in radians.

        Returns:
            ``(4, 4)`` float32 homogeneous transformation matrix.
        """
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        # Rz(yaw) @ Ry(pitch) @ Rx(roll)
        R = np.array(
            [
                [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                [-sp, cp * sr, cp * cr],
            ],
            dtype=np.float32,
        )

        T = np.eye(4, dtype=np.float32)
        T[:3, :3] = R
        T[0, 3] = x
        T[1, 3] = y
        T[2, 3] = z
        return T

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _load_config(config_path: str | Path) -> DatasetConfig:
        """Load and validate the YAML configuration file.

        Args:
            config_path: Path to ``dataset_config.yaml``.

        Returns:
            Parsed :class:`DatasetConfig`.

        Raises:
            FileNotFoundError: If the config file does not exist.
            KeyError: If the ``nclt`` top-level key is missing.
        """
        config_path = Path(config_path)
        if not config_path.exists():
            raise FileNotFoundError(
                f"Dataset config file not found: {config_path}"
            )

        with open(config_path, "r", encoding="utf-8") as fh:
            raw = yaml.safe_load(fh)

        if "nclt" not in raw:
            raise KeyError(
                "Config file is missing the top-level 'nclt' key. "
                f"Found keys: {list(raw.keys())}"
            )

        return DatasetConfig.from_dict(raw["nclt"])

    def _resolve_data_path(self) -> Path:
        """Determine the dataset root directory.

        Checks the Kaggle path first (for notebook execution), then falls
        back to the local path from the configuration.

        Returns:
            Resolved :class:`Path` to the dataset root.

        Raises:
            FileNotFoundError: If neither path exists.
        """
        kaggle_path = Path(self.config.kaggle_path)
        if kaggle_path.exists() and kaggle_path.is_dir():
            logger.info("Using Kaggle data path: %s", kaggle_path)
            return kaggle_path

        local_path = Path(self.config.local_path)
        if local_path.exists() and local_path.is_dir():
            logger.info("Using local data path: %s", local_path)
            return local_path

        raise FileNotFoundError(
            f"Dataset not found at either path.\n"
            f"  Kaggle: {kaggle_path}\n"
            f"  Local:  {local_path}\n"
            "Please download the dataset or update the config."
        )

    def _get_split_sessions(self) -> list[str]:
        """Return session list for the current split.

        Returns:
            List of session date strings.

        Raises:
            ValueError: If the split has no sessions configured.
        """
        mapping: dict[str, list[str]] = {
            "train": self.config.train_sessions,
            "val": self.config.val_sessions,
            "test": self.config.test_sessions,
        }
        sessions = mapping[self.split]
        if not sessions:
            raise ValueError(
                f"No sessions configured for split '{self.split}'."
            )
        return sessions

    def _build_index(self) -> None:
        """Scan the filesystem and build the flat sample index.

        For each session in the current split, loads poses from ``track.csv``
        and matches them to ``.bin`` files in the ``velodyne/`` directory by
        timestamp. Only timestamps that have a corresponding point cloud file
        are included.
        """
        for session in self.sessions:
            session_dir = self.data_root / "sessions" / session

            if not session_dir.exists():
                logger.warning(
                    "Session directory missing, skipping: %s", session_dir
                )
                continue

            # Load poses ---------------------------------------------------
            try:
                poses_array = self.load_poses(session_dir)
            except (FileNotFoundError, RuntimeError, ValueError) as exc:
                logger.warning(
                    "Could not load poses for session %s: %s", session, exc
                )
                continue

            # Build timestamp -> pose lookup --------------------------------
            velodyne_dir = session_dir / "velodyne"
            image_dir = session_dir / "images_small"

            if not velodyne_dir.exists():
                logger.warning(
                    "Velodyne directory missing, skipping session: %s",
                    session,
                )
                continue

            # Create a mapping from timestamp (int) to row index for fast
            # look-up. Timestamps are the first column of track.csv.
            timestamp_to_idx: dict[int, int] = {
                int(poses_array[i, 0]): i for i in range(poses_array.shape[0])
            }

            # Enumerate .bin files and match to poses -----------------------
            bin_files = sorted(velodyne_dir.glob("*.bin"))
            matched = 0

            for bin_path in bin_files:
                try:
                    ts = int(bin_path.stem)
                except ValueError:
                    logger.debug(
                        "Non-numeric bin filename, skipping: %s", bin_path.name
                    )
                    continue

                if ts not in timestamp_to_idx:
                    # Point cloud exists but no matching pose; skip.
                    continue

                row = poses_array[timestamp_to_idx[ts]]
                _, x, y, z, roll, pitch, yaw = row
                pose_matrix = self.pose_to_matrix(x, y, z, roll, pitch, yaw)

                self.samples.append(
                    SampleRecord(
                        session=session,
                        timestamp=ts,
                        velodyne_path=bin_path,
                        image_dir=image_dir,
                        pose=pose_matrix,
                    )
                )
                matched += 1

            logger.info(
                "Session %s: %d/%d point clouds matched to poses",
                session,
                matched,
                len(bin_files),
            )

        if not self.samples:
            logger.warning(
                "No samples found for split '%s'. Check data paths and "
                "session directories.",
                self.split,
            )

    def _preprocess_point_cloud(self, points: np.ndarray) -> np.ndarray:
        """Apply configured preprocessing to a raw point cloud.

        Steps (in order):
            1. Remove ground points (if configured).
            2. Random subsampling to ``max_points`` (if the cloud is larger).

        Voxel-based downsampling is left to the model's collation / transform
        pipeline (e.g. MinkowskiEngine quantization), so it is **not** applied
        here. The ``voxel_size`` parameter is stored for downstream use.

        Args:
            points: ``(N, 4)`` float32 array.

        Returns:
            Preprocessed ``(M, 4)`` float32 array where ``M <= max_points``.
        """
        if points.shape[0] == 0:
            return points

        # 1. Ground removal -------------------------------------------------
        if self.pc_config.remove_ground:
            mask = points[:, 2] > self.pc_config.ground_threshold
            points = points[mask]
            if points.shape[0] == 0:
                logger.debug("All points removed by ground filter.")
                return points

        # 2. Random subsampling ---------------------------------------------
        if points.shape[0] > self.pc_config.max_points:
            indices = np.random.choice(
                points.shape[0], size=self.pc_config.max_points, replace=False
            )
            points = points[indices]

        return points

    def _find_and_load_image(self, record: SampleRecord) -> np.ndarray | None:
        """Attempt to load the camera image matching a sample's timestamp.

        Searches for common image extensions in the session's
        ``images_small/`` directory using the sample's timestamp as the
        filename stem.

        Args:
            record: The sample record to find an image for.

        Returns:
            ``(H, W, 3)`` uint8 RGB array, or ``None`` if not found.
        """
        if not record.image_dir.exists():
            return None

        for ext in (".png", ".jpg", ".jpeg", ".tif", ".tiff"):
            candidate = record.image_dir / f"{record.timestamp}{ext}"
            if candidate.exists():
                return self.load_image(candidate)

        return None

    # ------------------------------------------------------------------
    # Dunder helpers
    # ------------------------------------------------------------------

    def __repr__(self) -> str:
        return (
            f"NCLTDataset(split='{self.split}', sessions={self.sessions}, "
            f"samples={len(self.samples)}, root='{self.data_root}')"
        )
