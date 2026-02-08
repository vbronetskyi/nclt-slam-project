"""NCLTPairsDataset for place recognition training on the NCLT dataset.

Loads point cloud pairs (anchor, positive, negatives) from the Kaggle-preprocessed
NCLT dataset. Supports both pre-annotated CSV pair files and on-the-fly pair
generation from pose distance thresholds.

Typical usage::

    from src.datasets.nclt_pairs import NCLTPairsDataset, pairs_collate_fn

    dataset = NCLTPairsDataset("configs/dataset_config.yaml", split="train")
    loader = DataLoader(dataset, batch_size=32, collate_fn=pairs_collate_fn)
"""

from __future__ import annotations

import csv
import logging
from pathlib import Path
from typing import Any, Callable

import numpy as np
import torch
import yaml
from scipy.spatial import KDTree
from torch.utils.data import Dataset

logger = logging.getLogger(__name__)

# Number of float32 fields per point in the NCLT velodyne binary format.
# Each point is stored as (x, y, z, intensity, ring).
_VELODYNE_POINT_FIELDS = 5
_BYTES_PER_FLOAT32 = 4


def load_point_cloud(path: Path) -> np.ndarray:
    """Load a binary velodyne point cloud file.

    The NCLT preprocessed .bin files store points as contiguous float32 values
    with 5 fields per point: (x, y, z, intensity, ring). Only the (x, y, z)
    coordinates are returned.

    Args:
        path: Absolute or relative path to a ``.bin`` point cloud file.

    Returns:
        An (N, 3) float32 array of XYZ coordinates.

    Raises:
        FileNotFoundError: If the file does not exist.
        ValueError: If the file size is not a multiple of the expected point
            stride.
    """
    path = Path(path)
    if not path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {path}")

    file_size = path.stat().st_size
    point_stride = _VELODYNE_POINT_FIELDS * _BYTES_PER_FLOAT32
    if file_size % point_stride != 0:
        raise ValueError(
            f"File size ({file_size} bytes) is not a multiple of point stride "
            f"({point_stride} bytes): {path}"
        )

    points = np.fromfile(str(path), dtype=np.float32)
    points = points.reshape(-1, _VELODYNE_POINT_FIELDS)
    return points[:, :3].copy()


def pairs_collate_fn(batch: list[dict[str, Any]]) -> dict[str, Any]:
    """Custom collate function for :class:`NCLTPairsDataset`.

    Stacks anchors and positives into batched tensors and collects negatives
    into a nested list (since the number of negatives can vary per sample).

    Args:
        batch: A list of sample dicts as returned by
            :meth:`NCLTPairsDataset.__getitem__`.

    Returns:
        A dict with the following keys:

        - ``anchors``: ``(B, N, 3)`` float tensor of anchor point clouds.
        - ``positives``: ``(B, N, 3)`` float tensor of positive point clouds.
        - ``negatives``: List of length B, each element a list of ``(N, 3)``
          float tensors (one per negative sample).
        - ``anchor_poses``: ``(B, 7)`` float tensor (x, y, z, roll, pitch, yaw,
          timestamp).
        - ``positive_poses``: ``(B, 7)`` float tensor.
    """
    anchors = torch.stack([s["anchor"] for s in batch], dim=0)
    positives = torch.stack([s["positive"] for s in batch], dim=0)
    negatives = [s["negatives"] for s in batch]
    anchor_poses = torch.stack([s["anchor_pose"] for s in batch], dim=0)
    positive_poses = torch.stack([s["positive_pose"] for s in batch], dim=0)

    return {
        "anchors": anchors,
        "positives": positives,
        "negatives": negatives,
        "anchor_poses": anchor_poses,
        "positive_poses": positive_poses,
    }


class NCLTPairsDataset(Dataset):
    """PyTorch dataset that yields (anchor, positive, negatives) tuples.

    Each sample consists of an anchor point cloud, a positive point cloud
    (within ``positive_threshold`` meters), and a configurable number of
    negative point clouds (farther than ``negative_threshold`` meters).

    Args:
        config_path: Path to the YAML dataset configuration file.
        split: One of ``"train"``, ``"val"``, or ``"test"``.
        transform: Optional callable applied to each raw point cloud
            (as an ``np.ndarray``) before conversion to a tensor.
        num_negatives: Number of negative examples per anchor.
    """

    _VALID_SPLITS = ("train", "val", "test")

    def __init__(
        self,
        config_path: str | Path,
        split: str = "train",
        transform: Callable[[np.ndarray], np.ndarray] | None = None,
        num_negatives: int = 5,
    ) -> None:
        if split not in self._VALID_SPLITS:
            raise ValueError(
                f"Invalid split '{split}'. Must be one of {self._VALID_SPLITS}."
            )

        self.split = split
        self.transform = transform
        self.num_negatives = num_negatives

        # ---- Load configuration ------------------------------------------------
        config_path = Path(config_path)
        if not config_path.exists():
            raise FileNotFoundError(f"Config file not found: {config_path}")

        with open(config_path, "r") as f:
            full_config = yaml.safe_load(f)

        self.config: dict[str, Any] = full_config["nclt"]
        self.positive_threshold: float = float(
            self.config["positive_threshold"]
        )
        self.negative_threshold: float = float(
            self.config["negative_threshold"]
        )
        self.max_points: int = int(
            self.config.get("point_cloud", {}).get("max_points", 50_000)
        )

        # ---- Resolve data root -------------------------------------------------
        self.data_root = self._resolve_data_root()
        logger.info("Using data root: %s", self.data_root)

        # ---- Determine sessions for this split ---------------------------------
        session_key = f"{split}_sessions"
        self.sessions: list[str] = self.config.get(
            session_key, self.config.get("sessions", [])
        )
        if not self.sessions:
            raise ValueError(
                f"No sessions found for split '{split}' in config."
            )
        logger.info(
            "Split '%s' — %d session(s): %s",
            split,
            len(self.sessions),
            ", ".join(self.sessions),
        )

        # ---- Load poses for each session ---------------------------------------
        self.poses: dict[str, list[dict[str, Any]]] = {}
        for session in self.sessions:
            self.poses[session] = self._load_session_poses(session)
        total_poses = sum(len(v) for v in self.poses.values())
        logger.info("Loaded %d total poses across all sessions.", total_poses)

        # ---- Load or generate pairs --------------------------------------------
        csv_path = self.data_root / f"{split}.csv"
        if csv_path.exists():
            logger.info("Loading pre-annotated pairs from %s", csv_path)
            self.pairs = self._load_pairs_from_csv(csv_path)
        else:
            logger.info(
                "Pair CSV not found at %s — generating pairs from poses.",
                csv_path,
            )
            self.pairs = self._generate_pairs(self.poses, self.sessions)

        logger.info(
            "NCLTPairsDataset [%s]: %d pairs ready.", split, len(self.pairs)
        )

    # ------------------------------------------------------------------
    # Public interface
    # ------------------------------------------------------------------

    def __len__(self) -> int:
        return len(self.pairs)

    def __getitem__(self, idx: int) -> dict[str, Any]:
        """Return a single training sample.

        Returns:
            A dict with keys ``anchor``, ``positive``, ``negatives``,
            ``anchor_pose``, and ``positive_pose``.
        """
        anchor_info, positive_info, negative_infos = self.pairs[idx]

        anchor_pc = self._prepare_point_cloud(anchor_info["path"])
        positive_pc = self._prepare_point_cloud(positive_info["path"])

        negative_pcs: list[torch.Tensor] = []
        for neg in negative_infos:
            negative_pcs.append(self._prepare_point_cloud(neg["path"]))

        anchor_pose = torch.tensor(
            [
                anchor_info["x"],
                anchor_info["y"],
                anchor_info["z"],
                anchor_info["roll"],
                anchor_info["pitch"],
                anchor_info["yaw"],
                anchor_info["timestamp"],
            ],
            dtype=torch.float64,
        )
        positive_pose = torch.tensor(
            [
                positive_info["x"],
                positive_info["y"],
                positive_info["z"],
                positive_info["roll"],
                positive_info["pitch"],
                positive_info["yaw"],
                positive_info["timestamp"],
            ],
            dtype=torch.float64,
        )

        return {
            "anchor": anchor_pc,
            "positive": positive_pc,
            "negatives": negative_pcs,
            "anchor_pose": anchor_pose,
            "positive_pose": positive_pose,
        }

    # ------------------------------------------------------------------
    # Pair loading / generation
    # ------------------------------------------------------------------

    def _load_pairs_from_csv(self, csv_path: Path) -> list[tuple]:
        """Parse a CSV file of pre-annotated place recognition pairs.

        Expected CSV columns::

            anchor_session, anchor_timestamp,
            positive_session, positive_timestamp,
            negative_sessions, negative_timestamps

        Negative columns contain semicolon-separated lists.

        Args:
            csv_path: Path to the CSV pair annotations file.

        Returns:
            A list of ``(anchor_info, positive_info, [neg_infos])`` tuples.
        """
        pairs: list[tuple] = []

        with open(csv_path, "r", newline="") as f:
            reader = csv.DictReader(f)
            for row_idx, row in enumerate(reader):
                try:
                    anchor = self._resolve_pose(
                        row["anchor_session"].strip(),
                        int(row["anchor_timestamp"].strip()),
                    )
                    positive = self._resolve_pose(
                        row["positive_session"].strip(),
                        int(row["positive_timestamp"].strip()),
                    )

                    negatives: list[dict[str, Any]] = []
                    neg_sessions = row["negative_sessions"].strip().split(";")
                    neg_timestamps = (
                        row["negative_timestamps"].strip().split(";")
                    )

                    for ns, nt in zip(neg_sessions, neg_timestamps):
                        ns, nt = ns.strip(), nt.strip()
                        if ns and nt:
                            negatives.append(
                                self._resolve_pose(ns, int(nt))
                            )

                    # Pad or truncate negatives to match num_negatives.
                    negatives = self._adjust_negatives(
                        negatives, anchor, positive
                    )
                    pairs.append((anchor, positive, negatives))

                except (KeyError, ValueError) as exc:
                    logger.warning(
                        "Skipping malformed row %d in %s: %s",
                        row_idx,
                        csv_path,
                        exc,
                    )

        return pairs

    def _generate_pairs(
        self,
        poses: dict[str, list[dict[str, Any]]],
        sessions: list[str],
    ) -> list[tuple]:
        """Generate positive / negative pairs from pose distances.

        For every anchor pose, a positive is the closest pose (in a different
        scan) that is within ``positive_threshold`` meters, and negatives are
        randomly sampled from poses farther than ``negative_threshold`` meters.

        Args:
            poses: Mapping of session name to list of pose dicts.
            sessions: Ordered list of session names to use.

        Returns:
            A list of ``(anchor_info, positive_info, [neg_infos])`` tuples.
        """
        # Flatten all poses into a single list with session labels.
        all_poses: list[dict[str, Any]] = []
        for session in sessions:
            all_poses.extend(poses[session])

        if len(all_poses) < 2:
            logger.warning("Not enough poses to generate pairs.")
            return []

        coords = np.array([[p["x"], p["y"], p["z"]] for p in all_poses])
        tree = KDTree(coords)

        pairs: list[tuple] = []
        rng = np.random.default_rng(seed=42)

        for anchor_idx, anchor_pose in enumerate(all_poses):
            anchor_xy = coords[anchor_idx]

            # --- Find positives (within positive_threshold) ----------------
            positive_indices = tree.query_ball_point(
                anchor_xy, r=self.positive_threshold
            )
            # Exclude the anchor itself.
            positive_indices = [
                i for i in positive_indices if i != anchor_idx
            ]
            if not positive_indices:
                continue

            # Pick the closest positive.
            dists = np.linalg.norm(
                coords[positive_indices] - anchor_xy, axis=1
            )
            best_pos_idx = positive_indices[int(np.argmin(dists))]

            # --- Find negatives (beyond negative_threshold) ----------------
            all_dists = np.linalg.norm(coords - anchor_xy, axis=1)
            negative_mask = all_dists > self.negative_threshold
            negative_indices = np.where(negative_mask)[0]

            if len(negative_indices) < self.num_negatives:
                # Not enough negatives; skip this anchor.
                continue

            chosen_neg_indices = rng.choice(
                negative_indices, size=self.num_negatives, replace=False
            )

            pairs.append(
                (
                    all_poses[anchor_idx],
                    all_poses[best_pos_idx],
                    [all_poses[int(ni)] for ni in chosen_neg_indices],
                )
            )

        logger.info("Generated %d pairs from %d poses.", len(pairs), len(all_poses))
        return pairs

    def _mine_hard_negatives(
        self,
        anchor_descriptor: np.ndarray,
        candidates: list[dict[str, Any]],
        k: int,
    ) -> list[dict[str, Any]]:
        """Select the hardest negatives from a candidate pool.

        Hard negatives are candidates that are closest to the anchor in
        descriptor (embedding) space while being far away in physical space
        (i.e., already confirmed as negatives by the distance threshold).

        Args:
            anchor_descriptor: 1-D feature vector for the anchor, shape
                ``(D,)``.
            candidates: List of candidate negative pose dicts. Each must
                contain a ``"descriptor"`` key with an ``np.ndarray`` of
                shape ``(D,)``.
            k: Number of hard negatives to return.

        Returns:
            A list of up to *k* candidate dicts, ordered from hardest
            (smallest descriptor distance) to easiest.
        """
        if not candidates:
            return []

        descriptors = np.stack([c["descriptor"] for c in candidates], axis=0)
        distances = np.linalg.norm(
            descriptors - anchor_descriptor[np.newaxis, :], axis=1
        )

        # Ascending order: smallest distance = hardest negative.
        sorted_indices = np.argsort(distances)
        selected = [candidates[int(i)] for i in sorted_indices[:k]]

        logger.debug(
            "Mined %d hard negatives (descriptor dist range: %.4f–%.4f).",
            len(selected),
            float(distances[sorted_indices[0]]),
            float(distances[sorted_indices[min(k, len(sorted_indices)) - 1]]),
        )
        return selected

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _resolve_data_root(self) -> Path:
        """Return the first existing data root from the config.

        Checks ``kaggle_path`` first (for Kaggle notebook environments), then
        falls back to ``local_path``.

        Returns:
            Resolved :class:`Path` to the dataset root.

        Raises:
            FileNotFoundError: If neither path exists.
        """
        for key in ("kaggle_path", "local_path"):
            candidate = Path(self.config.get(key, ""))
            if candidate.exists():
                return candidate.resolve()

        raise FileNotFoundError(
            "Dataset root not found. Checked: "
            f"kaggle_path={self.config.get('kaggle_path')}, "
            f"local_path={self.config.get('local_path')}"
        )

    def _load_session_poses(
        self, session: str
    ) -> list[dict[str, Any]]:
        """Load poses from a session's ``track.csv``.

        Args:
            session: Session date string, e.g. ``"2012-01-08"``.

        Returns:
            List of pose dicts with keys ``timestamp``, ``x``, ``y``, ``z``,
            ``roll``, ``pitch``, ``yaw``, ``session``, and ``path``.
        """
        track_path = self.data_root / session / "track.csv"
        if not track_path.exists():
            logger.warning("Track file not found: %s", track_path)
            return []

        poses: list[dict[str, Any]] = []
        velodyne_dir = self.data_root / session / "velodyne"

        with open(track_path, "r", newline="") as f:
            reader = csv.reader(f)
            header = next(reader, None)

            # Determine column indices. Accept both ordered and named headers.
            if header and not header[0].replace(".", "", 1).lstrip("-").isdigit():
                # Named header row.
                col_map = {name.strip().lower(): i for i, name in enumerate(header)}
                idx_ts = col_map.get("timestamp", 0)
                idx_x = col_map.get("x", 1)
                idx_y = col_map.get("y", 2)
                idx_z = col_map.get("z", 3)
                idx_roll = col_map.get("roll", 4)
                idx_pitch = col_map.get("pitch", 5)
                idx_yaw = col_map.get("yaw", 6)
            else:
                # No header — first row is data; rewind is not possible with
                # csv.reader, so we parse the first row as data below.
                idx_ts, idx_x, idx_y, idx_z = 0, 1, 2, 3
                idx_roll, idx_pitch, idx_yaw = 4, 5, 6
                if header:
                    # Re-process the first data row.
                    poses.append(
                        self._parse_pose_row(
                            header,
                            idx_ts, idx_x, idx_y, idx_z,
                            idx_roll, idx_pitch, idx_yaw,
                            session, velodyne_dir,
                        )
                    )

            for row in reader:
                if not row or not row[0].strip():
                    continue
                pose = self._parse_pose_row(
                    row,
                    idx_ts, idx_x, idx_y, idx_z,
                    idx_roll, idx_pitch, idx_yaw,
                    session, velodyne_dir,
                )
                if pose is not None:
                    poses.append(pose)

        logger.debug(
            "Session %s: loaded %d poses from %s.", session, len(poses), track_path
        )
        return poses

    @staticmethod
    def _parse_pose_row(
        row: list[str],
        idx_ts: int,
        idx_x: int,
        idx_y: int,
        idx_z: int,
        idx_roll: int,
        idx_pitch: int,
        idx_yaw: int,
        session: str,
        velodyne_dir: Path,
    ) -> dict[str, Any] | None:
        """Parse a single row of ``track.csv`` into a pose dict."""
        try:
            timestamp = int(row[idx_ts].strip())
            return {
                "timestamp": timestamp,
                "x": float(row[idx_x]),
                "y": float(row[idx_y]),
                "z": float(row[idx_z]),
                "roll": float(row[idx_roll]),
                "pitch": float(row[idx_pitch]),
                "yaw": float(row[idx_yaw]),
                "session": session,
                "path": velodyne_dir / f"{timestamp}.bin",
            }
        except (IndexError, ValueError) as exc:
            logger.debug("Skipping malformed track row: %s (%s)", row, exc)
            return None

    def _resolve_pose(
        self, session: str, timestamp: int
    ) -> dict[str, Any]:
        """Look up a pose dict by session and timestamp.

        Falls back to nearest-timestamp matching if an exact match is not
        found.

        Args:
            session: Session date string.
            timestamp: Integer timestamp.

        Returns:
            The matching pose dict.

        Raises:
            ValueError: If the session has no loaded poses.
        """
        session_poses = self.poses.get(session, [])
        if not session_poses:
            raise ValueError(f"No poses loaded for session '{session}'.")

        # Exact match (fast path).
        for pose in session_poses:
            if pose["timestamp"] == timestamp:
                return pose

        # Nearest timestamp (fallback).
        timestamps = np.array([p["timestamp"] for p in session_poses])
        nearest_idx = int(np.argmin(np.abs(timestamps - timestamp)))
        logger.debug(
            "Exact timestamp %d not found in session %s; using nearest (%d).",
            timestamp,
            session,
            session_poses[nearest_idx]["timestamp"],
        )
        return session_poses[nearest_idx]

    def _adjust_negatives(
        self,
        negatives: list[dict[str, Any]],
        anchor: dict[str, Any],
        positive: dict[str, Any],
    ) -> list[dict[str, Any]]:
        """Pad or truncate the negative list to ``self.num_negatives``.

        If there are fewer negatives than required, additional negatives are
        sampled from all loaded poses that satisfy the distance threshold.

        Args:
            negatives: Current list of negative pose dicts.
            anchor: Anchor pose dict.
            positive: Positive pose dict (unused but available for future
                filtering).

        Returns:
            A list of exactly ``self.num_negatives`` negative pose dicts.
        """
        if len(negatives) >= self.num_negatives:
            return negatives[: self.num_negatives]

        # Need more negatives; sample from all poses.
        anchor_xy = np.array([anchor["x"], anchor["y"], anchor["z"]])
        existing_timestamps = {n["timestamp"] for n in negatives}
        rng = np.random.default_rng()

        candidates: list[dict[str, Any]] = []
        for session_poses in self.poses.values():
            for pose in session_poses:
                if pose["timestamp"] in existing_timestamps:
                    continue
                dist = float(
                    np.linalg.norm(
                        np.array([pose["x"], pose["y"], pose["z"]]) - anchor_xy
                    )
                )
                if dist > self.negative_threshold:
                    candidates.append(pose)

        needed = self.num_negatives - len(negatives)
        if candidates:
            chosen_indices = rng.choice(
                len(candidates),
                size=min(needed, len(candidates)),
                replace=False,
            )
            negatives.extend([candidates[int(i)] for i in chosen_indices])

        # If still short, duplicate existing negatives to fill.
        while len(negatives) < self.num_negatives and negatives:
            negatives.append(negatives[len(negatives) % len(negatives)])

        return negatives[: self.num_negatives]

    def _prepare_point_cloud(self, path: Path) -> torch.Tensor:
        """Load, transform, and pad/truncate a point cloud to a fixed size.

        Args:
            path: Path to the ``.bin`` file.

        Returns:
            A float32 tensor of shape ``(max_points, 3)``.
        """
        points = load_point_cloud(path)

        if self.transform is not None:
            points = self.transform(points)

        num_points = points.shape[0]
        if num_points > self.max_points:
            # Deterministic subsampling via stride for reproducibility during
            # evaluation; random subsampling during training.
            if self.split == "train":
                indices = np.random.choice(
                    num_points, size=self.max_points, replace=False
                )
            else:
                step = num_points / self.max_points
                indices = np.round(np.arange(self.max_points) * step).astype(
                    np.int64
                )
            points = points[indices]
        elif num_points < self.max_points:
            # Pad with zeros.
            padding = np.zeros(
                (self.max_points - num_points, 3), dtype=np.float32
            )
            points = np.concatenate([points, padding], axis=0)

        return torch.from_numpy(points.astype(np.float32))
