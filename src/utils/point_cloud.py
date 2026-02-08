"""Utility functions for point cloud processing.

Provides loading, filtering, transformation, and conversion utilities
tailored to the NCLT dataset's Velodyne HDL-32E binary format.
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import TYPE_CHECKING

import numpy as np

if TYPE_CHECKING:
    import open3d as o3d

logger = logging.getLogger(__name__)


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------


def load_velodyne_bin(path: str | Path) -> np.ndarray:
    """Load an NCLT Velodyne binary point cloud file.

    NCLT .bin files store points as contiguous float32 values laid out as
    ``[x, y, z, intensity, x, y, z, intensity, ...]``.  The returned array
    has shape ``(N, 4)``.

    Args:
        path: Filesystem path to the ``.bin`` file.

    Returns:
        Point cloud as an ``(N, 4)`` float32 array with columns
        ``[x, y, z, intensity]``.

    Raises:
        FileNotFoundError: If *path* does not exist.
        ValueError: If the file size is not a multiple of 16 bytes
            (4 floats * 4 bytes) or the file is empty.
    """
    path = Path(path)

    if not path.exists():
        raise FileNotFoundError(f"Point cloud file not found: {path}")

    file_size = path.stat().st_size

    if file_size == 0:
        raise ValueError(f"Point cloud file is empty: {path}")

    if file_size % 16 != 0:
        raise ValueError(
            f"Corrupted point cloud file (size {file_size} bytes is not a "
            f"multiple of 16): {path}"
        )

    try:
        points = np.fromfile(str(path), dtype=np.float32).reshape(-1, 4)
    except Exception as exc:
        raise ValueError(f"Failed to parse point cloud file {path}: {exc}") from exc

    logger.debug("Loaded %d points from %s", len(points), path.name)
    return points


# ---------------------------------------------------------------------------
# Downsampling
# ---------------------------------------------------------------------------


def voxel_downsample(points: np.ndarray, voxel_size: float = 0.1) -> np.ndarray:
    """Reduce point density with a voxel grid filter.

    If Open3D is available the implementation delegates to its optimised
    ``voxel_down_sample``; otherwise a pure-NumPy fallback is used that
    keeps one arbitrary point per voxel cell.

    Args:
        points: Input point cloud of shape ``(N, 3)`` or ``(N, 4)``.
        voxel_size: Edge length of each cubic voxel in metres.  Must be
            positive.

    Returns:
        Down-sampled point cloud with the same number of columns as the
        input.

    Raises:
        ValueError: If *voxel_size* is not positive or *points* has an
            unexpected shape.
    """
    if voxel_size <= 0:
        raise ValueError(f"voxel_size must be positive, got {voxel_size}")

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    try:
        import open3d as o3d  # noqa: F811

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
        pcd_down = pcd.voxel_down_sample(voxel_size)
        indices = _closest_point_indices(points[:, :3], np.asarray(pcd_down.points))
        result = points[indices]
        logger.debug(
            "Voxel downsample (Open3D): %d -> %d points (voxel=%.3f m)",
            len(points),
            len(result),
            voxel_size,
        )
        return result
    except ImportError:
        logger.debug("Open3D not available; using NumPy voxel downsample fallback")

    # NumPy fallback: quantise coordinates to voxel indices, keep first per voxel.
    voxel_indices = np.floor(points[:, :3] / voxel_size).astype(np.int64)
    _, unique_idx = np.unique(voxel_indices, axis=0, return_index=True)
    unique_idx.sort()  # preserve original ordering
    result = points[unique_idx]
    logger.debug(
        "Voxel downsample (NumPy): %d -> %d points (voxel=%.3f m)",
        len(points),
        len(result),
        voxel_size,
    )
    return result


def _closest_point_indices(
    source: np.ndarray, targets: np.ndarray
) -> np.ndarray:
    """Return indices into *source* of the nearest neighbour for each *target*.

    Used internally to map Open3D voxel-centre outputs back to original
    points so that extra columns (e.g. intensity) are preserved.
    """
    from scipy.spatial import cKDTree

    tree = cKDTree(source)
    _, indices = tree.query(targets)
    return np.unique(indices)


# ---------------------------------------------------------------------------
# Ground removal
# ---------------------------------------------------------------------------


def remove_ground_plane(
    points: np.ndarray,
    threshold: float = -1.5,
    method: str = "height",
) -> np.ndarray:
    """Remove ground-plane points from a point cloud.

    Args:
        points: Input cloud of shape ``(N, 3)`` or ``(N, 4)``.
        threshold: Interpretation depends on *method*:

            * ``"height"``: points with ``z < threshold`` are considered
              ground and removed.
            * ``"ransac"``: maximum distance (metres) from the fitted
              plane for a point to be classified as ground.
        method: ``"height"`` for a simple z-threshold filter, or
            ``"ransac"`` for RANSAC plane fitting via Open3D.

    Returns:
        Filtered point cloud with ground points removed, same number of
        columns as input.

    Raises:
        ValueError: If *method* is not one of the supported values.
        ImportError: If *method* is ``"ransac"`` and Open3D is not
            installed.
    """
    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    if method == "height":
        mask = points[:, 2] >= threshold
        result = points[mask]
        logger.debug(
            "Ground removal (height z>=%.2f): %d -> %d points",
            threshold,
            len(points),
            len(result),
        )
        return result

    if method == "ransac":
        try:
            import open3d as o3d  # noqa: F811
        except ImportError as exc:
            raise ImportError(
                "Open3D is required for RANSAC ground removal but is not installed."
            ) from exc

        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

        # RANSAC plane segmentation.  The ground plane in typical outdoor
        # scans is the dominant plane, so a generous number of iterations
        # and a reasonable distance threshold usually suffice.
        plane_model, inlier_indices = pcd.segment_plane(
            distance_threshold=threshold if threshold > 0 else 0.2,
            ransac_n=3,
            num_iterations=1000,
        )
        a, b, c, d = plane_model
        logger.debug(
            "RANSAC ground plane: %.3fx + %.3fy + %.3fz + %.3f = 0 "
            "(%d inliers)",
            a,
            b,
            c,
            d,
            len(inlier_indices),
        )

        inlier_set = set(inlier_indices)
        mask = np.array(
            [i not in inlier_set for i in range(len(points))], dtype=bool
        )
        result = points[mask]
        logger.debug(
            "Ground removal (RANSAC dist=%.3f): %d -> %d points",
            threshold,
            len(points),
            len(result),
        )
        return result

    raise ValueError(
        f"Unknown ground removal method '{method}'. "
        f"Supported: 'height', 'ransac'."
    )


# ---------------------------------------------------------------------------
# Transformation
# ---------------------------------------------------------------------------


def transform_point_cloud(
    points: np.ndarray, transform: np.ndarray
) -> np.ndarray:
    """Apply a rigid-body (SE3) transformation to a point cloud.

    Args:
        points: Point cloud of shape ``(N, 3)`` or ``(N, 4)``.  When four
            columns are present the fourth column (typically intensity) is
            preserved unchanged.
        transform: A ``(4, 4)`` homogeneous transformation matrix.

    Returns:
        Transformed point cloud with the same shape as *points*.

    Raises:
        ValueError: If *transform* is not ``(4, 4)`` or *points* has an
            unexpected shape.
    """
    if transform.shape != (4, 4):
        raise ValueError(
            f"Transform must be (4, 4), got {transform.shape}"
        )

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    xyz = points[:, :3]
    # R @ p + t  for each point (vectorised).
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    xyz_transformed = (rotation @ xyz.T).T + translation

    if points.shape[1] == 4:
        result = np.column_stack([xyz_transformed, points[:, 3]])
    else:
        result = xyz_transformed

    return result.astype(points.dtype)


# ---------------------------------------------------------------------------
# Normal estimation
# ---------------------------------------------------------------------------


def compute_normals(
    points: np.ndarray, k_neighbors: int = 20
) -> np.ndarray:
    """Estimate surface normals for each point using Open3D.

    Args:
        points: Point cloud of shape ``(N, 3)`` or ``(N, 4)``.  Only the
            first three columns (xyz) are used.
        k_neighbors: Number of nearest neighbours used to fit local
            planes for normal estimation.

    Returns:
        An ``(N, 3)`` float64 array of unit normals, one per input point.

    Raises:
        ImportError: If Open3D is not installed.
        ValueError: If *points* has an unexpected shape or fewer than 3
            points.
    """
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for normal estimation but is not installed."
        ) from exc

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) < 3:
        raise ValueError(
            f"Need at least 3 points for normal estimation, got {len(points)}"
        )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))
    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamKNN(knn=k_neighbors)
    )
    normals = np.asarray(pcd.normals)

    logger.debug("Computed normals for %d points (k=%d)", len(points), k_neighbors)
    return normals


# ---------------------------------------------------------------------------
# Cropping
# ---------------------------------------------------------------------------


def crop_point_cloud(
    points: np.ndarray,
    min_bound: np.ndarray,
    max_bound: np.ndarray,
) -> np.ndarray:
    """Crop a point cloud to an axis-aligned bounding box.

    Points whose xyz coordinates satisfy
    ``min_bound <= point <= max_bound`` (element-wise) are retained.

    Args:
        points: Point cloud of shape ``(N, 3)`` or ``(N, 4)``.
        min_bound: Lower corner of the bounding box, shape ``(3,)``.
        max_bound: Upper corner of the bounding box, shape ``(3,)``.

    Returns:
        Cropped point cloud with the same number of columns as *points*.

    Raises:
        ValueError: If bounds have wrong shapes or *min_bound* exceeds
            *max_bound* in any dimension.
    """
    min_bound = np.asarray(min_bound, dtype=np.float64).ravel()
    max_bound = np.asarray(max_bound, dtype=np.float64).ravel()

    if min_bound.shape != (3,) or max_bound.shape != (3,):
        raise ValueError(
            f"Bounds must be length-3 vectors, got min={min_bound.shape}, "
            f"max={max_bound.shape}"
        )

    if np.any(min_bound > max_bound):
        raise ValueError(
            f"min_bound must be <= max_bound element-wise. "
            f"min={min_bound}, max={max_bound}"
        )

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    if len(points) == 0:
        return points.copy()

    xyz = points[:, :3]
    mask = np.all((xyz >= min_bound) & (xyz <= max_bound), axis=1)
    result = points[mask]

    logger.debug(
        "Crop: %d -> %d points (bounds min=%s max=%s)",
        len(points),
        len(result),
        min_bound,
        max_bound,
    )
    return result


# ---------------------------------------------------------------------------
# Random subsampling
# ---------------------------------------------------------------------------


def random_subsample(
    points: np.ndarray, max_points: int = 50_000
) -> np.ndarray:
    """Randomly subsample a point cloud if it exceeds *max_points*.

    If the cloud already has *max_points* or fewer points it is returned
    unchanged (as a copy).

    Args:
        points: Point cloud of shape ``(N, C)`` with arbitrary columns.
        max_points: Maximum number of points to keep.  Must be positive.

    Returns:
        Sub-sampled point cloud with at most *max_points* rows.

    Raises:
        ValueError: If *max_points* is not positive.
    """
    if max_points <= 0:
        raise ValueError(f"max_points must be positive, got {max_points}")

    if len(points) <= max_points:
        return points.copy()

    rng = np.random.default_rng()
    indices = rng.choice(len(points), size=max_points, replace=False)
    indices.sort()  # preserve original ordering
    result = points[indices]

    logger.debug(
        "Random subsample: %d -> %d points", len(points), len(result)
    )
    return result


# ---------------------------------------------------------------------------
# Open3D conversions
# ---------------------------------------------------------------------------


def to_open3d(points: np.ndarray) -> "o3d.geometry.PointCloud":
    """Convert a NumPy point cloud to an Open3D ``PointCloud``.

    Args:
        points: Array of shape ``(N, 3)`` or ``(N, 4)``.  Only the first
            three columns are used as point positions.

    Returns:
        An ``open3d.geometry.PointCloud`` instance.

    Raises:
        ImportError: If Open3D is not installed.
        ValueError: If *points* has an unexpected shape.
    """
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for point cloud conversion but is not "
            "installed."
        ) from exc

    if points.ndim != 2 or points.shape[1] not in (3, 4):
        raise ValueError(
            f"Expected points of shape (N, 3) or (N, 4), got {points.shape}"
        )

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points[:, :3].astype(np.float64))

    logger.debug("Converted %d points to Open3D PointCloud", len(points))
    return pcd


def from_open3d(pcd: "o3d.geometry.PointCloud") -> np.ndarray:
    """Convert an Open3D ``PointCloud`` back to a NumPy array.

    Args:
        pcd: An ``open3d.geometry.PointCloud`` instance.

    Returns:
        An ``(N, 3)`` float64 array of point coordinates.

    Raises:
        ImportError: If Open3D is not installed.
        TypeError: If *pcd* is not an ``open3d.geometry.PointCloud``.
    """
    try:
        import open3d as o3d  # noqa: F811
    except ImportError as exc:
        raise ImportError(
            "Open3D is required for point cloud conversion but is not "
            "installed."
        ) from exc

    if not isinstance(pcd, o3d.geometry.PointCloud):
        raise TypeError(
            f"Expected open3d.geometry.PointCloud, got {type(pcd).__name__}"
        )

    points = np.asarray(pcd.points)
    logger.debug("Converted Open3D PointCloud to %d points", len(points))
    return points
