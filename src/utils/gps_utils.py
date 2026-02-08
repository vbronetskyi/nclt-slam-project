"""GPS coordinate conversion utilities for the NCLT dataset.

Provides geodetic-to-local coordinate transforms (LLA -> ECEF -> ENU),
trajectory pose construction, and GPS data parsing for the Trimble GPS
data recorded on the University of Michigan North Campus.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# ---------------------------------------------------------------------------
# WGS-84 ellipsoid constants
# ---------------------------------------------------------------------------
WGS84_A = 6378137.0                        # semi-major axis (m)
WGS84_F = 1 / 298.257223563                # flattening
WGS84_E2 = 2 * WGS84_F - WGS84_F ** 2     # first eccentricity squared

# Mean Earth radius for haversine (m)
_EARTH_RADIUS_M = 6371000.0

# ---------------------------------------------------------------------------
# Default reference point for NCLT (University of Michigan North Campus)
# ---------------------------------------------------------------------------
NCLT_REF_LAT = 42.293195    # degrees
NCLT_REF_LON = -83.709657   # degrees
NCLT_REF_ALT = 270.0        # meters above WGS-84 ellipsoid


# ---------------------------------------------------------------------------
# Coordinate conversions
# ---------------------------------------------------------------------------

def lla_to_ecef(lat: float, lon: float, alt: float) -> np.ndarray:
    """Convert geodetic coordinates to Earth-Centered Earth-Fixed (ECEF).

    Uses the WGS-84 reference ellipsoid.

    Args:
        lat: Geodetic latitude in degrees.
        lon: Geodetic longitude in degrees.
        alt: Altitude above the WGS-84 ellipsoid in meters.

    Returns:
        NumPy array of shape (3,) containing [X, Y, Z] in meters.
    """
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    # Radius of curvature in the prime vertical
    N = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat ** 2)

    x = (N + alt) * cos_lat * cos_lon
    y = (N + alt) * cos_lat * sin_lon
    z = (N * (1.0 - WGS84_E2) + alt) * sin_lat

    return np.array([x, y, z], dtype=np.float64)


def ecef_to_enu(
    x: float,
    y: float,
    z: float,
    lat_ref: float,
    lon_ref: float,
    alt_ref: float,
) -> np.ndarray:
    """Convert ECEF coordinates to local East-North-Up (ENU).

    The ENU frame is defined by a reference origin given in geodetic
    coordinates.

    Args:
        x: ECEF X coordinate in meters.
        y: ECEF Y coordinate in meters.
        z: ECEF Z coordinate in meters.
        lat_ref: Reference latitude in degrees.
        lon_ref: Reference longitude in degrees.
        alt_ref: Reference altitude in meters.

    Returns:
        NumPy array of shape (3,) containing [East, North, Up] in meters.
    """
    # ECEF of reference origin
    ref_ecef = lla_to_ecef(lat_ref, lon_ref, alt_ref)
    dx = x - ref_ecef[0]
    dy = y - ref_ecef[1]
    dz = z - ref_ecef[2]

    lat_rad = np.radians(lat_ref)
    lon_rad = np.radians(lon_ref)

    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    # Rotation matrix from ECEF to ENU
    east  = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up    =  cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz

    return np.array([east, north, up], dtype=np.float64)


def lla_to_enu(
    lat: np.ndarray,
    lon: np.ndarray,
    alt: np.ndarray,
    lat_ref: float | None = None,
    lon_ref: float | None = None,
    alt_ref: float | None = None,
) -> np.ndarray:
    """Batch convert geodetic (LLA) coordinates to local ENU.

    If no reference origin is supplied, the first point in the arrays is
    used as the origin.

    Args:
        lat: Latitudes in degrees, shape (N,).
        lon: Longitudes in degrees, shape (N,).
        alt: Altitudes in meters, shape (N,).
        lat_ref: Reference latitude in degrees (default: first point).
        lon_ref: Reference longitude in degrees (default: first point).
        alt_ref: Reference altitude in meters (default: first point).

    Returns:
        NumPy array of shape (N, 3) with columns [East, North, Up] in
        meters.
    """
    lat = np.asarray(lat, dtype=np.float64).ravel()
    lon = np.asarray(lon, dtype=np.float64).ravel()
    alt = np.asarray(alt, dtype=np.float64).ravel()

    n = len(lat)
    if n == 0:
        return np.empty((0, 3), dtype=np.float64)

    if lat_ref is None:
        lat_ref = float(lat[0])
    if lon_ref is None:
        lon_ref = float(lon[0])
    if alt_ref is None:
        alt_ref = float(alt[0])

    # Vectorised ECEF conversion
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    sin_lat = np.sin(lat_rad)
    cos_lat = np.cos(lat_rad)
    sin_lon = np.sin(lon_rad)
    cos_lon = np.cos(lon_rad)

    N_prime = WGS84_A / np.sqrt(1.0 - WGS84_E2 * sin_lat ** 2)

    ecef_x = (N_prime + alt) * cos_lat * cos_lon
    ecef_y = (N_prime + alt) * cos_lat * sin_lon
    ecef_z = (N_prime * (1.0 - WGS84_E2) + alt) * sin_lat

    # Reference ECEF
    ref_ecef = lla_to_ecef(lat_ref, lon_ref, alt_ref)
    dx = ecef_x - ref_ecef[0]
    dy = ecef_y - ref_ecef[1]
    dz = ecef_z - ref_ecef[2]

    # Rotation matrix components
    ref_lat_rad = np.radians(lat_ref)
    ref_lon_rad = np.radians(lon_ref)
    slat = np.sin(ref_lat_rad)
    clat = np.cos(ref_lat_rad)
    slon = np.sin(ref_lon_rad)
    clon = np.cos(ref_lon_rad)

    east  = -slon * dx + clon * dy
    north = -slat * clon * dx - slat * slon * dy + clat * dz
    up    =  clat * clon * dx + clat * slon * dy + slat * dz

    return np.column_stack([east, north, up])


# ---------------------------------------------------------------------------
# Distance
# ---------------------------------------------------------------------------

def haversine_distance(
    lat1: float,
    lon1: float,
    lat2: float,
    lon2: float,
) -> float:
    """Compute great-circle distance between two points on the Earth.

    Uses the haversine formula with mean Earth radius 6 371 000 m.

    Args:
        lat1: Latitude of point 1 in degrees.
        lon1: Longitude of point 1 in degrees.
        lat2: Latitude of point 2 in degrees.
        lon2: Longitude of point 2 in degrees.

    Returns:
        Distance in meters.
    """
    lat1_r, lon1_r = np.radians(lat1), np.radians(lon1)
    lat2_r, lon2_r = np.radians(lat2), np.radians(lon2)

    dlat = lat2_r - lat1_r
    dlon = lon2_r - lon1_r

    a = (
        np.sin(dlat / 2.0) ** 2
        + np.cos(lat1_r) * np.cos(lat2_r) * np.sin(dlon / 2.0) ** 2
    )
    c = 2.0 * np.arctan2(np.sqrt(a), np.sqrt(1.0 - a))

    return float(_EARTH_RADIUS_M * c)


# ---------------------------------------------------------------------------
# Trajectory / pose helpers
# ---------------------------------------------------------------------------

def gps_to_pose(
    timestamps: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    alt: np.ndarray,
    heading: np.ndarray | None = None,
) -> np.ndarray:
    """Convert a GPS trajectory to SE(3) pose matrices.

    Translation is obtained by converting LLA to a local ENU frame
    centred at the NCLT default reference point.  Rotation encodes yaw
    only (roll = pitch = 0).

    Args:
        timestamps: Timestamps in microseconds, shape (N,), int64.
        lat: Latitudes in degrees, shape (N,).
        lon: Longitudes in degrees, shape (N,).
        alt: Altitudes in meters, shape (N,).
        heading: Heading (yaw) in radians, shape (N,).  If ``None``,
            heading is estimated from successive ENU positions using
            ``atan2(dN, dE)`` (note: atan2(north, east) gives bearing
            from east, but we want heading from north so we use
            ``atan2(dE, dN)``).

    Returns:
        Array of 4x4 homogeneous transformation matrices, shape
        (N, 4, 4).
    """
    timestamps = np.asarray(timestamps, dtype=np.int64).ravel()
    lat = np.asarray(lat, dtype=np.float64).ravel()
    lon = np.asarray(lon, dtype=np.float64).ravel()
    alt = np.asarray(alt, dtype=np.float64).ravel()

    n = len(timestamps)
    if n == 0:
        return np.empty((0, 4, 4), dtype=np.float64)

    # Convert to local ENU
    enu = lla_to_enu(lat, lon, alt, NCLT_REF_LAT, NCLT_REF_LON, NCLT_REF_ALT)

    # Resolve heading
    if heading is not None:
        yaw = np.asarray(heading, dtype=np.float64).ravel()
    else:
        yaw = _estimate_heading(enu)

    # Build SE(3) matrices (yaw-only rotation about the Up axis)
    poses = np.zeros((n, 4, 4), dtype=np.float64)
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)

    poses[:, 0, 0] = cos_yaw
    poses[:, 0, 1] = -sin_yaw
    poses[:, 1, 0] = sin_yaw
    poses[:, 1, 1] = cos_yaw
    poses[:, 2, 2] = 1.0
    poses[:, 3, 3] = 1.0
    poses[:, 0, 3] = enu[:, 0]
    poses[:, 1, 3] = enu[:, 1]
    poses[:, 2, 3] = enu[:, 2]

    return poses


def _estimate_heading(enu: np.ndarray) -> np.ndarray:
    """Estimate heading from successive ENU positions.

    Heading is defined as the angle measured clockwise from North (the
    positive-N axis), which in ENU coordinates equals
    ``atan2(dEast, dNorth)``.

    For the first point the heading of the second point is copied.  For
    a single-point trajectory the heading defaults to zero.

    Args:
        enu: Array of shape (N, 3) with columns [East, North, Up].

    Returns:
        Heading in radians, shape (N,).
    """
    n = len(enu)
    if n <= 1:
        return np.zeros(n, dtype=np.float64)

    deast = np.diff(enu[:, 0])
    dnorth = np.diff(enu[:, 1])
    heading = np.arctan2(deast, dnorth)

    # Pad: replicate the first computed heading for the initial point
    heading = np.concatenate([[heading[0]], heading])
    return heading


# ---------------------------------------------------------------------------
# CSV parsing
# ---------------------------------------------------------------------------

def parse_gps_csv(csv_path: str | Path) -> dict[str, np.ndarray]:
    """Parse an NCLT ``gps.csv`` file into structured arrays.

    The expected CSV columns (no header) are::

        utime, mode, num_satell, latitude, longitude, altitude,
        track, speed

    Args:
        csv_path: Path to the CSV file.

    Returns:
        Dictionary with keys:

        - ``timestamps`` -- int64 microsecond timestamps
        - ``mode``       -- int fix-mode indicator
        - ``num_satellites`` -- int satellite count
        - ``latitude``   -- float64 degrees
        - ``longitude``  -- float64 degrees
        - ``altitude``   -- float64 meters
        - ``heading``    -- float64 degrees (track over ground)
        - ``speed``      -- float64 m/s

    Raises:
        FileNotFoundError: If *csv_path* does not exist.
        ValueError: If the file has an unexpected number of columns.
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"GPS CSV not found: {csv_path}")

    data = np.loadtxt(csv_path, delimiter=",", dtype=np.float64)

    if data.ndim == 1:
        # Single row -- promote to 2-D
        data = data.reshape(1, -1)

    if data.shape[1] != 8:
        raise ValueError(
            f"Expected 8 columns in GPS CSV, got {data.shape[1]}"
        )

    result: dict[str, np.ndarray] = {
        "timestamps":     data[:, 0].astype(np.int64),
        "mode":           data[:, 1].astype(np.int32),
        "num_satellites": data[:, 2].astype(np.int32),
        "latitude":       data[:, 3],
        "longitude":      data[:, 4],
        "altitude":       data[:, 5],
        "heading":        data[:, 6],
        "speed":          data[:, 7],
    }

    logger.info(
        "Parsed %d GPS records from %s", len(result["timestamps"]), csv_path,
    )
    return result


# ---------------------------------------------------------------------------
# Filtering
# ---------------------------------------------------------------------------

def filter_gps_by_fix(
    timestamps: np.ndarray,
    mode: np.ndarray,
    lat: np.ndarray,
    lon: np.ndarray,
    alt: np.ndarray,
    min_mode: int = 1,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Filter GPS readings by fix quality.

    Fix-mode values used in the NCLT dataset:

    - 0 -- no fix
    - 1 -- autonomous GPS fix
    - 2 -- DGPS fix
    - 4 -- RTK fix

    Args:
        timestamps: Timestamps, shape (N,).
        mode: Fix-mode indicator, shape (N,).
        lat: Latitudes in degrees, shape (N,).
        lon: Longitudes in degrees, shape (N,).
        alt: Altitudes in meters, shape (N,).
        min_mode: Minimum acceptable fix mode (inclusive).

    Returns:
        Tuple of four filtered arrays
        ``(timestamps, lat, lon, alt)`` containing only the rows
        where ``mode >= min_mode``.
    """
    timestamps = np.asarray(timestamps)
    mode = np.asarray(mode)
    lat = np.asarray(lat, dtype=np.float64)
    lon = np.asarray(lon, dtype=np.float64)
    alt = np.asarray(alt, dtype=np.float64)

    mask = mode >= min_mode

    n_total = len(mode)
    n_kept = int(np.sum(mask))
    if n_kept < n_total:
        logger.debug(
            "GPS fix filter (min_mode=%d): kept %d / %d readings",
            min_mode, n_kept, n_total,
        )

    return (
        timestamps[mask],
        lat[mask],
        lon[mask],
        alt[mask],
    )
