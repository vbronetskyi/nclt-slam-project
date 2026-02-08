"""IMU processing utilities for the NCLT dataset.

Provides parsing, interpolation, integration, and preintegration routines
for the Microstrain MS25 IMU data collected on the NCLT Segway platform.

The MS25 CSV format (ms25.csv) has columns::

    utime, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, rot_x, rot_y, rot_z

where *utime* is in microseconds, magnetometer readings are in Gauss,
accelerometer readings are in m/s^2, and gyroscope readings are in rad/s.
"""

from __future__ import annotations

import logging
from pathlib import Path

import numpy as np

logger = logging.getLogger(__name__)

# Column layout in ms25.csv (after utime)
_MAG_COLS = (1, 2, 3)    # mag_x, mag_y, mag_z
_ACCEL_COLS = (4, 5, 6)  # accel_x, accel_y, accel_z
_GYRO_COLS = (7, 8, 9)   # rot_x, rot_y, rot_z

_US_TO_S = 1e-6  # microseconds to seconds


# ---------------------------------------------------------------------------
# Rotation helpers
# ---------------------------------------------------------------------------


def skew_symmetric(v: np.ndarray) -> np.ndarray:
    """Convert a 3-vector to a 3x3 skew-symmetric matrix.

    For a vector ``v = [v0, v1, v2]`` the result is::

        [[ 0, -v2,  v1],
         [ v2,  0, -v0],
         [-v1,  v0,  0]]

    Args:
        v: Input vector of shape ``(3,)``.

    Returns:
        Skew-symmetric matrix of shape ``(3, 3)``.
    """
    v = np.asarray(v, dtype=np.float64).ravel()
    return np.array([
        [0.0, -v[2], v[1]],
        [v[2], 0.0, -v[0]],
        [-v[1], v[0], 0.0],
    ], dtype=np.float64)


def rodrigues(axis_angle: np.ndarray) -> np.ndarray:
    """Convert an axis-angle vector to a rotation matrix via Rodrigues' formula.

    The direction of *axis_angle* gives the rotation axis and its norm
    gives the rotation angle in radians.

    For a unit axis ``k`` and angle ``theta``::

        R = I + sin(theta) * K + (1 - cos(theta)) * K^2

    where ``K = skew_symmetric(k)``.

    When ``theta`` is very small (< 1e-8) the first-order approximation
    ``R ~ I + K * theta`` is used for numerical stability.

    Args:
        axis_angle: Rotation vector of shape ``(3,)`` where
            ``||axis_angle||`` is the angle in radians.

    Returns:
        Rotation matrix of shape ``(3, 3)``.
    """
    axis_angle = np.asarray(axis_angle, dtype=np.float64).ravel()
    theta = np.linalg.norm(axis_angle)

    if theta < 1e-8:
        # First-order approximation for tiny angles
        K = skew_symmetric(axis_angle)
        return np.eye(3, dtype=np.float64) + K

    k = axis_angle / theta
    K = skew_symmetric(k)
    R = (
        np.eye(3, dtype=np.float64)
        + np.sin(theta) * K
        + (1.0 - np.cos(theta)) * (K @ K)
    )
    return R


# ---------------------------------------------------------------------------
# Parsing
# ---------------------------------------------------------------------------


def parse_imu_csv(csv_path: str | Path) -> dict[str, np.ndarray]:
    """Parse an NCLT ``ms25.csv`` file into structured arrays.

    The CSV is assumed to be comma-separated with no header and ten
    numeric columns:

    ``utime, mag_x, mag_y, mag_z, accel_x, accel_y, accel_z, rot_x, rot_y, rot_z``

    Args:
        csv_path: Filesystem path to the ``ms25.csv`` file.

    Returns:
        Dictionary with keys:

        - ``timestamps``: int64 microsecond timestamps, shape ``(N,)``.
        - ``magnetometer``: float64 magnetometer readings in Gauss,
          shape ``(N, 3)``.
        - ``accelerometer``: float64 accelerometer readings in m/s^2,
          shape ``(N, 3)``.
        - ``gyroscope``: float64 gyroscope readings in rad/s,
          shape ``(N, 3)``.

    Raises:
        FileNotFoundError: If *csv_path* does not exist.
        ValueError: If the file has an unexpected number of columns.
    """
    csv_path = Path(csv_path)
    if not csv_path.exists():
        raise FileNotFoundError(f"IMU file not found: {csv_path}")

    data = np.loadtxt(csv_path, delimiter=",", dtype=np.float64)

    if data.ndim == 1:
        data = data.reshape(1, -1)

    if data.shape[1] != 10:
        raise ValueError(
            f"Expected 10 columns in ms25.csv, got {data.shape[1]}"
        )

    result = {
        "timestamps": data[:, 0].astype(np.int64),
        "magnetometer": data[:, _MAG_COLS[0]:_MAG_COLS[-1] + 1].copy(),
        "accelerometer": data[:, _ACCEL_COLS[0]:_ACCEL_COLS[-1] + 1].copy(),
        "gyroscope": data[:, _GYRO_COLS[0]:_GYRO_COLS[-1] + 1].copy(),
    }

    logger.info(
        "Parsed %d IMU samples from %s (%.1f s span)",
        len(result["timestamps"]),
        csv_path.name,
        (result["timestamps"][-1] - result["timestamps"][0]) * _US_TO_S,
    )
    return result


# ---------------------------------------------------------------------------
# Interpolation
# ---------------------------------------------------------------------------


def interpolate_imu(
    timestamps: np.ndarray,
    values: np.ndarray,
    target_times: np.ndarray,
) -> np.ndarray:
    """Linearly interpolate IMU data at arbitrary target timestamps.

    Each column of *values* is interpolated independently along the
    *timestamps* axis.  Target timestamps that fall outside the range of
    *timestamps* are clamped to the boundary values (no extrapolation).

    Args:
        timestamps: Monotonically increasing source timestamps of shape
            ``(N,)``, int64 microseconds.
        values: Source values of shape ``(N, C)`` with *C* channels.
        target_times: Query timestamps of shape ``(M,)``, int64
            microseconds.

    Returns:
        Interpolated values of shape ``(M, C)``.
    """
    timestamps = np.asarray(timestamps, dtype=np.float64)
    target_times = np.asarray(target_times, dtype=np.float64)

    if values.ndim == 1:
        values = values[:, np.newaxis]

    n_channels = values.shape[1]
    result = np.empty((len(target_times), n_channels), dtype=np.float64)

    for c in range(n_channels):
        result[:, c] = np.interp(target_times, timestamps, values[:, c])

    return result


# ---------------------------------------------------------------------------
# Bias estimation
# ---------------------------------------------------------------------------


def compute_bias(
    timestamps: np.ndarray,
    gyro: np.ndarray,
    static_duration_us: int = 5_000_000,
) -> np.ndarray:
    """Estimate gyroscope bias from an initial static period.

    The robot is assumed to be stationary during the first
    *static_duration_us* microseconds of data.  The bias is computed as
    the mean angular velocity over that interval.

    Args:
        timestamps: Monotonic int64 microsecond timestamps, shape ``(N,)``.
        gyro: Angular velocity readings in rad/s, shape ``(N, 3)``.
        static_duration_us: Duration of the static window in
            microseconds.  Defaults to 5 seconds (5 000 000 us).

    Returns:
        Estimated gyroscope bias of shape ``(3,)`` in rad/s.
    """
    t0 = timestamps[0]
    mask = timestamps <= t0 + static_duration_us

    if np.sum(mask) < 2:
        logger.warning(
            "Fewer than 2 samples in static window (%d us); "
            "using all available data for bias estimation",
            static_duration_us,
        )
        mask = np.ones(len(timestamps), dtype=bool)

    bias = np.mean(gyro[mask], axis=0)
    logger.info(
        "Gyro bias estimated from %d samples (%.2f s): [%.6f, %.6f, %.6f] rad/s",
        np.sum(mask),
        np.sum(mask) * _US_TO_S,  # approximate; actual span may differ
        bias[0],
        bias[1],
        bias[2],
    )
    return bias


# ---------------------------------------------------------------------------
# Gravity alignment
# ---------------------------------------------------------------------------


def gravity_alignment(
    accel: np.ndarray,
    num_samples: int = 100,
) -> np.ndarray:
    """Estimate the rotation that aligns the body Z-axis with gravity.

    When the robot is stationary the accelerometer measures ``-g`` in the
    body frame.  This function averages the first *num_samples* readings
    to estimate the gravity direction, then computes a rotation matrix
    that maps the measured gravity direction onto ``[0, 0, -g]`` (i.e.
    aligns the body frame so that Z points opposite to gravity, which is
    the NED / ENU convention with Z-up).

    Args:
        accel: Accelerometer readings in m/s^2, shape ``(N, 3)``.  At
            least *num_samples* rows should correspond to a static
            period.
        num_samples: Number of initial samples to average.  Clamped to
            the length of *accel* if necessary.

    Returns:
        Rotation matrix of shape ``(3, 3)`` that transforms vectors from
        the original body frame into the gravity-aligned frame.
    """
    n = min(num_samples, len(accel))
    g_body = np.mean(accel[:n], axis=0)  # measured gravity in body frame
    g_mag = np.linalg.norm(g_body)

    if g_mag < 1e-6:
        logger.warning(
            "Measured gravity magnitude is near zero (%.4f m/s^2); "
            "returning identity",
            g_mag,
        )
        return np.eye(3, dtype=np.float64)

    # Normalised measured gravity direction in the body frame
    g_hat = g_body / g_mag

    # Target direction: gravity points along -Z in the aligned frame
    z_target = np.array([0.0, 0.0, -1.0])

    # Rotation axis is the cross product of measured and target directions
    axis = np.cross(g_hat, z_target)
    sin_angle = np.linalg.norm(axis)
    cos_angle = np.dot(g_hat, z_target)

    if sin_angle < 1e-8:
        if cos_angle > 0:
            # Already aligned
            return np.eye(3, dtype=np.float64)
        else:
            # 180-degree flip -- pick an arbitrary perpendicular axis
            perp = np.array([1.0, 0.0, 0.0])
            if np.abs(np.dot(g_hat, perp)) > 0.9:
                perp = np.array([0.0, 1.0, 0.0])
            axis = np.cross(g_hat, perp)
            axis /= np.linalg.norm(axis)
            return rodrigues(axis * np.pi)

    axis /= sin_angle
    angle = np.arctan2(sin_angle, cos_angle)
    R = rodrigues(axis * angle)

    logger.info(
        "Gravity alignment: measured |g|=%.4f m/s^2, rotation angle=%.2f deg",
        g_mag,
        np.degrees(angle),
    )
    return R


# ---------------------------------------------------------------------------
# Gyroscope integration
# ---------------------------------------------------------------------------


def integrate_gyroscope(
    timestamps: np.ndarray,
    gyro: np.ndarray,
    initial_orientation: np.ndarray | None = None,
) -> np.ndarray:
    """Integrate angular velocities to obtain orientation over time.

    Uses the Rodrigues rotation formula to compose incremental rotations
    from discrete gyroscope readings::

        R_{k+1} = R_k @ rodrigues(omega_k * dt)

    Args:
        timestamps: Monotonic int64 microsecond timestamps, shape ``(N,)``.
        gyro: Angular velocity readings ``[rot_x, rot_y, rot_z]`` in
            rad/s, shape ``(N, 3)``.
        initial_orientation: Optional ``(3, 3)`` rotation matrix for the
            initial orientation.  Defaults to the identity.

    Returns:
        Array of rotation matrices of shape ``(N, 3, 3)``.  The first
        element equals *initial_orientation*.
    """
    n = len(timestamps)
    orientations = np.empty((n, 3, 3), dtype=np.float64)

    if initial_orientation is not None:
        orientations[0] = initial_orientation
    else:
        orientations[0] = np.eye(3, dtype=np.float64)

    for k in range(n - 1):
        dt = (timestamps[k + 1] - timestamps[k]) * _US_TO_S
        dR = rodrigues(gyro[k] * dt)
        orientations[k + 1] = orientations[k] @ dR

    return orientations


# ---------------------------------------------------------------------------
# IMU preintegration
# ---------------------------------------------------------------------------


def imu_preintegration(
    timestamps: np.ndarray,
    accel: np.ndarray,
    gyro: np.ndarray,
    gravity: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    """Discrete IMU preintegration between keyframes.

    Follows the standard discrete-time integration equations used in
    LIO-SAM and similar tightly-coupled LiDAR-inertial systems:

    .. math::

        R_{k+1} &= R_k \\cdot \\mathrm{rodrigues}(\\omega_k \\, \\Delta t) \\\\
        v_{k+1} &= v_k + (R_k \\, a_k + g) \\, \\Delta t \\\\
        p_{k+1} &= p_k + v_k \\, \\Delta t
                    + \\tfrac{1}{2} (R_k \\, a_k + g) \\, \\Delta t^2

    where :math:`R_k` is the orientation, :math:`v_k` the velocity,
    :math:`p_k` the position, :math:`a_k` the accelerometer reading,
    :math:`\\omega_k` the gyroscope reading, and :math:`g` the gravity
    vector.

    Args:
        timestamps: Monotonic int64 microsecond timestamps, shape ``(N,)``.
        accel: Accelerometer readings in m/s^2, shape ``(N, 3)``.
        gyro: Gyroscope readings in rad/s, shape ``(N, 3)``.
        gravity: Gravity vector in m/s^2, shape ``(3,)``.  Defaults to
            ``[0, 0, -9.81]``.

    Returns:
        Dictionary with:

        - ``positions``: integrated positions, shape ``(N, 3)``.
        - ``velocities``: integrated velocities, shape ``(N, 3)``.
        - ``orientations``: rotation matrices, shape ``(N, 3, 3)``.
    """
    if gravity is None:
        gravity = np.array([0.0, 0.0, -9.81], dtype=np.float64)
    else:
        gravity = np.asarray(gravity, dtype=np.float64).ravel()

    n = len(timestamps)

    positions = np.zeros((n, 3), dtype=np.float64)
    velocities = np.zeros((n, 3), dtype=np.float64)
    orientations = np.zeros((n, 3, 3), dtype=np.float64)
    orientations[0] = np.eye(3, dtype=np.float64)

    for k in range(n - 1):
        dt = (timestamps[k + 1] - timestamps[k]) * _US_TO_S
        R_k = orientations[k]

        # Acceleration in the world frame
        accel_world = R_k @ accel[k] + gravity

        # Update orientation
        dR = rodrigues(gyro[k] * dt)
        orientations[k + 1] = R_k @ dR

        # Update velocity
        velocities[k + 1] = velocities[k] + accel_world * dt

        # Update position
        positions[k + 1] = (
            positions[k]
            + velocities[k] * dt
            + 0.5 * accel_world * dt * dt
        )

    logger.debug(
        "IMU preintegration: %d samples, %.3f s span, "
        "final pos=[%.3f, %.3f, %.3f]",
        n,
        (timestamps[-1] - timestamps[0]) * _US_TO_S,
        positions[-1, 0],
        positions[-1, 1],
        positions[-1, 2],
    )

    return {
        "positions": positions,
        "velocities": velocities,
        "orientations": orientations,
    }
