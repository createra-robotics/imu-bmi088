"""Quaternion math utilities (scalar-first convention: q = [w, x, y, z])."""

import numpy as np


def quat_to_euler(q: np.ndarray, degrees: bool = False) -> np.ndarray:
    """Convert quaternion to intrinsic ZYX Euler angles (roll, pitch, yaw).

    Args:
        q: Quaternion (w, x, y, z).
        degrees: If True, returns degrees instead of radians.

    Returns:
        ndarray of shape (3,): roll (X), pitch (Y), yaw (Z).

    """
    w, x, y, z = q
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = np.arcsin(np.clip(sinp, -1.0, 1.0))

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    out = np.array([roll, pitch, yaw], dtype=float)
    return np.rad2deg(out) if degrees else out


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """Convert quaternion to a 3x3 body-to-world rotation matrix.

    Args:
        q: Quaternion (w, x, y, z).

    Returns:
        ndarray of shape (3, 3).

    """
    w, x, y, z = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def quat_normalize(q: np.ndarray) -> np.ndarray:
    """Return a unit-norm copy of the quaternion."""
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q)
    if n == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / n


def quat_conjugate(q: np.ndarray) -> np.ndarray:
    """Return the conjugate (w, -x, -y, -z) of a quaternion."""
    w, x, y, z = q
    return np.array([w, -x, -y, -z], dtype=float)


def rotate_vector(q: np.ndarray, v: np.ndarray) -> np.ndarray:
    """Rotate a 3-vector by a quaternion (body-frame to world-frame)."""
    return quat_to_matrix(q) @ np.asarray(v, dtype=float)
