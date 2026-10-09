
import numpy as np


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    """
    Normalize a scalar-first quaternion [w, x, y, z].
    """
    q = np.asarray(q, dtype=float)

    if q.shape != (4,):
        raise ValueError("Quaternion must have shape (4,).")

    norm = np.linalg.norm(q)

    if norm < 1e-12:
        raise ValueError("Cannot normalize a zero quaternion.")

    return q / norm


def quaternion_multiply(
    q1: np.ndarray,
    q2: np.ndarray,
) -> np.ndarray:
    """
    Hamilton product of two scalar-first quaternions [w, x, y, z].
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    ])


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    """
    Convert a scalar-first quaternion [w, x, y, z]
    into a body-to-NED rotation matrix.

    The quaternion is normalized before conversion.
    """
    qw, qx, qy, qz = normalize_quaternion(q)

    return np.array([
        [
            1 - 2*(qy**2 + qz**2),
            2*(qx*qy - qw*qz),
            2*(qx*qz + qw*qy),
        ],
        [
            2*(qx*qy + qw*qz),
            1 - 2*(qx**2 + qz**2),
            2*(qy*qz - qw*qx),
        ],
        [
            2*(qx*qz - qw*qy),
            2*(qy*qz + qw*qx),
            1 - 2*(qx**2 + qy**2),
        ],
    ])