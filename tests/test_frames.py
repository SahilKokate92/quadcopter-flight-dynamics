
import numpy as np
from src.quadcopter.frames import quaternion_to_rotation_matrix
from src.quadcopter.frames import (
    normalize_quaternion,
    quaternion_multiply,
)


def test_normalize_quaternion():
    q = np.array([2.0, 0.0, 0.0, 0.0])
    result = normalize_quaternion(q)

    np.testing.assert_allclose(
        result,
        [1.0, 0.0, 0.0, 0.0],
    )


def test_quaternion_multiply_identity():
    identity = np.array([1.0, 0.0, 0.0, 0.0])
    q = np.array([0.70710678, 0.70710678, 0.0, 0.0])

    result = quaternion_multiply(identity, q)

    np.testing.assert_allclose(result, q, atol=1e-7)


def test_normalized_quaternion_has_unit_length():
    q = np.array([1.0, 2.0, 3.0, 4.0])
    result = normalize_quaternion(q)

    assert np.isclose(np.linalg.norm(result), 1.0)


def test_zero_quaternion_raises_error():
    q = np.zeros(4)

    try:
        normalize_quaternion(q)
    except ValueError:
        pass
    else:
        raise AssertionError("Expected ValueError for zero quaternion.")


def test_identity_quaternion_rotation_matrix():
    q = np.array([1.0, 0.0, 0.0, 0.0])

    R = quaternion_to_rotation_matrix(q)

    np.testing.assert_allclose(R, np.eye(3), atol=1e-12)


def test_rotation_matrix_is_orthogonal():
    q = normalize_quaternion(np.array([0.8, 0.2, -0.3, 0.4]))

    R = quaternion_to_rotation_matrix(q)

    np.testing.assert_allclose(
        R.T @ R,
        np.eye(3),
        atol=1e-12,
    )


def test_rotation_matrix_determinant():
    q = np.array([0.70710678, 0.0, 0.0, 0.70710678])

    R = quaternion_to_rotation_matrix(q)

    assert np.isclose(np.linalg.det(R), 1.0, atol=1e-7)


def test_positive_90_degree_yaw():
    q = np.array([
        np.sqrt(0.5),
        0.0,
        0.0,
        np.sqrt(0.5),
    ])

    R = quaternion_to_rotation_matrix(q)

    # Body forward axis expressed in NED coordinates.
    forward_body = np.array([1.0, 0.0, 0.0])
    forward_ned = R @ forward_body

    np.testing.assert_allclose(
        forward_ned,
        [0.0, 1.0, 0.0],
        atol=1e-12,
    )