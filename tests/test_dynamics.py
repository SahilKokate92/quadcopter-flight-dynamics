
import numpy as np

from config.vehicle import MASS_KG, GRAVITY_M_S2, INERTIA_KG_M2
from src.quadcopter.dynamics import rigid_body_derivatives


IDENTITY_QUATERNION = np.array([1.0, 0.0, 0.0, 0.0])


def calculate_derivatives(force_body, torque_body):
    return rigid_body_derivatives(
        position_ned=np.zeros(3),
        velocity_body=np.zeros(3),
        quaternion_nb=IDENTITY_QUATERNION,
        angular_velocity_body=np.zeros(3),
        force_body=np.array(force_body, dtype=float),
        torque_body=np.array(torque_body, dtype=float),
    )


def test_hover_equilibrium():
    # Upward thrust balances gravity at level attitude.
    derivatives = calculate_derivatives(
        force_body=[0.0, 0.0, -MASS_KG * GRAVITY_M_S2],
        torque_body=[0.0, 0.0, 0.0],
    )

    np.testing.assert_allclose(
        derivatives["velocity_dot_body"],
        np.zeros(3),
        atol=1e-12,
    )

    np.testing.assert_allclose(
        derivatives["angular_velocity_dot_body"],
        np.zeros(3),
        atol=1e-12,
    )


def test_free_fall_acceleration():
    # No external force: gravity causes positive-down acceleration.
    derivatives = calculate_derivatives(
        force_body=[0.0, 0.0, 0.0],
        torque_body=[0.0, 0.0, 0.0],
    )

    np.testing.assert_allclose(
        derivatives["velocity_dot_body"],
        [0.0, 0.0, GRAVITY_M_S2],
        atol=1e-12,
    )


def test_roll_torque_acceleration():
    torque = np.array([0.1, 0.0, 0.0])

    derivatives = calculate_derivatives(
        force_body=[0.0, 0.0, -MASS_KG * GRAVITY_M_S2],
        torque_body=torque,
    )

    expected_roll_acceleration = (
        torque[0] / INERTIA_KG_M2[0][0]
    )

    np.testing.assert_allclose(
        derivatives["angular_velocity_dot_body"],
        [expected_roll_acceleration, 0.0, 0.0],
        atol=1e-12,
    )


def test_zero_input_gives_zero_quaternion_derivative():
    derivatives = calculate_derivatives(
        force_body=[0.0, 0.0, -MASS_KG * GRAVITY_M_S2],
        torque_body=[0.0, 0.0, 0.0],
    )

    np.testing.assert_allclose(
        derivatives["quaternion_dot_nb"],
        np.zeros(4),
        atol=1e-12,
    )


def test_yaw_rate_produces_correct_quaternion_derivative():
    yaw_rate = 0.8

    derivatives = rigid_body_derivatives(
        position_ned=np.zeros(3),
        velocity_body=np.zeros(3),
        quaternion_nb=IDENTITY_QUATERNION,
        angular_velocity_body=np.array([0.0, 0.0, yaw_rate]),
        force_body=np.array([
            0.0,
            0.0,
            -MASS_KG * GRAVITY_M_S2,
        ]),
        torque_body=np.zeros(3),
    )

    expected = np.array([
        0.0,
        0.0,
        0.0,
        yaw_rate / 2.0,
    ])

    np.testing.assert_allclose(
        derivatives["quaternion_dot_nb"],
        expected,
        atol=1e-12,
    )