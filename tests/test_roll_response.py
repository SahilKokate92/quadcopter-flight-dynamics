
import numpy as np

from config.vehicle import IXX_KG_M2
from src.quadcopter.rotors import rotor_forces_and_torques
from src.quadcopter.dynamics import rigid_body_derivatives
from src.quadcopter.simulator import simulate


def test_left_side_thrust_difference_produces_negative_roll_torque():
    # Rotors 3 and 4 are on the left side (positive body y).
    speeds = np.array([500.0, 500.0, 600.0, 600.0])

    result = rotor_forces_and_torques(speeds)

    assert result["torque_body_nm"][0] < 0.0


def test_positive_roll_torque_produces_positive_roll_acceleration():
    state_result = rigid_body_derivatives(
        position_ned=np.zeros(3),
        velocity_body=np.zeros(3),
        quaternion_nb=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity_body=np.zeros(3),
        force_body=np.zeros(3),
        torque_body=np.array([0.01, 0.0, 0.0]),
    )

    expected_acceleration = 0.01 / IXX_KG_M2

    np.testing.assert_allclose(
        state_result["angular_velocity_dot_body"][0],
        expected_acceleration,
        atol=1e-12,
    )


def test_constant_roll_torque_produces_negative_roll_rate():
    hover_speed = np.sqrt(
        1.2 * 9.80665 / (4.0 * 8.0e-6)
    )

    commanded_rotor_speeds = np.array([
        hover_speed,
        hover_speed,
        hover_speed + 20.0,
        hover_speed + 20.0,
    ])

    initial_state = np.zeros(17)
    initial_state[6] = 1.0
    initial_state[13:17] = hover_speed

    times, states = simulate(
        initial_state=initial_state,
        commanded_rotor_speeds_rad_s=commanded_rotor_speeds,
        duration_s=0.2,
        dt=0.01,
    )

    assert times.shape == (21,)
    assert states[-1, 10] < 0.0
    assert states[-1, 10] < states[0, 10]


def test_roll_simulation_preserves_quaternion_norm():
    hover_speed = np.sqrt(
        1.2 * 9.80665 / (4.0 * 8.0e-6)
    )

    commanded_rotor_speeds = np.array([
        hover_speed,
        hover_speed,
        hover_speed + 20.0,
        hover_speed + 20.0,
    ])

    initial_state = np.zeros(17)
    initial_state[6] = 1.0
    initial_state[13:17] = hover_speed

    _, states = simulate(
        initial_state=initial_state,
        commanded_rotor_speeds_rad_s=commanded_rotor_speeds,
        duration_s=0.2,
        dt=0.01,
    )

    quaternion_norms = np.linalg.norm(
        states[:, 6:10], axis=1
    )

    np.testing.assert_allclose(
        quaternion_norms,
        1.0,
        atol=1e-12,
    )
