import numpy as np
import pytest

from src.quadcopter.simulation import state_derivative


def test_state_derivative_returns_17_finite_values():
    state = np.zeros(17)
    state[6] = 1.0  # Identity quaternion
    state[13:17] = 500.0  # Current rotor speeds

    commanded_speeds = np.full(4, 600.0)

    result = state_derivative(
        t=0.0,
        state=state,
        commanded_rotor_speeds_rad_s=commanded_speeds,
    )

    assert result.shape == (17,)
    assert np.all(np.isfinite(result))


def test_state_derivative_rejects_invalid_state_size():
    with pytest.raises(ValueError):
        state_derivative(
            t=0.0,
            state=np.zeros(13),
            commanded_rotor_speeds_rad_s=np.full(4, 500.0),
        )


def test_state_derivative_motor_speeds_approach_command():
    state = np.zeros(17)
    state[6] = 1.0
    state[13:17] = 500.0

    commands = np.full(4, 600.0)

    result = state_derivative(
        t=0.0,
        state=state,
        commanded_rotor_speeds_rad_s=commands,
    )

    assert np.all(result[13:17] > 0.0)


def test_state_derivative_rejects_invalid_command_shape():
    state = np.zeros(17)
    state[6] = 1.0
    state[13:17] = 500.0

    with pytest.raises(ValueError):
        state_derivative(
            t=0.0,
            state=state,
            commanded_rotor_speeds_rad_s=np.array([500.0, 500.0]),
        )


def test_time_varying_motor_command_increases_motor_speed():
    import numpy as np

    from src.quadcopter.simulator import simulate

    hover_speed = np.sqrt(
        1.2 * 9.80665 / (4.0 * 8.0e-6)
    )

    initial_state = np.zeros(17)
    initial_state[6] = 1.0
    initial_state[13:17] = hover_speed

    def motor_command(t):
        if t < 0.1:
            return np.full(4, hover_speed)

        return np.full(4, hover_speed + 100.0)

    _, states = simulate(
        initial_state=initial_state,
        commanded_rotor_speeds_rad_s=np.full(4, hover_speed),
        duration_s=0.3,
        dt=0.001,
        command_function=motor_command,
    )

    assert np.all(states[-1, 13:17] > hover_speed)