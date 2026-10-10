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