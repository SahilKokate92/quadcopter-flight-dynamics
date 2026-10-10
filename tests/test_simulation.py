import numpy as np
import pytest

from src.quadcopter.simulation import state_derivative


def test_state_derivative_returns_13_finite_values():
    state = np.zeros(13)
    state[6] = 1.0  # Identity quaternion

    rotor_speeds = np.full(4, 500.0)

    result = state_derivative(
        t=0.0,
        state=state,
        rotor_speeds_rad_s=rotor_speeds,
    )

    assert result.shape == (13,)
    assert np.all(np.isfinite(result))


def test_state_derivative_rejects_invalid_state_size():
    with pytest.raises(ValueError):
        state_derivative(
            t=0.0,
            state=np.zeros(12),
            rotor_speeds_rad_s=np.full(4, 500.0),
        )