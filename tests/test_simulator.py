import numpy as np
import pytest

from config.vehicle import MASS_KG, GRAVITY_M_S2, THRUST_COEFFICIENT
from src.quadcopter.simulator import simulate


def test_ideal_hover_remains_stationary():
    state = np.zeros(13)
    state[6] = 1.0  # Identity quaternion

    # Equal rotor speeds for total thrust equal to weight.
    hover_speed = np.sqrt(
        MASS_KG * GRAVITY_M_S2 / (4.0 * THRUST_COEFFICIENT)
    )
    rotor_speeds = np.full(4, hover_speed)

    times, states = simulate(
        initial_state=state,
        rotor_speeds_rad_s=rotor_speeds,
        duration_s=1.0,
        dt=0.01,
    )

    assert states.shape == (101, 13)
    assert times.shape == (101,)

    # The vehicle should remain near its initial stationary state.
    np.testing.assert_allclose(states[:, 0:6], 0.0, atol=1e-8)
    np.testing.assert_allclose(states[:, 6], 1.0, atol=1e-8)
    np.testing.assert_allclose(states[:, 7:10], 0.0, atol=1e-8)
    np.testing.assert_allclose(states[:, 10:13], 0.0, atol=1e-8)


def test_simulator_rejects_invalid_initial_state():
    with pytest.raises(ValueError):
        simulate(
            initial_state=np.zeros(12),
            rotor_speeds_rad_s=np.full(4, 500.0),
            duration_s=1.0,
            dt=0.01,
        )