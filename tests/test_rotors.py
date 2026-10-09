
import numpy as np
import pytest

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
)
from src.quadcopter.rotors import rotor_forces_and_torques


def test_zero_rotor_speed_produces_zero_force_and_torque():
    result = rotor_forces_and_torques(np.zeros(4))

    np.testing.assert_allclose(result["force_body_n"], np.zeros(3))
    np.testing.assert_allclose(result["torque_body_nm"], np.zeros(3))
    assert result["thrust_n"] == 0.0


def test_equal_speeds_produce_balanced_torque():
    speeds = np.full(4, 500.0)

    result = rotor_forces_and_torques(speeds)

    assert result["thrust_n"] > 0.0
    np.testing.assert_allclose(
        result["torque_body_nm"],
        np.zeros(3),
        atol=1e-12,
    )


def test_total_thrust_matches_rotor_thrust_sum():
    speeds = np.array([400.0, 500.0, 600.0, 700.0])

    result = rotor_forces_and_torques(speeds)

    expected_thrust = THRUST_COEFFICIENT * np.sum(speeds**2)

    assert result["thrust_n"] == pytest.approx(expected_thrust)


def test_negative_speed_is_rejected():
    with pytest.raises(ValueError):
        rotor_forces_and_torques(
            np.array([500.0, -100.0, 500.0, 500.0])
        )


def test_hover_speed_estimate():
    # Ideal equal-speed hover: each rotor supplies one quarter of weight.
    speed = np.sqrt(
        MASS_KG * GRAVITY_M_S2
        / (4.0 * THRUST_COEFFICIENT)
    )

    result = rotor_forces_and_torques(np.full(4, speed))

    assert result["thrust_n"] == pytest.approx(
        MASS_KG * GRAVITY_M_S2
    )