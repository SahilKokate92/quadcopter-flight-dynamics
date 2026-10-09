
import numpy as np

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
)
from src.quadcopter.vehicle import vehicle_derivatives


def test_vehicle_hovers_at_ideal_rotor_speed():
    hover_speed = np.sqrt(
        MASS_KG * GRAVITY_M_S2
        / (4.0 * THRUST_COEFFICIENT)
    )

    result = vehicle_derivatives(
        position_ned=np.zeros(3),
        velocity_body=np.zeros(3),
        quaternion_nb=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity_body=np.zeros(3),
        rotor_speeds_rad_s=np.full(4, hover_speed),
    )

    np.testing.assert_allclose(
        result["velocity_dot_body"],
        np.zeros(3),
        atol=1e-10,
    )

    np.testing.assert_allclose(
        result["angular_velocity_dot_body"],
        np.zeros(3),
        atol=1e-10,
    )


def test_hover_has_zero_position_derivative():
    hover_speed = np.sqrt(
        MASS_KG * GRAVITY_M_S2
        / (4.0 * THRUST_COEFFICIENT)
    )

    result = vehicle_derivatives(
        position_ned=np.zeros(3),
        velocity_body=np.zeros(3),
        quaternion_nb=np.array([1.0, 0.0, 0.0, 0.0]),
        angular_velocity_body=np.zeros(3),
        rotor_speeds_rad_s=np.full(4, hover_speed),
    )

    np.testing.assert_allclose(
        result["position_dot_ned"],
        np.zeros(3),
        atol=1e-12,
    )