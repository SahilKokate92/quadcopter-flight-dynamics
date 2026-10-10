import numpy as np

from src.quadcopter.simulator import simulate


def test_quadcopter_maintains_hover_equilibrium():
    mass_kg = 1.2
    gravity_m_s2 = 9.80665
    thrust_coefficient = 8.0e-6

    hover_speed = np.sqrt(
        mass_kg * gravity_m_s2
        / (4.0 * thrust_coefficient)
    )

    initial_state = np.zeros(17)
    initial_state[6] = 1.0
    initial_state[13:17] = hover_speed

    commanded_rotor_speeds = np.full(4, hover_speed)

    times, states = simulate(
        initial_state=initial_state,
        commanded_rotor_speeds_rad_s=commanded_rotor_speeds,
        duration_s=2.0,
        dt=0.01,
    )

    # The vehicle should remain close to its initial altitude.
    np.testing.assert_allclose(
        states[-1, 2],
        0.0,
        atol=1e-6,
    )

    # Vertical body velocity should remain near zero.
    np.testing.assert_allclose(
        states[-1, 5],
        0.0,
        atol=1e-6,
    )

    # Angular velocity should remain near zero.
    np.testing.assert_allclose(
        states[-1, 10:13],
        0.0,
        atol=1e-6,
    )

    # Quaternion should remain normalized.
    quaternion_norms = np.linalg.norm(
        states[:, 6:10],
        axis=1,
    )

    np.testing.assert_allclose(
        quaternion_norms,
        1.0,
        atol=1e-12,
    )
