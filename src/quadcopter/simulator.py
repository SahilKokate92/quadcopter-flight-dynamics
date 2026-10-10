import numpy as np

from src.quadcopter.integrators import rk4_step
from src.quadcopter.simulation import state_derivative


def simulate(
    initial_state,
    rotor_speeds_rad_s,
    duration_s,
    dt,
):
    """
    Simulate quadcopter motion with fixed rotor speeds.

    Returns:
        times: Simulation timestamps.
        states: State history with shape (number_of_steps + 1, 13).
    """

    initial_state = np.asarray(initial_state, dtype=float)
    rotor_speeds_rad_s = np.asarray(
        rotor_speeds_rad_s, dtype=float
    )

    if initial_state.shape != (13,):
        raise ValueError("Initial state must contain 13 elements.")

    if not np.all(np.isfinite(initial_state)):
        raise ValueError("Initial state must be finite.")

    if rotor_speeds_rad_s.shape != (4,):
        raise ValueError("Rotor speeds must contain 4 elements.")

    if not np.all(np.isfinite(rotor_speeds_rad_s)):
        raise ValueError("Rotor speeds must be finite.")

    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("Duration must be finite and positive.")

    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("Time step must be finite and positive.")

    number_of_steps = int(round(duration_s / dt))

    if number_of_steps < 1 or not np.isclose(
        number_of_steps * dt, duration_s, rtol=1e-9, atol=1e-12
    ):
        raise ValueError(
            "Duration must be an integer multiple of dt."
        )

    times = np.arange(number_of_steps + 1, dtype=float) * dt
    states = np.zeros((number_of_steps + 1, 13), dtype=float)
    states[0] = initial_state

    for index in range(number_of_steps):
        states[index + 1] = rk4_step(
            derivative=lambda t, state: state_derivative(
                t, state, rotor_speeds_rad_s
            ),
            t=times[index],
            state=states[index],
            dt=dt,
        )

        # Keep the quaternion unit length after each full step.
        quaternion = states[index + 1, 6:10]
        quaternion_norm = np.linalg.norm(quaternion)

        if quaternion_norm < 1e-12:
            raise ValueError("Quaternion norm became too small.")

        states[index + 1, 6:10] = quaternion / quaternion_norm

    return times, states