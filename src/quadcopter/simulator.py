
import numpy as np

from src.quadcopter.integrators import rk4_step
from src.quadcopter.simulation import state_derivative
from src.quadcopter.state import STATE_SIZE


def simulate(
    initial_state,
    commanded_rotor_speeds_rad_s,
    duration_s,
    dt,
    command_function=None,
):
    """
    Simulate quadcopter motion with first-order motor dynamics.

    Args:
        initial_state:
            Initial 17-element state vector.
        commanded_rotor_speeds_rad_s:
            Four fixed rotor-speed commands in rad/s.
            Used when command_function is None.
        duration_s:
            Total simulation duration in seconds.
        dt:
            Integration time step in seconds.
        command_function:
            Optional callable command_function(t) returning
            four commanded rotor speeds at time t.

    Returns:
        times:
            Simulation timestamps.
        states:
            State history with shape (number_of_steps + 1, 17).
    """

    initial_state = np.asarray(initial_state, dtype=float)
    commanded_rotor_speeds_rad_s = np.asarray(
        commanded_rotor_speeds_rad_s,
        dtype=float,
    )

    # Validate initial state.
    if initial_state.shape != (STATE_SIZE,):
        raise ValueError(
            f"Initial state must contain {STATE_SIZE} elements."
        )

    if not np.all(np.isfinite(initial_state)):
        raise ValueError("Initial state must be finite.")

    # Validate initial quaternion.
    quaternion_norm = np.linalg.norm(initial_state[6:10])

    if not np.isclose(
        quaternion_norm, 1.0, rtol=1e-9, atol=1e-12
    ):
        raise ValueError("Initial quaternion must have unit norm.")

    # Validate initial rotor speeds.
    initial_rotor_speeds = initial_state[13:17]

    if np.any(initial_rotor_speeds < 0.0):
        raise ValueError("Initial rotor speeds cannot be negative.")

    # Validate fixed commands.
    if commanded_rotor_speeds_rad_s.shape != (4,):
        raise ValueError("Rotor commands must contain 4 elements.")

    if not np.all(np.isfinite(commanded_rotor_speeds_rad_s)):
        raise ValueError("Rotor commands must be finite.")

    # Validate simulation duration and time step.
    if not np.isfinite(duration_s) or duration_s <= 0.0:
        raise ValueError("Duration must be finite and positive.")

    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("Time step must be finite and positive.")

    number_of_steps = int(round(duration_s / dt))

    if number_of_steps < 1 or not np.isclose(
        number_of_steps * dt,
        duration_s,
        rtol=1e-9,
        atol=1e-12,
    ):
        raise ValueError(
            "Duration must be an integer multiple of dt."
        )

    # Resolve and validate commands at each RK4 evaluation time.
    def get_commands(t):
        if command_function is None:
            commands = commanded_rotor_speeds_rad_s
        else:
            commands = command_function(t)

        commands = np.asarray(commands, dtype=float)

        if commands.shape != (4,):
            raise ValueError(
                "Command function must return 4 rotor speeds."
            )

        if not np.all(np.isfinite(commands)):
            raise ValueError(
                "Commanded rotor speeds must be finite."
            )

        if np.any(commands < 0.0):
            raise ValueError(
                "Commanded rotor speeds cannot be negative."
            )

        return commands

    # Allocate simulation history.
    times = np.arange(number_of_steps + 1, dtype=float) * dt
    states = np.zeros(
        (number_of_steps + 1, STATE_SIZE),
        dtype=float,
    )
    states[0] = initial_state

    # Integrate vehicle and motor dynamics together.
    for index in range(number_of_steps):
        states[index + 1] = rk4_step(
            derivative=lambda t, state: state_derivative(
                t,
                state,
                get_commands(t),
            ),
            t=times[index],
            state=states[index],
            dt=dt,
        )

        # Normalize quaternion after each full integration step.
        quaternion = states[index + 1, 6:10]
        quaternion_norm = np.linalg.norm(quaternion)

        if quaternion_norm < 1e-12:
            raise ValueError("Quaternion norm became too small.")

        states[index + 1, 6:10] = (
            quaternion / quaternion_norm
        )

    return times, states
