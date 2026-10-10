import numpy as np

from config.vehicle import (
    MAX_ROTOR_SPEED_RAD_S,
    MOTOR_TIME_CONSTANT_S,
)


def motor_speed_derivative(
    motor_speeds_rad_s,
    commanded_speeds_rad_s,
):
    """
    Calculate rotor speed derivatives using a first-order motor model.

    Parameters
    ----------
    motor_speeds_rad_s : array-like, shape (4,)
        Current rotor speeds in rad/s.

    commanded_speeds_rad_s : array-like, shape (4,)
        Commanded rotor speeds in rad/s.

    Returns
    -------
    numpy.ndarray, shape (4,)
        Rotor speed derivatives in rad/s^2.
    """
    speeds = np.asarray(motor_speeds_rad_s, dtype=float)
    commands = np.asarray(commanded_speeds_rad_s, dtype=float)

    if speeds.shape != (4,) or commands.shape != (4,):
        raise ValueError("Exactly four motor speeds are required.")

    if not np.all(np.isfinite(speeds)) or not np.all(np.isfinite(commands)):
        raise ValueError("Motor speeds must be finite.")

    if np.any(speeds < 0.0) or np.any(commands < 0.0):
        raise ValueError("Motor speeds cannot be negative.")

    if np.any(speeds > MAX_ROTOR_SPEED_RAD_S):
        raise ValueError("Motor speeds exceed the configured maximum.")

    if np.any(commands > MAX_ROTOR_SPEED_RAD_S):
        raise ValueError("Commanded speeds exceed the configured maximum.")

    if MOTOR_TIME_CONSTANT_S <= 0.0:
        raise ValueError("Motor time constant must be positive.")

    return (commands - speeds) / MOTOR_TIME_CONSTANT_S