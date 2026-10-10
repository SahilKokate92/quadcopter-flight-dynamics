import numpy as np

from src.quadcopter.state import pack_state, unpack_state
from src.quadcopter.vehicle import vehicle_derivatives
from src.quadcopter.motors import motor_speed_derivative


def state_derivative(t, state, commanded_rotor_speeds_rad_s):
    """
    Calculate derivatives of the 17-state quadcopter model.

    State layout:
        0:3   Position NED (m)
        3:6   Velocity body (m/s)
        6:10  Quaternion [qw, qx, qy, qz]
        10:13 Angular velocity body (rad/s)
        13:17 Actual rotor speeds (rad/s)

    Commanded rotor speeds are external inputs.
    """
    del t  # The current model is autonomous for fixed commands.

    variables = unpack_state(state)

    actual_speeds = variables["rotor_speeds_rad_s"]
    commanded_speeds = np.asarray(
        commanded_rotor_speeds_rad_s, dtype=float
    )

    if commanded_speeds.shape != (4,):
        raise ValueError("Commanded rotor speeds must contain 4 elements.")

    if not np.all(np.isfinite(commanded_speeds)):
        raise ValueError("Commanded rotor speeds must be finite.")

    result = vehicle_derivatives(
        position_ned=variables["position_ned"],
        velocity_body=variables["velocity_body"],
        quaternion_nb=variables["quaternion_nb"],
        angular_velocity_body=variables["angular_velocity_body"],
        rotor_speeds_rad_s=actual_speeds,
    )

    motor_dot = motor_speed_derivative(
        actual_speeds,
        commanded_speeds,
    )

    return pack_state(
        position_ned=result["position_dot_ned"],
        velocity_body=result["velocity_dot_body"],
        quaternion_nb=result["quaternion_dot_nb"],
        angular_velocity_body=result["angular_velocity_dot_body"],
        rotor_speeds_rad_s=motor_dot,
    )