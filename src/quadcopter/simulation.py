from src.quadcopter.state import pack_state, unpack_state
from src.quadcopter.vehicle import vehicle_derivatives


def state_derivative(t, state, rotor_speeds_rad_s):
    """
    Calculate derivatives of the 13-state quadcopter vector.

    State layout:
        0:3   Position NED (m)
        3:6   Velocity body (m/s)
        6:10  Quaternion [qw, qx, qy, qz]
        10:13 Angular velocity body (rad/s)

    Rotor speeds are fixed inputs at this stage.
    """
    del t  # Dynamics do not explicitly depend on time.

    variables = unpack_state(state)

    result = vehicle_derivatives(
        position_ned=variables["position_ned"],
        velocity_body=variables["velocity_body"],
        quaternion_nb=variables["quaternion_nb"],
        angular_velocity_body=variables["angular_velocity_body"],
        rotor_speeds_rad_s=rotor_speeds_rad_s,
    )

    return pack_state(
        position_ned=result["position_dot_ned"],
        velocity_body=result["velocity_dot_body"],
        quaternion_nb=result["quaternion_dot_nb"],
        angular_velocity_body=result["angular_velocity_dot_body"],
    )