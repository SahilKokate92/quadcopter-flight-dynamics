import numpy as np

from src.quadcopter.dynamics import rigid_body_derivatives
from src.quadcopter.rotors import rotor_forces_and_torques


def vehicle_derivatives(
    position_ned: np.ndarray,
    velocity_body: np.ndarray,
    quaternion_nb: np.ndarray,
    angular_velocity_body: np.ndarray,
    rotor_speeds_rad_s: np.ndarray,
) -> dict:
    """
    Calculate quadcopter state derivatives from rotor speeds.
    """

    propulsion = rotor_forces_and_torques(rotor_speeds_rad_s)

    dynamics = rigid_body_derivatives(
        position_ned=position_ned,
        velocity_body=velocity_body,
        quaternion_nb=quaternion_nb,
        angular_velocity_body=angular_velocity_body,
        force_body=propulsion["force_body_n"],
        torque_body=propulsion["torque_body_nm"],
    )

    return {
        **dynamics,
        **propulsion,
    }