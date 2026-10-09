
import numpy as np

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    INERTIA_KG_M2,
)

from src.quadcopter.frames import (
    normalize_quaternion,
    quaternion_multiply,
    quaternion_to_rotation_matrix,
)


def rigid_body_derivatives(
    position_ned: np.ndarray,
    velocity_body: np.ndarray,
    quaternion_nb: np.ndarray,
    angular_velocity_body: np.ndarray,
    force_body: np.ndarray,
    torque_body: np.ndarray,
) -> dict:
    """
    Calculate the derivatives of a rigid quadcopter's 6-DOF state.

    Frames:
        Inertial: NED (North-East-Down)
        Body: FRD (Forward-Right-Down)

    Quaternion:
        Scalar-first [w, x, y, z]

    Force and torque:
        Expressed in body coordinates.
    """

    position_ned = np.asarray(position_ned, dtype=float)
    velocity_body = np.asarray(velocity_body, dtype=float)
    angular_velocity_body = np.asarray(
        angular_velocity_body, dtype=float
    )
    force_body = np.asarray(force_body, dtype=float)
    torque_body = np.asarray(torque_body, dtype=float)

    quaternion_nb = normalize_quaternion(quaternion_nb)

    rotation_nb = quaternion_to_rotation_matrix(quaternion_nb)

    gravity_ned = np.array([
        0.0,
        0.0,
        GRAVITY_M_S2,
    ])

    inertia = np.asarray(INERTIA_KG_M2, dtype=float)

    # Position derivative in the NED frame.
    position_dot_ned = rotation_nb @ velocity_body

    # Gravity transformed from NED into body coordinates.
    gravity_body = rotation_nb.T @ gravity_ned

    # Translational acceleration in body coordinates.
    velocity_dot_body = (
        force_body / MASS_KG
        + gravity_body
        - np.cross(angular_velocity_body, velocity_body)
    )

    # Rotational acceleration from Euler's rigid-body equation.
    angular_momentum = inertia @ angular_velocity_body

    angular_velocity_dot_body = np.linalg.solve(
        inertia,
        torque_body
        - np.cross(angular_velocity_body, angular_momentum),
    )

    # Quaternion derivative: q_dot = 0.5 * q ⊗ [0, omega_body].
    omega_quaternion = np.array([
        0.0,
        angular_velocity_body[0],
        angular_velocity_body[1],
        angular_velocity_body[2],
    ])

    quaternion_dot_nb = 0.5 * quaternion_multiply(
        quaternion_nb,
        omega_quaternion,
    )

    return {
        "position_dot_ned": position_dot_ned,
        "velocity_dot_body": velocity_dot_body,
        "quaternion_dot_nb": quaternion_dot_nb,
        "angular_velocity_dot_body": angular_velocity_dot_body,
    }