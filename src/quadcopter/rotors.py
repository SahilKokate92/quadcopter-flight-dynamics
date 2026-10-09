
import numpy as np

from config.vehicle import (
    THRUST_COEFFICIENT,
    TORQUE_COEFFICIENT,
    MAX_ROTOR_SPEED_RAD_S,
    ROTOR_OFFSET_M,
    ROTOR_SPIN_SIGNS,
)


# Rotor order:
# 1: front-right
# 2: rear-right
# 3: rear-left
# 4: front-left

ROTOR_POSITIONS_B = np.array([
    [ROTOR_OFFSET_M, -ROTOR_OFFSET_M, 0.0],
    [-ROTOR_OFFSET_M, -ROTOR_OFFSET_M, 0.0],
    [-ROTOR_OFFSET_M, ROTOR_OFFSET_M, 0.0],
    [ROTOR_OFFSET_M, ROTOR_OFFSET_M, 0.0],
])


def rotor_forces_and_torques(
    rotor_speeds_rad_s: np.ndarray,
) -> dict:
    """
    Calculate total thrust and body torque from four rotor speeds.

    Rotor speeds are in rad/s.

    Returns:
        thrust_n: total thrust magnitude in newtons
        force_body_n: total force vector in body FRD coordinates
        torque_body_nm: total body torque in N*m
        rotor_thrusts_n: individual rotor thrust magnitudes
    """
    speeds = np.asarray(rotor_speeds_rad_s, dtype=float)

    if speeds.shape != (4,):
        raise ValueError("Exactly four rotor speeds are required.")

    if not np.all(np.isfinite(speeds)):
        raise ValueError("Rotor speeds must be finite.")

    if np.any(speeds < 0.0):
        raise ValueError("Rotor speeds cannot be negative.")

    speeds = np.clip(speeds, 0.0, MAX_ROTOR_SPEED_RAD_S)

    rotor_thrusts = THRUST_COEFFICIENT * speeds**2
    rotor_reaction_torques = (
        TORQUE_COEFFICIENT * speeds**2
        * np.asarray(ROTOR_SPIN_SIGNS, dtype=float)
    )

    # Each rotor produces thrust along negative body z.
    force_body = np.array([
        0.0,
        0.0,
        -np.sum(rotor_thrusts),
    ])

    # Moment from each rotor's position crossed with its thrust force.
    rotor_force_vectors = np.column_stack([
        np.zeros(4),
        np.zeros(4),
        -rotor_thrusts,
    ])

    arm_torques = np.cross(
        ROTOR_POSITIONS_B,
        rotor_force_vectors,
    )

    total_arm_torque = np.sum(arm_torques, axis=0)

    # Signed reaction torque about the body z axis.
    total_reaction_torque = np.array([
        0.0,
        0.0,
        np.sum(rotor_reaction_torques),
    ])

    torque_body = total_arm_torque + total_reaction_torque

    return {
        "thrust_n": float(np.sum(rotor_thrusts)),
        "force_body_n": force_body,
        "torque_body_nm": torque_body,
        "rotor_thrusts_n": rotor_thrusts,
    }