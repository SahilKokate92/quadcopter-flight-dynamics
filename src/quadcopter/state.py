import numpy as np


STATE_SIZE = 13


def pack_state(
    position_ned,
    velocity_body,
    quaternion_nb,
    angular_velocity_body,
):
    """Combine the quadcopter variables into a 13-element state vector."""

    position_ned = np.asarray(position_ned, dtype=float)
    velocity_body = np.asarray(velocity_body, dtype=float)
    quaternion_nb = np.asarray(quaternion_nb, dtype=float)
    angular_velocity_body = np.asarray(
        angular_velocity_body, dtype=float
    )

    expected_shapes = {
        "position_ned": (3,),
        "velocity_body": (3,),
        "quaternion_nb": (4,),
        "angular_velocity_body": (3,),
    }

    variables = {
        "position_ned": position_ned,
        "velocity_body": velocity_body,
        "quaternion_nb": quaternion_nb,
        "angular_velocity_body": angular_velocity_body,
    }

    for name, expected_shape in expected_shapes.items():
        value = variables[name]

        if value.shape != expected_shape:
            raise ValueError(
                f"{name} must have shape {expected_shape}."
            )

        if not np.all(np.isfinite(value)):
            raise ValueError(
                f"{name} must contain only finite values."
            )

    return np.concatenate([
        position_ned,
        velocity_body,
        quaternion_nb,
        angular_velocity_body,
    ])


def unpack_state(state):
    """Extract the variables from a 13-element state vector."""

    state = np.asarray(state, dtype=float)

    if state.shape != (STATE_SIZE,):
        raise ValueError(
            f"State must have shape ({STATE_SIZE},)."
        )

    if not np.all(np.isfinite(state)):
        raise ValueError(
            "State must contain only finite values."
        )

    return {
        "position_ned": state[0:3],
        "velocity_body": state[3:6],
        "quaternion_nb": state[6:10],
        "angular_velocity_body": state[10:13],
    }
