import numpy as np


STATE_SIZE = 17


def pack_state(
    position_ned,
    velocity_body,
    quaternion_nb,
    angular_velocity_body,
    rotor_speeds_rad_s,
):
    """Combine vehicle and motor variables into a 17-element state."""

    variables = {
        "position_ned": (position_ned, (3,)),
        "velocity_body": (velocity_body, (3,)),
        "quaternion_nb": (quaternion_nb, (4,)),
        "angular_velocity_body": (angular_velocity_body, (3,)),
        "rotor_speeds_rad_s": (rotor_speeds_rad_s, (4,)),
    }

    arrays = {}

    for name, (value, expected_shape) in variables.items():
        value = np.asarray(value, dtype=float)

        if value.shape != expected_shape:
            raise ValueError(
                f"{name} must have shape {expected_shape}."
            )

        if not np.all(np.isfinite(value)):
            raise ValueError(
                f"{name} must contain only finite values."
            )

        arrays[name] = value

    return np.concatenate([
        arrays["position_ned"],
        arrays["velocity_body"],
        arrays["quaternion_nb"],
        arrays["angular_velocity_body"],
        arrays["rotor_speeds_rad_s"],
    ])


def unpack_state(state):
    """Extract vehicle and motor variables from a 17-element state."""

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
        "rotor_speeds_rad_s": state[13:17],
    }