import numpy as np
import pytest

from src.quadcopter.state import (
    STATE_SIZE,
    pack_state,
    unpack_state,
)


def test_pack_and_unpack_state():
    position = np.array([1.0, 2.0, 3.0])
    velocity = np.array([4.0, 5.0, 6.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    angular_velocity = np.array([0.1, 0.2, 0.3])
    rotor_speeds = np.array([400.0, 410.0, 420.0, 430.0])

    state = pack_state(
        position,
        velocity,
        quaternion,
        angular_velocity,
        rotor_speeds,
    )

    result = unpack_state(state)

    assert STATE_SIZE == 17
    assert state.shape == (17,)
    np.testing.assert_array_equal(result["position_ned"], position)
    np.testing.assert_array_equal(result["velocity_body"], velocity)
    np.testing.assert_array_equal(result["quaternion_nb"], quaternion)
    np.testing.assert_array_equal(
        result["angular_velocity_body"], angular_velocity
    )
    np.testing.assert_array_equal(
        result["rotor_speeds_rad_s"], rotor_speeds
    )


def test_pack_state_rejects_wrong_shape():
    with pytest.raises(ValueError):
        pack_state(
            np.zeros(2),
            np.zeros(3),
            np.array([1.0, 0.0, 0.0, 0.0]),
            np.zeros(3),
            np.zeros(4),
        )


def test_unpack_state_rejects_wrong_size():
    with pytest.raises(ValueError):
        unpack_state(np.zeros(13))


def test_unpack_state_rejects_nonfinite_values():
    state = np.zeros(17)
    state[6] = 1.0
    state[15] = np.nan

    with pytest.raises(ValueError):
        unpack_state(state)