import numpy as np
import pytest

from src.quadcopter.state import pack_state, unpack_state


def test_pack_and_unpack_state():
    position = np.array([1.0, 2.0, 3.0])
    velocity = np.array([4.0, 5.0, 6.0])
    quaternion = np.array([1.0, 0.0, 0.0, 0.0])
    angular_velocity = np.array([0.1, 0.2, 0.3])

    state = pack_state(
        position, velocity, quaternion, angular_velocity
    )
    result = unpack_state(state)

    assert state.shape == (13,)
    np.testing.assert_array_equal(result["position_ned"], position)
    np.testing.assert_array_equal(result["velocity_body"], velocity)
    np.testing.assert_array_equal(result["quaternion_nb"], quaternion)
    np.testing.assert_array_equal(
        result["angular_velocity_body"], angular_velocity
    )


def test_pack_state_rejects_wrong_shape():
    with pytest.raises(ValueError):
        pack_state(
            [1.0, 2.0],
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0],
        )


def test_unpack_state_rejects_wrong_size():
    with pytest.raises(ValueError):
        unpack_state(np.zeros(12))


def test_unpack_state_rejects_nonfinite_values():
    state = np.zeros(13)
    state[0] = np.nan

    with pytest.raises(ValueError):
        unpack_state(state)