import numpy as np
import pytest

from config.vehicle import MOTOR_TIME_CONSTANT_S
from src.quadcopter.motors import motor_speed_derivative


def test_motor_derivative_is_zero_at_commanded_speed():
    speeds = np.full(4, 500.0)

    result = motor_speed_derivative(speeds, speeds)

    np.testing.assert_array_equal(result, np.zeros(4))


def test_motor_accelerates_toward_higher_command():
    speeds = np.full(4, 400.0)
    commands = np.full(4, 600.0)

    result = motor_speed_derivative(speeds, commands)

    np.testing.assert_allclose(
        result,
        np.full(4, 200.0 / MOTOR_TIME_CONSTANT_S),
    )


def test_motor_decelerates_toward_lower_command():
    speeds = np.full(4, 600.0)
    commands = np.full(4, 400.0)

    result = motor_speed_derivative(speeds, commands)

    assert np.all(result < 0.0)


def test_motor_rejects_negative_speed():
    speeds = np.array([400.0, 400.0, -1.0, 400.0])
    commands = np.full(4, 500.0)

    with pytest.raises(ValueError):
        motor_speed_derivative(speeds, commands)


def test_motor_rejects_wrong_shape():
    with pytest.raises(ValueError):
        motor_speed_derivative(
            np.array([400.0, 400.0, 400.0]),
            np.full(4, 500.0),
        )