
import numpy as np
import pytest

from src.quadcopter.integrators import rk4_step


def test_rk4_constant_derivative():
    state = np.array([1.0, 2.0, 3.0])

    result = rk4_step(
        derivative=lambda t, x: np.array([2.0, -1.0, 0.5]),
        t=0.0,
        state=state,
        dt=0.1,
    )

    expected = np.array([1.2, 1.9, 3.05])
    np.testing.assert_allclose(result, expected, atol=1e-12)


def test_rk4_exponential_growth():
    # Exact solution: x(t) = exp(t), with x(0) = 1.
    state = np.array([1.0])
    t = 0.0
    dt = 0.1

    for _ in range(10):
        state = rk4_step(
            derivative=lambda t, x: x,
            t=t,
            state=state,
            dt=dt,
        )
        t += dt

    np.testing.assert_allclose(
        state[0],
        np.exp(1.0),
        rtol=1e-6,
    )


def test_rk4_time_dependent_derivative():
    # dx/dt = t; x(0) = 0 gives x(1) = 0.5.
    state = np.array([0.0])

    result = rk4_step(
        derivative=lambda t, x: np.array([t]),
        t=0.0,
        state=state,
        dt=1.0,
    )

    np.testing.assert_allclose(result, [0.5], atol=1e-12)


def test_rk4_does_not_modify_original_state():
    state = np.array([1.0, 2.0])
    original = state.copy()

    rk4_step(
        derivative=lambda t, x: x,
        t=0.0,
        state=state,
        dt=0.1,
    )

    np.testing.assert_array_equal(state, original)


def test_rk4_rejects_nonpositive_time_step():
    state = np.array([1.0])

    for dt in (0.0, -0.1):
        with pytest.raises(ValueError):
            rk4_step(
                derivative=lambda t, x: x,
                t=0.0,
                state=state,
                dt=dt,
            )


def test_rk4_rejects_wrong_derivative_shape():
    with pytest.raises(ValueError):
        rk4_step(
            derivative=lambda t, x: np.array([1.0, 2.0]),
            t=0.0,
            state=np.array([1.0]),
            dt=0.1,
        )


def test_rk4_rejects_nonfinite_derivative():
    with pytest.raises(ValueError):
        rk4_step(
            derivative=lambda t, x: np.array([np.nan]),
            t=0.0,
            state=np.array([1.0]),
            dt=0.1,
        )

def test_rk4_fourth_order_convergence():
    def integrate(dt):
        state = np.array([1.0])
        t = 0.0

        while t < 1.0 - 1e-12:
            state = rk4_step(
                derivative=lambda time, x: x,
                t=t,
                state=state,
                dt=dt,
            )
            t += dt

        return abs(state[0] - np.exp(1.0))

    error_coarse = integrate(0.1)
    error_fine = integrate(0.05)

    # Halving the time step should reduce RK4 error
    # by approximately a factor of 16 in the asymptotic regime.
    assert error_coarse / error_fine > 12.0
