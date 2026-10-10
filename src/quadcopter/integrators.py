
import numpy as np


def rk4_step(derivative, t, state, dt):
    """
    Advance a state vector by one fourth-order Runge-Kutta step.

    Parameters
    ----------
    derivative : callable
        Function f(t, state) that returns the state derivative.
    t : float
        Current simulation time in seconds.
    state : array_like
        Current state vector.
    dt : float
        Integration time step in seconds.

    Returns
    -------
    numpy.ndarray
        Estimated state at time t + dt.
    """

    state = np.asarray(state, dtype=float)

    if state.ndim != 1:
        raise ValueError("State must be a one-dimensional array.")

    if state.size == 0:
        raise ValueError("State cannot be empty.")

    if not np.all(np.isfinite(state)):
        raise ValueError("State must contain only finite values.")

    if not np.isfinite(t):
        raise ValueError("Time must be finite.")

    if not np.isfinite(dt) or dt <= 0.0:
        raise ValueError("Time step must be finite and positive.")

    def evaluate(time, value):
        result = np.asarray(
            derivative(time, value),
            dtype=float,
        )

        if result.shape != state.shape:
            raise ValueError(
                "Derivative must have the same shape as the state."
            )

        if not np.all(np.isfinite(result)):
            raise ValueError(
                "Derivative must contain only finite values."
            )

        return result

    k1 = evaluate(t, state)
    k2 = evaluate(t + dt / 2.0, state + dt * k1 / 2.0)
    k3 = evaluate(t + dt / 2.0, state + dt * k2 / 2.0)
    k4 = evaluate(t + dt, state + dt * k3)

    next_state = state + (dt / 6.0) * (
        k1 + 2.0 * k2 + 2.0 * k3 + k4
    )

    return next_state
