import argparse
import csv
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

from src.quadcopter.integrators import rk4_step
from src.quadcopter.simulation import state_derivative


STATE_NAMES = [
    "pn_m", "pe_m", "pd_m",
    "ub_m_s", "vb_m_s", "wb_m_s",
    "qw", "qx", "qy", "qz",
    "p_rad_s", "q_rad_s", "r_rad_s",
    "rotor1_rad_s", "rotor2_rad_s",
    "rotor3_rad_s", "rotor4_rad_s",
]

# Column names used by the existing C++ trajectory CSV.
CSV_COLUMNS = [
    "pn", "pe", "pd",
    "ub", "vb", "wb",
    "qw", "qx", "qy", "qz",
    "p", "q", "r",
    "rotor1_rad_s", "rotor2_rad_s",
    "rotor3_rad_s", "rotor4_rad_s",
]


def normalize_quaternion(state):
    """Normalize the scalar-first quaternion in a copied 17-state vector."""
    state = state.copy()
    quaternion = state[6:10]
    norm = np.linalg.norm(quaternion)

    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError(
            "Invalid quaternion encountered in Python simulation."
        )

    state[6:10] = quaternion / norm
    return state


def run_python_trajectory(dt, steps):
    """Run the Python reference with the same initial conditions and inputs."""
    state = np.array([
        0.0, 0.0, 0.0,          # Position NED (m)
        8.0, 1.0, 0.5,          # Body velocity (m/s)
        1.0, 0.0, 0.0, 0.0,     # Quaternion [qw, qx, qy, qz]
        0.1, -0.2, 0.3,         # Body angular velocity (rad/s)
        0.0, 0.0, 0.0, 0.0,     # Rotor speeds (rad/s)
    ], dtype=float)

    commands = np.array([600.0, 600.0, 600.0, 600.0])

    times = [0.0]
    states = [state.copy()]

    def derivative(t, x):
        return state_derivative(t, x, commands)

    for i in range(steps):
        t = i * dt
        state = rk4_step(derivative, t, state, dt)

        # Match the C++ integrator's quaternion normalization after each step.
        state = normalize_quaternion(state)

        times.append((i + 1) * dt)
        states.append(state.copy())

    return np.asarray(times), np.asarray(states)


def load_cpp_csv(path):
    """Load the C++ trajectory using its existing CSV column names."""
    with path.open("r", newline="") as file:
        reader = csv.DictReader(file)
        rows = list(reader)

    if not rows:
        raise ValueError(f"No trajectory rows found in {path}")

    required_columns = ["time_s"] + CSV_COLUMNS
    missing = [
        name for name in required_columns
        if name not in (reader.fieldnames or [])
    ]

    if missing:
        raise ValueError(f"C++ CSV is missing columns: {missing}")

    times = np.array([float(row["time_s"]) for row in rows])
    states = np.array([
        [float(row[name]) for name in CSV_COLUMNS]
        for row in rows
    ])

    if not np.all(np.isfinite(times)):
        raise ValueError("C++ CSV contains non-finite time values.")

    if not np.all(np.isfinite(states)):
        raise ValueError("C++ CSV contains non-finite state values.")

    return times, states


def main():
    parser = argparse.ArgumentParser(
        description="Compare full C++ and Python 17-state trajectories."
    )

    parser.add_argument(
        "--csv",
        default="controller/cpp/build/cpp_quadcopter_trajectory.csv",
        help="Path to the C++ trajectory CSV, relative to project root.",
    )
    parser.add_argument("--dt", type=float, default=0.001)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--tolerance", type=float, default=1e-10)

    args = parser.parse_args()

    if not np.isfinite(args.dt) or args.dt <= 0.0:
        parser.error("--dt must be finite and positive.")

    if not np.isfinite(args.duration) or args.duration <= 0.0:
        parser.error("--duration must be finite and positive.")

    if not np.isfinite(args.tolerance) or args.tolerance <= 0.0:
        parser.error("--tolerance must be finite and positive.")

    step_count_float = args.duration / args.dt
    steps = round(step_count_float)

    if not np.isclose(
        step_count_float, steps, rtol=0.0, atol=1e-9
    ):
        parser.error("Duration must be an integer multiple of dt.")

    csv_path = (PROJECT_ROOT / args.csv).resolve()

    if not csv_path.is_file():
        parser.error(f"C++ trajectory CSV not found: {csv_path}")

    try:
        cpp_times, cpp_states = load_cpp_csv(csv_path)
        py_times, py_states = run_python_trajectory(args.dt, steps)
    except (ValueError, OSError) as exc:
        print(f"FAIL: {exc}")
        return 1

    if cpp_states.shape[1] != len(STATE_NAMES):
        print(
            "FAIL: unexpected number of C++ state columns "
            f"({cpp_states.shape[1]}; expected {len(STATE_NAMES)})."
        )
        return 1

    if len(cpp_times) != len(py_times):
        print(
            "FAIL: sample count differs "
            f"(C++={len(cpp_times)}, Python={len(py_times)})."
        )
        return 1

    if cpp_states.shape != py_states.shape:
        print(
            "FAIL: state-array shapes differ "
            f"(C++={cpp_states.shape}, Python={py_states.shape})."
        )
        return 1

    time_error = float(np.max(np.abs(cpp_times - py_times)))
    state_errors = np.max(np.abs(cpp_states - py_states), axis=0)

    print(f"C++ samples: {len(cpp_times)}")
    print(f"Python samples: {len(py_times)}")
    print(f"Duration: {args.duration:.6f} s")
    print(f"Time step: {args.dt:.6f} s")
    print(f"Maximum time error: {time_error:.12e} s")
    print()
    print("Maximum absolute error by state:")

    for name, error in zip(STATE_NAMES, state_errors):
        print(f"  {name:18s}: {error:.12e}")

    maximum_state_error = float(np.max(state_errors))

    print()
    print(f"Maximum state error: {maximum_state_error:.12e}")
    print(f"Tolerance:           {args.tolerance:.12e}")

    if time_error > args.tolerance:
        print("FAIL: time arrays differ beyond tolerance.")
        return 1

    if not np.all(np.isfinite(state_errors)):
        print("FAIL: non-finite state error detected.")
        return 1

    if maximum_state_error > args.tolerance:
        print("FAIL: trajectory mismatch exceeds tolerance.")
        return 1

    print("PASS: C++ and Python trajectories agree within tolerance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())