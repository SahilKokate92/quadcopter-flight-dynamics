import argparse
import csv
import subprocess
import sys
from pathlib import Path

import numpy as np

from pathlib import Path
import sys

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.quadcopter.dynamics import rigid_body_derivatives
from src.quadcopter.integrators import rk4_step


DT = 0.001
STEPS = 1000

INITIAL_STATE = np.array([
    0.0, 0.0, 0.0,       # Position NED (m)
    8.0, 1.0, 0.5,       # Body velocity (m/s)
    1.0, 0.0, 0.0, 0.0,  # Quaternion [qw, qx, qy, qz]
    0.1, -0.2, 0.3,      # Body angular rates (rad/s)
], dtype=float)

FORCE_BODY_N = np.array([1.5, -0.8, -10.0])
TORQUE_BODY_NM = np.array([0.02, -0.015, 0.01])

STATE_COLUMNS = [
    "pn", "pe", "pd",
    "ub", "vb", "wb",
    "qw", "qx", "qy", "qz",
    "p", "q", "r",
]


def python_derivative(_time, state):
    result = rigid_body_derivatives(
        position_ned=state[0:3],
        velocity_body=state[3:6],
        quaternion_nb=state[6:10],
        angular_velocity_body=state[10:13],
        force_body=FORCE_BODY_N,
        torque_body=TORQUE_BODY_NM,
    )

    return np.concatenate([
        result["position_dot_ned"],
        result["velocity_dot_body"],
        result["quaternion_dot_nb"],
        result["angular_velocity_dot_body"],
    ])


def normalize_quaternion(state):
    norm = np.linalg.norm(state[6:10])

    if norm < 1e-12:
        raise RuntimeError("Python quaternion norm became too small.")

    state[6:10] /= norm
    return state


def load_cpp_trajectory(path):
    with path.open(newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)

    times = np.array([float(row["time_s"]) for row in rows])

    states = np.array([
        [float(row[column]) for column in STATE_COLUMNS]
        for row in rows
    ])

    return times, states


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--executable",
        required=True,
        help="Path to the C++ trajectory executable",
    )
    parser.add_argument(
        "--csv",
        required=True,
        help="Path to the CSV output from the C++ executable",
    )
    args = parser.parse_args()

    executable = Path(args.executable)
    csv_path = Path(args.csv)

    subprocess.run(
        [str(executable),],
        check=True,
        stdout=csv_path.open("w", encoding="utf-8", newline=""),
    )

    cpp_times, cpp_states = load_cpp_trajectory(csv_path)

    expected_times = np.arange(STEPS + 1) * DT

    if cpp_times.shape != expected_times.shape:
        raise RuntimeError(
            f"Expected {STEPS + 1} samples; got {len(cpp_times)}."
        )

    if not np.allclose(cpp_times, expected_times, rtol=0.0, atol=1e-12):
        raise RuntimeError("C++ timestamps do not match expected times.")

    python_states = np.zeros_like(cpp_states)
    python_states[0] = INITIAL_STATE

    for index in range(STEPS):
        next_state = rk4_step(
            python_derivative,
            cpp_times[index],
            python_states[index],
            DT,
        )
        python_states[index + 1] = normalize_quaternion(next_state)

    errors = np.abs(cpp_states - python_states)
    max_error_by_state = np.max(errors, axis=0)
    overall_error = float(np.max(errors))

    print("C++ vs Python rigid-body trajectory comparison")
    print(f"Samples: {len(cpp_times)}")
    print(f"Time step: {DT} s")
    print(f"Duration: {cpp_times[-1]:.6f} s")
    print(f"Maximum absolute state error: {overall_error:.12e}")
    print()
    print("Maximum absolute error by state:")

    for name, error in zip(STATE_COLUMNS, max_error_by_state):
        print(f"  {name:>3s}: {error:.12e}")

    tolerance = 1e-10

    if overall_error <= tolerance:
        print(f"\nPASS: error <= {tolerance:.1e}")
    else:
        print(f"\nFAIL: error > {tolerance:.1e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
