import csv
import subprocess
import sys
from pathlib import Path

import numpy as np

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from src.quadcopter.integrators import rk4_step
from src.quadcopter.simulation import state_derivative


DT = 0.001
STEPS = 1000
TOLERANCE = 1e-10

COMMANDS = np.array([650.0, 600.0, 600.0, 600.0])

STATE_NAMES = [
    "pn_m", "pe_m", "pd_m",
    "ub_m_s", "vb_m_s", "wb_m_s",
    "qw", "qx", "qy", "qz",
    "p_rad_s", "q_rad_s", "r_rad_s",
    "rotor1_rad_s", "rotor2_rad_s",
    "rotor3_rad_s", "rotor4_rad_s",
]

CSV_COLUMNS = [
    "pn", "pe", "pd",
    "ub", "vb", "wb",
    "qw", "qx", "qy", "qz",
    "p", "q", "r",
    "rotor1_rad_s", "rotor2_rad_s",
    "rotor3_rad_s", "rotor4_rad_s",
]


def normalize_quaternion(state):
    state = state.copy()
    quaternion = state[6:10]
    norm = np.linalg.norm(quaternion)

    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError("Invalid quaternion in Python simulation.")

    state[6:10] = quaternion / norm
    return state


def run_python_simulation():
    state = np.array([
        0.0, 0.0, 0.0,
        8.0, 1.0, 0.5,
        1.0, 0.0, 0.0, 0.0,
        0.1, -0.2, 0.3,
        0.0, 0.0, 0.0, 0.0,
    ], dtype=float)

    states = [state.copy()]

    def derivative(time, current_state):
        return state_derivative(time, current_state, COMMANDS)

    for index in range(STEPS):
        time = index * DT
        state = rk4_step(derivative, time, state, DT)
        state = normalize_quaternion(state)
        states.append(state.copy())

    return np.asarray(states)


def load_cpp_trajectory(csv_path):
    with csv_path.open("r", newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        rows = list(reader)

        if not rows:
            raise ValueError("C++ trajectory CSV is empty.")

        required = ["time_s"] + CSV_COLUMNS
        missing = [column for column in required
                   if column not in (reader.fieldnames or [])]

        if missing:
            raise ValueError(f"Missing C++ CSV columns: {missing}")

    times = np.array([float(row["time_s"]) for row in rows])
    states = np.array([
        [float(row[column]) for column in CSV_COLUMNS]
        for row in rows
    ])

    if not np.all(np.isfinite(times)):
        raise ValueError("C++ times contain non-finite values.")

    if not np.all(np.isfinite(states)):
        raise ValueError("C++ states contain non-finite values.")

    return times, states


def main():
    executable = (
        PROJECT_ROOT
        / "controller"
        / "cpp"
        / "build"
        / "print_asymmetric_trajectory.exe"
    )

    csv_path = (
        PROJECT_ROOT
        / "controller"
        / "cpp"
        / "build"
        / "cpp_asymmetric_trajectory.csv"
    )

    if not executable.is_file():
        print(f"FAIL: executable not found: {executable}")
        return 1

    try:
        with csv_path.open("w", newline="", encoding="utf-8") as output:
            subprocess.run(
                [str(executable)],
                stdout=output,
                check=True,
                text=True,
            )

        cpp_times, cpp_states = load_cpp_trajectory(csv_path)
        python_states = run_python_simulation()

    except (OSError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"FAIL: {exc}")
        return 1

    expected_times = np.arange(STEPS + 1) * DT

    if cpp_states.shape != python_states.shape:
        print(
            "FAIL: state dimensions differ: "
            f"C++={cpp_states.shape}, Python={python_states.shape}"
        )
        return 1

    if not np.allclose(cpp_times, expected_times, rtol=0.0, atol=1e-12):
        print("FAIL: C++ timestamps differ from expected timestamps.")
        return 1

    errors = np.abs(cpp_states - python_states)
    errors_by_state = np.max(errors, axis=0)
    maximum_error = float(np.max(errors_by_state))

    print("Asymmetric rotor response: C++ vs Python")
    print(f"Commands (rad/s): {COMMANDS.tolist()}")
    print(f"Duration: {STEPS * DT:.3f} s")
    print(f"Time step: {DT:.3f} s")
    print(f"Samples per model: {len(cpp_times)}")
    print()
    print("Maximum absolute error by state:")

    for name, error in zip(STATE_NAMES, errors_by_state):
        print(f"  {name:18s}: {error:.12e}")

    print()
    print(f"Maximum state error: {maximum_error:.12e}")
    print(f"Tolerance:           {TOLERANCE:.12e}")

    if maximum_error > TOLERANCE:
        print("FAIL: C++ and Python trajectories differ beyond tolerance.")
        return 1

    print("PASS: C++ and Python trajectories agree within tolerance.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
