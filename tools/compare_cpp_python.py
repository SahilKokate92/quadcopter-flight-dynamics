import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.quadcopter.dynamics import rigid_body_derivatives


ATOL = 1e-10
RTOL = 1e-10

# Every test uses:
# position(3), velocity(3), quaternion(4),
# angular velocity(3), force(3), torque(3).
CASES = {
    "level_static": {
        "position": [0.0, 0.0, 0.0],
        "velocity": [0.0, 0.0, 0.0],
        "quaternion": [1.0, 0.0, 0.0, 0.0],
        "omega": [0.0, 0.0, 0.0],
        "force": [0.0, 0.0, -1.2 * 9.80665],
        "torque": [0.0, 0.0, 0.0],
    },
    "translated_rotating": {
        "position": [10.0, -2.0, 3.0],
        "velocity": [12.0, -1.5, 0.7],
        "quaternion": [0.9238795325112867, 0.0, 0.0,
                       0.3826834323650898],
        "omega": [0.2, -0.3, 0.4],
        "force": [1.5, -0.8, -10.0],
        "torque": [0.02, -0.01, 0.03],
    },
    "coupled_rotation": {
        "position": [-1.0, 4.0, 2.5],
        "velocity": [3.0, 2.0, -0.5],
        "quaternion": [0.8, 0.2, -0.3, 0.4],
        "omega": [0.0, 2.0, 3.0],
        "force": [0.5, 1.2, -8.0],
        "torque": [0.0, 0.0, 0.0],
    },
}


def flatten_case(case):
    return (
        case["position"]
        + case["velocity"]
        + case["quaternion"]
        + case["omega"]
        + case["force"]
        + case["torque"]
    )


def python_derivatives(case):
    result = rigid_body_derivatives(
        position_ned=np.asarray(case["position"], dtype=float),
        velocity_body=np.asarray(case["velocity"], dtype=float),
        quaternion_nb=np.asarray(case["quaternion"], dtype=float),
        angular_velocity_body=np.asarray(case["omega"], dtype=float),
        force_body=np.asarray(case["force"], dtype=float),
        torque_body=np.asarray(case["torque"], dtype=float),
    )

    return np.concatenate([
        result["position_dot_ned"],
        result["velocity_dot_body"],
        result["quaternion_dot_nb"],
        result["angular_velocity_dot_body"],
    ])


def cpp_derivatives(executable, case):
    arguments = [str(executable)]
    arguments.extend(str(value) for value in flatten_case(case))

    completed = subprocess.run(
        arguments,
        check=True,
        capture_output=True,
        text=True,
    )

    values = np.fromstring(completed.stdout.strip(), sep=",")

    if values.size != 13:
        raise RuntimeError(
            f"C++ executable returned {values.size} values; expected 13. "
            f"Output: {completed.stdout!r}"
        )

    if not np.all(np.isfinite(values)):
        raise RuntimeError("C++ output contains non-finite values.")

    return values


def main():
    parser = argparse.ArgumentParser(
        description="Compare Python and C++ rigid-body derivatives."
    )
    parser.add_argument(
        "--executable",
        required=True,
        help="Path to the compiled C++ comparison executable.",
    )
    args = parser.parse_args()

    executable = Path(args.executable).resolve()

    if not executable.is_file():
        raise FileNotFoundError(
            f"C++ executable not found: {executable}"
        )

    all_passed = True

    for name, case in CASES.items():
        python_values = python_derivatives(case)
        cpp_values = cpp_derivatives(executable, case)

        differences = np.abs(python_values - cpp_values)
        max_error = float(np.max(differences))

        passed = np.allclose(
            python_values,
            cpp_values,
            atol=ATOL,
            rtol=RTOL,
        )

        print(
            f"{name}: {'PASS' if passed else 'FAIL'} "
            f"(maximum absolute error = {max_error:.17g})"
        )

        if not passed:
            all_passed = False
            for index, (python_value, cpp_value) in enumerate(
                zip(python_values, cpp_values)
            ):
                if not np.isclose(
                    python_value,
                    cpp_value,
                    atol=ATOL,
                    rtol=RTOL,
                ):
                    print(
                        f"  component {index}: "
                        f"Python={python_value:.17g}, "
                        f"C++={cpp_value:.17g}, "
                        f"error={abs(python_value - cpp_value):.17g}"
                    )

    print(
        "\nOverall result: "
        + ("PASS" if all_passed else "FAIL")
    )

    return 0 if all_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())