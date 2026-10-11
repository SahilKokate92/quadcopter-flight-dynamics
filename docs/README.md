# Technical Documentation

This directory contains the technical documentation for the Quadcopter Flight Dynamics project.

The documents cover the numerical model, C++ tests, open-loop simulation workflow, and the assumptions and limitations of the current implementation.

## Contents

1. [Document map](#document-map)
2. [Documentation](#documentation)
3. [Quick Start](#quick-start)
4. [Documentation Status](#documentation-status)

---

## Document map

| Area | Document | Scope |
|---|---|---|
| Model | [Mathematical Model](mathematical_model.md) | Kinematics, dynamics, rotor and motor models, and integration. |
| Model | [Numerical Model](NUMERICAL_MODEL.md) | C++ implementation of the 17-state model. |
| Model | [Coordinate Frames](coordinate_frames.md) | NED and FRD conventions, quaternion convention, state indexing, rotor ordering. |
| Model | [Software Architecture](architecture.md) | Directories, state flow, state vector, and design boundaries. |
| Verification | [C++ Test Suite](CPP_TESTS.md) | Objectives, reference cases, and acceptance criteria for each C++ test. |
| Verification | [Python–C++ Derivative Validation](python_cpp_derivative_validation.md) | Instantaneous derivative comparison between the Python and C++ implementations. |
| Verification | [Validation and Assumptions](VALIDATION_AND_ASSUMPTIONS.md) | Conventions, assumptions, numerical limitations, and validation scope. |
| Simulation | [Open-Loop Simulation](OPEN_LOOP_SIMULATION.md) | Asymmetric rotor simulation, logging, and plots. |
| Simulation | [Example Scripts](examples.md) | Python example and validation scripts. |

## Documentation

### 1. [Numerical Model](NUMERICAL_MODEL.md)

Describes the mathematical model implemented in C++, including:

- The 17-state vector.
- NED inertial and FRD body coordinate conventions.
- Quaternion attitude representation.
- Translational and rotational dynamics.
- Rotor thrust and reaction torque.
- Motor-speed dynamics.
- RK4 numerical integration.

### 2. [C++ Test Suite](CPP_TESTS.md)

Documents the C++ test suite, its objectives, numerical reference cases, expected behavior, and acceptance criteria.

Use this document to understand what the current tests check and how to execute the test suite.

### 3. [Open-Loop Simulation](OPEN_LOOP_SIMULATION.md)

Describes the asymmetric rotor simulation, including:

- Initial conditions and model parameters.
- Rotor-speed commands.
- Integration settings.
- CSV state logging.
- Python visualization and generated PNG plots.
- Interpretation and limitations of the simulation results.

### 4. [Validation and Assumptions](VALIDATION_AND_ASSUMPTIONS.md)

Describes the model's coordinate conventions, assumptions, numerical limitations, and current validation scope.

It also outlines additional work needed to establish stronger confidence in the model, including time-step convergence, reference-model comparison, parameter verification, and experimental validation.

## Quick Start

From the repository root, configure and build the C++ project:

```bash
cmake -S controller/cpp -B controller/cpp/build
cmake --build controller/cpp/build
```

Run the registered tests:

```bash
ctest --test-dir controller/cpp/build --output-on-failure
```

Build and run the open-loop example:

```bash
cmake --build controller/cpp/build --target open_loop_asymmetric
```

On Windows:

```powershell
.\controller\cpp\build\open_loop_asymmetric.exe
```

For visualization:

```bash
python controller/cpp/examples/visualize_open_loop.py
```

See [Open-Loop Simulation](OPEN_LOOP_SIMULATION.md) for the full procedure and output details.

## Documentation Status

> [!IMPORTANT]
> These documents describe the current implementation and selected numerical checks. Passing tests indicates that the tested assertions are satisfied; it does not independently establish physical accuracy or experimental validation.
