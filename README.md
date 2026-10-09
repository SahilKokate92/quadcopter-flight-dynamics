# Quadcopter Flight Dynamics Simulator

A Python-based nonlinear quadcopter flight dynamics simulator under development. The project implements quaternion-based attitude representation, six-degree-of-freedom (6-DOF) rigid-body dynamics, and a four-rotor propulsion model.

---

## Table of Contents

- [Current Features](#current-features)
- [Mathematical Model](#mathematical-model)
- [Coordinate Conventions](#coordinate-conventions)
- [Project Structure](#project-structure)
- [Testing](#testing)
- [Development Roadmap](#development-roadmap)
- [Parameters](#parameters)
- [Status](#status)

---

## Current Features

- NED inertial frame and FRD body frame conventions
- Scalar-first quaternion operations
- Body-to-NED rotation matrix
- 6-DOF rigid-body translational and rotational dynamics
- Four-rotor thrust and reaction-torque model
- Vehicle model combining propulsion and rigid-body dynamics
- Automated unit tests using pytest

---

## Mathematical Model

The current rigid-body model uses the following equations.

### Position kinematics

$$
\dot{\mathbf{p}}_n = R_{NB}\,\mathbf{v}_b
$$

### Translational dynamics

$$
\dot{\mathbf{v}}_b = \frac{\mathbf{F}_b}{m} + R_{NB}^{T}\,\mathbf{g}_n - \boldsymbol{\omega}_b \times \mathbf{v}_b
$$

### Rotational dynamics

$$
I\,\dot{\boldsymbol{\omega}}_b = \boldsymbol{\tau}_b - \boldsymbol{\omega}_b \times \left(I\,\boldsymbol{\omega}_b\right)
$$

### Quaternion kinematics

$$
\dot{q}_{NB} = \frac{1}{2}\, q_{NB} \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega}_b \end{bmatrix}
$$

### Rotor thrust model

$$
T_i = k_f\,\Omega_i^2
$$

### Symbols

| Symbol | Description |
| :--- | :--- |
| $m$ | Vehicle mass |
| $I$ | Body-frame inertia matrix |
| $R_{NB}$ | Body-to-NED rotation matrix |
| $\mathbf{F}_b$ | Total body-frame force |
| $\boldsymbol{\tau}_b$ | Total body-frame torque |
| $\Omega_i$ | Rotor angular speed |

---

## Coordinate Conventions

| Item | Convention |
| :--- | :--- |
| Inertial frame | North-East-Down (NED) |
| Body frame | Forward-Right-Down (FRD) |
| Quaternion ordering | $[q_w, q_x, q_y, q_z]$ |
| Gravity in NED | $[0, 0, g]^T$ |
| Rotor thrust direction | Negative body $z$-axis |

---

## Project Structure

| Path | Description |
| :--- | :--- |
| `config/vehicle.py` | Vehicle and propulsion parameters |
| `src/quadcopter/frames.py` | Quaternion operations and coordinate transformations |
| `src/quadcopter/dynamics.py` | Rigid-body equations of motion |
| `src/quadcopter/rotors.py` | Rotor thrust and torque calculations |
| `src/quadcopter/vehicle.py` | Integrated vehicle derivative calculations |
| `tests/` | Automated unit tests |

---

## Testing

Run the test suite from the project root:

```bash
python -m pytest -v
```

The current development checkpoint has 20 passing tests.

---

## Development Roadmap

- [ ] Implement and validate an RK4 numerical integrator
- [ ] Add motor dynamics
- [ ] Build a time-domain simulation loop
- [ ] Validate open-loop flight responses
- [ ] Add trajectory plots and 3D visualization
- [ ] Develop and evaluate flight-control algorithms

---

## Parameters

Vehicle and propulsion values are provisional illustrative parameters, not measured hardware data.

---

## Status

The simulator is under active development. The current version provides foundational mathematical models and unit tests; complete time-domain simulation and flight-response validation remain future work.
