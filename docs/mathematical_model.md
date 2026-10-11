# Mathematical Model

[Documentation index](README.md)

## Contents

1. [Assumptions and parameter status](#1-assumptions-and-parameter-status)
2. [Position kinematics](#2-position-kinematics)
3. [Translational dynamics](#3-translational-dynamics)
4. [Rotational dynamics](#4-rotational-dynamics)
5. [Quaternion kinematics](#5-quaternion-kinematics)
6. [Rotor thrust and reaction torque](#6-rotor-thrust-and-reaction-torque)
7. [First-order motor dynamics](#7-first-order-motor-dynamics)
8. [Numerical integration](#8-numerical-integration)
9. [Current omissions](#9-current-omissions)

---

## Notation

| Symbol | Meaning |
|---|---|
| $\mathbf{p}_n$ | Position in NED coordinates |
| $\mathbf{v}_b$ | Velocity expressed in body coordinates |
| $\mathbf{R}_{NB}$ | Rotation matrix mapping body-frame vectors into NED |
| $\mathbf{F}_b$ | Total applied body force |
| $m$ | Mass |
| $\mathbf{g}_n$ | Gravity in NED, $[0,0,g]^T$ |
| $\boldsymbol{\omega}_b$ | Body angular velocity, $[p,q,r]^T$ |
| $\boldsymbol{\tau}_b$ | Total body torque |
| $\mathbf{I}$ | Inertia matrix |
| $q_{NB}$ | Scalar-first attitude quaternion, $[q_w,q_x,q_y,q_z]^T$ |
| $\Omega_i$, $\Omega_{cmd,i}$ | Actual and commanded speed of rotor $i$ |
| $k_f$ | Configured thrust coefficient |
| $\tau_m$ | Configured motor time constant |
| $\mathbf{r}_i$, $\mathbf{F}_i$ | Position and force of rotor $i$ |

---

## 1. Assumptions and parameter status

The model represents a **rigid quadcopter with four rotors**. It uses body-axis translational velocity, body angular rates, a quaternion attitude representation, and first-order rotor-speed dynamics.

> [!WARNING]
> The current vehicle parameters are **illustrative placeholders** and have not been identified from a physical vehicle.

The inertia matrix is diagonal in the configured body frame:

$$
\mathbf{I} =
\begin{bmatrix}
I_{xx} & 0 & 0 \\
0 & I_{yy} & 0 \\
0 & 0 & I_{zz}
\end{bmatrix}.
$$

## 2. Position kinematics

Let $\mathbf{p}_n$ be position in NED coordinates, $\mathbf{v}_b$ be velocity expressed in body coordinates, and $\mathbf{R}_{NB}$ map body-frame vectors into NED coordinates:

$$
\dot{\mathbf{p}}_n = \mathbf{R}_{NB}\,\mathbf{v}_b.
$$

## 3. Translational dynamics

The implementation uses:

$$
\dot{\mathbf{v}}_b = \frac{\mathbf{F}_b}{m} + \mathbf{R}_{NB}^{T}\,\mathbf{g}_n - \boldsymbol{\omega}_b \times \mathbf{v}_b,
$$

where $\mathbf{F}_b$ is total applied body force, $m$ is mass, and $\mathbf{g}_n=[0,0,g]^T$ in the NED frame. This form accounts for gravity expressed in body coordinates and the rotating body-frame velocity derivative.

| Term | Physical meaning |
|---|---|
| $\mathbf{F}_b / m$ | Acceleration from applied (rotor) force |
| $\mathbf{R}_{NB}^{T}\mathbf{g}_n$ | Gravity expressed in body coordinates |
| $-\boldsymbol{\omega}_b \times \mathbf{v}_b$ | Rotating body-frame velocity derivative |

The rotor thrust direction is negative body $z$. For a level vehicle, this produces upward acceleration when total thrust exceeds weight.

## 4. Rotational dynamics

The rigid-body rotational equation is:

$$
\mathbf{I}\,\dot{\boldsymbol{\omega}}_b = \boldsymbol{\tau}_b - \boldsymbol{\omega}_b \times (\mathbf{I}\,\boldsymbol{\omega}_b),
$$

or equivalently,

$$
\dot{\boldsymbol{\omega}}_b = \mathbf{I}^{-1}\left[\boldsymbol{\tau}_b - \boldsymbol{\omega}_b \times (\mathbf{I}\,\boldsymbol{\omega}_b)\right].
$$

Here $\boldsymbol{\omega}_b=[p,q,r]^T$ and $\boldsymbol{\tau}_b$ is the total body torque from rotor-arm moments and rotor reaction torques.

## 5. Quaternion kinematics

The attitude quaternion is scalar-first, $q_{NB}=[q_w,q_x,q_y,q_z]^T$. The current implementation uses:

$$
\dot{q}_{NB} = \frac{1}{2}\, q_{NB} \otimes
\begin{bmatrix} 0 \\ \boldsymbol{\omega}_b \end{bmatrix},
$$

where $\otimes$ denotes quaternion multiplication. The simulator normalizes the quaternion after integration steps to limit numerical norm drift.

## 6. Rotor thrust and reaction torque

For rotor $i$ with actual angular speed $\Omega_i$:

$$
T_i = k_f\,\Omega_i^2,
$$

where $k_f$ is the configured thrust coefficient. The total body force is:

$$
\mathbf{F}_b =
\begin{bmatrix}
0 \\ 0 \\ -\sum_{i=1}^{4} T_i
\end{bmatrix}.
$$

The rotor-arm torque is calculated from each rotor position $\mathbf{r}_i$ and force $\mathbf{F}_i$:

$$
\boldsymbol{\tau}_{arm} = \sum_{i=1}^{4} \mathbf{r}_i \times \mathbf{F}_i.
$$

The model also includes rotor reaction torque proportional to squared rotor speed, using the configured torque coefficient and spin signs. The total body torque is the sum of arm torque and reaction torque.

## 7. First-order motor dynamics

For commanded rotor speed $\Omega_{cmd,i}$ and actual rotor speed $\Omega_i$, the motor model uses:

$$
\dot{\Omega}_i = \frac{\Omega_{cmd,i} - \Omega_i}{\tau_m},
$$

where $\tau_m$ is the configured motor time constant. Actual rotor speed therefore approaches a command gradually rather than changing instantaneously.

## 8. Numerical integration

The simulator integrates the combined 17-element state using the configured numerical integrator. The example scripts use a fixed time step, commonly $\Delta t = 0.001\,\mathrm{s}$, and Runge–Kutta 4 (RK4) integration.

## 9. Current omissions

> [!CAUTION]
> The current model does not establish a complete real-world flight model.

In particular, the documented model does not include:

- aerodynamic drag,
- blade flapping,
- ground effect,
- detailed motor electrical dynamics,
- sensor models, or
- a closed-loop flight controller.

These should be treated as future extensions unless they are explicitly implemented and tested.
