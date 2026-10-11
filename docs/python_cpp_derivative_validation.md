# Python–C++ Rigid-Body Derivative Validation

[Documentation index](README.md)

## Contents

1. [Purpose](#1-purpose)
2. [State and coordinate conventions](#2-state-and-coordinate-conventions)
3. [Model equations](#3-model-equations)
4. [Comparison procedure](#4-comparison-procedure)
5. [Numerical acceptance criterion](#5-numerical-acceptance-criterion)
6. [Test cases](#6-test-cases)
7. [Limitations](#7-limitations)

---

## 1. Purpose

This procedure compares the instantaneous rigid-body derivatives produced by the Python reference implementation and the C++ implementation.

> [!NOTE]
> It does not validate the full integrated trajectory or the physical accuracy of the vehicle parameters.

## 2. State and coordinate conventions

| Item | Convention |
|---|---|
| Inertial frame | North-East-Down (NED). |
| Body frame | Forward-Right-Down (FRD). |
| Quaternion order | Scalar-first `[qw, qx, qy, qz]`. |
| Quaternion multiplication | Hamilton product. |

| Quantity | Unit |
|---|---|
| Position | m. |
| Velocity | m/s. |
| Angular velocity | rad/s. |
| Force | N. |
| Torque | N·m. |
| Translational acceleration | m/s². |
| Angular acceleration | rad/s². |

## 3. Model equations

**Position kinematics:**

$$
\dot{\mathbf r}^{NED}=R_{NB}\mathbf v^B
$$

**Body-frame translational dynamics:**

$$
\dot{\mathbf v}^B =
\frac{\mathbf F^B}{m}
+R_{NB}^{T}\mathbf g^{NED}
-\boldsymbol\omega^B\times\mathbf v^B
$$

**Rotational dynamics for diagonal inertia:**

$$
\dot{\boldsymbol\omega}^{B}
=I^{-1}
\left(\boldsymbol\tau^B
-\boldsymbol\omega^B\times(I\boldsymbol\omega^B)\right)
$$

**Quaternion kinematics:**

$$
\dot q_{NB}=\frac{1}{2}q_{NB}\otimes
\begin{bmatrix}0\\p\\q\\r\end{bmatrix}
$$

The quaternion is normalized before calculating the rotation matrix and quaternion derivative, matching the current implementation.

## 4. Comparison procedure

The Python script passes the same state, body force, and body torque to the C++ executable. Both implementations use the same rigid-body parameters.

The 13 derivative components are compared in this order:

| Order | Derivative | Values |
|---:|---|---:|
| 1 | Position derivative | 3 |
| 2 | Body velocity derivative | 3 |
| 3 | Quaternion derivative | 4 |
| 4 | Angular velocity derivative | 3 |

## 5. Numerical acceptance criterion

The comparison uses:

| Tolerance | Value |
|---|---|
| Absolute tolerance | $10^{-10}$ |
| Relative tolerance | $10^{-10}$ |

A case passes when all 13 components satisfy the NumPy `allclose` criterion with these tolerances.

## 6. Test cases

| Case | Description |
|---|---|
| `level_static` | Level attitude with thrust balancing gravity. |
| `translated_rotating` | Nonzero body velocity, attitude, angular rates, force, and torque. |
| `coupled_rotation` | Non-identity quaternion and nonzero angular rates to exercise rotational coupling. |

## 7. Limitations

> [!IMPORTANT]
> Passing these tests establishes numerical agreement for the tested instantaneous derivatives only.

It does not establish:

- Full 17-state trajectory equivalence.
- Numerical integration equivalence.
- Motor dynamics equivalence.
- Experimental validation of thrust or torque coefficients.
- Real-world flight safety or vehicle performance.
