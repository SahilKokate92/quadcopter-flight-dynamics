# 🧭 Coordinate Frames and State Definition

[⬅ Back to documentation index](README.md)

## 📑 Contents

1. [Coordinate conventions](#1-coordinate-conventions)
2. [Rotation matrix](#2-rotation-matrix)
3. [Quaternion convention](#3-quaternion-convention)
4. [State vector indexing](#4-state-vector-indexing)
5. [Rotor ordering](#5-rotor-ordering)

---

## 1. Coordinate conventions

| Frame | Convention | $x$ | $y$ | $z$ |
|---|---|---|---|---|
| **Inertial** | NED | North | East | Down |
| **Body** | FRD | Forward | Right | Down |

### 1.1 Inertial frame: NED

- $x$: North
- $y$: East
- $z$: Down
- Gravity:

$$
\mathbf{g}_n = [0,\,0,\,+g]^T
$$

### 1.2 Body frame: FRD

- $x$: Forward
- $y$: Right
- $z$: Down
- Rotor thrust acts along **negative body $z$**.

## 2. Rotation matrix

$\mathbf{R}_{NB}$ maps a vector expressed in body coordinates into NED coordinates:

$$
\mathbf{v}_n = \mathbf{R}_{NB}\,\mathbf{v}_b.
$$

The transpose maps NED vectors into body coordinates:

$$
\mathbf{v}_b = \mathbf{R}_{NB}^{T}\,\mathbf{v}_n.
$$

## 3. Quaternion convention

The quaternion is stored **scalar-first**:

$$
q_{NB} = [q_w,\; q_x,\; q_y,\; q_z]^T.
$$

A normalized quaternion has unit norm:

$$
q_w^2 + q_x^2 + q_y^2 + q_z^2 = 1.
$$

> [!NOTE]
> The quaternions $q$ and $-q$ represent the same physical orientation, so a negative scalar component is not by itself an attitude error.

## 4. State vector indexing

The simulator state contains 17 scalar values.

| Index range | State variable | Units |
|---|---|---|
| `0:3` | NED position `[north, east, down]` | m |
| `3:6` | Body velocity `[u, v, w]` | m/s |
| `6:10` | Quaternion `[qw, qx, qy, qz]` | dimensionless |
| `10:13` | Body angular velocity `[p, q, r]` | rad/s |
| `13:17` | Rotor angular speeds `[Ω1, Ω2, Ω3, Ω4]` | rad/s |

## 5. Rotor ordering

The configured rotor order is:

| Rotor | Position |
|:-:|---|
| 1 | Front-right |
| 2 | Rear-right |
| 3 | Rear-left |
| 4 | Front-left |

> [!TIP]
> Rotor geometry is defined in `config/vehicle.py` and `src/quadcopter/rotors.py`. Validation scripts should reuse the configured rotor offset and positions instead of independently redefining geometry; this avoids inconsistent torque calculations.
