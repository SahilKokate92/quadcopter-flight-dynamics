# Numerical Model of the Quadcopter

[Documentation index](README.md)

## Contents

1. [Overview](#1-overview)
2. [State Vector](#2-state-vector)
3. [Coordinate Conventions](#3-coordinate-conventions)
4. [Attitude and Rotation Matrix](#4-attitude-and-rotation-matrix)
5. [Quaternion Kinematics](#5-quaternion-kinematics)
6. [Translational Dynamics](#6-translational-dynamics)
7. [Rotational Dynamics](#7-rotational-dynamics)
8. [Rotor Thrust Model](#8-rotor-thrust-model)
9. [Rotor Geometry and Moments](#9-rotor-geometry-and-moments)
10. [Motor-Speed Dynamics](#10-motor-speed-dynamics)
11. [Coupled Vehicle Dynamics](#11-coupled-vehicle-dynamics)
12. [RK4 Integration](#12-rk4-integration)
13. [Parameter Configuration](#13-parameter-configuration)
14. [Model Scope and Limitations](#14-model-scope-and-limitations)
15. [Verification and Validation](#15-verification-and-validation)
16. [Conclusion](#16-conclusion)

---

## 1. Overview

The C++ implementation models a quadcopter as a coupled nonlinear rigid-body system with four motor-driven rotors.

The model contains:

- Three-dimensional translational motion.
- Rigid-body rotational motion.
- Quaternion-based attitude kinematics.
- Rotor thrust and reaction torque.
- First-order motor-speed dynamics.
- Fourth-order Runge–Kutta (RK4) integration.

The primary implementation is located in `controller/cpp/`.

| Component | Source file |
|---|---|
| State definitions | `include/quadcopter/rigid_body_dynamics.hpp`, `include/quadcopter/quadcopter_state.hpp` |
| Rigid-body equations | `src/rigid_body_dynamics.cpp` |
| Rotor forces and moments | `src/rotor_dynamics.cpp` |
| Motor response | `src/motor_dynamics.cpp` |
| Coupled vehicle model | `src/quadcopter_dynamics.cpp` |
| Numerical integration | `src/rk4_integrator.cpp` |

## 2. State Vector

The complete vehicle state contains 17 scalar values.

$$
\mathbf{x}=
\begin{bmatrix}
p_n&p_e&p_d&
u_b&v_b&w_b&
q_w&q_x&q_y&q_z&
p&q&r&
\Omega_1&\Omega_2&\Omega_3&\Omega_4
\end{bmatrix}^{T}
$$

The state is grouped as follows.

| Variables | Meaning | Unit |
|---|---|---|
| $p_n,p_e,p_d$ | Position in the inertial NED frame | m |
| $u_b,v_b,w_b$ | Translational velocity in the body frame | m/s |
| $q_w,q_x,q_y,q_z$ | Scalar-first attitude quaternion | Dimensionless |
| $p,q,r$ | Body angular rates | rad/s |
| $\Omega_1,\ldots,\Omega_4$ | Actual rotor angular speeds | rad/s |

The state is represented in C++ by `QuadcopterState`, which combines `RigidBodyState` and `RotorSpeeds`.

## 3. Coordinate Conventions

### 3.1 Inertial frame

The inertial coordinate system uses North-East-Down (NED):

$$
\mathbf{p}_{NED}=
\begin{bmatrix}
p_n\\p_e\\p_d
\end{bmatrix}
$$

Positive $p_d$ points downward.

### 3.2 Body frame

The body coordinate system uses Forward-Right-Down (FRD):

$$
\mathbf{v}_b=
\begin{bmatrix}
u_b\\v_b\\w_b
\end{bmatrix}
$$

The body angular velocity is:

$$
\boldsymbol{\omega}_b=
\begin{bmatrix}
p\\q\\r
\end{bmatrix}
$$

Body forces and moments are also expressed in FRD coordinates.

## 4. Attitude and Rotation Matrix

The attitude is stored as a scalar-first Hamilton quaternion:

$$
\mathbf{q}_{nb}=
\begin{bmatrix}
q_w\\q_x\\q_y\\q_z
\end{bmatrix}
$$

The rotation matrix implemented by `rotation_body_to_ned()` maps a body-frame vector into the NED frame:

$$
\mathbf{R}_{b\rightarrow n}=
\begin{bmatrix}
1-2(q_y^2+q_z^2) &
2(q_xq_y-q_wq_z) &
2(q_xq_z+q_wq_y)\\
2(q_xq_y+q_wq_z) &
1-2(q_x^2+q_z^2) &
2(q_yq_z-q_wq_x)\\
2(q_xq_z-q_wq_y) &
2(q_yq_z+q_wq_x) &
1-2(q_x^2+q_y^2)
\end{bmatrix}
$$

The position derivative is calculated by transforming body velocity into NED:

$$
\boxed{
\dot{\mathbf{p}}_{NED}
=
\mathbf{R}_{b\rightarrow n}\mathbf{v}_b
}
$$

The rotation matrix is constructed using a normalized copy of the quaternion.

## 5. Quaternion Kinematics

The source code uses the quaternion differential equation:

$$
\boxed{
\dot{\mathbf{q}}_{nb}
=
\frac{1}{2}
\left(
\mathbf{q}_{nb}\otimes
\begin{bmatrix}
0\\p\\q\\r
\end{bmatrix}
\right)
}
$$

Here, $\otimes$ denotes the scalar-first Hamilton quaternion product.

Expanding the implemented product gives:

$$
\dot q_w=-\frac{1}{2}(q_xp+q_yq+q_zr)
$$

$$
\dot q_x=\frac{1}{2}(q_wp+q_yr-q_zq)
$$

$$
\dot q_y=\frac{1}{2}(q_wq+q_zp-q_xr)
$$

$$
\dot q_z=\frac{1}{2}(q_wr+q_xq_y^{\,0}+q_y p-q_xq)
$$

The last expression above should be interpreted using the actual Hamilton-product implementation. In expanded form, the correct fourth component from that implementation is:

$$
\boxed{
\dot q_z=\frac{1}{2}(q_wr+q_xq-q_yp)
}
$$

The unit-quaternion condition is:

$$
q_w^2+q_x^2+q_y^2+q_z^2=1
$$

The RK4 integrator normalizes the quaternion after each completed integration step. The derivative calculation also normalizes the quaternion used to construct the rotation matrix and rejects invalid or near-zero quaternions.

## 6. Translational Dynamics

The model calculates gravity in the NED frame as:

$$
\mathbf{g}_{NED}=
\begin{bmatrix}
0\\0\\g
\end{bmatrix}
$$

where $g$ is the configured positive gravity magnitude.

Gravity is transformed into body coordinates:

$$
\boxed{
\mathbf{g}_b=
\mathbf{R}_{b\rightarrow n}^{T}\mathbf{g}_{NED}
}
$$

The body-frame translational acceleration is implemented as:

$$
\boxed{
\dot{\mathbf{v}}_b=
\frac{\mathbf{F}_b}{m}
+\mathbf{g}_b
-\boldsymbol{\omega}_b\times\mathbf{v}_b
}
$$

where:

- $\mathbf{F}_b$ is the total body-frame force.
- $m$ is vehicle mass.
- $\mathbf{g}_b$ is gravity expressed in body coordinates.
- $\boldsymbol{\omega}_b$ is body angular velocity.
- $\mathbf{v}_b$ is body translational velocity.

The cross-product term accounts for the rotating body coordinate system.

The implementation uses:

$$
\boldsymbol{\omega}_b\times\mathbf{v}_b=
\begin{bmatrix}
q w_b-r v_b\\
r u_b-p w_b\\
p v_b-q u_b
\end{bmatrix}
$$

Therefore, the component equations are:

$$
\dot u_b=
\frac{F_x}{m}+g_{b,x}-(q w_b-r v_b)
$$

$$
\dot v_b=
\frac{F_y}{m}+g_{b,y}-(r u_b-p w_b)
$$

$$
\dot w_b=
\frac{F_z}{m}+g_{b,z}-(p v_b-q u_b)
$$

The force components and gravity components must use the same body-frame convention.

## 7. Rotational Dynamics

The model uses a diagonal inertia tensor:

$$
\mathbf{I}=
\begin{bmatrix}
I_x&0&0\\
0&I_y&0\\
0&0&I_z
\end{bmatrix}
$$

Euler's rigid-body equation is implemented as:

$$
\boxed{
\mathbf{I}\dot{\boldsymbol{\omega}}_b
=
\boldsymbol{\tau}_b
-
\boldsymbol{\omega}_b
\times
(\mathbf{I}\boldsymbol{\omega}_b)
}
$$

where $\boldsymbol{\tau}_b$ is the total body torque.

The angular momentum vector is:

$$
\mathbf{H}_b=\mathbf{I}\boldsymbol{\omega}_b
$$

The implementation evaluates:

$$
\dot{\boldsymbol{\omega}}_b
=
\mathbf{I}^{-1}
\left[
\boldsymbol{\tau}_b-
\boldsymbol{\omega}_b\times\mathbf{H}_b
\right]
$$

For diagonal inertia, the component equations are:

$$
\boxed{
\dot p=
\frac{\tau_x+(I_y-I_z)qr}{I_x}
}
$$

$$
\boxed{
\dot q=
\frac{\tau_y+(I_z-I_x)rp}{I_y}
}
$$

$$
\boxed{
\dot r=
\frac{\tau_z+(I_x-I_y)pq}{I_z}
}
$$

These equations include gyroscopic coupling between body angular rates.

## 8. Rotor Thrust Model

Each rotor produces thrust according to the quadratic model:

$$
\boxed{T_i=k_T\Omega_i^2}
$$

where $k_T$ is the thrust coefficient and $\Omega_i$ is the actual rotor speed.

The total thrust is:

$$
T_{\mathrm{total}}=\sum_{i=1}^{4}T_i
$$

The rotor thrust acts along negative body z:

$$
\boxed{
\mathbf{F}_b=
\begin{bmatrix}
0\\0\\-T_{\mathrm{total}}
\end{bmatrix}
}
$$

The implementation clamps rotor speeds to the interval from zero to the configured maximum before calculating thrust.

## 9. Rotor Geometry and Moments

The rotor positions used in `rotor_dynamics.cpp` are:

$$
\mathbf{r}_1=
\begin{bmatrix}d\\-d\\0\end{bmatrix},
\quad
\mathbf{r}_2=
\begin{bmatrix}-d\\-d\\0\end{bmatrix}
$$

$$
\mathbf{r}_3=
\begin{bmatrix}-d\\d\\0\end{bmatrix},
\quad
\mathbf{r}_4=
\begin{bmatrix}d\\d\\0\end{bmatrix}
$$

The rotor force for rotor $i$ is:

$$
\mathbf{F}_i=
\begin{bmatrix}
0\\0\\-T_i
\end{bmatrix}
$$

Its arm moment is calculated using:

$$
\boxed{
\boldsymbol{\tau}_{i,\mathrm{arm}}
=
\mathbf{r}_i\times\mathbf{F}_i
}
$$

The total arm moment is:

$$
\boldsymbol{\tau}_{\mathrm{arm}}
=
\sum_{i=1}^{4}
\left(\mathbf{r}_i\times\mathbf{F}_i\right)
$$

The configured parameter $d$ is the magnitude of each x and y coordinate, not the full rotor distance from the origin. The radial distance implied by these coordinates is:

$$
r_{\mathrm{arm}}=\sqrt{2}\,d
$$

The code also models reaction torque:

$$
Q_i=k_Q\Omega_i^2s_i
$$

where $s_i$ is the configured spin sign of rotor $i$.

The net yaw reaction torque is:

$$
\boxed{
\tau_z=\tau_{z,\mathrm{arm}}+\sum_{i=1}^{4}Q_i
}
$$

The configured spin signs are:

$$
[s_1,s_2,s_3,s_4]=[+1,-1,+1,-1]
$$

> [!NOTE]
> The spin-sign convention determines the direction of each modeled reaction torque. It must be kept consistent with the selected rotor numbering and physical rotation directions.

## 10. Motor-Speed Dynamics

The motor model uses a first-order response:

$$
\boxed{
\dot{\Omega}_i=
\frac{\Omega_{i,\mathrm{cmd}}-\Omega_i}{\tau_m}
}
$$

where:

- $\Omega_i$ is actual rotor speed.
- $\Omega_{i,\mathrm{cmd}}$ is commanded rotor speed.
- $\tau_m$ is the motor time constant.

The implementation rejects non-finite, negative, or above-limit actual and commanded speeds.

The model does not instantaneously set actual speed equal to the command. Rotor speed evolves according to the motor derivative and is integrated as part of the 17-state model.

## 11. Coupled Vehicle Dynamics

`QuadcopterDynamics::derivatives()` combines the rotor, motor, and rigid-body models.

At each derivative evaluation, it:

1. Calculates thrust and torque from actual rotor speeds.
2. Forms the body wrench from the rotor force and torque.
3. Computes rigid-body state derivatives.
4. Computes the four motor-speed derivatives from actual and commanded speeds.
5. Returns the derivatives of the complete state.

The complete system can be expressed as:

$$
\boxed{
\dot{\mathbf{x}}=
f(\mathbf{x},\boldsymbol{\Omega}_{\mathrm{cmd}})
}
$$

The state is coupled because rotor speeds affect forces and moments, forces and moments affect rigid-body motion, and the motor equations determine how actual rotor speeds respond to commands.

## 12. RK4 Integration

The integrator advances the full state using classical fourth-order Runge–Kutta.

For a general state equation:

$$
\dot{\mathbf{x}}=f(t,\mathbf{x})
$$

the stages are:

$$
\mathbf{k}_1=f(t_n,\mathbf{x}_n)
$$

$$
\mathbf{k}_2=
f\left(t_n+\frac{\Delta t}{2},
\mathbf{x}_n+\frac{\Delta t}{2}\mathbf{k}_1\right)
$$

$$
\mathbf{k}_3=
f\left(t_n+\frac{\Delta t}{2},
\mathbf{x}_n+\frac{\Delta t}{2}\mathbf{k}_2\right)
$$

$$
\mathbf{k}_4=
f(t_n+\Delta t,\mathbf{x}_n+\Delta t\mathbf{k}_3)
$$

The update is:

$$
\boxed{
\mathbf{x}_{n+1}
=
\mathbf{x}_n+
\frac{\Delta t}{6}
\left(
\mathbf{k}_1+2\mathbf{k}_2+2\mathbf{k}_3+\mathbf{k}_4
\right)
}
$$

The C++ implementation applies this update to position, velocity, quaternion, angular rates, and all four rotor speeds. It then normalizes the quaternion.

The time step must be finite and positive. A non-finite or invalid quaternion after integration raises an error.

## 13. Parameter Configuration

The open-loop example configures the model as follows.

| Parameter | Value | Unit |
|---|---:|---|
| Mass | 1.2 | kg |
| Gravity magnitude | 9.80665 | m/s² |
| $I_x$ | 0.015 | kg·m² |
| $I_y$ | 0.015 | kg·m² |
| $I_z$ | 0.025 | kg·m² |
| $k_T$ | $8.0\times10^{-6}$ | N/(rad/s)² |
| $k_Q$ | $1.5\times10^{-7}$ | N·m/(rad/s)² |
| Maximum rotor speed | 1200 | rad/s |
| Rotor coordinate offset, $d$ | 0.141421356 | m |
| Motor time constant | 0.04 | s |

> [!NOTE]
> These are the configured values in the open-loop example. Their presence in the source code does not by itself establish that they are identified or experimentally validated parameters for a particular physical quadcopter.

## 14. Model Scope and Limitations

The implemented model includes rigid-body dynamics, rotor thrust and reaction torque, first-order motor-speed response, and quaternion attitude representation.

> [!WARNING]
> The following should not be assumed to be modeled unless implemented elsewhere in the source:

- Aerodynamic drag on the vehicle body.
- Blade-element or momentum-theory rotor aerodynamics.
- Rotor-to-rotor aerodynamic interaction.
- Ground effect.
- Wind and atmospheric disturbances.
- Flexible structural dynamics.
- Electrical motor and battery dynamics beyond the configured speed-response model.
- Closed-loop flight stabilization in the open-loop simulation.

The model's behavior depends on its parameters, equations, initial conditions, commanded rotor speeds, and integration settings.

## 15. Verification and Validation

The project contains component tests, analytical comparison tests, and a full-model numerical-stability test. These tests are documented separately in `CPP_TESTS.md`.

The open-loop simulation is documented in `OPEN_LOOP_SIMULATION.md`.

> [!IMPORTANT]
> Passing tests and obtaining finite simulation results provide evidence about software behavior and numerical execution. They do not establish agreement with experimental flight data.

Additional confidence should be developed through:

1. Time-step convergence studies.
2. Independent analytical checks for selected trajectories and rotations.
3. Comparison against a trusted reference simulation.
4. Parameter-source documentation or system identification.
5. Experimental validation when measured data are available.

## 16. Conclusion

The current C++ model integrates a 17-state nonlinear quadcopter system using RK4. It combines body-frame translational and rotational equations with quaternion attitude kinematics, rotor force and moment calculations, and first-order motor dynamics.

The equations documented here follow the implementation in the current C++ source. Model parameters and assumptions should be revisited whenever the implementation changes.
