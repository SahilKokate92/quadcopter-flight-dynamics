# C++ Test Suite and Numerical Validation

[Documentation index](README.md)

## Contents

1. [Purpose](#1-purpose)
2. [Test Environment](#2-test-environment)
3. [Test Execution](#3-test-execution)
4. [Test 1 — Roll-Rate Controller](#4-test-1--roll-rate-controller)
5. [Test 2 — Roll-Rate Simulation](#5-test-2--roll-rate-simulation)
6. [Test 3 — Rigid-Body Dynamics](#6-test-3--rigid-body-dynamics)
7. [Test 4 — RK4 Integrator](#7-test-4--rk4-integrator)
8. [Test 5 — Rotor Dynamics](#8-test-5--rotor-dynamics)
9. [Test 6 — Motor Dynamics](#9-test-6--motor-dynamics)
10. [Test 7 — Rotor Mixer](#10-test-7--rotor-mixer)
11. [Test 8 — Attitude Response](#11-test-8--attitude-response)
12. [Test 9 — Rotational Dynamics Validation](#12-test-9--rotational-dynamics-validation)
13. [Test 10 — Translational Dynamics Validation](#13-test-10--translational-dynamics-validation)
14. [Test 11 — Rotor Force and Moment Numerical Validation](#14-test-11--rotor-force-and-moment-numerical-validation)
15. [Test 12 — Quaternion Axis Integration](#15-test-12--quaternion-axis-integration)
16. [Test 13 — Collective Thrust Response](#16-test-13--collective-thrust-response)
17. [Test 14 — Full-Model Numerical Stability](#17-test-14--full-model-numerical-stability)
18. [Test Coverage Summary](#18-test-coverage-summary)
19. [Interpreting Test Results](#19-interpreting-test-results)
20. [Recording Test Results](#20-recording-test-results)

---

## 1. Purpose

This document describes the C++ tests for the quadcopter flight-dynamics model. The tests cover controller behavior, rigid-body dynamics, numerical integration, rotor and motor models, attitude evolution, and full-model numerical stability.

The test suite uses analytical equations, known numerical cases, physical sign conventions, and numerical tolerances to check individual components.

> [!IMPORTANT]
> A passing test demonstrates that the tested implementation satisfies its programmed assertions for the selected cases. It does not, by itself, establish agreement with experimental flight data or prove that the complete model represents a real quadcopter accurately.

## 2. Test Environment

The C++ implementation uses C++17 and CMake.

Core model parameters used by several tests are:

| Parameter | Value | Unit |
|---|---:|---|
| Vehicle mass | 1.2 | kg |
| Gravitational acceleration | 9.80665 | m/s² |
| Roll-axis inertia, $I_x$ | 0.015 | kg·m² |
| Pitch-axis inertia, $I_y$ | 0.015 | kg·m² |
| Yaw-axis inertia, $I_z$ | 0.025 | kg·m² |
| Rotor thrust coefficient, $k_T$ | $8.0\times10^{-6}$ | N/(rad/s)² |
| Rotor reaction-torque coefficient, $k_Q$ | $1.5\times10^{-7}$ | N·m/(rad/s)² |
| Maximum rotor speed | 1200 | rad/s |
| Rotor arm offset | 0.141421356 | m |

The rotor spin signs are $[+1,-1,+1,-1]$.

Not every test uses every parameter. Controller and simplified simulation tests define their own parameters where necessary.

## 3. Test Execution

Run the following commands from the repository root.

Configure the project:

```bash
cmake -S controller/cpp -B controller/cpp/build
```

Build the test executables:

```bash
cmake --build controller/cpp/build
```

Execute the registered tests:

```bash
ctest --test-dir controller/cpp/build --output-on-failure
```

To run an individual test, use its registered CTest name. For example:

```bash
ctest --test-dir controller/cpp/build -R rotor_dynamics_test --output-on-failure
```

The test executables use assertions and explicit error checks. A test failure indicates that at least one expected condition was not satisfied.

## 4. Test 1 — Roll-Rate Controller

**Source:** `tests/test_roll_rate_controller.cpp`

### Objective

Verify the proportional roll-rate controller, torque saturation, input validation, and output limits.

### Test inputs

The controller uses:

- Proportional gain: $K_p=0.006$
- Maximum torque magnitude: $\tau_{\max}=0.002\ \text{N·m}$

The proportional control law is:

$$
\tau_{\mathrm{cmd}}=K_p(p_{\mathrm{desired}}-p_{\mathrm{measured}})
$$

The output is limited to the allowed torque interval:

$$
-\tau_{\max}\leq \tau_{\mathrm{cmd}}\leq\tau_{\max}
$$

### Cases and expected behavior

| Case | Desired rate | Measured rate | Expected torque |
|---|---:|---:|---:|
| Positive rate error | 0.5 rad/s | 0.2 rad/s | +0.0018 N·m |
| Negative rate error | 0.2 rad/s | 0.5 rad/s | −0.0018 N·m |
| Zero rate error | 0.3 rad/s | 0.3 rad/s | 0 N·m |
| Positive saturation | 2 rad/s | 0 rad/s | +0.002 N·m |
| Negative saturation | 0 rad/s | 2 rad/s | −0.002 N·m |

Additional cases verify that a negative gain, zero torque limit, and non-finite measured rate are rejected.

The test also sweeps desired and measured rates from −10 to +10 rad/s in increments of 0.1 rad/s and verifies that the torque remains within its limits.

> **Acceptance criteria:** Correct torque sign and magnitude, correct saturation, invalid-input rejection, and bounded output.

## 5. Test 2 — Roll-Rate Simulation

**Source:** `tests/test_roll_rate_simulation.cpp`

### Objective

Check whether the roll-rate controller can track a constant target in a simplified rotational model.

### Simulation parameters

| Parameter | Value |
|---|---:|
| Desired roll rate | 1.0 rad/s |
| Roll-axis inertia | 0.002 kg·m² |
| Proportional gain | 0.006 |
| Maximum torque | 0.002 N·m |
| Time step | 0.01 s |
| Duration | 3.0 s |
| Settling tolerance | 0.02 rad/s |

The simplified rotational model is:

$$
\dot p=\frac{\tau}{I_x}
$$

The rate is advanced using:

$$
p_{k+1}=p_k+\dot p_k\Delta t
$$

The simulation checks that the torque remains within its limit, the final tracking error is no greater than 0.02 rad/s, and a settling time can be identified within the simulation interval.

The settling-time calculation requires the rate to remain within the specified tolerance for the rest of the simulated time history.

> **Acceptance criteria:** The final tracking error is at most 0.02 rad/s, and the computed settling time lies between 0 and 3 seconds.

## 6. Test 3 — Rigid-Body Dynamics

**Source:** `tests/test_rigid_body_dynamics.cpp`

### Objective

Check the core rigid-body derivative calculations.

This test is intended to verify the basic mapping from state and applied body wrench to the derivatives of the rigid-body state.

The state includes position, body-frame translational velocity, attitude quaternion, and body angular rates.

> **Acceptance criteria:** The conditions implemented in the source test must be satisfied. Consult the individual assertions in `test_rigid_body_dynamics.cpp` for the exact input values and expected numerical outputs.

## 7. Test 4 — RK4 Integrator

**Source:** `tests/test_rk4_integrator.cpp`

### Objective

Verify the fourth-order Runge–Kutta integration implementation and its behavior on selected dynamical cases.

The classical RK4 update is:

$$
\begin{aligned}
k_1 &= f(t_n,x_n)\\
k_2 &= f\left(t_n+\frac{\Delta t}{2},
x_n+\frac{\Delta t}{2}k_1\right)\\
k_3 &= f\left(t_n+\frac{\Delta t}{2},
x_n+\frac{\Delta t}{2}k_2\right)\\
k_4 &= f(t_n+\Delta t,x_n+\Delta t k_3)
\end{aligned}
$$

The state update is:

$$
x_{n+1}=x_n+
\frac{\Delta t}{6}(k_1+2k_2+2k_3+k_4)
$$

The test source contains the actual integration cases and assertions. These should be used as the authority for its specific numerical expectations.

> **Acceptance criteria:** All integration assertions in the test source pass.

## 8. Test 5 — Rotor Dynamics

**Source:** `tests/test_rotor_dynamics.cpp`

### Objective

Validate the mapping from four rotor angular speeds to total thrust, body force, and body moments.

The thrust generated by an individual rotor is modeled as:

$$
T_i=k_T\Omega_i^2
$$

The total thrust is:

$$
T_{\mathrm{total}}=\sum_{i=1}^{4}k_T\Omega_i^2
$$

For the model's thrust direction, the body force under equal positive rotor speeds is:

$$
\mathbf F_b=
\begin{bmatrix}
0\\
0\\
-T_{\mathrm{total}}
\end{bmatrix}
$$

### Cases

1. Zero rotor speeds produce zero thrust and moments.
2. Four equal rotor speeds of 500 rad/s produce symmetric collective thrust.
3. A single active rotor produces the expected roll and pitch moment signs.
4. Equal rotor speeds cancel the modeled yaw reaction moment.
5. Asymmetric rotor speeds produce moments consistent with the implemented rotor geometry and spin signs.
6. Speeds above the 1200 rad/s maximum are clipped to the configured limit.
7. Negative rotor speed input is rejected.

For equal rotor speeds of 500 rad/s, the expected collective thrust is:

$$
T_{\mathrm{total}}
=4(8.0\times10^{-6})(500)^2
=8.0\ \text{N}
$$

> **Acceptance criteria:** The calculated thrust, force, moment signs, speed clipping, and input validation satisfy the assertions in the test source. The numerical comparison tolerance is relative-scaled using $10^{-10}$.

## 9. Test 6 — Motor Dynamics

**Source:** `tests/test_motor_dynamics.cpp`

### Objective

Verify the motor model that governs the response of actual rotor speed to commanded rotor speed.

A common first-order motor model is:

$$
\dot{\Omega}=\frac{\Omega_{\mathrm{cmd}}-\Omega}{\tau_m}
$$

where $\tau_m$ is the motor time constant. The equation describes the general first-order response; the precise implementation and test expectations are defined in the C++ source.

> **Acceptance criteria:** The motor dynamics satisfy the source test's response, limiting, and validation assertions.

## 10. Test 7 — Rotor Mixer

**Source:** `tests/test_rotor_mixer.cpp`

### Objective

Verify the collective and differential effects of selected rotor-speed combinations.

The test uses the rotor parameters:

- $k_T=8.0\times10^{-6}$
- $k_Q=1.5\times10^{-7}$
- Maximum rotor speed: 1200 rad/s
- Rotor arm offset: 0.141421356 m

### Cases

| Case | Rotor speeds (rad/s) | Expected behavior |
|---|---|---|
| Collective thrust | [500, 500, 500, 500] | Positive total thrust, zero net moments |
| Roll combination | [600, 600, 500, 500] | Positive roll moment, zero pitch and yaw moments |
| Pitch combination | [600, 500, 500, 600] | Positive pitch moment, zero roll and yaw moments |
| Yaw combination | [600, 500, 600, 500] | Positive yaw moment, zero roll and pitch moments |

> **Acceptance criteria:** The collective force and individual moment combinations satisfy the test assertions. The comparison tolerance is relative-scaled using $10^{-9}$.

## 11. Test 8 — Attitude Response

**Source:** `tests/test_attitude_response.cpp`

### Objective

Check the response of the vehicle attitude to the modeled dynamics.

The test focuses on the behavior specified by its actual initial conditions, applied inputs, and attitude assertions.

Attitude behavior depends on the relationship between body angular rates and quaternion evolution. For a unit quaternion, the attitude representation must also remain consistent with the model's quaternion convention.

> **Acceptance criteria:** All attitude-response assertions in the source test pass. Specific numerical expectations should be recorded from the source test rather than inferred from the test name.

## 12. Test 9 — Rotational Dynamics Validation

**Source:** `tests/test_rotational_dynamics_validation.cpp`

### Objective

Compare computed body angular accelerations against the analytical rigid-body rotational equations.

For a diagonal inertia tensor:

$$
I_x\dot p=\tau_x+(I_y-I_z)qr
$$

$$
I_y\dot q=\tau_y+(I_z-I_x)rp
$$

$$
I_z\dot r=\tau_z+(I_x-I_y)pq
$$

Therefore:

$$
\dot p=\frac{\tau_x+(I_y-I_z)qr}{I_x}
$$

$$
\dot q=\frac{\tau_y+(I_z-I_x)rp}{I_y}
$$

$$
\dot r=\frac{\tau_z+(I_x-I_y)pq}{I_z}
$$

### Cases

The test validates:

1. General rotational coupling with all three angular rates and torques nonzero.
2. Torque-free rotation with nonzero angular rates.
3. Pure roll torque with nonzero body rates.
4. Pure pitch torque with nonzero body rates.
5. Pure yaw torque with nonzero body rates.

The general case uses:

$$
[p,q,r]=[1.2,-0.7,2.1]\ \text{rad/s}
$$

$$
[\tau_x,\tau_y,\tau_z]
=[0.08,-0.04,0.03]\ \text{N·m}
$$

The source compares the computed and analytical angular accelerations using an absolute tolerance of $10^{-10}$.

> **Acceptance criteria:** Each computed angular acceleration agrees with its analytical value within the specified tolerance.

## 13. Test 10 — Translational Dynamics Validation

**Source:** `tests/test_translational_dynamics_validation.cpp`

### Objective

Validate body-frame translational acceleration against Newton's second law and the rotational transport term.

The body-frame translational equation is:

$$
\dot{\mathbf v}_b=
\frac{\mathbf F_b}{m}
+\mathbf g_b
-\boldsymbol{\omega}_b\times\mathbf v_b
$$

The test uses an identity quaternion, so the body axes initially align with the NED axes.

### Inputs

$$
m=1.2\ \text{kg}
$$

$$
\mathbf v_b=
\begin{bmatrix}
4\\-2\\1.5
\end{bmatrix}\ \text{m/s}
$$

$$
\boldsymbol{\omega}_b=
\begin{bmatrix}
0.7\\-0.4\\0.9
\end{bmatrix}\ \text{rad/s}
$$

$$
\mathbf F_b=
\begin{bmatrix}
2.4\\-1.2\\3.6
\end{bmatrix}\ \text{N}
$$

The test calculates the cross product independently and checks all three acceleration components against the model output.

For the supplied inputs:

$$
\boldsymbol{\omega}_b\times\mathbf v_b
=
\begin{bmatrix}
-1.2\\2.55\\-0.2
\end{bmatrix}
$$

Using NED gravity, the expected acceleration is:

$$
\dot{\mathbf v}_b=
\begin{bmatrix}
3.2\\
-3.55\\
12.20665
\end{bmatrix}\ \text{m/s}^2
$$

> **Acceptance criteria:** Each acceleration component agrees with the independently calculated value within $10^{-10}$.

## 14. Test 11 — Rotor Force and Moment Numerical Validation

**Source:** `tests/test_rotor_force_moment_validation.cpp`

### Objective

Validate rotor force and moment calculations using explicit analytical reference values.

The test uses:

$$
T_i=k_T\Omega_i^2
$$

The roll and pitch moments are calculated from the rotor position and thrust-force cross products. The yaw moment is calculated from the configured rotor reaction-torque signs.

### Cases

**Collective thrust**

For four rotors at 500 rad/s:

$$
T_i=8.0\times10^{-6}(500)^2=2.0\ \text{N}
$$

$$
T_{\mathrm{total}}=4(2.0)=8.0\ \text{N}
$$

The expected body force is $[0,0,-8.0]$ N, with zero net moments.

**Roll moment**

Rotor speeds:

$$
[600,600,500,500]\ \text{rad/s}
$$

The expected roll moment is:

$$
\tau_x=2d k_T(600^2-500^2)
$$

**Pitch moment**

Rotor speeds:

$$
[600,500,500,600]\ \text{rad/s}
$$

The expected pitch moment is:

$$
\tau_y=2d k_T(600^2-500^2)
$$

**Yaw moment**

Rotor speeds:

$$
[600,500,600,500]\ \text{rad/s}
$$

The expected yaw moment is:

$$
\tau_z=2k_Q(600^2-500^2)
$$

The test also checks that rotor speeds above the 1200 rad/s limit produce the same thrust as speeds clipped to that limit.

> **Acceptance criteria:** All calculated forces, moments, and saturation behavior agree with the analytical reference values within the relative-scaled tolerance of $10^{-10}$.

## 15. Test 12 — Quaternion Axis Integration

**Source:** `tests/test_quaternion_axis_integration.cpp`

### Objective

Check quaternion attitude integration for the rotation cases implemented by the test.

A scalar-first quaternion is written as:

$$
\mathbf q=
\begin{bmatrix}
q_w&q_x&q_y&q_z
\end{bmatrix}^{T}
$$

Quaternion evolution is related to body angular velocity through the quaternion kinematic equations. The exact sign convention must remain consistent with the implementation's body axes and multiplication convention.

The test should be interpreted using its actual angular-rate inputs, integration settings, expected quaternion components, and normalization checks.

> **Acceptance criteria:** All axis-integration assertions in the test source pass.

## 16. Test 13 — Collective Thrust Response

**Source:** `tests/test_collective_thrust_response.cpp`

### Objective

Check the vehicle's response when the commanded rotor speeds receive a collective increase.

The test examines the modeled motor transient and the resulting vehicle response, including:

- Rotor response toward the commanded speeds.
- Vertical body-velocity response under the model's force and gravity conventions.
- Quaternion normalization.
- Absence of unintended angular motion for the tested symmetric input.

The test's exact command offset, duration, time step, and tolerances are defined in its source file and should be preserved when documenting a particular run.

> **Acceptance criteria:** The collective-response assertions in the source test pass.

## 17. Test 14 — Full-Model Numerical Stability

**Source:** `tests/test_full_model_numerical_stability.cpp`

### Objective

Check whether the coupled quadcopter model maintains numerically valid state values during the asymmetric-input simulation case implemented by the test.

The complete state has 17 components:

$$
\mathbf x=
\begin{bmatrix}
p_n&p_e&p_d&
u_b&v_b&w_b&
q_w&q_x&q_y&q_z&
p&q&r&
\Omega_1&\Omega_2&\Omega_3&\Omega_4
\end{bmatrix}^{T}
$$

The test checks that state values remain finite and that the attitude quaternion remains normalized within its configured tolerance.

For a unit quaternion:

$$
q_w^2+q_x^2+q_y^2+q_z^2=1
$$

> **Acceptance criteria:** No tested state component becomes non-finite, and the quaternion satisfies the normalization assertion.

> [!NOTE]
> Passing this test establishes numerical validity for the tested case and duration only. It does not prove that the model remains stable for every possible input or that the resulting motion is physically accurate.

## 18. Test Coverage Summary

| No. | Test | Main purpose |
|---:|---|---|
| 1 | Roll-rate controller | Proportional control, saturation, and input validation |
| 2 | Roll-rate simulation | Simplified closed-loop tracking and settling |
| 3 | Rigid-body dynamics | Core rigid-body derivative calculations |
| 4 | RK4 integrator | Numerical state integration |
| 5 | Rotor dynamics | Rotor thrust, moments, limits, and signs |
| 6 | Motor dynamics | Rotor-speed response |
| 7 | Rotor mixer | Collective and differential rotor combinations |
| 8 | Attitude response | Attitude response under modeled dynamics |
| 9 | Rotational dynamics validation | Analytical angular-acceleration comparison |
| 10 | Translational dynamics validation | Analytical translational-acceleration comparison |
| 11 | Rotor force and moment validation | Explicit analytical force and moment references |
| 12 | Quaternion axis integration | Quaternion evolution for tested axes |
| 13 | Collective thrust response | Vehicle response to collective input |
| 14 | Full-model numerical stability | Finite states and quaternion normalization |

## 19. Interpreting Test Results

The suite includes both component tests and analytical validation tests.

Component tests check specific software behavior. Analytical validation tests compare selected calculations against independently evaluated equations. Full-model tests check the behavior of coupled states under selected inputs.

These checks provide different kinds of evidence and should not be treated as interchangeable.

The following limitations apply:

- Test coverage is limited to the conditions explicitly exercised by the source code.
- Numerical tolerances are not equivalent to physical-model uncertainty.
- Analytical agreement does not independently validate the selected vehicle parameters.
- Numerical stability does not guarantee physical accuracy.
- A successful test run does not replace comparison against trusted reference simulations or experimental measurements.

## 20. Recording Test Results

After running CTest, record the observed results separately from the expected results.

| Item | Result |
|---|---|
| Test date | Record the date of the run |
| Compiler and version | Record the compiler used |
| Build configuration | Record the configuration |
| Total registered tests | 14 |
| Passed | Record the actual count |
| Failed | Record the actual count |
| Command used | `ctest --test-dir controller/cpp/build --output-on-failure` |

> [!NOTE]
> The total of 14 refers to the test executables registered in the current CMake configuration. It does not mean there are only 14 individual assertions or test cases.

> [!WARNING]
> Do not mark all tests as passing until the current build and CTest output have been checked.
