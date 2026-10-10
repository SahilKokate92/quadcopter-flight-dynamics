# ✅ Model Validation and Verification

[⬅ Back to documentation index](README.md)

## 📑 Contents

1. [Purpose](#1-purpose)
2. [Automated tests](#2-automated-tests)
3. [Hover equilibrium](#3-hover-equilibrium)
4. [Vertical step response](#4-vertical-step-response)
5. [Roll response](#5-roll-response)
6. [Important interpretation](#6-important-interpretation)

---

## 1. Purpose

The checks below verify selected behaviors and internal consistency of the current implementation.

> [!WARNING]
> They are **not** a substitute for comparison against measured flight data or a validated reference simulator.

## 2. Automated tests

Run from the project root:

```powershell
python -m pytest -v
```

Use the actual test summary printed by the current checkout when recording the test count; it can change as tests are added.

## 3. Hover equilibrium

**Script:** `examples/hover_validation.py`

The vehicle is initialized at level attitude with zero velocity and body rates. Each rotor is initialized at the hover speed calculated from:

$$
\Omega_{hover} = \sqrt{\frac{m\,g}{4\,k_f}}.
$$

For the current illustrative values, the expected hover speed is approximately `606.423429 rad/s`. In the recorded run, altitude change, vertical velocity, and angular rates remained approximately zero, and the script printed `HOVER VALIDATION PASSED`.

| Item | Result |
|---|---|
| Expected hover speed | ≈ `606.423429 rad/s` |
| Altitude change | ≈ 0 |
| Vertical velocity | ≈ 0 |
| Angular rates | ≈ 0 |
| Script output | `HOVER VALIDATION PASSED` |

This verifies equilibrium for the implemented idealized model and chosen parameters.

## 4. Vertical step response

**Script:** `examples/vertical_step_response.py`

The commanded rotor speed is increased by `100 rad/s` between `0.5 s` and `1.5 s`, then returned to hover speed. The command profile is:

$$
\Omega_{cmd}(t) =
\begin{cases}
\Omega_{hover} + 100\ \mathrm{rad/s}, & 0.5\ \mathrm{s} \le t \le 1.5\ \mathrm{s} \\[4pt]
\Omega_{hover}, & \text{otherwise}
\end{cases}
$$

The recorded run showed:

| Metric | Recorded value |
|---|---|
| Maximum/final altitude gain (3-second run) | ≈ `6.8417 m` |
| Upward velocity at `1.5 s` | ≈ `3.3561 m/s` |
| Upward velocity at `3.0 s` | ≈ `3.4903 m/s` |
| Final rotor speeds | ≈ `606.423429 rad/s` per rotor |
| Final upward acceleration | ≈ 0 |

The thrust-derived acceleration and velocity-derived acceleration curves closely overlapped. The remaining upward velocity after the motors return to hover speed is physically consistent with zero acceleration in a model without aerodynamic drag.

## 5. Roll response

**Script:** `examples/roll_dynamics_validation.py`

The left rotor pair receives a temporary positive speed increment. With the current FRD convention and rotor ordering, the simulated roll rate becomes **negative**. For the smaller input test, the recorded values were:

| Metric | Recorded value |
|---|---|
| Maximum absolute roll rate | ≈ `0.368332 rad/s` |
| Final roll rate | ≈ `-0.368332 rad/s` |
| Final pitch and yaw rates | ≈ 0 |
| Numerical roll acceleration at `0.70 s` | ≈ `-1.816829 rad/s²` |
| Full-equation roll acceleration at `0.70 s` | ≈ `-1.824432 rad/s²` |

The close agreement between acceleration estimated from sampled angular rates and acceleration calculated from the rigid-body equation supports consistency of the torque and rotational-dynamics implementation for this case.

## 6. Important interpretation

- Numerical differentiation of sampled rates can introduce errors, especially near command transitions.
- A constant nonzero roll rate after a torque pulse is expected in the current open-loop model because there is no active rate controller or explicit rotational damping.
- These checks validate selected equations and expected responses, not real-world accuracy across all flight conditions.
- If the model changes, rerun the tests and update this document with new observed values rather than retaining stale results.
