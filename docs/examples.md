# ▶️ Example Scripts

[⬅ Back to documentation index](README.md)

## 📑 Contents

1. [Running the examples](#1-running-the-examples)
2. [Current examples](#2-current-examples)
3. [Reproducibility notes](#3-reproducibility-notes)

---

## 1. Running the examples

Run commands from the repository root, for example:

```powershell
python examples/hover_validation.py
```

## 2. Current examples

| Script | Purpose |
|---|---|
| `examples/free_fall_test.py` | Demonstrates a free-fall or gravity-related scenario. Check the script's initial conditions before interpreting the result. |
| `examples/hover_test.py` | Demonstrates a basic hover-related simulation. |
| `examples/hover_validation.py` | Checks hover equilibrium and reports pass/fail metrics. |
| `examples/motor_response_test.py` | Demonstrates first-order rotor motor response. |
| `examples/roll_dynamics_validation.py` | Tests roll response and compares numerical roll acceleration with the rigid-body equation. |
| `examples/roll_test.py` | Demonstrates a roll-response case. |
| `examples/thrust_response_test.py` | Demonstrates the effect of a temporary increase in rotor-speed commands. |
| `examples/vertical_step_response.py` | Plots altitude, upward velocity, rotor speeds, and vertical acceleration for a vertical thrust step. |

## 3. Reproducibility notes

- Run scripts from the repository root so imports resolve consistently. Scripts that are designed for direct execution may add the repository root to `sys.path`.
- Matplotlib-based examples open figure windows.
- Results depend on the configured vehicle parameters, time step, initial state, and command profile.

> [!NOTE]
> The examples are engineering demonstrations, not hardware flight instructions.
