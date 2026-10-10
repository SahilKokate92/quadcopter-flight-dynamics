<div align="center">

# 🚁 Quadcopter Flight Dynamics Simulator

### Technical Documentation

*Six-degree-of-freedom rigid-body model · quaternion attitude · first-order rotor dynamics · RK4 integration*

![Language](https://img.shields.io/badge/language-Python-3776AB?logo=python&logoColor=white)
![Model](https://img.shields.io/badge/model-6--DOF%20nonlinear-blue)
![Integrator](https://img.shields.io/badge/integrator-RK4-informational)
![Frames](https://img.shields.io/badge/frames-NED%20%2F%20FRD-success)
![Status](https://img.shields.io/badge/validation-open--loop%20checks%20only-orange)

</div>

---

This folder contains technical documentation for the **Quadcopter Flight Dynamics Simulator**.

## 📑 Table of Contents

1. [Documents](#-documents)
2. [Documentation Map](#-documentation-map)
3. [Scope](#-scope)

---

## 📚 Documents

| # | Document | Description |
|:-:|---|---|
| 1 | [🏗️ Architecture](architecture.md) | Software modules and data flow. |
| 2 | [📐 Mathematical Model](mathematical_model.md) | Equations implemented by the current model. |
| 3 | [🧭 Coordinate Frames and State](coordinate_frames.md) | Frame conventions, quaternion ordering, and state vector. |
| 4 | [✅ Validation](validation.md) | Current checks, observed results, and limitations. |
| 5 | [▶️ Examples](examples.md) | How to run the example scripts. |

## 🗺️ Documentation Map

```mermaid
flowchart LR
    R[README]:::root --> A[Architecture]
    R --> M[Mathematical Model]
    R --> C[Coordinate Frames and State]
    R --> V[Validation]
    R --> E[Examples]
    C -. conventions .-> M
    A -. state vector .-> C
    M -. equations checked by .-> V
    E -. scripts used in .-> V
    classDef root fill:#1f6feb,color:#fff,stroke:#1f6feb;
```

## 🎯 Scope

> [!IMPORTANT]
> These documents describe the **current Python implementation** and the **tests performed so far**. They do **not** claim validation against measured flight data. Vehicle and propulsion parameters are **provisional illustrative values**, not measured hardware data.
