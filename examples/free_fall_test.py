import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import matplotlib.pyplot as plt

from config.vehicle import GRAVITY_M_S2
from src.quadcopter.simulator import simulate


# Initial state: stationary, level, at the origin.
initial_state = np.zeros(13)
initial_state[6] = 1.0

# All rotors switched off.
rotor_speeds = np.zeros(4)

duration_s = 1.0
dt = 0.01

times, states = simulate(
    initial_state=initial_state,
    rotor_speeds_rad_s=rotor_speeds,
    duration_s=duration_s,
    dt=dt,
)

down_position = states[:, 2]
down_velocity = states[:, 5]

expected_down_position = 0.5 * GRAVITY_M_S2 * duration_s**2
expected_down_velocity = GRAVITY_M_S2 * duration_s

print(f"Final down position: {down_position[-1]:.6f} m")
print(f"Expected down position: {expected_down_position:.6f} m")
print(f"Final down velocity: {down_velocity[-1]:.6f} m/s")
print(f"Expected down velocity: {expected_down_velocity:.6f} m/s")

np.testing.assert_allclose(
    down_position[-1], expected_down_position, atol=1e-6
)
np.testing.assert_allclose(
    down_velocity[-1], expected_down_velocity, atol=1e-6
)

fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

axes[0].plot(times, down_position)
axes[0].set_ylabel("Down position (m)")
axes[0].grid(True)

axes[1].plot(times, down_velocity)
axes[1].set_ylabel("Down velocity (m/s)")
axes[1].set_xlabel("Time (s)")
axes[1].grid(True)

fig.suptitle("Quadcopter Free-Fall Validation")
fig.tight_layout()
plt.show()