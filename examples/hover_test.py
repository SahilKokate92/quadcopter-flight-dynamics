import sys
from pathlib import Path

# Add the project root to Python's import path.
PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import matplotlib.pyplot as plt

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
)
from src.quadcopter.simulator import simulate


# Initial state: stationary at the origin, level attitude.
initial_state = np.zeros(13)
initial_state[6] = 1.0  # Identity quaternion [qw, qx, qy, qz]

# Equal rotor speeds for ideal hover.
hover_speed = np.sqrt(
    MASS_KG * GRAVITY_M_S2 / (4.0 * THRUST_COEFFICIENT)
)
rotor_speeds = np.full(4, hover_speed)

# Run the simulation.
times, states = simulate(
    initial_state=initial_state,
    rotor_speeds_rad_s=rotor_speeds,
    duration_s=1.0,
    dt=0.01,
)

position_ned = states[:, 0:3]
velocity_body = states[:, 3:6]
quaternion = states[:, 6:10]

# Print final-state diagnostics.
print(f"Hover rotor speed: {hover_speed:.2f} rad/s")
print(f"Final NED position: {position_ned[-1]} m")
print(f"Final body velocity: {velocity_body[-1]} m/s")
print(f"Final quaternion: {quaternion[-1]}")
print(f"Final quaternion norm: {np.linalg.norm(quaternion[-1]):.8f}")

# Plot position.
fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

position_labels = ["North", "East", "Down"]
for i, label in enumerate(position_labels):
    axes[i].plot(times, position_ned[:, i])
    axes[i].set_ylabel(f"{label} (m)")
    axes[i].grid(True)

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Quadcopter Hover — Position")
fig.tight_layout()
plt.show()

# Plot body velocity.
fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)

velocity_labels = ["Forward", "Right", "Down"]
for i, label in enumerate(velocity_labels):
    axes[i].plot(times, velocity_body[:, i])
    axes[i].set_ylabel(f"{label} (m/s)")
    axes[i].grid(True)

axes[-1].set_xlabel("Time (s)")
fig.suptitle("Quadcopter Hover — Body Velocity")
fig.tight_layout()
plt.show()