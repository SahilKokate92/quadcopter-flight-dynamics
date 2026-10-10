import sys
from pathlib import Path

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
from src.quadcopter.rotors import rotor_forces_and_torques


# Start from stationary, level hover.
initial_state = np.zeros(13)
initial_state[6] = 1.0

hover_speed = np.sqrt(
    MASS_KG * GRAVITY_M_S2 / (4.0 * THRUST_COEFFICIENT)
)

# Increase left-side rotor speeds to create a negative roll torque.
rotor_speeds = np.array([
    hover_speed,
    hover_speed,
    hover_speed + 20.0,
    hover_speed + 20.0,
])

propulsion = rotor_forces_and_torques(rotor_speeds)

print("Rotor speeds (rad/s):", rotor_speeds)
print("Total thrust (N):", propulsion["thrust_n"])
print("Body torque (N m):", propulsion["torque_body_nm"])

times, states = simulate(
    initial_state=initial_state,
    rotor_speeds_rad_s=rotor_speeds,
    duration_s=1.0,
    dt=0.01,
)

angular_velocity = states[:, 10:13]
quaternion = states[:, 6:10]

fig, axes = plt.subplots(2, 1, figsize=(9, 7), sharex=True)

axes[0].plot(times, angular_velocity[:, 0], label="Roll rate p")
axes[0].set_ylabel("Roll rate (rad/s)")
axes[0].grid(True)
axes[0].legend()

axes[1].plot(times, quaternion[:, 0], label="qw")
axes[1].plot(times, quaternion[:, 1], label="qx")
axes[1].set_ylabel("Quaternion components")
axes[1].set_xlabel("Time (s)")
axes[1].grid(True)
axes[1].legend()

fig.suptitle("Quadcopter Roll Response")
fig.tight_layout()
plt.show()

print("Final body angular velocity (rad/s):", angular_velocity[-1])
print("Final quaternion:", quaternion[-1])
print("Final quaternion norm:", np.linalg.norm(quaternion[-1]))
