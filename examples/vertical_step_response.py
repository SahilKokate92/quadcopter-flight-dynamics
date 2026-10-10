
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import matplotlib.pyplot as plt

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
)
from src.quadcopter.simulator import simulate


# Vehicle and hover parameters

hover_speed = np.sqrt(
    MASS_KG * GRAVITY_M_S2
    / (4.0 * THRUST_COEFFICIENT)
)

initial_state = np.zeros(17)
initial_state[6] = 1.0  # Identity quaternion
initial_state[13:17] = hover_speed


# Motor command: temporary vertical thrust step

def motor_command(t):
    if 0.5 <= t < 1.5:
        return np.full(4, hover_speed + 100.0)

    return np.full(4, hover_speed)


# Run simulation

times, states = simulate(
    initial_state=initial_state,
    commanded_rotor_speeds_rad_s=np.full(4, hover_speed),
    duration_s=3.0,
    dt=0.001,
    command_function=motor_command,
)

# Extract response

altitude_up = -states[:, 2]
vertical_velocity_up = -states[:, 5]
rotor_speeds = states[:, 13:17]

vertical_acceleration_up = np.gradient(
    vertical_velocity_up,
    times,
)

# Ideal total thrust based on actual rotor speeds
total_thrust = (
    THRUST_COEFFICIENT
    * np.sum(rotor_speeds**2, axis=1)
)

ideal_vertical_acceleration = (
    total_thrust - MASS_KG * GRAVITY_M_S2
) / MASS_KG


# Print validation metrics

print(f"Hover speed: {hover_speed:.6f} rad/s")
print(f"Maximum altitude gain: {np.max(altitude_up):.4f} m")
print(f"Final altitude gain: {altitude_up[-1]:.4f} m")
print(f"Vertical velocity at 1.5 s: "
      f"{np.interp(1.5, times, vertical_velocity_up):.4f} m/s")
print(f"Vertical velocity at 3.0 s: "
      f"{vertical_velocity_up[-1]:.4f} m/s")

print("\nAcceleration comparison:")
for sample_time in [0.5, 1.0, 1.5, 2.0, 3.0]:
    index = np.argmin(np.abs(times - sample_time))
    print(
        f"t = {times[index]:.2f} s | "
        f"From velocity = {vertical_acceleration_up[index]:+.6f} m/s^2 | "
        f"From thrust = {ideal_vertical_acceleration[index]:+.6f} m/s^2"
    )

print("\nFinal rotor speeds (rad/s):")
print(rotor_speeds[-1])

#  Plot the response

fig, axes = plt.subplots(4, 1, figsize=(10, 12), sharex=True)

axes[0].plot(times, altitude_up)
axes[0].set_ylabel("Altitude gain (m)")
axes[0].grid(True)

axes[1].plot(times, vertical_velocity_up)
axes[1].set_ylabel("Upward velocity (m/s)")
axes[1].grid(True)

axes[2].plot(times, rotor_speeds[:, 0], label="Rotor 1")
axes[2].plot(times, rotor_speeds[:, 1], "--", label="Rotor 2")
axes[2].axhline(hover_speed, linestyle=":", label="Hover speed")
axes[2].set_ylabel("Rotor speed (rad/s)")
axes[2].legend()
axes[2].grid(True)

axes[3].plot(
    times,
    ideal_vertical_acceleration,
    label="From thrust",
)
axes[3].plot(
    times,
    vertical_acceleration_up,
    "--",
    label="From velocity derivative",
)
axes[3].set_ylabel("Acceleration (m/s²)")
axes[3].set_xlabel("Time (s)")
axes[3].legend()
axes[3].grid(True)

fig.suptitle("Quadcopter Vertical Step Response")
fig.tight_layout()
plt.show()
