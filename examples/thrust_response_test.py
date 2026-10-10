import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import matplotlib.pyplot as plt

from src.quadcopter.simulator import simulate


MASS_KG = 1.2
GRAVITY_M_S2 = 9.80665
THRUST_COEFFICIENT = 8.0e-6

HOVER_SPEED = np.sqrt(
    MASS_KG * GRAVITY_M_S2
    / (4.0 * THRUST_COEFFICIENT)
)

initial_state = np.zeros(17)
initial_state[6] = 1.0
initial_state[13:17] = HOVER_SPEED


def motor_command(t):
    if 0.5 <= t < 1.5:
        return np.full(4, HOVER_SPEED + 100.0)

    return np.full(4, HOVER_SPEED)


times, states = simulate(
    initial_state=initial_state,
    commanded_rotor_speeds_rad_s=np.full(4, HOVER_SPEED),
    duration_s=3.0,
    dt=0.001,
    command_function=motor_command,
)

altitude_up_m = -states[:, 2]
vertical_velocity_up_m_s = -(
    states[:, 3] * 0.0 + states[:, 5]
)

rotor_speeds = states[:, 13:17]

fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)

axes[0].plot(times, rotor_speeds[:, 0], label="Rotor 1")
axes[0].plot(times, rotor_speeds[:, 1], label="Rotor 2")
axes[0].set_ylabel("Rotor speed (rad/s)")
axes[0].legend()
axes[0].grid(True)

axes[1].plot(times, altitude_up_m)
axes[1].set_ylabel("Altitude above start (m)")
axes[1].grid(True)

axes[2].plot(times, vertical_velocity_up_m_s)
axes[2].set_ylabel("Vertical velocity (m/s)")
axes[2].set_xlabel("Time (s)")
axes[2].grid(True)

plt.tight_layout()
plt.show()

print(f"Final altitude change: {altitude_up_m[-1]:.4f} m")
print(
    "Maximum altitude change: "
    f"{np.max(altitude_up_m):.4f} m"
)

print(
    f"Vertical velocity at 1.5 s: "
    f"{vertical_velocity_up_m_s[np.argmin(np.abs(times - 1.5))]:.4f} m/s"
)

print(
    f"Vertical velocity at 3.0 s: "
    f"{vertical_velocity_up_m_s[-1]:.4f} m/s"
)


# Check the final rotor speeds and vertical acceleration.
final_rotor_speeds = states[-1, 13:17]

final_thrust_n = 8.0e-6 * np.sum(
    final_rotor_speeds**2
)

final_vertical_acceleration = (
    final_thrust_n / MASS_KG - GRAVITY_M_S2
)

print("Final rotor speeds (rad/s):", final_rotor_speeds)
print(f"Final total thrust: {final_thrust_n:.6f} N")
print(
    "Final ideal vertical acceleration: "
    f"{final_vertical_acceleration:.8f} m/s^2"
)


# Inspect the final attitude and body-frame velocity.
print("Final quaternion [qw, qx, qy, qz]:")
print(states[-1, 6:10])

print("Final body velocity [u, v, w] (m/s):")
print(states[-1, 3:6])

print("Final angular velocity [p, q, r] (rad/s):")
print(states[-1, 10:13])


# Estimate upward acceleration from the simulated vertical velocity.
vertical_acceleration_up_m_s2 = np.gradient(
    vertical_velocity_up_m_s,
    times,
)

print(
    "Vertical acceleration at 1.5 s: "
    f"{vertical_acceleration_up_m_s2[np.argmin(np.abs(times - 1.5))]:.6f} m/s^2"
)

print(
    "Vertical acceleration at 2.0 s: "
    f"{vertical_acceleration_up_m_s2[np.argmin(np.abs(times - 2.0))]:.6f} m/s^2"
)

print(
    "Vertical acceleration at 3.0 s: "
    f"{vertical_acceleration_up_m_s2[-1]:.6f} m/s^2"
)
