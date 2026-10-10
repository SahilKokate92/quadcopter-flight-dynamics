
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np
import matplotlib.pyplot as plt

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
    INERTIA_KG_M2,
    ROTOR_OFFSET_M,
)
from src.quadcopter.simulator import simulate


# ==================================================
# 1. INITIALIZE VEHICLE AT HOVER
# ==================================================

hover_speed = np.sqrt(
    MASS_KG * GRAVITY_M_S2
    / (4.0 * THRUST_COEFFICIENT)
)

initial_state = np.zeros(17)
initial_state[6] = 1.0
initial_state[13:17] = hover_speed


# ==================================================
# 2. DEFINE MOTOR COMMAND
# ==================================================


def motor_command(t):
    commands = np.full(4, hover_speed)

    # Apply a smaller differential-thrust input.
    # Increase the left rotor pair by 10 rad/s
    # for 0.2 seconds.
    if 0.5 <= t < 0.7:
        commands[2] += 10.0
        commands[3] += 10.0

    return commands


# ==================================================
# 3. RUN SIMULATION
# ==================================================

times, states = simulate(
    initial_state=initial_state,
    commanded_rotor_speeds_rad_s=np.full(4, hover_speed),
    duration_s=1.5,
    dt=0.001,
    command_function=motor_command,
)


# ==================================================
# 4. EXTRACT SIMULATED STATES
# ==================================================

angular_rates = states[:, 10:13]

roll_rate = angular_rates[:, 0]
pitch_rate = angular_rates[:, 1]
yaw_rate = angular_rates[:, 2]

quaternions = states[:, 6:10]
rotor_speeds = states[:, 13:17]

# Numerical angular acceleration from the simulated rates.
angular_accelerations_numerical = np.gradient(
    angular_rates,
    times,
    axis=0,
)

roll_acceleration_numerical = (
    angular_accelerations_numerical[:, 0]
)


# ==================================================
# 5. CALCULATE ROTOR TORQUES
# ==================================================

# Use the configured rotor arm length.
a = ROTOR_OFFSET_M

rotor_positions = np.array([
    [ a, -a, 0.0],  # Front-right
    [-a, -a, 0.0],  # Rear-right
    [-a,  a, 0.0],  # Rear-left
    [ a,  a, 0.0],  # Front-left
])

# Thrust from each rotor at each simulation time.
rotor_thrusts = (
    THRUST_COEFFICIENT * rotor_speeds**2
)

# Force vector shape: (time steps, rotors, axes).
rotor_force_vectors = np.zeros(
    (len(times), 4, 3)
)

rotor_force_vectors[:, :, 2] = -rotor_thrusts

# Moment from each rotor arm: r x F.
arm_torques = np.cross(
    rotor_positions[np.newaxis, :, :],
    rotor_force_vectors,
)

# Sum arm torques across all four rotors.
total_arm_torque = np.sum(
    arm_torques,
    axis=1,
)

# Include reaction torque about body z from rotor drag.
# This does not directly contribute to roll torque.
from config.vehicle import (
    TORQUE_COEFFICIENT,
    ROTOR_SPIN_SIGNS,
)

reaction_torque_z = np.sum(
    TORQUE_COEFFICIENT
    * rotor_speeds**2
    * np.asarray(ROTOR_SPIN_SIGNS)[np.newaxis, :],
    axis=1,
)

torque_body = total_arm_torque.copy()
torque_body[:, 2] += reaction_torque_z


# ==================================================
# 6. FULL RIGID-BODY ROTATIONAL EQUATION
# ==================================================

inertia = np.asarray(
    INERTIA_KG_M2,
    dtype=float,
)

# Angular momentum in body coordinates:
# H = I @ omega
angular_momentum = angular_rates @ inertia.T

# Gyroscopic coupling:
# omega x (I @ omega)
gyroscopic_term = np.cross(
    angular_rates,
    angular_momentum,
)

# omega_dot = I^-1 [torque - omega x (I omega)]
angular_accelerations_expected = np.linalg.solve(
    inertia,
    (torque_body - gyroscopic_term).T,
).T

roll_acceleration_expected = (
    angular_accelerations_expected[:, 0]
)


# ==================================================
# 7. PRINT VALIDATION RESULTS
# ==================================================

print("=" * 55)
print("QUADCOPTER ROLL DYNAMICS VALIDATION")
print("=" * 55)

print(f"Hover speed: {hover_speed:.6f} rad/s")
print(f"Rotor offset: {ROTOR_OFFSET_M:.6f} m")
print(f"Ixx: {inertia[0, 0]:.6f} kg m^2")

print("\nSimulated rotational response:")
print(
    f"Maximum absolute roll rate: "
    f"{np.max(np.abs(roll_rate)):.6f} rad/s"
)
print(f"Final roll rate: {roll_rate[-1]:.6f} rad/s")
print(f"Final pitch rate: {pitch_rate[-1]:.6f} rad/s")
print(f"Final yaw rate: {yaw_rate[-1]:.6f} rad/s")
print(f"Final quaternion: {quaternions[-1]}")

print("\nRoll acceleration comparison:")

for sample_time in [0.5, 0.7, 1.0, 1.2, 2.0]:
    index = np.argmin(
        np.abs(times - sample_time)
    )

    print(
        f"t = {times[index]:.2f} s | "
        f"Numerical = "
        f"{roll_acceleration_numerical[index]:+.6f} rad/s^2 | "
        f"Rigid-body equation = "
        f"{roll_acceleration_expected[index]:+.6f} rad/s^2"
    )


# ==================================================
# 8. PLOT RESULTS
# ==================================================

fig, axes = plt.subplots(
    3,
    1,
    figsize=(10, 11),
    sharex=True,
)

# Body angular rates.
axes[0].plot(
    times,
    roll_rate,
    label="Roll rate p",
)
axes[0].plot(
    times,
    pitch_rate,
    label="Pitch rate q",
)
axes[0].plot(
    times,
    yaw_rate,
    label="Yaw rate r",
)

axes[0].set_ylabel("Body rates (rad/s)")
axes[0].legend()
axes[0].grid(True)

# Quaternion components.
quaternion_labels = ["qw", "qx", "qy", "qz"]

for i, label in enumerate(quaternion_labels):
    axes[1].plot(
        times,
        quaternions[:, i],
        label=label,
    )

axes[1].set_ylabel("Quaternion components")
axes[1].legend()
axes[1].grid(True)

# Compare roll acceleration.
axes[2].plot(
    times,
    roll_acceleration_numerical,
    label="From angular-rate derivative",
)

axes[2].plot(
    times,
    roll_acceleration_expected,
    "--",
    label="Full rigid-body equation",
)

axes[2].set_ylabel("Roll acceleration (rad/s²)")
axes[2].set_xlabel("Time (s)")
axes[2].legend()
axes[2].grid(True)

fig.suptitle(
    "Quadcopter Roll Dynamics Validation"
)

fig.tight_layout()
plt.show()
