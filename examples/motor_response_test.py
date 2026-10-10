import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT))

import numpy as np
import matplotlib.pyplot as plt

from config.vehicle import MOTOR_TIME_CONSTANT_S
from src.quadcopter.integrators import rk4_step
from src.quadcopter.motors import motor_speed_derivative


initial_speed = 300.0
commanded_speed = 700.0
dt = 0.001
duration_s = 0.20

current_speeds = np.full(4, initial_speed)
commands = np.full(4, commanded_speed)

number_of_steps = int(round(duration_s / dt))
times = np.arange(number_of_steps + 1) * dt
history = np.zeros((number_of_steps + 1, 4))
history[0] = current_speeds

for index in range(number_of_steps):
    current_speeds = rk4_step(
        derivative=lambda t, speeds: motor_speed_derivative(
            speeds, commands
        ),
        t=times[index],
        state=current_speeds,
        dt=dt,
    )
    history[index + 1] = current_speeds

analytical_speed = commanded_speed + (
    initial_speed - commanded_speed
) * np.exp(-times / MOTOR_TIME_CONSTANT_S)

print(f"Motor time constant: {MOTOR_TIME_CONSTANT_S:.3f} s")
print(f"Final simulated speed: {history[-1, 0]:.6f} rad/s")
print(f"Final analytical speed: {analytical_speed[-1]:.6f} rad/s")

np.testing.assert_allclose(
    history[:, 0], analytical_speed, rtol=1e-8, atol=1e-8
)

plt.figure(figsize=(9, 5))
plt.plot(times, history[:, 0], label="RK4 motor model")
plt.plot(times, analytical_speed, "--", label="Analytical response")
plt.axhline(commanded_speed, linestyle=":", label="Commanded speed")
plt.xlabel("Time (s)")
plt.ylabel("Rotor speed (rad/s)")
plt.title("First-Order Motor Response")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()