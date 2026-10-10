
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np

from config.vehicle import (
    MASS_KG,
    GRAVITY_M_S2,
    THRUST_COEFFICIENT,
)
from src.quadcopter.simulator import simulate

hover_speed = np.sqrt(
    MASS_KG * GRAVITY_M_S2
    / (4.0 * THRUST_COEFFICIENT)
)

initial_state = np.zeros(17)
initial_state[6] = 1.0
initial_state[13:17] = hover_speed

times, states = simulate(
    initial_state=initial_state,
    commanded_rotor_speeds_rad_s=np.full(4, hover_speed),
    duration_s=5.0,
    dt=0.001,
)

altitude_up = -states[:, 2]
vertical_velocity_up = -states[:, 5]

print(f"Hover speed: {hover_speed:.6f} rad/s")
print(f"Final altitude change: {altitude_up[-1]:.9f} m")
print(f"Final vertical velocity: {vertical_velocity_up[-1]:.9f} m/s")
print(f"Final angular rates: {states[-1, 10:13]}")
print(f"Maximum altitude deviation: {np.max(np.abs(altitude_up)):.9f} m")

assert np.max(np.abs(altitude_up)) < 1e-5
assert np.max(np.abs(vertical_velocity_up)) < 1e-5
assert np.max(np.abs(states[:, 10:13])) < 1e-5

print("HOVER VALIDATION PASSED")
