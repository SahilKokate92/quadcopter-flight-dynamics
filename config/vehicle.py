
# Vehicle parameters for the quadcopter simulator.
# Values are illustrative placeholders.

MASS_KG = 1.2

GRAVITY_M_S2 = 9.80665

# Rotor geometry: X configuration.
# Distance from the center to each motor along body x and y.
ROTOR_OFFSET_M = 0.141421356

# Provisional propulsion parameters; not measured hardware values.
THRUST_COEFFICIENT = 8.0e-6       # N / (rad/s)^2
TORQUE_COEFFICIENT = 1.5e-7       # N*m / (rad/s)^2
MAX_ROTOR_SPEED_RAD_S = 1200.0
MOTOR_TIME_CONSTANT_S = 0.04

# Rotor spin signs used to calculate reaction torque.
# Rotor order: front-right, rear-right, rear-left, front-left.
ROTOR_SPIN_SIGNS = (1, -1, 1, -1)

# Moments of inertia about the body-frame axes.
# Assumption: body axes coincide with principal inertia axes.
IXX_KG_M2 = 0.015
IYY_KG_M2 = 0.015
IZZ_KG_M2 = 0.025

# Inertia matrix, expressed in the body frame.
INERTIA_KG_M2 = (
    (IXX_KG_M2, 0.0, 0.0),
    (0.0, IYY_KG_M2, 0.0),
    (0.0, 0.0, IZZ_KG_M2),
)