#include "quadcopter/quadcopter_dynamics.hpp"

namespace quadcopter {

QuadcopterDynamics::QuadcopterDynamics(
    const RigidBodyDynamics& rigid_body_dynamics,
    const RotorDynamics& rotor_dynamics,
    const MotorDynamics& motor_dynamics
)
    : rigid_body_dynamics_(rigid_body_dynamics),
      rotor_dynamics_(rotor_dynamics),
      motor_dynamics_(motor_dynamics) {}

QuadcopterDerivatives QuadcopterDynamics::derivatives(
    const QuadcopterState& state,
    const RotorSpeeds& commanded_speeds_rad_s
) const {
    // Calculate total force and torque produced by the four rotors.
    const RotorWrench rotor_wrench =
        rotor_dynamics_.calculate(state.rotor_speeds_rad_s);

    // Convert the rotor wrench into rigid-body derivatives.
    const BodyWrench body_wrench{
        rotor_wrench.force_body_n,
        rotor_wrench.torque_body_nm
    };

    const RigidBodyDerivatives rigid_body_derivatives =
        rigid_body_dynamics_.derivatives(
            state.rigid_body,
            body_wrench
        );

    // Calculate the four motor-speed derivatives.
    const RotorSpeeds rotor_speed_derivatives =
        motor_dynamics_.derivatives(
            state.rotor_speeds_rad_s,
            commanded_speeds_rad_s
        );

    return {
        rigid_body_derivatives,
        rotor_speed_derivatives
    };
}

}  // namespace quadcopter