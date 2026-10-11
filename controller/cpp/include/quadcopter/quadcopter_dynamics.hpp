#pragma once

#include "quadcopter/motor_dynamics.hpp"
#include "quadcopter/quadcopter_state.hpp"
#include "quadcopter/rigid_body_dynamics.hpp"

namespace quadcopter {

struct QuadcopterDerivatives {
    RigidBodyDerivatives rigid_body;
    RotorSpeeds rotor_speed_derivatives_rad_s2;
};

class QuadcopterDynamics {
public:
    QuadcopterDynamics(
        const RigidBodyDynamics& rigid_body_dynamics,
        const RotorDynamics& rotor_dynamics,
        const MotorDynamics& motor_dynamics
    );

    QuadcopterDerivatives derivatives(
        const QuadcopterState& state,
        const RotorSpeeds& commanded_speeds_rad_s
    ) const;

private:
    const RigidBodyDynamics& rigid_body_dynamics_;
    const RotorDynamics& rotor_dynamics_;
    const MotorDynamics& motor_dynamics_;
};

}  // namespace quadcopter