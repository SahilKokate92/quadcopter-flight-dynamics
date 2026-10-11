#pragma once

#include "quadcopter/rotor_dynamics.hpp"

namespace quadcopter {

struct MotorParameters {
    double time_constant_s;
    double max_rotor_speed_rad_s;
};

class MotorDynamics {
public:
    explicit MotorDynamics(const MotorParameters& parameters);

    RotorSpeeds derivatives(
        const RotorSpeeds& current_speeds_rad_s,
        const RotorSpeeds& commanded_speeds_rad_s
    ) const;

private:
    MotorParameters parameters_;
};

}  // namespace quadcopter
