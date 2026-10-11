#include "quadcopter/motor_dynamics.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace quadcopter {

MotorDynamics::MotorDynamics(const MotorParameters& parameters)
    : parameters_(parameters) {
    if (!std::isfinite(parameters_.time_constant_s) ||
        parameters_.time_constant_s <= 0.0 ||
        !std::isfinite(parameters_.max_rotor_speed_rad_s) ||
        parameters_.max_rotor_speed_rad_s <= 0.0) {
        throw std::invalid_argument("Invalid motor parameters.");
    }
}

RotorSpeeds MotorDynamics::derivatives(
    const RotorSpeeds& current_speeds_rad_s,
    const RotorSpeeds& commanded_speeds_rad_s
) const {
    RotorSpeeds speed_derivatives{};

    for (std::size_t i = 0; i < 4; ++i) {
        const double current = current_speeds_rad_s[i];
        const double command = commanded_speeds_rad_s[i];

        if (!std::isfinite(current) || !std::isfinite(command)) {
            throw std::invalid_argument(
                "Motor speeds and commands must be finite."
            );
        }

        if (current < 0.0 || command < 0.0 ||
            current > parameters_.max_rotor_speed_rad_s ||
            command > parameters_.max_rotor_speed_rad_s) {
            throw std::invalid_argument(
                "Motor speeds must be within configured limits."
            );
        }

        speed_derivatives[i] =
            (command - current) / parameters_.time_constant_s;
    }

    return speed_derivatives;
}

}  // namespace quadcopter
