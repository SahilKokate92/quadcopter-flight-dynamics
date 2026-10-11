#include "quadcopter/rotor_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace quadcopter {
namespace {

Vec3 cross_product(const Vec3& a, const Vec3& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

void validate_parameters(const RotorParameters& p) {
    if (!std::isfinite(p.thrust_coefficient_n_per_rad_s_squared) ||
        p.thrust_coefficient_n_per_rad_s_squared < 0.0 ||
        !std::isfinite(p.torque_coefficient_nm_per_rad_s_squared) ||
        p.torque_coefficient_nm_per_rad_s_squared < 0.0 ||
        !std::isfinite(p.max_rotor_speed_rad_s) ||
        p.max_rotor_speed_rad_s <= 0.0 ||
        !std::isfinite(p.rotor_offset_m) ||
        p.rotor_offset_m <= 0.0) {
        throw std::invalid_argument("Invalid rotor parameters.");
    }

    for (double sign : p.rotor_spin_signs) {
        if (!std::isfinite(sign)) {
            throw std::invalid_argument("Rotor spin signs must be finite.");
        }
    }
}

}  // namespace

RotorDynamics::RotorDynamics(const RotorParameters& parameters)
    : parameters_(parameters) {
    validate_parameters(parameters_);
}

RotorWrench RotorDynamics::calculate(
    const RotorSpeeds& rotor_speeds_rad_s
) const {
    RotorWrench result{};
    RotorThrusts reaction_torques{};

    const double d = parameters_.rotor_offset_m;

    const std::array<Vec3, 4> positions{{
        { d, -d, 0.0},  // Front-right
        {-d, -d, 0.0},  // Rear-right
        {-d,  d, 0.0},  // Rear-left
        { d,  d, 0.0}   // Front-left
    }};

    for (std::size_t i = 0; i < 4; ++i) {
        const double speed = rotor_speeds_rad_s[i];

        if (!std::isfinite(speed) || speed < 0.0) {
            throw std::invalid_argument(
                "Rotor speeds must be finite and non-negative."
            );
        }

        const double bounded_speed = std::clamp(
            speed, 0.0, parameters_.max_rotor_speed_rad_s
        );

        const double speed_squared = bounded_speed * bounded_speed;

        result.individual_thrusts_n[i] =
            parameters_.thrust_coefficient_n_per_rad_s_squared
            * speed_squared;

        reaction_torques[i] =
            parameters_.torque_coefficient_nm_per_rad_s_squared
            * speed_squared
            * parameters_.rotor_spin_signs[i];

        const Vec3 rotor_force{
            0.0,
            0.0,
            -result.individual_thrusts_n[i]
        };

        const Vec3 arm_torque = cross_product(positions[i], rotor_force);

        for (std::size_t axis = 0; axis < 3; ++axis) {
            result.torque_body_nm[axis] += arm_torque[axis];
        }

        result.total_thrust_n += result.individual_thrusts_n[i];
    }

    result.force_body_n = {
        0.0,
        0.0,
        -result.total_thrust_n
    };

    result.torque_body_nm[2] +=
        reaction_torques[0] + reaction_torques[1]
        + reaction_torques[2] + reaction_torques[3];

    return result;
}

}  // namespace quadcopter
