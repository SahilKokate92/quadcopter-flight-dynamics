#pragma once

#include <array>

namespace quadcopter {

using Vec3 = std::array<double, 3>;
using RotorSpeeds = std::array<double, 4>;
using RotorThrusts = std::array<double, 4>;

struct RotorParameters {
    double thrust_coefficient_n_per_rad_s_squared;
    double torque_coefficient_nm_per_rad_s_squared;
    double max_rotor_speed_rad_s;
    double rotor_offset_m;
    std::array<double, 4> rotor_spin_signs;
};

struct RotorWrench {
    double total_thrust_n;
    Vec3 force_body_n;
    Vec3 torque_body_nm;
    RotorThrusts individual_thrusts_n;
};

class RotorDynamics {
public:
    explicit RotorDynamics(const RotorParameters& parameters);

    RotorWrench calculate(
        const RotorSpeeds& rotor_speeds_rad_s
    ) const;

private:
    RotorParameters parameters_;
};

}  // namespace quadcopter
