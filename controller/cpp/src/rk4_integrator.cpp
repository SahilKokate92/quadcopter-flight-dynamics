#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <cstddef>
#include <stdexcept>

namespace quadcopter {
namespace {

QuadcopterState add_scaled(
    const QuadcopterState& state,
    const QuadcopterDerivatives& derivative,
    double scale
) {
    QuadcopterState result = state;

    for (std::size_t i = 0; i < 3; ++i) {
        result.rigid_body.position_ned_m[i] +=
            scale * derivative.rigid_body.position_dot_ned_m_s[i];

        result.rigid_body.velocity_body_m_s[i] +=
            scale * derivative.rigid_body.velocity_dot_body_m_s2[i];

        result.rigid_body.angular_velocity_body_rad_s[i] +=
            scale * derivative.rigid_body.angular_velocity_dot_body_rad_s2[i];
    }

    for (std::size_t i = 0; i < 4; ++i) {
        result.rigid_body.quaternion_nb[i] +=
            scale * derivative.rigid_body.quaternion_dot_nb[i];

        result.rotor_speeds_rad_s[i] +=
            scale * derivative.rotor_speed_derivatives_rad_s2[i];
    }

    return result;
}

QuadcopterState weighted_sum(
    const QuadcopterState& initial,
    const QuadcopterDerivatives& k1,
    const QuadcopterDerivatives& k2,
    const QuadcopterDerivatives& k3,
    const QuadcopterDerivatives& k4,
    double dt
) {
    QuadcopterState result = initial;

    for (std::size_t i = 0; i < 3; ++i) {
        result.rigid_body.position_ned_m[i] += dt / 6.0 *
            (k1.rigid_body.position_dot_ned_m_s[i]
             + 2.0 * k2.rigid_body.position_dot_ned_m_s[i]
             + 2.0 * k3.rigid_body.position_dot_ned_m_s[i]
             + k4.rigid_body.position_dot_ned_m_s[i]);

        result.rigid_body.velocity_body_m_s[i] += dt / 6.0 *
            (k1.rigid_body.velocity_dot_body_m_s2[i]
             + 2.0 * k2.rigid_body.velocity_dot_body_m_s2[i]
             + 2.0 * k3.rigid_body.velocity_dot_body_m_s2[i]
             + k4.rigid_body.velocity_dot_body_m_s2[i]);

        result.rigid_body.angular_velocity_body_rad_s[i] += dt / 6.0 *
            (k1.rigid_body.angular_velocity_dot_body_rad_s2[i]
             + 2.0 * k2.rigid_body.angular_velocity_dot_body_rad_s2[i]
             + 2.0 * k3.rigid_body.angular_velocity_dot_body_rad_s2[i]
             + k4.rigid_body.angular_velocity_dot_body_rad_s2[i]);
    }

    for (std::size_t i = 0; i < 4; ++i) {
        result.rigid_body.quaternion_nb[i] += dt / 6.0 *
            (k1.rigid_body.quaternion_dot_nb[i]
             + 2.0 * k2.rigid_body.quaternion_dot_nb[i]
             + 2.0 * k3.rigid_body.quaternion_dot_nb[i]
             + k4.rigid_body.quaternion_dot_nb[i]);

        result.rotor_speeds_rad_s[i] += dt / 6.0 *
            (k1.rotor_speed_derivatives_rad_s2[i]
             + 2.0 * k2.rotor_speed_derivatives_rad_s2[i]
             + 2.0 * k3.rotor_speed_derivatives_rad_s2[i]
             + k4.rotor_speed_derivatives_rad_s2[i]);
    }

    double norm_squared = 0.0;

    for (double value : result.rigid_body.quaternion_nb) {
        if (!std::isfinite(value)) {
            throw std::runtime_error(
                "RK4 produced a non-finite quaternion."
            );
        }

        norm_squared += value * value;
    }

    const double norm = std::sqrt(norm_squared);

    if (!std::isfinite(norm) || norm < 1e-12) {
        throw std::runtime_error(
            "RK4 produced an invalid quaternion."
        );
    }

    for (double& value : result.rigid_body.quaternion_nb) {
        value /= norm;
    }

    return result;
}

}  // namespace

Rk4Integrator::Rk4Integrator(const QuadcopterDynamics& dynamics)
    : dynamics_(dynamics) {}

QuadcopterState Rk4Integrator::step(
    const QuadcopterState& state,
    const RotorSpeeds& commanded_speeds_rad_s,
    double time_step_s
) const {
    if (!std::isfinite(time_step_s) || time_step_s <= 0.0) {
        throw std::invalid_argument(
            "RK4 time step must be finite and positive."
        );
    }

    const auto k1 =
        dynamics_.derivatives(state, commanded_speeds_rad_s);

    const auto state2 =
        add_scaled(state, k1, 0.5 * time_step_s);
    const auto k2 =
        dynamics_.derivatives(state2, commanded_speeds_rad_s);

    const auto state3 =
        add_scaled(state, k2, 0.5 * time_step_s);
    const auto k3 =
        dynamics_.derivatives(state3, commanded_speeds_rad_s);

    const auto state4 =
        add_scaled(state, k3, time_step_s);
    const auto k4 =
        dynamics_.derivatives(state4, commanded_speeds_rad_s);

    return weighted_sum(state, k1, k2, k3, k4, time_step_s);
}

}  // namespace quadcopter