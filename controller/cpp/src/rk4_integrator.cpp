#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <stdexcept>

namespace quadcopter {
namespace {

RigidBodyState add_scaled(
    const RigidBodyState& state,
    const RigidBodyDerivatives& derivative,
    double scale
) {
    RigidBodyState result = state;

    for (std::size_t i = 0; i < 3; ++i) {
        result.position_ned_m[i] +=
            scale * derivative.position_dot_ned_m_s[i];

        result.velocity_body_m_s[i] +=
            scale * derivative.velocity_dot_body_m_s2[i];

        result.angular_velocity_body_rad_s[i] +=
            scale * derivative.angular_velocity_dot_body_rad_s2[i];
    }

    for (std::size_t i = 0; i < 4; ++i) {
        result.quaternion_nb[i] +=
            scale * derivative.quaternion_dot_nb[i];
    }

    return result;
}

RigidBodyState weighted_sum(
    const RigidBodyState& initial,
    const RigidBodyDerivatives& k1,
    const RigidBodyDerivatives& k2,
    const RigidBodyDerivatives& k3,
    const RigidBodyDerivatives& k4,
    double dt
) {
    RigidBodyState result = initial;

    for (std::size_t i = 0; i < 3; ++i) {
        result.position_ned_m[i] += dt / 6.0 *
            (k1.position_dot_ned_m_s[i]
             + 2.0 * k2.position_dot_ned_m_s[i]
             + 2.0 * k3.position_dot_ned_m_s[i]
             + k4.position_dot_ned_m_s[i]);

        result.velocity_body_m_s[i] += dt / 6.0 *
            (k1.velocity_dot_body_m_s2[i]
             + 2.0 * k2.velocity_dot_body_m_s2[i]
             + 2.0 * k3.velocity_dot_body_m_s2[i]
             + k4.velocity_dot_body_m_s2[i]);

        result.angular_velocity_body_rad_s[i] += dt / 6.0 *
            (k1.angular_velocity_dot_body_rad_s2[i]
             + 2.0 * k2.angular_velocity_dot_body_rad_s2[i]
             + 2.0 * k3.angular_velocity_dot_body_rad_s2[i]
             + k4.angular_velocity_dot_body_rad_s2[i]);
    }

    for (std::size_t i = 0; i < 4; ++i) {
        result.quaternion_nb[i] += dt / 6.0 *
            (k1.quaternion_dot_nb[i]
             + 2.0 * k2.quaternion_dot_nb[i]
             + 2.0 * k3.quaternion_dot_nb[i]
             + k4.quaternion_dot_nb[i]);
    }

    double norm_squared = 0.0;

    for (double value : result.quaternion_nb) {
        if (!std::isfinite(value)) {
            throw std::runtime_error(
                "RK4 produced a non-finite quaternion."
            );
        }
        norm_squared += value * value;
    }

    const double norm = std::sqrt(norm_squared);

    if (norm < 1e-12) {
        throw std::runtime_error(
            "RK4 produced a zero quaternion."
        );
    }

    for (double& value : result.quaternion_nb) {
        value /= norm;
    }

    return result;
}

}  // namespace

Rk4Integrator::Rk4Integrator(const RigidBodyDynamics& dynamics)
    : dynamics_(dynamics) {}

RigidBodyState Rk4Integrator::step(
    const RigidBodyState& state,
    const BodyWrench& wrench,
    double time_step_s
) const {
    if (!std::isfinite(time_step_s) || time_step_s <= 0.0) {
        throw std::invalid_argument(
            "RK4 time step must be finite and positive."
        );
    }

    const auto k1 = dynamics_.derivatives(state, wrench);

    const auto state2 = add_scaled(state, k1, 0.5 * time_step_s);
    const auto k2 = dynamics_.derivatives(state2, wrench);

    const auto state3 = add_scaled(state, k2, 0.5 * time_step_s);
    const auto k3 = dynamics_.derivatives(state3, wrench);

    const auto state4 = add_scaled(state, k3, time_step_s);
    const auto k4 = dynamics_.derivatives(state4, wrench);

    return weighted_sum(state, k1, k2, k3, k4, time_step_s);
}

}  // namespace quadcopter
