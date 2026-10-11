#include "quadcopter/rigid_body_dynamics.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    constexpr double mass = 1.2;
    constexpr double gravity = 9.80665;
    constexpr double tolerance = 1e-10;

    quadcopter::RigidBodyParameters parameters{
        mass,
        gravity,
        {0.015, 0.015, 0.025}
    };

    quadcopter::RigidBodyDynamics dynamics(parameters);

    quadcopter::RigidBodyState state{
        {0.0, 0.0, 0.0},
        {4.0, -2.0, 1.5},
        {1.0, 0.0, 0.0, 0.0},
        {0.7, -0.4, 0.9}
    };

    quadcopter::BodyWrench wrench{
        {2.4, -1.2, 3.6},
        {0.0, 0.0, 0.0}
    };

    const auto result = dynamics.derivatives(state, wrench);

    const auto& v = state.velocity_body_m_s;
    const auto& w = state.angular_velocity_body_rad_s;
    const auto& F = wrench.force_body_n;

    // omega cross velocity
    const std::array<double, 3> cross{
        w[1] * v[2] - w[2] * v[1],
        w[2] * v[0] - w[0] * v[2],
        w[0] * v[1] - w[1] * v[0]
    };

    // Identity quaternion: body axes align with NED.
    const std::array<double, 3> expected{
        F[0] / mass - cross[0],
        F[1] / mass - cross[1],
        F[2] / mass + gravity - cross[2]
    };

    for (std::size_t i = 0; i < 3; ++i) {
        if (std::abs(result.velocity_dot_body_m_s2[i] - expected[i])
            > tolerance) {
            std::cerr << "Translational dynamics mismatch on axis "
                      << i << ": expected " << expected[i]
                      << ", got "
                      << result.velocity_dot_body_m_s2[i] << '\n';
            return 1;
        }
    }

    std::cout << "Translational dynamics validation passed.\n";
    return 0;
}
