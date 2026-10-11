#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

constexpr double kTolerance = 1e-8;

void require_near(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << message
                  << ": expected " << expected
                  << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
}

void test_free_fall() {
    const RigidBodyParameters parameters{
        1.2,
        9.80665,
        {0.015, 0.015, 0.025}
    };

    const RigidBodyDynamics dynamics(parameters);
    const Rk4Integrator integrator(dynamics);

    const RigidBodyState initial{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    const BodyWrench wrench{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    const double dt = 0.01;
    const int steps = 100;

    RigidBodyState state = initial;

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(state, wrench, dt);
    }

    const double elapsed_time = dt * steps;

    // In NED coordinates, positive z points down.
    const double expected_down_velocity =
        parameters.gravity_m_s2 * elapsed_time;

    const double expected_down_position =
        0.5 * parameters.gravity_m_s2
        * elapsed_time * elapsed_time;

    require_near(
        state.velocity_body_m_s[2],
        expected_down_velocity,
        kTolerance,
        "Free-fall down velocity"
    );

    require_near(
        state.position_ned_m[2],
        expected_down_position,
        kTolerance,
        "Free-fall down position"
    );
}

void test_constant_yaw_rate() {
    const RigidBodyParameters parameters{
        1.2,
        9.80665,
        {0.015, 0.015, 0.025}
    };

    const RigidBodyDynamics dynamics(parameters);
    const Rk4Integrator integrator(dynamics);

    const double yaw_rate = 0.5;
    const RigidBodyState initial{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, yaw_rate}
    };

    const BodyWrench wrench{
        {0.0, 0.0, -parameters.mass_kg * parameters.gravity_m_s2},
        {0.0, 0.0, 0.0}
    };

    const double dt = 0.001;
    const int steps = 1000;
    RigidBodyState state = initial;

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(state, wrench, dt);
    }

    const double total_angle = yaw_rate * dt * steps;

    require_near(
        state.quaternion_nb[0],
        std::cos(total_angle / 2.0),
        1e-8,
        "Yaw quaternion scalar"
    );

    require_near(
        state.quaternion_nb[3],
        std::sin(total_angle / 2.0),
        1e-8,
        "Yaw quaternion z component"
    );

    double norm_squared = 0.0;
    for (double component : state.quaternion_nb) {
        norm_squared += component * component;
    }

    require_near(
        std::sqrt(norm_squared),
        1.0,
        1e-12,
        "Quaternion norm"
    );
}

}  // namespace

int main() {
    try {
        test_free_fall();
        test_constant_yaw_rate();

        std::cout << "All RK4 integrator tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "RK4 test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
