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

struct TestSystem {
    RigidBodyDynamics rigid_body{
        RigidBodyParameters{
            1.2, 9.80665, {0.015, 0.015, 0.025}
        }
    };

    RotorDynamics rotors{
        RotorParameters{
            8.0e-6,
            1.5e-7,
            1200.0,
            0.141421356,
            {1.0, -1.0, 1.0, -1.0}
        }
    };

    MotorDynamics motors{MotorParameters{0.04, 1200.0}};

    QuadcopterDynamics dynamics{rigid_body, rotors, motors};
    Rk4Integrator integrator{dynamics};
};

QuadcopterState make_state(
    Vec3 position,
    Vec3 velocity,
    Quaternion quaternion,
    Vec3 angular_velocity,
    RotorSpeeds rotor_speeds
) {
    return {
        RigidBodyState{
            position, velocity, quaternion, angular_velocity
        },
        rotor_speeds
    };
}

void test_free_fall() {
    TestSystem system;

    const QuadcopterState initial = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    );

    // Zero rotor commands and speeds produce no thrust.
    const RotorSpeeds commands{0.0, 0.0, 0.0, 0.0};

    const double dt = 0.01;
    const int steps = 100;
    QuadcopterState state = initial;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, commands, dt);
    }

    const double elapsed_time = dt * steps;
    const double expected_velocity =
        9.80665 * elapsed_time;
    const double expected_position =
        0.5 * 9.80665 * elapsed_time * elapsed_time;

    require_near(
        state.rigid_body.velocity_body_m_s[2],
        expected_velocity,
        kTolerance,
        "Free-fall down velocity"
    );

    require_near(
        state.rigid_body.position_ned_m[2],
        expected_position,
        kTolerance,
        "Free-fall down position"
    );
}

void test_constant_yaw_rate() {
    TestSystem system;

    constexpr double yaw_rate = 0.5;

    // Equal rotor speeds generate zero net thrust and yaw reaction torque.
    const RotorSpeeds hoverless_speeds{
        500.0, 500.0, 500.0, 500.0
    };

    const QuadcopterState initial = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, yaw_rate},
        hoverless_speeds
    );

    const double dt = 0.001;
    const int steps = 1000;
    QuadcopterState state = initial;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, hoverless_speeds, dt);
    }

    const double total_angle = yaw_rate * dt * steps;

    require_near(
        state.rigid_body.quaternion_nb[0],
        std::cos(total_angle / 2.0),
        1e-8,
        "Yaw quaternion scalar"
    );

    require_near(
        state.rigid_body.quaternion_nb[3],
        std::sin(total_angle / 2.0),
        1e-8,
        "Yaw quaternion z component"
    );

    double norm_squared = 0.0;
    for (double component : state.rigid_body.quaternion_nb) {
        norm_squared += component * component;
    }

    require_near(
        std::sqrt(norm_squared),
        1.0,
        1e-12,
        "Quaternion norm"
    );
}

void test_motor_speed_integration() {
    TestSystem system;

    const QuadcopterState initial = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    );

    const RotorSpeeds commands{400.0, 400.0, 400.0, 400.0};
    constexpr double tau = 0.04;
    constexpr double duration = 0.1;
    constexpr double dt = 0.001;
    constexpr int steps = 100;

    QuadcopterState state = initial;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, commands, dt);
    }

    const double expected_speed =
        400.0 * (1.0 - std::exp(-duration / tau));

    for (double speed : state.rotor_speeds_rad_s) {
        require_near(
            speed,
            expected_speed,
            1e-6,
            "Motor first-order response"
        );
    }
}

}  // namespace

int main() {
    try {
        test_free_fall();
        test_constant_yaw_rate();
        test_motor_speed_integration();

        std::cout << "All 17-state RK4 integrator tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "RK4 test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}