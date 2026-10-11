#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

struct TestSystem {
    RigidBodyDynamics rigid_body{
        RigidBodyParameters{1.2, 9.80665, {0.015, 0.015, 0.025}}
    };

    RotorDynamics rotors{
        RotorParameters{
            8.0e-6, 1.5e-7, 1200.0, 0.141421356,
            {1.0, -1.0, 1.0, -1.0}
        }
    };

    MotorDynamics motors{MotorParameters{0.04, 1200.0}};
    QuadcopterDynamics dynamics{rigid_body, rotors, motors};
    Rk4Integrator integrator{dynamics};
};

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

double quaternion_norm(const Quaternion& q) {
    double sum = 0.0;
    for (double component : q) {
        sum += component * component;
    }
    return std::sqrt(sum);
}

void test_isolated_attitude_axes() {
    TestSystem system;

    constexpr double mass = 1.2;
    constexpr double gravity = 9.80665;
    constexpr double kT = 8.0e-6;

    const double hover_speed =
        std::sqrt(mass * gravity / (4.0 * kT));

    const RotorSpeeds hover{
        hover_speed, hover_speed, hover_speed, hover_speed
    };

    struct Case {
        const char* name;
        RotorSpeeds commands;
        int target_axis;
    };

    // Rotor order: front-right, rear-right, rear-left, front-left.
    const Case cases[] = {
        {"roll",  {650.0, 650.0, 550.0, 550.0}, 0},
        {"pitch", {650.0, 550.0, 550.0, 650.0}, 1},
        {"yaw",   {650.0, 550.0, 650.0, 550.0}, 2}
    };

    constexpr double dt = 0.001;
    constexpr int steps = 100;
    constexpr double cross_axis_tolerance = 1e-10;

    for (const Case& test : cases) {
        QuadcopterState state{
            RigidBodyState{
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {1.0, 0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0}
            },
            hover
        };

        for (int i = 0; i < steps; ++i) {
            state = system.integrator.step(state, test.commands, dt);
        }

        const auto& rates =
            state.rigid_body.angular_velocity_body_rad_s;

        require(
            rates[test.target_axis] > 0.0,
            "Target angular rate did not become positive"
        );

        for (int axis = 0; axis < 3; ++axis) {
            if (axis != test.target_axis) {
                require(
                    std::abs(rates[axis]) < cross_axis_tolerance,
                    "Unexpected cross-axis angular rate"
                );
            }
        }

        require(
            std::abs(quaternion_norm(state.rigid_body.quaternion_nb) - 1.0)
                < 1e-12,
            "Quaternion lost unit norm"
        );

        std::cout << test.name
                  << " response [p, q, r]: "
                  << rates[0] << ", "
                  << rates[1] << ", "
                  << rates[2] << " rad/s\n";
    }
}

}  // namespace

int main() {
    try {
        test_isolated_attitude_axes();
        std::cout << "All attitude response tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Attitude response test failed: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
