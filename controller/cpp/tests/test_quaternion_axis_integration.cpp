#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

constexpr double kTolerance = 1e-10;

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

void require_near(
    double actual,
    double expected,
    const char* message
) {
    if (std::abs(actual - expected) > kTolerance) {
        std::cerr << message << ": expected " << expected
                  << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
}

void test_constant_body_axis_rotation() {
    TestSystem system;

    constexpr double dt = 0.001;
    constexpr int steps = 1000;
    constexpr double duration = dt * steps;
    constexpr double rate = 0.5;
    constexpr double half_angle = rate * duration / 2.0;

    const RotorSpeeds constant_speeds{
        500.0, 500.0, 500.0, 500.0
    };

    struct Case {
        const char* name;
        int rate_axis;
        int quaternion_axis;
    };

    const Case cases[] = {
        {"Roll", 0, 1},
        {"Pitch", 1, 2}
    };

    for (const Case& test : cases) {
        Vec3 angular_rates{0.0, 0.0, 0.0};
        angular_rates[test.rate_axis] = rate;

        QuadcopterState state{
            RigidBodyState{
                {0.0, 0.0, 0.0},
                {0.0, 0.0, 0.0},
                {1.0, 0.0, 0.0, 0.0},
                angular_rates
            },
            constant_speeds
        };

        for (int i = 0; i < steps; ++i) {
            state = system.integrator.step(
                state, constant_speeds, dt
            );
        }

        const Quaternion& q = state.rigid_body.quaternion_nb;

        require_near(
            q[0], std::cos(half_angle),
            test.name
        );

        require_near(
            q[test.quaternion_axis], std::sin(half_angle),
            test.name
        );

        for (int axis = 1; axis < 4; ++axis) {
            if (axis != test.quaternion_axis) {
                require_near(
                    q[axis], 0.0,
                    "Unexpected quaternion component"
                );
            }
        }

        double norm_squared = 0.0;
        for (double component : q) {
            norm_squared += component * component;
        }

        require_near(
            std::sqrt(norm_squared), 1.0,
            "Quaternion norm"
        );

        std::cout << test.name
                  << " integration passed; quaternion = ["
                  << q[0] << ", " << q[1] << ", "
                  << q[2] << ", " << q[3] << "]\n";
    }
}

}  // namespace

int main() {
    try {
        test_constant_body_axis_rotation();
        std::cout << "All quaternion axis integration tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Quaternion integration test failed: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
