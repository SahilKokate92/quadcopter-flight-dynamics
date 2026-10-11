#include "quadcopter/rk4_integrator.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

void require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

void require_finite(double value, const char* message) {
    require(std::isfinite(value), message);
}

void test_full_model_numerical_stability() {
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

    constexpr double mass = 1.2;
    constexpr double gravity = 9.80665;
    constexpr double kT = 8.0e-6;
    constexpr double dt = 0.001;
    constexpr int steps = 3000;

    const double hover_speed =
        std::sqrt(mass * gravity / (4.0 * kT));

    QuadcopterState state{
        RigidBodyState{
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}
        },
        {hover_speed, hover_speed, hover_speed, hover_speed}
    };

    // Apply a sustained asymmetric command to exercise coupled dynamics.
    const RotorSpeeds commands{
        hover_speed + 30.0,
        hover_speed - 20.0,
        hover_speed + 10.0,
        hover_speed - 10.0
    };

    for (int step = 0; step < steps; ++step) {
        state = integrator.step(state, commands, dt);

        const auto& body = state.rigid_body;

        for (double value : body.position_ned_m) {
            require_finite(value, "Non-finite position");
        }

        for (double value : body.velocity_body_m_s) {
            require_finite(value, "Non-finite body velocity");
        }

        for (double value : body.quaternion_nb) {
            require_finite(value, "Non-finite quaternion component");
        }

        for (double value : body.angular_velocity_body_rad_s) {
            require_finite(value, "Non-finite angular velocity");
        }

        for (double value : state.rotor_speeds_rad_s) {
            require_finite(value, "Non-finite rotor speed");
        }

        double norm_squared = 0.0;
        for (double component : body.quaternion_nb) {
            norm_squared += component * component;
        }

        require(
            std::abs(std::sqrt(norm_squared) - 1.0) < 1e-10,
            "Quaternion normalization lost during simulation"
        );
    }

    std::cout
        << "Full-model numerical stability passed over "
        << steps << " steps (" << steps * dt << " s).\n";
}

}  // namespace

int main() {
    try {
        test_full_model_numerical_stability();
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Full-model stability test failed: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}