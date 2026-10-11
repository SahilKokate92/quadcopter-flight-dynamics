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

void test_collective_thrust_response() {
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
    constexpr double thrust_coefficient = 8.0e-6;
    constexpr double dt = 0.001;
    constexpr int steps = 500;

    const double hover_speed =
        std::sqrt((mass * gravity) / (4.0 * thrust_coefficient));

    const RotorSpeeds initial_speeds{
        hover_speed, hover_speed, hover_speed, hover_speed
    };

    const RotorSpeeds commanded_speeds{
        hover_speed + 100.0,
        hover_speed + 100.0,
        hover_speed + 100.0,
        hover_speed + 100.0
    };

    QuadcopterState state{
        RigidBodyState{
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}
        },
        initial_speeds
    };

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(state, commanded_speeds, dt);
    }

    const auto& body = state.rigid_body;

    // In NED coordinates, negative vertical velocity means upward motion.
    require(body.velocity_body_m_s[2] < 0.0,
            "Increased collective thrust should produce upward motion");

    for (double speed : state.rotor_speeds_rad_s) {
        require(speed > hover_speed,
                "Rotor speed should rise above hover speed");
        require(speed < commanded_speeds[0],
                "Rotor speed should approach command gradually");
    }

    const auto& q = body.quaternion_nb;
    double norm_squared = 0.0;
    for (double component : q) {
        norm_squared += component * component;
    }

    require(std::abs(std::sqrt(norm_squared) - 1.0) < 1e-10,
            "Quaternion must remain normalized");

        require(std::abs(body.angular_velocity_body_rad_s[0]) < 1e-8 &&
            std::abs(body.angular_velocity_body_rad_s[1]) < 1e-8 &&
            std::abs(body.angular_velocity_body_rad_s[2]) < 1e-8,
            "Symmetric collective command should not induce rotation");

    std::cout << "Collective thrust response passed.\n"
              << "Vertical body velocity: "
              << body.velocity_body_m_s[2] << " m/s\n";
}

}  // namespace

int main() {
    try {
        test_collective_thrust_response();
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "Collective thrust response failed: "
                  << error.what() << '\n';
        return EXIT_FAILURE;
    }
}