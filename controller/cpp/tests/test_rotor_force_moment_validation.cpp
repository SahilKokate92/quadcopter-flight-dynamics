#include "quadcopter/rotor_dynamics.hpp"

#include <array>
#include <cassert>
#include <cmath>
#include <iostream>

using namespace quadcopter;

namespace {

constexpr double TOL = 1e-10;

bool near(double actual, double expected) {
    return std::abs(actual - expected) <=
           TOL * (1.0 + std::abs(expected));
}

RotorParameters test_parameters() {
    return {
        8.0e-6,       // Thrust coefficient
        1.5e-7,       // Reaction torque coefficient
        1200.0,       // Maximum rotor speed
        0.141421356,  // Rotor offset
        {1.0, -1.0, 1.0, -1.0}
    };
}

void test_exact_collective_thrust() {
    RotorDynamics dynamics(test_parameters());

    constexpr double speed = 500.0;
    constexpr double thrust_per_rotor = 8.0e-6 * speed * speed;
    constexpr double total_thrust = 4.0 * thrust_per_rotor;

    const RotorWrench wrench =
        dynamics.calculate({speed, speed, speed, speed});

    for (std::size_t i = 0; i < 4; ++i) {
        assert(near(wrench.individual_thrusts_n[i],
                    thrust_per_rotor));
    }

    assert(near(wrench.total_thrust_n, total_thrust));
    assert(near(wrench.force_body_n[0], 0.0));
    assert(near(wrench.force_body_n[1], 0.0));
    assert(near(wrench.force_body_n[2], -total_thrust));

    for (double torque : wrench.torque_body_nm) {
        assert(near(torque, 0.0));
    }
}

void test_exact_roll_moment() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench wrench =
        dynamics.calculate({600.0, 600.0, 500.0, 500.0});

    constexpr double d = 0.141421356;
    constexpr double kT = 8.0e-6;

    const double thrust_difference =
        kT * (600.0 * 600.0 - 500.0 * 500.0);

    const double expected_roll = 2.0 * d * thrust_difference;

    assert(near(wrench.torque_body_nm[0], expected_roll));
    assert(near(wrench.torque_body_nm[1], 0.0));
    assert(near(wrench.torque_body_nm[2], 0.0));
}

void test_exact_pitch_moment() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench wrench =
        dynamics.calculate({600.0, 500.0, 500.0, 600.0});

    constexpr double d = 0.141421356;
    constexpr double kT = 8.0e-6;

    const double thrust_difference =
        kT * (600.0 * 600.0 - 500.0 * 500.0);

    const double expected_pitch = 2.0 * d * thrust_difference;

    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(near(wrench.torque_body_nm[1], expected_pitch));
    assert(near(wrench.torque_body_nm[2], 0.0));
}

void test_exact_yaw_moment() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench wrench =
        dynamics.calculate({600.0, 500.0, 600.0, 500.0});

    constexpr double kQ = 1.5e-7;

    const double expected_yaw =
        2.0 * kQ * (600.0 * 600.0 - 500.0 * 500.0);

    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(near(wrench.torque_body_nm[1], 0.0));
    assert(near(wrench.torque_body_nm[2], expected_yaw));
}

void test_rotor_speed_saturation() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench at_limit =
        dynamics.calculate({1200.0, 1200.0, 1200.0, 1200.0});

    const RotorWrench above_limit =
        dynamics.calculate({1500.0, 1500.0, 1500.0, 1500.0});

    assert(near(at_limit.total_thrust_n,
                above_limit.total_thrust_n));

    for (std::size_t i = 0; i < 4; ++i) {
        assert(near(at_limit.individual_thrusts_n[i],
                    above_limit.individual_thrusts_n[i]));
    }
}

}  // namespace

int main() {
    test_exact_collective_thrust();
    test_exact_roll_moment();
    test_exact_pitch_moment();
    test_exact_yaw_moment();
    test_rotor_speed_saturation();

    std::cout << "Rotor force-and-moment numerical validation passed.\n";
    return 0;
}
