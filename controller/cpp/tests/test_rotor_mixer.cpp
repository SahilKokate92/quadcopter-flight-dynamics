#include "quadcopter/rotor_dynamics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>

using namespace quadcopter;

namespace {

constexpr double TOL = 1e-9;

bool near(double actual, double expected) {
    return std::abs(actual - expected) <=
           TOL * (1.0 + std::abs(expected));
}

RotorParameters test_parameters() {
    return {
        8.0e-6,
        1.5e-7,
        1200.0,
        0.141421356,
        {1.0, -1.0, 1.0, -1.0}
    };
}

void test_collective_thrust() {
    RotorDynamics dynamics(test_parameters());

    constexpr double speed = 500.0;

    const RotorWrench wrench =
        dynamics.calculate({speed, speed, speed, speed});

    assert(wrench.total_thrust_n > 0.0);
    assert(near(wrench.force_body_n[0], 0.0));
    assert(near(wrench.force_body_n[1], 0.0));
    assert(near(
        wrench.force_body_n[2],
        -wrench.total_thrust_n
    ));

    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(near(wrench.torque_body_nm[1], 0.0));
    assert(near(wrench.torque_body_nm[2], 0.0));
}

void test_roll_combination() {
    RotorDynamics dynamics(test_parameters());

    // Increase rotors 1 and 2 together.
    const RotorWrench wrench =
        dynamics.calculate({600.0, 600.0, 500.0, 500.0});

    assert(wrench.torque_body_nm[0] > 0.0);
    assert(near(wrench.torque_body_nm[1], 0.0));
    assert(near(wrench.torque_body_nm[2], 0.0));
}

void test_pitch_combination() {
    RotorDynamics dynamics(test_parameters());

    // Increase rotors 1 and 4 together.
    const RotorWrench wrench =
        dynamics.calculate({600.0, 500.0, 500.0, 600.0});

    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(wrench.torque_body_nm[1] > 0.0);
    assert(near(wrench.torque_body_nm[2], 0.0));
}

void test_yaw_combination() {
    RotorDynamics dynamics(test_parameters());

    // Increase rotors 1 and 3 together.
    const RotorWrench wrench =
        dynamics.calculate({600.0, 500.0, 600.0, 500.0});

    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(near(wrench.torque_body_nm[1], 0.0));
    assert(wrench.torque_body_nm[2] > 0.0);
}

}  // namespace

int main() {
    test_collective_thrust();
    test_roll_combination();
    test_pitch_combination();
    test_yaw_combination();

    std::cout << "All rotor mixer validation tests passed.\n";
    return 0;
}
