#include "quadcopter/rotor_dynamics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

using namespace quadcopter;

namespace {

constexpr double TOL = 1e-10;

bool near(double actual, double expected) {
    return std::abs(actual - expected) <=
           TOL * (1.0 + std::abs(expected));
}

RotorParameters test_parameters() {
    return {
        8.0e-6,              // N / (rad/s)^2
        1.5e-7,              // N*m / (rad/s)^2
        1200.0,              // rad/s
        0.141421356,         // m
        {1.0, -1.0, 1.0, -1.0}
    };
}

void test_zero_speed() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench wrench = dynamics.calculate({0.0, 0.0, 0.0, 0.0});

    assert(near(wrench.total_thrust_n, 0.0));
    assert(near(wrench.force_body_n[2], 0.0));

    for (double torque : wrench.torque_body_nm) {
        assert(near(torque, 0.0));
    }
}

void test_equal_speed_thrust() {
    const RotorParameters p = test_parameters();
    RotorDynamics dynamics(p);

    constexpr double speed = 500.0;
    const RotorWrench wrench =
        dynamics.calculate({speed, speed, speed, speed});

    const double expected_thrust =
        4.0 * p.thrust_coefficient_n_per_rad_s_squared
        * speed * speed;

    assert(near(wrench.total_thrust_n, expected_thrust));
    assert(near(wrench.force_body_n[0], 0.0));
    assert(near(wrench.force_body_n[1], 0.0));
    assert(near(wrench.force_body_n[2], -expected_thrust));

    // Symmetric thrust cancels roll and pitch moments.
    assert(near(wrench.torque_body_nm[0], 0.0));
    assert(near(wrench.torque_body_nm[1], 0.0));
}

void test_roll_pitch_torque_signs() {
    RotorDynamics dynamics(test_parameters());

    // Rotor 1 is at (+d, -d, 0).
    // Its upward thrust force is along negative body z.
    const RotorWrench wrench =
        dynamics.calculate({500.0, 0.0, 0.0, 0.0});

    assert(wrench.torque_body_nm[0] > 0.0);
    assert(wrench.torque_body_nm[1] > 0.0);
}

void test_yaw_reaction_torque() {
    const RotorParameters p = test_parameters();
    RotorDynamics dynamics(p);

    const double speed = 500.0;

    // With signs (+1, -1, +1, -1), equal speeds cancel yaw torque.
    const RotorWrench balanced =
        dynamics.calculate({speed, speed, speed, speed});

    assert(near(balanced.torque_body_nm[2], 0.0));

    // Rotor 1 alone produces positive reaction torque about body z.
    const RotorWrench single =
        dynamics.calculate({speed, 0.0, 0.0, 0.0});

    const double expected =
        p.torque_coefficient_nm_per_rad_s_squared * speed * speed;

    assert(near(single.torque_body_nm[2], expected));
}

void test_speed_clipping() {
    RotorDynamics dynamics(test_parameters());

    const RotorWrench at_limit =
        dynamics.calculate({1200.0, 0.0, 0.0, 0.0});

    const RotorWrench above_limit =
        dynamics.calculate({1500.0, 0.0, 0.0, 0.0});

    assert(near(
        at_limit.total_thrust_n,
        above_limit.total_thrust_n
    ));
}

void test_negative_speed_rejected() {
    RotorDynamics dynamics(test_parameters());

    bool rejected = false;

    try {
        (void)dynamics.calculate({-1.0, 0.0, 0.0, 0.0});
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);
}

}  // namespace

int main() {
    test_zero_speed();
    test_equal_speed_thrust();
    test_roll_pitch_torque_signs();
    test_yaw_reaction_torque();
    test_speed_clipping();
    test_negative_speed_rejected();

    std::cout << "All rotor dynamics tests passed.\n";
    return 0;
}
