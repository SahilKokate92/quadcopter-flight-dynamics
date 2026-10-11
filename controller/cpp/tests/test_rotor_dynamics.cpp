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

    const RotorWrench wrench =
        dynamics.calculate({0.0, 0.0, 0.0, 0.0});

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
    // Its thrust force acts along negative body z.
    const RotorWrench wrench =
        dynamics.calculate({500.0, 0.0, 0.0, 0.0});

    assert(wrench.torque_body_nm[0] > 0.0);
    assert(wrench.torque_body_nm[1] > 0.0);
}

void test_yaw_reaction_torque() {
    const RotorParameters p = test_parameters();
    RotorDynamics dynamics(p);

    constexpr double speed = 500.0;

    // Equal rotor speeds cancel yaw reaction torque.
    const RotorWrench balanced =
        dynamics.calculate({speed, speed, speed, speed});

    assert(near(balanced.torque_body_nm[2], 0.0));

    // Rotor 1 alone produces positive yaw reaction torque.
    const RotorWrench single =
        dynamics.calculate({speed, 0.0, 0.0, 0.0});

    const double expected =
        p.torque_coefficient_nm_per_rad_s_squared
        * speed * speed;

    assert(near(single.torque_body_nm[2], expected));
}

void test_asymmetric_rotor_torques() {
    const RotorParameters p = test_parameters();
    RotorDynamics dynamics(p);

    // Rotor 1 spins faster than the other three rotors.
    const RotorSpeeds speeds{600.0, 500.0, 500.0, 500.0};
    const RotorWrench wrench = dynamics.calculate(speeds);

    const double d = p.rotor_offset_m;
    const double kT = p.thrust_coefficient_n_per_rad_s_squared;
    const double kQ = p.torque_coefficient_nm_per_rad_s_squared;

    const double w1_sq = speeds[0] * speeds[0];
    const double w2_sq = speeds[1] * speeds[1];
    const double w3_sq = speeds[2] * speeds[2];
    const double w4_sq = speeds[3] * speeds[3];

    // Moments calculated from rotor position cross thrust force.
    const double expected_roll =
        d * kT * (w1_sq + w2_sq - w3_sq - w4_sq);

    const double expected_pitch =
        d * kT * (w1_sq - w2_sq - w3_sq + w4_sq);

    // Rotor reaction torques use spin signs (+1, -1, +1, -1).
    const double expected_yaw =
        kQ * (w1_sq - w2_sq + w3_sq - w4_sq);

    assert(near(wrench.torque_body_nm[0], expected_roll));
    assert(near(wrench.torque_body_nm[1], expected_pitch));
    assert(near(wrench.torque_body_nm[2], expected_yaw));
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
    test_asymmetric_rotor_torques();
    test_speed_clipping();
    test_negative_speed_rejected();

    std::cout << "All rotor dynamics tests passed.\n";
    return 0;
}