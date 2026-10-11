#include "quadcopter/motor_dynamics.hpp"

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

MotorDynamics make_motor_dynamics() {
    return MotorDynamics({
        0.04,    // Motor time constant (s)
        1200.0   // Maximum rotor speed (rad/s)
    });
}

void test_zero_error() {
    const auto motors = make_motor_dynamics();

    const RotorSpeeds result = motors.derivatives(
        {500.0, 500.0, 500.0, 500.0},
        {500.0, 500.0, 500.0, 500.0}
    );

    for (double derivative : result) {
        assert(near(derivative, 0.0));
    }
}

void test_motor_acceleration() {
    const auto motors = make_motor_dynamics();

    const RotorSpeeds result = motors.derivatives(
        {0.0, 0.0, 0.0, 0.0},
        {400.0, 400.0, 400.0, 400.0}
    );

    for (double derivative : result) {
        assert(near(derivative, 10000.0));
    }
}

void test_motor_deceleration() {
    const auto motors = make_motor_dynamics();

    const RotorSpeeds result = motors.derivatives(
        {800.0, 800.0, 800.0, 800.0},
        {400.0, 400.0, 400.0, 400.0}
    );

    for (double derivative : result) {
        assert(near(derivative, -10000.0));
    }
}

void test_independent_motor_commands() {
    const auto motors = make_motor_dynamics();

    const RotorSpeeds result = motors.derivatives(
        {100.0, 200.0, 300.0, 400.0},
        {200.0, 200.0, 400.0, 200.0}
    );

    assert(near(result[0], 2500.0));
    assert(near(result[1], 0.0));
    assert(near(result[2], 2500.0));
    assert(near(result[3], -5000.0));
}

void test_invalid_speed_rejected() {
    const auto motors = make_motor_dynamics();

    bool rejected = false;

    try {
        (void)motors.derivatives(
            {-1.0, 0.0, 0.0, 0.0},
            {100.0, 100.0, 100.0, 100.0}
        );
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);
}

void test_speed_above_limit_rejected() {
    const auto motors = make_motor_dynamics();

    bool rejected = false;

    try {
        (void)motors.derivatives(
            {1201.0, 0.0, 0.0, 0.0},
            {100.0, 100.0, 100.0, 100.0}
        );
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);
}

}  // namespace

int main() {
    test_zero_error();
    test_motor_acceleration();
    test_motor_deceleration();
    test_independent_motor_commands();
    test_invalid_speed_rejected();
    test_speed_above_limit_rejected();

    std::cout << "All motor dynamics tests passed.\n";
    return 0;
}
