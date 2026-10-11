
#include "quadcopter/rigid_body_dynamics.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

constexpr double kTolerance = 1e-10;

bool near(double actual, double expected) {
    return std::abs(actual - expected) < kTolerance;
}

quadcopter::RigidBodyParameters default_parameters() {
    return {
        1.2,                 // Mass (kg)
        9.80665,             // Gravity (m/s^2)
        {0.015, 0.015, 0.025} // Principal moments (kg*m^2)
    };
}

quadcopter::RigidBodyState stationary_level_state() {
    return {
        {0.0, 0.0, 0.0},    // Position NED (m)
        {0.0, 0.0, 0.0},    // Velocity body (m/s)
        {1.0, 0.0, 0.0, 0.0}, // Identity quaternion [w,x,y,z]
        {0.0, 0.0, 0.0}     // Angular velocity body (rad/s)
    };
}

void test_hover_force_balance() {
    const quadcopter::RigidBodyDynamics dynamics(default_parameters());
    auto state = stationary_level_state();

    const quadcopter::BodyWrench wrench{
        {0.0, 0.0, -1.2 * 9.80665},
        {0.0, 0.0, 0.0}
    };

    const auto result = dynamics.derivatives(state, wrench);

    for (double value : result.position_dot_ned_m_s) {
        assert(near(value, 0.0));
    }

    for (double value : result.velocity_dot_body_m_s2) {
        assert(near(value, 0.0));
    }

    for (double value : result.quaternion_dot_nb) {
        assert(near(value, 0.0));
    }

    for (double value : result.angular_velocity_dot_body_rad_s2) {
        assert(near(value, 0.0));
    }
}

void test_free_fall_acceleration() {
    const quadcopter::RigidBodyDynamics dynamics(default_parameters());
    const auto state = stationary_level_state();

    const quadcopter::BodyWrench zero_wrench{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    const auto result = dynamics.derivatives(state, zero_wrench);

    assert(near(result.velocity_dot_body_m_s2[0], 0.0));
    assert(near(result.velocity_dot_body_m_s2[1], 0.0));
    assert(near(result.velocity_dot_body_m_s2[2], 9.80665));
}

void test_quaternion_rate_about_body_x() {
    const quadcopter::RigidBodyDynamics dynamics(default_parameters());
    auto state = stationary_level_state();

    state.angular_velocity_body_rad_s = {1.0, 0.0, 0.0};

    const quadcopter::BodyWrench wrench{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    const auto result = dynamics.derivatives(state, wrench);

    assert(near(result.quaternion_dot_nb[0], 0.0));
    assert(near(result.quaternion_dot_nb[1], 0.5));
    assert(near(result.quaternion_dot_nb[2], 0.0));
    assert(near(result.quaternion_dot_nb[3], 0.0));
}

void test_rotational_coupling() {
    const quadcopter::RigidBodyDynamics dynamics(default_parameters());
    auto state = stationary_level_state();

    // q = 2 rad/s, r = 3 rad/s, p = 0.
    state.angular_velocity_body_rad_s = {0.0, 2.0, 3.0};

    const quadcopter::BodyWrench wrench{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    const auto result = dynamics.derivatives(state, wrench);

    // alpha_x = (Iyy - Izz) * q * r / Ixx = -4 rad/s^2.
    assert(near(result.angular_velocity_dot_body_rad_s2[0], -4.0));
    assert(near(result.angular_velocity_dot_body_rad_s2[1], 0.0));
    assert(near(result.angular_velocity_dot_body_rad_s2[2], 0.0));
}

void test_invalid_mass_is_rejected() {
    auto parameters = default_parameters();
    parameters.mass_kg = 0.0;

    bool rejected = false;

    try {
        const quadcopter::RigidBodyDynamics dynamics(parameters);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);
}

void test_zero_quaternion_is_rejected() {
    const quadcopter::RigidBodyDynamics dynamics(default_parameters());
    auto state = stationary_level_state();
    state.quaternion_nb = {0.0, 0.0, 0.0, 0.0};

    const quadcopter::BodyWrench wrench{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0}
    };

    bool rejected = false;

    try {
        dynamics.derivatives(state, wrench);
    } catch (const std::invalid_argument&) {
        rejected = true;
    }

    assert(rejected);
}

}  // namespace

int main() {
    test_hover_force_balance();
    test_free_fall_acceleration();
    test_quaternion_rate_about_body_x();
    test_rotational_coupling();
    test_invalid_mass_is_rejected();
    test_zero_quaternion_is_rejected();

    std::cout << "All rigid-body dynamics tests passed.\n";
    return 0;
}
