#include "quadcopter/rigid_body_dynamics.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

constexpr double kTolerance = 1e-10;

void require_near(
    double actual,
    double expected,
    const char* label
) {
    if (std::abs(actual - expected) > kTolerance) {
        std::cerr << label << ": expected " << expected
                  << ", got " << actual << '\n';
        throw std::runtime_error(label);
    }
}

void validate_case(
    const Vec3& rates,
    const Vec3& torque,
    const char* label
) {
    const RigidBodyParameters parameters{
        1.2, 9.80665, {0.015, 0.015, 0.025}
    };

    const RigidBodyDynamics dynamics(parameters);

    const RigidBodyState state{
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        rates
    };

    const BodyWrench wrench{
        {0.0, 0.0, 0.0},
        torque
    };

    const auto result = dynamics.derivatives(state, wrench);
    const auto& actual =
        result.angular_velocity_dot_body_rad_s2;

    const double Ix = parameters.inertia_diagonal_kg_m2[0];
    const double Iy = parameters.inertia_diagonal_kg_m2[1];
    const double Iz = parameters.inertia_diagonal_kg_m2[2];

    const double p = rates[0];
    const double q = rates[1];
    const double r = rates[2];

    const Vec3 expected{
        (torque[0] + (Iy - Iz) * q * r) / Ix,
        (torque[1] + (Iz - Ix) * r * p) / Iy,
        (torque[2] + (Ix - Iy) * p * q) / Iz
    };

    for (int axis = 0; axis < 3; ++axis) {
        require_near(actual[axis], expected[axis], label);
    }
}

void test_general_rotational_coupling() {
    // All three rates and torques are nonzero.
    validate_case(
        {1.2, -0.7, 2.1},
        {0.08, -0.04, 0.03},
        "General rotational-coupling case"
    );
}

void test_zero_torque_coupling() {
    // Torque-free rotation: angular acceleration comes from
    // gyroscopic coupling alone.
    validate_case(
        {1.2, -0.7, 2.1},
        {0.0, 0.0, 0.0},
        "Torque-free rotational-coupling case"
    );
}

void test_pure_axis_torques_with_nonzero_rates() {
    const Vec3 rates{0.8, -1.1, 1.6};

    validate_case(rates, {0.05, 0.0, 0.0}, "Pure roll torque");
    validate_case(rates, {0.0, 0.05, 0.0}, "Pure pitch torque");
    validate_case(rates, {0.0, 0.0, 0.05}, "Pure yaw torque");
}

}  // namespace

int main() {
    try {
        test_general_rotational_coupling();
        test_zero_torque_coupling();
        test_pure_axis_torques_with_nonzero_rates();

        std::cout
            << "All analytical rotational-dynamics validation tests passed.\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Validation failed: " << error.what() << '\n';
        return 1;
    }
}
