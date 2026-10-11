#include "quadcopter/rk4_integrator.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

using namespace quadcopter;

constexpr double kTolerance = 1e-8;

void require_near(
    double actual,
    double expected,
    double tolerance,
    const char* message
) {
    if (std::abs(actual - expected) > tolerance) {
        std::cerr << message
                  << ": expected " << expected
                  << ", got " << actual << '\n';
        throw std::runtime_error(message);
    }
}

struct TestSystem {
    RigidBodyDynamics rigid_body{
        RigidBodyParameters{
            1.2, 9.80665, {0.015, 0.015, 0.025}
        }
    };

    RotorDynamics rotors{
        RotorParameters{
            8.0e-6,
            1.5e-7,
            1200.0,
            0.141421356,
            {1.0, -1.0, 1.0, -1.0}
        }
    };

    MotorDynamics motors{MotorParameters{0.04, 1200.0}};

    QuadcopterDynamics dynamics{rigid_body, rotors, motors};
    Rk4Integrator integrator{dynamics};
};

QuadcopterState make_state(
    Vec3 position,
    Vec3 velocity,
    Quaternion quaternion,
    Vec3 angular_velocity,
    RotorSpeeds rotor_speeds
) {
    return {
        RigidBodyState{
            position, velocity, quaternion, angular_velocity
        },
        rotor_speeds
    };
}

void test_free_fall() {
    TestSystem system;

    QuadcopterState state = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    );

    const RotorSpeeds commands{0.0, 0.0, 0.0, 0.0};

    constexpr double dt = 0.01;
    constexpr int steps = 100;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, commands, dt);
    }

    constexpr double elapsed_time = dt * steps;
    constexpr double gravity = 9.80665;

    const double expected_velocity = gravity * elapsed_time;
    const double expected_position =
        0.5 * gravity * elapsed_time * elapsed_time;

    require_near(
        state.rigid_body.velocity_body_m_s[2],
        expected_velocity,
        kTolerance,
        "Free-fall down velocity"
    );

    require_near(
        state.rigid_body.position_ned_m[2],
        expected_position,
        kTolerance,
        "Free-fall down position"
    );
}

void test_constant_yaw_rate() {
    TestSystem system;

    constexpr double yaw_rate = 0.5;

    const RotorSpeeds constant_speeds{
        500.0, 500.0, 500.0, 500.0
    };

    QuadcopterState state = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, yaw_rate},
        constant_speeds
    );

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, constant_speeds, dt);
    }

    const double total_angle = yaw_rate * dt * steps;

    require_near(
        state.rigid_body.quaternion_nb[0],
        std::cos(total_angle / 2.0),
        1e-8,
        "Yaw quaternion scalar"
    );

    require_near(
        state.rigid_body.quaternion_nb[3],
        std::sin(total_angle / 2.0),
        1e-8,
        "Yaw quaternion z component"
    );

    double norm_squared = 0.0;

    for (double component : state.rigid_body.quaternion_nb) {
        norm_squared += component * component;
    }

    require_near(
        std::sqrt(norm_squared),
        1.0,
        1e-12,
        "Quaternion norm"
    );
}

void test_motor_speed_integration() {
    TestSystem system;

    QuadcopterState state = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0, 0.0}
    );

    const RotorSpeeds commands{400.0, 400.0, 400.0, 400.0};

    constexpr double tau = 0.04;
    constexpr double duration = 0.1;
    constexpr double dt = 0.001;
    constexpr int steps = 100;

    for (int i = 0; i < steps; ++i) {
        state = system.integrator.step(state, commands, dt);
    }

    const double expected_speed =
        400.0 * (1.0 - std::exp(-duration / tau));

    for (double speed : state.rotor_speeds_rad_s) {
        require_near(
            speed,
            expected_speed,
            1e-6,
            "Motor first-order response"
        );
    }
}

void test_hover_equilibrium() {
    TestSystem system;

    constexpr double mass = 1.2;
    constexpr double gravity = 9.80665;
    constexpr double thrust_coefficient = 8.0e-6;

    const double hover_speed = std::sqrt(
        (mass * gravity) / (4.0 * thrust_coefficient)
    );

    const RotorSpeeds hover_speeds{
        hover_speed, hover_speed, hover_speed, hover_speed
    };

    QuadcopterState state = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        hover_speeds
    );

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    double max_position_drift = 0.0;
    double max_velocity_drift = 0.0;
    double max_angular_rate_drift = 0.0;
    double max_rotor_speed_drift = 0.0;
    double max_quaternion_drift = 0.0;

    for (int step = 0; step < steps; ++step) {
        state = system.integrator.step(state, hover_speeds, dt);

        for (std::size_t i = 0; i < 3; ++i) {
            max_position_drift = std::max(
                max_position_drift,
                std::abs(state.rigid_body.position_ned_m[i])
            );

            max_velocity_drift = std::max(
                max_velocity_drift,
                std::abs(state.rigid_body.velocity_body_m_s[i])
            );

            max_angular_rate_drift = std::max(
                max_angular_rate_drift,
                std::abs(
                    state.rigid_body.angular_velocity_body_rad_s[i]
                )
            );
        }

        for (std::size_t i = 0; i < 4; ++i) {
            max_rotor_speed_drift = std::max(
                max_rotor_speed_drift,
                std::abs(state.rotor_speeds_rad_s[i] - hover_speed)
            );
        }

        const Quaternion& q = state.rigid_body.quaternion_nb;

        max_quaternion_drift = std::max(
            max_quaternion_drift,
            std::max(
                std::abs(q[0] - 1.0),
                std::max(
                    std::abs(q[1]),
                    std::max(std::abs(q[2]), std::abs(q[3]))
                )
            )
        );
    }

    require_near(
        max_position_drift, 0.0, 1e-8,
        "Maximum hover position drift"
    );

    require_near(
        max_velocity_drift, 0.0, 1e-8,
        "Maximum hover velocity drift"
    );

    require_near(
        max_angular_rate_drift, 0.0, 1e-8,
        "Maximum hover angular-rate drift"
    );

    require_near(
        max_rotor_speed_drift, 0.0, 1e-8,
        "Maximum hover rotor-speed drift"
    );

    require_near(
        max_quaternion_drift, 0.0, 1e-8,
        "Maximum hover quaternion drift"
    );

    std::cout << "Hover speed: " << hover_speed
              << " rad/s per rotor\n";
    std::cout << "Maximum position drift: "
              << max_position_drift << " m\n";
    std::cout << "Maximum velocity drift: "
              << max_velocity_drift << " m/s\n";
    std::cout << "Maximum angular-rate drift: "
              << max_angular_rate_drift << " rad/s\n";
    std::cout << "Maximum rotor-speed drift: "
              << max_rotor_speed_drift << " rad/s\n";
    std::cout << "Maximum quaternion component drift: "
              << max_quaternion_drift << '\n';
}

void test_known_angular_accelerations() {
    TestSystem system;

    // Rotor 1 is faster than the other three rotors.
    // Commands equal actual speeds so motor lag does not affect
    // the instantaneous rigid-body angular acceleration.
    const RotorSpeeds rotor_speeds{
        600.0, 500.0, 500.0, 500.0
    };

    const QuadcopterState state = make_state(
        {0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0, 0.0},
        {0.0, 0.0, 0.0},
        rotor_speeds
    );

    const auto derivatives =
        system.dynamics.derivatives(state, rotor_speeds);

    const auto& angular_acceleration =
        derivatives.rigid_body.angular_velocity_dot_body_rad_s2;

    constexpr double expected_roll_acceleration = 8.296719552;
    constexpr double expected_pitch_acceleration = 8.296719552;
    constexpr double expected_yaw_acceleration = 0.66;

    require_near(
        angular_acceleration[0],
        expected_roll_acceleration,
        kTolerance,
        "Known roll angular acceleration"
    );

    require_near(
        angular_acceleration[1],
        expected_pitch_acceleration,
        kTolerance,
        "Known pitch angular acceleration"
    );

    require_near(
        angular_acceleration[2],
        expected_yaw_acceleration,
        kTolerance,
        "Known yaw angular acceleration"
    );

    std::cout << "Known angular accelerations:\n";
    std::cout << "  Roll: " << angular_acceleration[0]
              << " rad/s^2\n";
    std::cout << "  Pitch: " << angular_acceleration[1]
              << " rad/s^2\n";
    std::cout << "  Yaw: " << angular_acceleration[2]
              << " rad/s^2\n";
}

void test_asymmetric_rotor_command_response() {
    constexpr double mass = 1.2;
    constexpr double gravity = 9.80665;
    constexpr double thrust_coefficient = 8.0e-6;

    const double hover_speed = std::sqrt(
        (mass * gravity) / (4.0 * thrust_coefficient)
    );

    const RotorSpeeds hover_speeds{
        hover_speed, hover_speed, hover_speed, hover_speed
    };

    // Expected angular-rate signs [p, q, r] for each rotor.
    // Rotor layout and spin directions are defined in RotorDynamics.
    const int expected_signs[4][3] = {
        { 1,  1,  1},  // Rotor 1: front-right
        { 1, -1, -1},  // Rotor 2: rear-right
        {-1, -1,  1},  // Rotor 3: rear-left
        {-1,  1, -1}   // Rotor 4: front-left
    };

    constexpr double dt = 0.001;
    constexpr int steps = 100;

    for (std::size_t rotor = 0; rotor < 4; ++rotor) {
        TestSystem system;

        const QuadcopterState initial = make_state(
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            hover_speeds
        );

        RotorSpeeds commands = hover_speeds;
        commands[rotor] = hover_speed + 50.0;

        QuadcopterState state = initial;

        for (int step = 0; step < steps; ++step) {
            state = system.integrator.step(state, commands, dt);
        }

        const auto& rates =
            state.rigid_body.angular_velocity_body_rad_s;

        for (std::size_t axis = 0; axis < 3; ++axis) {
            const int expected = expected_signs[rotor][axis];

            if (expected * rates[axis] <= 0.0) {
                std::cerr
                    << "Rotor " << (rotor + 1)
                    << ", axis " << axis
                    << ": expected sign " << expected
                    << ", got " << rates[axis] << " rad/s\n";

                throw std::runtime_error(
                    "Unexpected full-vehicle angular response"
                );
            }
        }

        std::cout << "Rotor " << (rotor + 1)
                  << " response [p, q, r]: "
                  << rates[0] << ", "
                  << rates[1] << ", "
                  << rates[2] << " rad/s\n";
    }
}

}  // namespace

int main() {
    try {
        test_free_fall();
        test_constant_yaw_rate();
        test_motor_speed_integration();
        test_hover_equilibrium();
        test_known_angular_accelerations();
        test_asymmetric_rotor_command_response();

        std::cout << "All 17-state RK4 integrator tests passed.\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "RK4 test failure: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
