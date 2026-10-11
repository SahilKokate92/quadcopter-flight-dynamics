#include "quadcopter/rigid_body_dynamics.hpp"

#include <array>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <stdexcept>

namespace {

double read_value(char* argument) {
    char* end = nullptr;
    const double value = std::strtod(argument, &end);

    if (end == argument || *end != '\0') {
        throw std::invalid_argument("Invalid numeric argument.");
    }

    return value;
}

}  // namespace

int main(int argc, char* argv[]) {
    // 19 values:
    // position(3), velocity(3), quaternion(4),
    // angular velocity(3), force(3), torque(3).
    if (argc != 20) {
        std::cerr
            << "Expected 19 numeric arguments: position, velocity, "
            << "quaternion, angular velocity, force, torque.\n";
        return 1;
    }

    try {
        std::array<double, 19> input{};

        for (std::size_t i = 0; i < input.size(); ++i) {
            input[i] = read_value(argv[i + 1]);
        }

        const quadcopter::RigidBodyParameters parameters{
            1.2,
            9.80665,
            {0.015, 0.015, 0.025}
        };

        const quadcopter::RigidBodyState state{
            {input[0], input[1], input[2]},
            {input[3], input[4], input[5]},
            {input[6], input[7], input[8], input[9]},
            {input[10], input[11], input[12]}
        };

        const quadcopter::BodyWrench wrench{
            {input[13], input[14], input[15]},
            {input[16], input[17], input[18]}
        };

        const quadcopter::RigidBodyDynamics dynamics(parameters);
        const auto result = dynamics.derivatives(state, wrench);

        std::cout << std::setprecision(17);

        bool first = true;

        const auto print_group = [&first](const auto& values) {
            for (double value : values) {
                if (!first) {
                    std::cout << ',';
                }
                std::cout << value;
                first = false;
            }
        };

        print_group(result.position_dot_ned_m_s);
        print_group(result.velocity_dot_body_m_s2);
        print_group(result.quaternion_dot_nb);
        print_group(result.angular_velocity_dot_body_rad_s2);

        std::cout << '\n';
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Comparison executable error: "
                  << error.what() << '\n';
        return 2;
    }
}