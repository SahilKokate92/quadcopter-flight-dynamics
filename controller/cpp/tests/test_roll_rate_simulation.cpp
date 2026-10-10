
#include "quadcopter/roll_rate_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    constexpr double dt = 0.01;
    constexpr double simulation_time = 3.0;
    constexpr double inertia_x = 0.002;
    constexpr double desired_rate = 1.0;
    constexpr double maximum_torque = 0.002;
    constexpr double settling_tolerance = 0.02;

    const quadcopter::RollRateController controller(
        0.006,
        maximum_torque
    );

    std::vector<double> rates;
    double roll_rate = 0.0;

    const int steps =
        static_cast<int>(simulation_time / dt);

    for (int i = 0; i <= steps; ++i) {
        rates.push_back(roll_rate);

        if (i < steps) {
            const double torque =
                controller.update(desired_rate, roll_rate);

            assert(std::abs(torque) <= maximum_torque + 1e-12);

            const double acceleration = torque / inertia_x;
            roll_rate += acceleration * dt;
        }
    }

    const double final_error =
        std::abs(desired_rate - rates.back());

    assert(final_error <= settling_tolerance);

    double settling_time = -1.0;

    for (std::size_t i = 0; i < rates.size(); ++i) {
        bool remains_within_tolerance = true;

        for (std::size_t j = i; j < rates.size(); ++j) {
            if (std::abs(desired_rate - rates[j]) >
                settling_tolerance) {
                remains_within_tolerance = false;
                break;
            }
        }

        if (remains_within_tolerance) {
            settling_time = i * dt;
            break;
        }
    }

    assert(settling_time >= 0.0);
    assert(settling_time <= simulation_time);

    std::cout << "Roll-rate simulation tests passed.\n";
    std::cout << "Final tracking error: "
              << final_error << " rad/s\n";
    std::cout << "Settling time: "
              << settling_time << " s\n";

    return 0;
}
