
#include "quadcopter/roll_rate_controller.hpp"

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

struct SimulationSample {
    double time;
    double desired_rate;
    double measured_rate;
    double torque;
};

int main() {
    constexpr double dt = 0.01;
    constexpr double simulation_time = 3.0;
    constexpr double inertia_x = 0.002;
    constexpr double desired_roll_rate = 1.0;
    constexpr double settling_tolerance = 0.02; // rad/s

    const quadcopter::RollRateController controller(
        0.006,
        0.002
    );

    double roll_rate = 0.0;
    std::vector<SimulationSample> samples;

    const int steps =
        static_cast<int>(simulation_time / dt);

    for (int i = 0; i <= steps; ++i) {
        const double time = i * dt;

        const double torque = controller.update(
            desired_roll_rate,
            roll_rate
        );

        samples.push_back({
            time,
            desired_roll_rate,
            roll_rate,
            torque
        });

        // Do not integrate beyond the final sample.
        if (i < steps) {
            const double roll_acceleration =
                torque / inertia_x;

            // Forward Euler integration.
            roll_rate += roll_acceleration * dt;
        }
    }

    // Save the simulation history.
    std::ofstream csv("roll_rate_response.csv");

    if (!csv.is_open()) {
        std::cerr << "Error: Could not create CSV file.\n";
        return 1;
    }

    csv << "time_s,desired_rate_rad_s,"
           "measured_rate_rad_s,torque_Nm,error_rad_s\n";

    csv << std::fixed << std::setprecision(6);

    for (const auto& sample : samples) {
        const double error =
            sample.desired_rate - sample.measured_rate;

        csv << sample.time << ','
            << sample.desired_rate << ','
            << sample.measured_rate << ','
            << sample.torque << ','
            << error << '\n';
    }

    csv.close();

    if (!csv) {
        std::cerr << "Error: Failed while writing CSV file.\n";
        return 1;
    }

    // Calculate final tracking error.
    const auto& final_sample = samples.back();

    const double final_error = std::abs(
        final_sample.desired_rate -
        final_sample.measured_rate
    );

    // Find the earliest sample after which the error
    // remains within the selected tolerance.
    double settling_time = -1.0;

    for (std::size_t i = 0; i < samples.size(); ++i) {
        bool remains_within_tolerance = true;

        for (std::size_t j = i; j < samples.size(); ++j) {
            const double error = std::abs(
                samples[j].desired_rate -
                samples[j].measured_rate
            );

            if (error > settling_tolerance) {
                remains_within_tolerance = false;
                break;
            }
        }

        if (remains_within_tolerance) {
            settling_time = samples[i].time;
            break;
        }
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "\nRoll-rate simulation summary\n";
    std::cout << "----------------------------\n";
    std::cout << "Simulation duration: "
              << simulation_time << " s\n";
    std::cout << "Desired roll rate: "
              << desired_roll_rate << " rad/s\n";
    std::cout << "Final measured rate: "
              << final_sample.measured_rate << " rad/s\n";
    std::cout << "Final absolute error: "
              << final_error << " rad/s\n";
    std::cout << "Settling tolerance: +/-"
              << settling_tolerance << " rad/s\n";

    if (settling_time >= 0.0) {
        std::cout << "Settling time: "
                  << settling_time << " s\n";
    } else {
        std::cout << "Settling time: Not reached "
                     "during simulation\n";
    }

    std::cout << "CSV file: roll_rate_response.csv\n";

    return 0;
}
