#include "quadcopter/motor_dynamics.hpp"
#include "quadcopter/quadcopter_dynamics.hpp"
#include "quadcopter/quadcopter_state.hpp"
#include "quadcopter/rigid_body_dynamics.hpp"
#include "quadcopter/rk4_integrator.hpp"
#include "quadcopter/rotor_dynamics.hpp"

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>

int main() {
    using namespace quadcopter;

    constexpr double duration_s = 15.0;
    constexpr double dt_s = 0.001;
    constexpr int steps = 15000;

    const RigidBodyParameters rigid_body_parameters{
        1.2,
        9.80665,
        {0.015, 0.015, 0.025}
    };

    const RotorParameters rotor_parameters{
        8.0e-6,
        1.5e-7,
        1200.0,
        0.141421356,
        {1.0, -1.0, 1.0, -1.0}
    };

    const MotorParameters motor_parameters{
        0.04,
        1200.0
    };

    const RigidBodyDynamics rigid_body_dynamics(
        rigid_body_parameters
    );
    const RotorDynamics rotor_dynamics(rotor_parameters);
    const MotorDynamics motor_dynamics(motor_parameters);

    const QuadcopterDynamics dynamics(
        rigid_body_dynamics,
        rotor_dynamics,
        motor_dynamics
    );

    const Rk4Integrator integrator(dynamics);

    const double hover_speed_rad_s = std::sqrt(
        rigid_body_parameters.mass_kg *
        rigid_body_parameters.gravity_m_s2 /
        (4.0 *
         rotor_parameters.thrust_coefficient_n_per_rad_s_squared)
    );

    QuadcopterState state{
        {
            {0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0, 0.0},
            {0.0, 0.0, 0.0}
        },
        {
            hover_speed_rad_s,
            hover_speed_rad_s,
            hover_speed_rad_s,
            hover_speed_rad_s
        }
    };

    // Asymmetric rotor-speed commands around hover.
    const RotorSpeeds commanded_speeds_rad_s{
        hover_speed_rad_s + 30.0,
        hover_speed_rad_s - 20.0,
        hover_speed_rad_s + 10.0,
        hover_speed_rad_s - 10.0
    };

    const std::filesystem::path output_path =
        "controller/cpp/examples/open_loop_asymmetric_state.csv";

    std::ofstream csv(output_path);

    if (!csv) {
        std::cerr
            << "Failed to open CSV output: "
            << output_path.string() << '\n';
        std::cerr
            << "Run this executable from the repository root.\n";
        return EXIT_FAILURE;
    }

    csv << std::setprecision(17);

    csv
        << "time_s,"
        << "pn,pe,pd,"
        << "ub,vb,wb,"
        << "qw,qx,qy,qz,"
        << "p,q,r,"
        << "rotor1_rad_s,rotor2_rad_s,"
        << "rotor3_rad_s,rotor4_rad_s\n";

    const auto write_state =
        [&csv](double time_s, const QuadcopterState& s) {
            csv << time_s;

            for (double value : s.rigid_body.position_ned_m) {
                csv << ',' << value;
            }

            for (double value : s.rigid_body.velocity_body_m_s) {
                csv << ',' << value;
            }

            for (double value : s.rigid_body.quaternion_nb) {
                csv << ',' << value;
            }

            for (
                double value :
                s.rigid_body.angular_velocity_body_rad_s
            ) {
                csv << ',' << value;
            }

            for (double value : s.rotor_speeds_rad_s) {
                csv << ',' << value;
            }

            csv << '\n';
        };

    write_state(0.0, state);

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(
            state,
            commanded_speeds_rad_s,
            dt_s
        );

        write_state((i + 1) * dt_s, state);
    }

    csv.close();

    if (!csv) {
        std::cerr << "Error writing CSV output.\n";
        return EXIT_FAILURE;
    }

    std::cout << "Open-loop simulation completed.\n";
    std::cout << "Duration: " << duration_s << " s\n";
    std::cout << "Time step: " << dt_s << " s\n";
    std::cout << "Integration steps: " << steps << '\n';
    std::cout << "Hover speed: "
              << hover_speed_rad_s << " rad/s\n";
    std::cout << "CSV saved to: "
              << output_path.string() << '\n';

    return EXIT_SUCCESS;
}