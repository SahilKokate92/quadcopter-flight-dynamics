#include "quadcopter/motor_dynamics.hpp"
#include "quadcopter/quadcopter_dynamics.hpp"
#include "quadcopter/quadcopter_state.hpp"
#include "quadcopter/rigid_body_dynamics.hpp"
#include "quadcopter/rk4_integrator.hpp"
#include "quadcopter/rotor_dynamics.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>

int main() {
    using namespace quadcopter;

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

    const RigidBodyDynamics rigid_body_dynamics(rigid_body_parameters);
    const RotorDynamics rotor_dynamics(rotor_parameters);
    const MotorDynamics motor_dynamics(motor_parameters);

    const QuadcopterDynamics dynamics(
        rigid_body_dynamics,
        rotor_dynamics,
        motor_dynamics
    );

    const Rk4Integrator integrator(dynamics);

    QuadcopterState state{
        {
            {0.0, 0.0, 0.0},
            {8.0, 1.0, 0.5},
            {1.0, 0.0, 0.0, 0.0},
            {0.1, -0.2, 0.3}
        },
        {0.0, 0.0, 0.0, 0.0}
    };

    const RotorSpeeds commanded_speeds_rad_s{
        650.0, 600.0, 600.0, 600.0
    };

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    std::cout << std::setprecision(17);

    std::cout
        << "time_s,"
        << "pn,pe,pd,"
        << "ub,vb,wb,"
        << "qw,qx,qy,qz,"
        << "p,q,r,"
        << "rotor1_rad_s,rotor2_rad_s,"
        << "rotor3_rad_s,rotor4_rad_s\n";

    const auto print_state = [](double time, const QuadcopterState& s) {
        std::cout << time;

        for (double value : s.rigid_body.position_ned_m) {
            std::cout << ',' << value;
        }

        for (double value : s.rigid_body.velocity_body_m_s) {
            std::cout << ',' << value;
        }

        for (double value : s.rigid_body.quaternion_nb) {
            std::cout << ',' << value;
        }

        for (double value : s.rigid_body.angular_velocity_body_rad_s) {
            std::cout << ',' << value;
        }

        for (double value : s.rotor_speeds_rad_s) {
            std::cout << ',' << value;
        }

        std::cout << '\n';
    };

    print_state(0.0, state);

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(state, commanded_speeds_rad_s, dt);
        print_state((i + 1) * dt, state);
    }

    return EXIT_SUCCESS;
}