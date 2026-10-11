#include "quadcopter/rigid_body_dynamics.hpp"
#include "quadcopter/rk4_integrator.hpp"

#include <cstdlib>
#include <iomanip>
#include <iostream>

int main() {
    using namespace quadcopter;

    const RigidBodyParameters parameters{
        1.2,
        9.80665,
        {0.015, 0.015, 0.025}
    };

    const RigidBodyDynamics dynamics(parameters);
    const Rk4Integrator integrator(dynamics);

    RigidBodyState state{
        {0.0, 0.0, 0.0},
        {8.0, 1.0, 0.5},
        {1.0, 0.0, 0.0, 0.0},
        {0.1, -0.2, 0.3}
    };

    const BodyWrench wrench{
        {1.5, -0.8, -10.0},
        {0.02, -0.015, 0.01}
    };

    constexpr double dt = 0.001;
    constexpr int steps = 1000;

    std::cout << std::setprecision(17);

    // CSV columns: time, position, body velocity, quaternion,
    // and body angular velocity.
    std::cout
        << "time_s,"
        << "pn,pe,pd,"
        << "ub,vb,wb,"
        << "qw,qx,qy,qz,"
        << "p,q,r\n";

    auto print_state = [](double time, const RigidBodyState& s) {
        std::cout << time;

        for (double value : s.position_ned_m) {
            std::cout << ',' << value;
        }
        for (double value : s.velocity_body_m_s) {
            std::cout << ',' << value;
        }
        for (double value : s.quaternion_nb) {
            std::cout << ',' << value;
        }
        for (double value : s.angular_velocity_body_rad_s) {
            std::cout << ',' << value;
        }

        std::cout << '\n';
    };

    print_state(0.0, state);

    for (int i = 0; i < steps; ++i) {
        state = integrator.step(state, wrench, dt);
        print_state((i + 1) * dt, state);
    }

    return EXIT_SUCCESS;
}
