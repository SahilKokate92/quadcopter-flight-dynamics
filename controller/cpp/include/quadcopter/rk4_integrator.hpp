#ifndef QUADCOPTER_RK4_INTEGRATOR_HPP
#define QUADCOPTER_RK4_INTEGRATOR_HPP

#include "quadcopter/rigid_body_dynamics.hpp"

namespace quadcopter {

// Advances the rigid-body state by one fixed RK4 time step.
class Rk4Integrator {
public:
    explicit Rk4Integrator(const RigidBodyDynamics& dynamics);

    RigidBodyState step(
        const RigidBodyState& state,
        const BodyWrench& wrench,
        double time_step_s
    ) const;

private:
    const RigidBodyDynamics& dynamics_;
};

}  // namespace quadcopter

#endif  // QUADCOPTER_RK4_INTEGRATOR_HPP
