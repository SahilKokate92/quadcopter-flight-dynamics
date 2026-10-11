#ifndef QUADCOPTER_RK4_INTEGRATOR_HPP
#define QUADCOPTER_RK4_INTEGRATOR_HPP

#include "quadcopter/quadcopter_dynamics.hpp"
#include "quadcopter/quadcopter_state.hpp"

namespace quadcopter {

// Advances the complete 17-state quadcopter model by one RK4 step.
class Rk4Integrator {
public:
    explicit Rk4Integrator(const QuadcopterDynamics& dynamics);

    QuadcopterState step(
        const QuadcopterState& state,
        const RotorSpeeds& commanded_speeds_rad_s,
        double time_step_s
    ) const;

private:
    const QuadcopterDynamics& dynamics_;
};

}  // namespace quadcopter

#endif  // QUADCOPTER_RK4_INTEGRATOR_HPP