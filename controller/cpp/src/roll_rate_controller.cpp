
#include "quadcopter/roll_rate_controller.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadcopter {

RollRateController::RollRateController(
    double proportional_gain,
    double maximum_torque
)
    : proportional_gain_(proportional_gain),
      maximum_torque_(maximum_torque) {
    if (!std::isfinite(proportional_gain_) || proportional_gain_ < 0.0) {
        throw std::invalid_argument(
            "Proportional gain must be finite and non-negative."
        );
    }

    if (!std::isfinite(maximum_torque_) || maximum_torque_ <= 0.0) {
        throw std::invalid_argument(
            "Maximum torque must be finite and positive."
        );
    }
}

double RollRateController::update(
    double desired_rate,
    double measured_rate
) const {
    if (!std::isfinite(desired_rate) || !std::isfinite(measured_rate)) {
        throw std::invalid_argument(
            "Desired and measured rates must be finite."
        );
    }

    const double error = desired_rate - measured_rate;
    const double requested_torque = proportional_gain_ * error;

    return std::clamp(
        requested_torque,
        -maximum_torque_,
        maximum_torque_
    );
}

}  // namespace quadcopter
