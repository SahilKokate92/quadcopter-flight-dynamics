
#include "quadcopter/roll_rate_controller.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <stdexcept>

int main() {
    const double gain = 0.006;
    const double maximum_torque = 0.002;

    const quadcopter::RollRateController controller(
        gain,
        maximum_torque
    );

    // Test 1: Positive rate error produces positive torque.
    assert(std::abs(controller.update(0.5, 0.2) - 0.0018) < 1e-12);

    // Test 2: Negative rate error produces negative torque.
    assert(std::abs(controller.update(0.2, 0.5) + 0.0018) < 1e-12);

    // Test 3: Zero rate error produces zero torque.
    assert(std::abs(controller.update(0.3, 0.3)) < 1e-12);

    // Test 4: Large positive command is saturated.
    assert(std::abs(controller.update(2.0, 0.0) - maximum_torque) < 1e-12);

    // Test 5: Large negative command is saturated.
    assert(std::abs(controller.update(0.0, 2.0) + maximum_torque) < 1e-12);

    // Test 6: Negative gain must be rejected.
    bool invalid_gain_rejected = false;

    try {
        const quadcopter::RollRateController invalid_controller(
            -1.0,
            maximum_torque
        );
    } catch (const std::invalid_argument&) {
        invalid_gain_rejected = true;
    }

    assert(invalid_gain_rejected);

    // Test 7: Zero maximum torque must be rejected.
    bool invalid_limit_rejected = false;

    try {
        const quadcopter::RollRateController invalid_controller(
            gain,
            0.0
        );
    } catch (const std::invalid_argument&) {
        invalid_limit_rejected = true;
    }

    assert(invalid_limit_rejected);

    // Test 8: Non-finite measured rate must be rejected.
    bool invalid_rate_rejected = false;

    try {
        controller.update(0.0, NAN);
    } catch (const std::invalid_argument&) {
        invalid_rate_rejected = true;
    }

    assert(invalid_rate_rejected);
    // Test 9: Torque output remains within limits across many inputs.
    for (int desired = -100; desired <= 100; ++desired) {
        for (int measured = -100; measured <= 100; ++measured) {
            const double desired_rate = desired * 0.1;
            const double measured_rate = measured * 0.1;

            const double torque = controller.update(
                desired_rate,
                measured_rate
            );

            assert(torque >= -maximum_torque - 1e-12);
            assert(torque <= maximum_torque + 1e-12);
        }
    }


    std::cout << "All roll-rate controller tests passed.\n";
    return 0;
}
