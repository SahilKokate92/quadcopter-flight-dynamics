
#ifndef QUADCOPTER_ROLL_RATE_CONTROLLER_HPP
#define QUADCOPTER_ROLL_RATE_CONTROLLER_HPP

namespace quadcopter {

class RollRateController {
public:
    explicit RollRateController(
        double proportional_gain,
        double maximum_torque
    );

    double update(double desired_rate, double measured_rate) const;

private:
    double proportional_gain_;
    double maximum_torque_;
};

}  // namespace quadcopter

#endif  // QUADCOPTER_ROLL_RATE_CONTROLLER_HPP
