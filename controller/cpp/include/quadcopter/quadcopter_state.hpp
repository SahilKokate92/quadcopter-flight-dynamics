#pragma once

#include "quadcopter/rigid_body_dynamics.hpp"
#include "quadcopter/rotor_dynamics.hpp"

namespace quadcopter {

struct QuadcopterState {
    RigidBodyState rigid_body;
    RotorSpeeds rotor_speeds_rad_s;
};

}  // namespace quadcopter