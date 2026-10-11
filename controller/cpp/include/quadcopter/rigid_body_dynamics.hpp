
#ifndef QUADCOPTER_RIGID_BODY_DYNAMICS_HPP
#define QUADCOPTER_RIGID_BODY_DYNAMICS_HPP

#include <array>

namespace quadcopter {

// Three-dimensional vector: components are x, y, z.
using Vec3 = std::array<double, 3>;

// Scalar-first Hamilton quaternion: [w, x, y, z].
using Quaternion = std::array<double, 4>;

// Physical properties of the rigid vehicle.
// SI units: kg, m/s^2, kg*m^2.
struct RigidBodyParameters {
    double mass_kg;
    double gravity_m_s2;
    Vec3 inertia_diagonal_kg_m2;
};

// Rigid-body state, with position in NED and velocity/rates in body FRD.
struct RigidBodyState {
    Vec3 position_ned_m;
    Vec3 velocity_body_m_s;
    Quaternion quaternion_nb;
    Vec3 angular_velocity_body_rad_s;
};

// Forces and moments are expressed in body FRD coordinates.
struct BodyWrench {
    Vec3 force_body_n;
    Vec3 torque_body_nm;
};

// Time derivatives of the rigid-body state.
struct RigidBodyDerivatives {
    Vec3 position_dot_ned_m_s;
    Vec3 velocity_dot_body_m_s2;
    Quaternion quaternion_dot_nb;
    Vec3 angular_velocity_dot_body_rad_s2;
};

class RigidBodyDynamics {
public:
    explicit RigidBodyDynamics(RigidBodyParameters parameters);

    RigidBodyDerivatives derivatives(
        const RigidBodyState& state,
        const BodyWrench& wrench
    ) const;

private:
    RigidBodyParameters parameters_;
};

}  // namespace quadcopter

#endif  // QUADCOPTER_RIGID_BODY_DYNAMICS_HPP
