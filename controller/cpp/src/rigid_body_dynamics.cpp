
#include "quadcopter/rigid_body_dynamics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace quadcopter {
namespace {

constexpr double kMinimumQuaternionNorm = 1e-12;

void validate_vector(const Vec3& vector, const char* name) {
    for (double value : vector) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(
                std::string(name) + " must contain finite values."
            );
        }
    }
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vec3 subtract(const Vec3& a, const Vec3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vec3 scale(double factor, const Vec3& vector) {
    return {
        factor * vector[0],
        factor * vector[1],
        factor * vector[2]
    };
}

Vec3 cross(const Vec3& a, const Vec3& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

Quaternion normalize_quaternion(const Quaternion& q) {
    double norm_squared = 0.0;

    for (double value : q) {
        if (!std::isfinite(value)) {
            throw std::invalid_argument(
                "Quaternion must contain finite values."
            );
        }
        norm_squared += value * value;
    }

    const double norm = std::sqrt(norm_squared);

    if (norm < kMinimumQuaternionNorm) {
        throw std::invalid_argument(
            "Cannot normalize a zero quaternion."
        );
    }

    return {
        q[0] / norm,
        q[1] / norm,
        q[2] / norm,
        q[3] / norm
    };
}

// Hamilton product of scalar-first quaternions [w, x, y, z].
Quaternion quaternion_multiply(
    const Quaternion& a,
    const Quaternion& b
) {
    const double w1 = a[0];
    const double x1 = a[1];
    const double y1 = a[2];
    const double z1 = a[3];

    const double w2 = b[0];
    const double x2 = b[1];
    const double y2 = b[2];
    const double z2 = b[3];

    return {
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    };
}

// Rotation matrix mapping body FRD vectors to inertial NED.
std::array<std::array<double, 3>, 3> rotation_body_to_ned(
    const Quaternion& input
) {
    const Quaternion q = normalize_quaternion(input);

    const double w = q[0];
    const double x = q[1];
    const double y = q[2];
    const double z = q[3];

    return {{
        {{1.0 - 2.0*(y*y + z*z),
          2.0*(x*y - w*z),
          2.0*(x*z + w*y)}},

        {{2.0*(x*y + w*z),
          1.0 - 2.0*(x*x + z*z),
          2.0*(y*z - w*x)}},

        {{2.0*(x*z - w*y),
          2.0*(y*z + w*x),
          1.0 - 2.0*(x*x + y*y)}}
    }};
}

Vec3 matrix_vector_multiply(
    const std::array<std::array<double, 3>, 3>& matrix,
    const Vec3& vector
) {
    Vec3 result{0.0, 0.0, 0.0};

    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            result[row] += matrix[row][col] * vector[col];
        }
    }

    return result;
}

Vec3 transpose_matrix_vector_multiply(
    const std::array<std::array<double, 3>, 3>& matrix,
    const Vec3& vector
) {
    Vec3 result{0.0, 0.0, 0.0};

    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            result[row] += matrix[col][row] * vector[col];
        }
    }

    return result;
}

}  // namespace

RigidBodyDynamics::RigidBodyDynamics(RigidBodyParameters parameters)
    : parameters_(parameters) {
    if (!std::isfinite(parameters_.mass_kg) ||
        parameters_.mass_kg <= 0.0) {
        throw std::invalid_argument(
            "Mass must be finite and positive."
        );
    }

    if (!std::isfinite(parameters_.gravity_m_s2) ||
        parameters_.gravity_m_s2 <= 0.0) {
        throw std::invalid_argument(
            "Gravity magnitude must be finite and positive."
        );
    }

    for (double inertia : parameters_.inertia_diagonal_kg_m2) {
        if (!std::isfinite(inertia) || inertia <= 0.0) {
            throw std::invalid_argument(
                "All principal moments of inertia must be finite and positive."
            );
        }
    }
}

RigidBodyDerivatives RigidBodyDynamics::derivatives(
    const RigidBodyState& state,
    const BodyWrench& wrench
) const {
    validate_vector(state.position_ned_m, "Position");
    validate_vector(state.velocity_body_m_s, "Body velocity");
    validate_vector(state.angular_velocity_body_rad_s, "Angular velocity");
    validate_vector(wrench.force_body_n, "Body force");
    validate_vector(wrench.torque_body_nm, "Body torque");

    const Quaternion q = normalize_quaternion(state.quaternion_nb);
    const auto rotation = rotation_body_to_ned(q);

    const Vec3 gravity_ned{
        0.0,
        0.0,
        parameters_.gravity_m_s2
    };

    // Position derivative: body velocity expressed in NED.
    const Vec3 position_dot_ned =
        matrix_vector_multiply(rotation, state.velocity_body_m_s);

    // Gravity expressed in body coordinates.
    const Vec3 gravity_body =
        transpose_matrix_vector_multiply(rotation, gravity_ned);

    // Translational dynamics in the rotating body frame.
    const Vec3 force_acceleration = scale(
        1.0 / parameters_.mass_kg,
        wrench.force_body_n
    );

    const Vec3 rotational_velocity_term = cross(
        state.angular_velocity_body_rad_s,
        state.velocity_body_m_s
    );

    const Vec3 velocity_dot_body = subtract(
        add(force_acceleration, gravity_body),
        rotational_velocity_term
    );

    // Euler's rigid-body rotational equation for diagonal inertia.
    const Vec3 angular_momentum{
        parameters_.inertia_diagonal_kg_m2[0]
            * state.angular_velocity_body_rad_s[0],
        parameters_.inertia_diagonal_kg_m2[1]
            * state.angular_velocity_body_rad_s[1],
        parameters_.inertia_diagonal_kg_m2[2]
            * state.angular_velocity_body_rad_s[2]
    };

    const Vec3 gyroscopic_term = cross(
        state.angular_velocity_body_rad_s,
        angular_momentum
    );

    const Vec3 angular_velocity_dot_body{
        (wrench.torque_body_nm[0] - gyroscopic_term[0])
            / parameters_.inertia_diagonal_kg_m2[0],
        (wrench.torque_body_nm[1] - gyroscopic_term[1])
            / parameters_.inertia_diagonal_kg_m2[1],
        (wrench.torque_body_nm[2] - gyroscopic_term[2])
            / parameters_.inertia_diagonal_kg_m2[2]
    };

    // q_dot = 0.5 * q Hamilton-multiplied by [0, p, q, r].
    const Quaternion omega_quaternion{
        0.0,
        state.angular_velocity_body_rad_s[0],
        state.angular_velocity_body_rad_s[1],
        state.angular_velocity_body_rad_s[2]
    };

    const Quaternion product = quaternion_multiply(q, omega_quaternion);

    const Quaternion quaternion_dot_nb{
        0.5 * product[0],
        0.5 * product[1],
        0.5 * product[2],
        0.5 * product[3]
    };

    return {
        position_dot_ned,
        velocity_dot_body,
        quaternion_dot_nb,
        angular_velocity_dot_body
    };
}

}  // namespace quadcopter
