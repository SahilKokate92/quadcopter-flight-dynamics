cmake_minimum_required(VERSION 3.16)

project(quadcopter_flight_dynamics LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

# Core rigid-body and vehicle dynamics library
add_library(rigid_body_dynamics
    src/rigid_body_dynamics.cpp
    src/rk4_integrator.cpp
    src/rotor_dynamics.cpp
    src/motor_dynamics.cpp
    src/quadcopter_dynamics.cpp
)

target_include_directories(rigid_body_dynamics
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Roll-rate controller library
add_library(roll_rate_controller
    src/roll_rate_controller.cpp
)

target_include_directories(roll_rate_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 1. Roll rate controller
add_executable(test_roll_rate_controller
    tests/test_roll_rate_controller.cpp
)
target_link_libraries(test_roll_rate_controller
    PRIVATE roll_rate_controller
)
add_test(NAME roll_rate_controller_test
    COMMAND test_roll_rate_controller
)

# 2. Roll rate simulation
add_executable(test_roll_rate_simulation
    tests/test_roll_rate_simulation.cpp
)
target_link_libraries(test_roll_rate_simulation
    PRIVATE roll_rate_controller
)
add_test(NAME roll_rate_simulation_test
    COMMAND test_roll_rate_simulation
)

# 3. Rigid body dynamics
add_executable(test_rigid_body_dynamics
    tests/test_rigid_body_dynamics.cpp
)
target_link_libraries(test_rigid_body_dynamics
    PRIVATE rigid_body_dynamics
)
add_test(NAME rigid_body_dynamics_test
    COMMAND test_rigid_body_dynamics
)

# 4. RK4 integrator
add_executable(test_rk4_integrator
    tests/test_rk4_integrator.cpp
)
target_link_libraries(test_rk4_integrator
    PRIVATE rigid_body_dynamics
)
add_test(NAME rk4_integrator_test
    COMMAND test_rk4_integrator
)

# 5. Rotor dynamics
add_executable(test_rotor_dynamics
    tests/test_rotor_dynamics.cpp
)
target_link_libraries(test_rotor_dynamics
    PRIVATE rigid_body_dynamics
)
add_test(NAME rotor_dynamics_test
    COMMAND test_rotor_dynamics
)

# 6. Motor dynamics
add_executable(test_motor_dynamics
    tests/test_motor_dynamics.cpp
)
target_link_libraries(test_motor_dynamics
    PRIVATE rigid_body_dynamics
)
add_test(NAME motor_dynamics_test
    COMMAND test_motor_dynamics
)

# 7. Rotor mixer
add_executable(test_rotor_mixer
    tests/test_rotor_mixer.cpp
)
target_link_libraries(test_rotor_mixer
    PRIVATE rigid_body_dynamics
)
add_test(NAME rotor_mixer_test
    COMMAND test_rotor_mixer
)

# 8. Attitude response
add_executable(test_attitude_response
    tests/test_attitude_response.cpp
)
target_link_libraries(test_attitude_response
    PRIVATE rigid_body_dynamics
)
add_test(NAME attitude_response_test
    COMMAND test_attitude_response
)

# 9. Rotational dynamics validation
add_executable(test_rotational_dynamics_validation
    tests/test_rotational_dynamics_validation.cpp
)
target_link_libraries(test_rotational_dynamics_validation
    PRIVATE rigid_body_dynamics
)
add_test(NAME rotational_dynamics_validation_test
    COMMAND test_rotational_dynamics_validation
)

# 10. Translational dynamics validation
add_executable(test_translational_dynamics_validation
    tests/test_translational_dynamics_validation.cpp
)
target_link_libraries(test_translational_dynamics_validation
    PRIVATE rigid_body_dynamics
)
add_test(NAME translational_dynamics_validation_test
    COMMAND test_translational_dynamics_validation
)


# 11. Rotor force-and-moment numerical validation
add_executable(test_rotor_force_moment_validation
    tests/test_rotor_force_moment_validation.cpp
)
target_link_libraries(test_rotor_force_moment_validation
    PRIVATE rigid_body_dynamics
)
add_test(NAME rotor_force_moment_validation_test
    COMMAND test_rotor_force_moment_validation
)

# 12. Quaternion axis integration
add_executable(test_quaternion_axis_integration
    tests/test_quaternion_axis_integration.cpp
)
target_link_libraries(test_quaternion_axis_integration
    PRIVATE rigid_body_dynamics
)
add_test(NAME quaternion_axis_integration_test
    COMMAND test_quaternion_axis_integration
)

# 13. Collective thrust response
add_executable(test_collective_thrust_response
    tests/test_collective_thrust_response.cpp
)
target_link_libraries(test_collective_thrust_response
    PRIVATE rigid_body_dynamics
)
add_test(NAME collective_thrust_response_test
    COMMAND test_collective_thrust_response
)

# 14. Full-model numerical stability
add_executable(test_full_model_numerical_stability
    tests/test_full_model_numerical_stability.cpp
)
target_link_libraries(test_full_model_numerical_stability
    PRIVATE rigid_body_dynamics
)
add_test(NAME full_model_numerical_stability_test
    COMMAND test_full_model_numerical_stability
)

# Open-loop asymmetric quadcopter simulation
add_executable(open_loop_asymmetric
    examples/open_loop_asymmetric.cpp
)

target_link_libraries(open_loop_asymmetric
    PRIVATE rigid_body_dynamics
)