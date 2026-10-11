
cmake_minimum_required(VERSION 3.20)

project(QuadcopterController
    VERSION 0.1.0
    LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

enable_testing()

add_library(roll_rate_controller
    src/roll_rate_controller.cpp
)

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

target_include_directories(roll_rate_controller
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_executable(test_roll_rate_controller
    tests/test_roll_rate_controller.cpp
)

target_link_libraries(test_roll_rate_controller
    PRIVATE roll_rate_controller
)

add_test(
    NAME roll_rate_controller_test
    COMMAND test_roll_rate_controller
)

add_executable(simulate_roll_rate
    src/simulate_roll_rate.cpp
)

target_link_libraries(simulate_roll_rate
    PRIVATE roll_rate_controller
)

add_executable(test_roll_rate_simulation
    tests/test_roll_rate_simulation.cpp
)

target_link_libraries(test_roll_rate_simulation
    PRIVATE roll_rate_controller
)

add_test(
    NAME roll_rate_simulation_test
    COMMAND test_roll_rate_simulation
)

add_executable(test_rigid_body_dynamics
    tests/test_rigid_body_dynamics.cpp
)

target_link_libraries(test_rigid_body_dynamics
    PRIVATE rigid_body_dynamics
)

add_test(
    NAME rigid_body_dynamics_test
    COMMAND test_rigid_body_dynamics
)

add_executable(compare_rigid_body_derivatives
    src/print_rigid_body_derivatives.cpp
)

target_link_libraries(compare_rigid_body_derivatives
    PRIVATE rigid_body_dynamics
)

add_executable(test_rk4_integrator
    tests/test_rk4_integrator.cpp
)

target_link_libraries(test_rk4_integrator
    PRIVATE rigid_body_dynamics
)

add_test(
    NAME rk4_integrator_test
    COMMAND test_rk4_integrator
)


add_executable(print_rigid_body_trajectory
    src/print_rigid_body_trajectory.cpp
)

target_link_libraries(print_rigid_body_trajectory
    PRIVATE rigid_body_dynamics
)

add_executable(test_rotor_dynamics
    tests/test_rotor_dynamics.cpp
)

target_link_libraries(test_rotor_dynamics
    PRIVATE rigid_body_dynamics
)

add_test(
    NAME rotor_dynamics_test
    COMMAND test_rotor_dynamics
)

add_executable(test_motor_dynamics
    tests/test_motor_dynamics.cpp
)

target_link_libraries(test_motor_dynamics
    PRIVATE rigid_body_dynamics
)

add_test(
    NAME motor_dynamics_test
    COMMAND test_motor_dynamics
)