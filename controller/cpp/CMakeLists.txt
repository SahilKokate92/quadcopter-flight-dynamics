
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
