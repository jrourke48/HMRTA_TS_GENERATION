cmake_minimum_required(VERSION 3.10)
project(HMRTATests)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(spot REQUIRED)

# Include directories
include_directories(
    ${PROJECT_SOURCE_DIR}/../../../Automatons
    ${PROJECT_SOURCE_DIR}/../../../Transition_Systems
    ${PROJECT_SOURCE_DIR}/../../include
)

# Test executables
add_executable(test_automaton_states test_automaton_states.cpp)
add_executable(test_number_robots test_number_robots.cpp)
add_executable(test_transition_system_regions test_transition_system_regions.cpp)
add_executable(test_average_capabilities test_average_capabilities.cpp)
add_executable(test_robot_homogeneity test_robot_homogeneity.cpp)
add_executable(run_all_tests run_all_tests.cpp)

# Link libraries
target_link_libraries(test_automaton_states spot bddx)
target_link_libraries(test_number_robots spot bddx)
target_link_libraries(test_transition_system_regions spot bddx)
target_link_libraries(test_average_capabilities spot bddx)
target_link_libraries(test_robot_homogeneity spot bddx)

# Set output directory
set_target_properties(
    test_automaton_states test_number_robots test_transition_system_regions
    test_average_capabilities test_robot_homogeneity run_all_tests
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
