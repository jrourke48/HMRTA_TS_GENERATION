cmake_minimum_required(VERSION 3.10)
project(PlanningSystem)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find required packages
find_package(Spot REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(BDD REQUIRED)

# Source files
set(CORE_SOURCES
    VoronoiPruningPipeline.cpp
    IntegratedPlanningSystem.cpp
    PlanningUtilities.cpp
)

# Create library
add_library(planning_system STATIC ${CORE_SOURCES})
target_link_libraries(planning_system PUBLIC Spot::Spot Eigen3::Eigen)
target_include_directories(planning_system PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Examples
add_executable(voronoi_example VoronoiPruningExample.cpp)
target_link_libraries(voronoi_example planning_system Spot::Spot Eigen3::Eigen)

add_executable(complete_workflow CompleteWorkflowExample.cpp)
target_link_libraries(complete_workflow planning_system Spot::Spot Eigen3::Eigen)

# Installation
install(TARGETS planning_system 
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES 
    VoronoiPruningPipeline.h
    IntegratedPlanningSystem.h
    PlanningUtilities.h
    DESTINATION include/planning_system
)

# Custom target for running examples
add_custom_target(run_examples
    COMMAND voronoi_example
    COMMAND complete_workflow
    DEPENDS voronoi_example complete_workflow
)

message(STATUS "PlanningSystem module configured")
message(STATUS "  - Core library: planning_system")
message(STATUS "  - Examples: voronoi_example, complete_workflow")
message(STATUS "  - Run with: make run_examples")
