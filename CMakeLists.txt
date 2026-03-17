cmake_minimum_required(VERSION 3.15)
project(HMRTA_TS_GENERATION)

# C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -g3")

# Output directory
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/output)

# ============================================================================
# Spot Library Configuration
# ============================================================================
# Adjust these paths for your system
set(SPOT_ROOT "C:/Users/johnn/Spot Library/spot-2.14.4.dev")

find_library(SPOT_LIB NAMES spot PATHS "${SPOT_ROOT}/lib" NO_DEFAULT_PATH)
find_library(BDDX_LIB NAMES bddx PATHS "${SPOT_ROOT}/lib" NO_DEFAULT_PATH)

if(NOT SPOT_LIB OR NOT BDDX_LIB)
    message(WARNING "Spot libraries not found at ${SPOT_ROOT}/lib")
    message("Please ensure Spot is installed correctly or update SPOT_ROOT in CMakeLists.txt")
endif()

# Include directories
include_directories(
    "${SPOT_ROOT}"
    "${SPOT_ROOT}/buddy/src"
    "${CMAKE_CURRENT_SOURCE_DIR}"
)

# ============================================================================
# Common Source Files (used by multiple targets)
# ============================================================================
set(COMMON_SOURCES
    TS.cpp
    BuchiAutomaton.cpp
    ProductAutomaton.cpp
)

set(TS_SOURCES
    Transition_Systems/GridWorldTransitionSystem.cpp
)

# ============================================================================
# Build Targets
# ============================================================================

# Target: LTLtoBuchiAutomaton (default)
add_executable(LTLtoBuchiAutomaton 
    LTLtoBuchiAutomaton.cpp
    ${TS_SOURCES}
)
target_link_libraries(LTLtoBuchiAutomaton PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: TestingAutomaton
add_executable(TestingAutomaton
    TestingAutomaton.cpp
    ${TS_SOURCES}
    ${COMMON_SOURCES}
)
target_link_libraries(TestingAutomaton PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: GlobalState
add_executable(GlobalState GlobalState.cpp)
target_link_libraries(GlobalState PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: ParsingandPrintingLTL
add_executable(ParsingandPrintingLTL Learning_Spot/ParsingandPrintingLTL.cpp)
target_link_libraries(ParsingandPrintingLTL PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: RelablingFormulas
add_executable(RelablingFormulas Learning_Spot/RelablingFormulas.cpp)
target_link_libraries(RelablingFormulas PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: TestingEquivalance
add_executable(TestingEquivalance Learning_Spot/TestingEquivalance.cpp)
target_link_libraries(TestingEquivalance PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: BasicTransitionSystem
add_executable(BasicTransitionSystem Transition_Systems/BasicTransitionSystem.cpp)
target_link_libraries(BasicTransitionSystem PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: GeneralTransitionSystem
add_executable(GeneralTransitionSystem Transition_Systems/GeneralTransitionSystem.cpp)
target_link_libraries(GeneralTransitionSystem PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: GridWorldTransitionSystem
add_executable(GridWorldTransitionSystem Transition_Systems/GridWorldTransitionSystem.cpp)
target_link_libraries(GridWorldTransitionSystem PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# Target: SpotGridWorldTS
add_executable(SpotGridWorldTS Transition_Systems/SpotGridWorldTS.cpp)
target_link_libraries(SpotGridWorldTS PRIVATE ${SPOT_LIB} ${BDDX_LIB})

# ============================================================================
# Custom Targets (for convenience)
# ============================================================================

# Add custom target for all executables
add_custom_target(all-targets DEPENDS
    LTLtoBuchiAutomaton
    TestingAutomaton
    GlobalState
    ParsingandPrintingLTL
    RelablingFormulas
    TestingEquivalance
    BasicTransitionSystem
    GeneralTransitionSystem
    GridWorldTransitionSystem
    SpotGridWorldTS
)

# ============================================================================
# Build Instructions
# ============================================================================
# cmake -B build           (configure)
# cmake --build build      (compile all)
# cmake --build build --target LTLtoBuchiAutomaton (compile specific target)
# ============================================================================
