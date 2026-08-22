# HMRTA Test Suite - Build and Run Instructions

## Overview
This directory contains C++ test programs for evaluating the HMRTA algorithm across five independent variables:

1. **test_automaton_states.cpp** - Automaton complexity (5-150 states)
2. **test_number_robots.cpp** - Fleet scalability (3-20 robots)
3. **test_transition_system_regions.cpp** - Environment complexity (5-40 regions)
4. **test_average_capabilities.cpp** - Capability diversity (1-5 avg per robot)
5. **test_robot_homogeneity.cpp** - Fleet specialization (0.2-3.0 score)
6. **run_all_tests.cpp** - Master orchestrator to run all tests

## Prerequisites
- C++20 compiler (g++, clang, or MSVC)
- Spot library (for LTL/Buchi automaton operations)
- CMake 3.10+ (optional, for building)
- Python 3 with pandas and matplotlib (for plotting results)

## Building

### Option 1: Using CMake (Recommended)
```bash
cd organizedtests
mkdir build
cd build
cmake ..
cmake --build .
cd ..
```

### Option 2: Manual Compilation
```bash
# Compile individual test programs
g++ -std=c++20 -Wall -Wextra test_automaton_states.cpp -o test_automaton_states -lspot -lbddx
g++ -std=c++20 -Wall -Wextra test_number_robots.cpp -o test_number_robots -lspot -lbddx
g++ -std=c++20 -Wall -Wextra test_transition_system_regions.cpp -o test_transition_system_regions -lspot -lbddx
g++ -std=c++20 -Wall -Wextra test_average_capabilities.cpp -o test_average_capabilities -lspot -lbddx
g++ -std=c++20 -Wall -Wextra test_robot_homogeneity.cpp -o test_robot_homogeneity -lspot -lbddx
g++ -std=c++20 -Wall -Wextra run_all_tests.cpp -o run_all_tests
```

## Running Tests

### Run All Tests at Once
```bash
./run_all_tests
```

### Run Individual Tests
```bash
./test_automaton_states
./test_number_robots
./test_transition_system_regions
./test_average_capabilities
./test_robot_homogeneity
```

## Output Files

Each test generates a CSV file with results:
- `automaton_states_results.csv` - Automaton states test data
- `number_robots_results.csv` - Robot count test data
- `transition_system_regions_results.csv` - Region count test data
- `average_capabilities_results.csv` - Capability test data
- `robot_homogeneity_results.csv` - Homogeneity test data
- `test_run_summary.txt` - Overall execution summary

## Plotting Results

After tests complete, generate plots:
```bash
python3 plot_results.py
```

This creates a `plots/` directory with PNG files showing:
- Computation time trends
- Algorithm efficiency scores
- Scalability metrics
- Other performance indicators

## Test Parameters

### Automaton States
- **Fixed Variables**: 6 robots, 6 regions, 3 Buchi configurations
- **Variable**: 15 different automaton state counts (5-150)

### Number of Robots
- **Fixed Variables**: 15 automaton states, 6 regions, 3 Buchi configurations
- **Variable**: 8 different fleet sizes (3-20 robots)

### Transition System Regions
- **Fixed Variables**: 6 robots, 15 automaton states, 3 Buchi configurations
- **Variable**: 10 different region counts (5-40)

### Average Capabilities
- **Fixed Variables**: 6 robots, 6 regions, 15 automaton states, 3 Buchi configurations
- **Variable**: 10 different capability distributions (1.0-5.0 avg per robot)

### Robot Homogeneity
- **Fixed Variables**: 6 robots, 6 regions, 15 automaton states, 12 total capabilities, 3 Buchi configurations
- **Variable**: 10 different homogeneity scores (0.2-3.0)

## Customizing Tests

To modify test parameters, edit the `std::vector` containing test values in each `.cpp` file:

```cpp
// Example: Change automaton states range
std::vector<int> automatonStates = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 115, 125, 135, 150};
```

## Data Collection

The CSV files include columns for:
- Test number
- Independent variable value
- Computation time (milliseconds)
- Memory usage (MB)
- Algorithm-specific metrics (quality score, scalability, efficiency, etc.)
- Status (PENDING, COMPLETED, FAILED)

## Total Test Runs
- **Independent variable tests**: 53 total configurations
- **With 3 Buchi automata variants per test**: ~159 total runs
- **Estimated runtime**: Several hours depending on system performance

## Integration with TestingAutomaton

The test files are currently placeholders that output "PENDING". To activate them:

1. Link against TestingAutomaton library or include its source
2. Implement the actual test functions (e.g., `runTestWithAutomatonStates()`)
3. Uncomment the timing code and results logging
4. Recompile and run

## Troubleshooting

**Error: Spot library not found**
- Install Spot: `apt-get install spot libspot-dev` (Ubuntu/Debian)
- Or build from source: https://spot.lrde.epita.fr/

**Python plotting fails**
- Install requirements: `pip install pandas matplotlib`

**Tests show PENDING status**
- Need to implement actual test logic in each `.cpp` file
- Currently, files generate empty CSV templates

## Future Enhancements

- [ ] Implement actual algorithm calls in test functions
- [ ] Add real-time progress reporting
- [ ] Support for parallel test execution
- [ ] Database storage for long-term result tracking
- [ ] Statistical analysis of results
- [ ] Comparison plots between different runs
