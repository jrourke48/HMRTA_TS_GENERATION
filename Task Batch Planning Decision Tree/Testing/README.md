# Testing Module

## Overview

The Testing module provides comprehensive test coverage for the multi-robot task allocation system. Tests validate task allocation algorithms, automata handling, tree construction, and integration between components.

### Key Responsibilities

1. **Algorithm Validation** - Verify correctness of task allocation
2. **Automata Testing** - Test finite and infinite Büchi automata
3. **Integration Testing** - Validate component interactions
4. **Performance Testing** - Measure scalability and efficiency
5. **Regression Testing** - Catch bugs in modifications

## Test Files

### Optimal_Path_Test.cpp

**Purpose**: Main test suite for task allocation with different automata types

**Status**: Infinite automata test 1 enabled; tests 2-4 commented

#### Test Categories

**Finite Automata Tests** (One-time missions):
```cpp
// F("p0") & F("p1") - Visit two locations
// F("p0") & F("p1") & F("p2") & F("p3") - Visit four locations
```

**Infinite Automata Tests** (Cyclic missions):
```cpp
// Infinite Test 1: G(F("p0")) - Patrol one location forever
// Infinite Test 2: G(F("p0") & F("p1")) - Patrol two locations forever
// Infinite Test 3: G(F("p0") & F("p1") & F("p2")) - Patrol three locations
// Infinite Test 4: Complex nesting with conditionals
```

#### Running Tests

```bash
# Compile the test suite
make test-optimal-path

# Run all tests
./output/OptimalPathTest

# Run with debug output (requires recompilation with debug flags)
./output/OptimalPathTest 2>&1 | tee test_output.log
```

#### Test Output Interpretation

**Success Indicators**:
- No segmentation faults
- Tree construction completes
- Solution path found and validated
- Acceptance sets properly managed

**Diagnostic Output**:
```
=== Test: Infinite Automaton - Simple Patrol ===
Initial Automaton State: 0
Initial TS State: 0
Number of edges: 4
Processing edges...
  Edge 0: "p0" -> AP IDs: {0}, Acceptance: {}
  Edge 1: "p1" -> AP IDs: {1}, Acceptance: {}
  ...
Tree expansion completed
Nodes in frontier: 2
Final path length: 8
Test PASSED
```

---

### TestTaskAllocationAlgorithms.cpp

**Purpose**: Unit tests for individual algorithm functions

**Tests**:
- `collectAPsFromEdgesByIndexWithAcceptance()` - Edge extraction
- `unrelatedTaskSearch()` - Main search algorithm
- `pruneSubtree()` - Pruning logic validation

#### Running Tests

```bash
make TestTaskAllocation
./output/TestTaskAllocation
```

---

### TestPlanningTree.cpp

**Purpose**: Decision tree structure and operations testing

**Tests**:
- Tree insertion and deletion
- Node traversal
- Path extraction
- Frontier management
- Memory cleanup

#### Sample Test Structure

```cpp
void testTreeConstruction() {
    PlanningDecisionTree* tree = new PlanningDecisionTree();
    
    // Add nodes
    Tree_Node* root = new Tree_Node(0, 0, 0.0);
    tree->insertNode(root);
    
    Tree_Node* child1 = new Tree_Node(1, 1, 5.0);
    tree->insertNode(child1);
    
    // Verify
    assert(tree->getNodeCount() == 2);
    assert(tree->getFrontierSize() >= 1);
    
    // Cleanup
    delete tree;
}
```

---

### TestMultiRobotSystem.cpp

**Purpose**: Multi-robot coordination and capability testing

**Tests**:
- Robot creation and properties
- Team capability queries
- Robot-task assignment
- Capability-based filtering

---

### TestIntegration.cpp

**Purpose**: End-to-end system testing

**Tests**:
- Full pipeline: LTL → Automaton → Planning
- Multi-component interaction
- Solution validation
- Performance profiling

---

## Test Data Preparation

### Creating Test Environments

```cpp
// Helper: Create simple 3x3 TS
TS* createSimpleTS() {
    TS* ts = new TS();
    ts->addState(0, "");              // Initial
    ts->addState(1, "location_A");    // Goal 1
    ts->addState(2, "location_B");    // Goal 2
    
    ts->addTransition(0, 1, "move_A", 5.0);
    ts->addTransition(1, 2, "move_B", 8.0);
    ts->addTransition(2, 0, "return", 10.0);
    
    return ts;
}

// Helper: Create simple robot
Robot* createTestRobot(uint32_t id) {
    Robot* robot = new Robot(id, "Robot_" + to_string(id), Point(0, 0));
    robot->addCapability(ROBOT_CAPABILITY::MOVE);
    robot->addCapability(ROBOT_CAPABILITY::SENSE);
    return robot;
}

// Helper: Create simple environment
Environment* createTestEnvironment() {
    GridWorld* grid = new GridWorld(100, 100, 50);
    TS* ts = createSimpleTS();
    return new Environment(ts, grid);
}
```

### Test Automata Library

```cpp
// Collection of pre-configured test automata
struct TestAutomaton {
    string ltl_formula;
    bool is_finite;
    int expected_states;
    int expected_edges;
};

vector<TestAutomaton> getTestAutomata() {
    return {
        {"F(\"p0\") & F(\"p1\")", true, 5, 8},
        {"G(F(\"p0\"))", false, 2, 3},
        {"F(\"p0\" & X(\"p1\"))", true, 4, 6},
        // ... more test cases ...
    };
}
```

---

## Running Specific Tests

### By Category

```bash
# Only unit tests
make TestTaskAllocation

# Only tree tests
make TestPlanningTree

# Only multi-robot tests
make TestMultiRobotSystem

# Only integration tests
make TestIntegration

# All tests
make test-all
```

### With Filtering

```bash
# Run tests matching pattern (if using gtest framework)
./output/OptimalPathTest --gtest_filter=*Finite*

# Run specific test
./output/OptimalPathTest --gtest_filter=TestName
```

### With Coverage

```bash
# Compile with coverage flags
g++ -std=c++20 --coverage -o test_coverage *.cpp

# Run tests
./test_coverage

# Generate coverage report
gcov *.cpp
lcov --directory . --capture --output-file coverage.info
```

---

## Test Scenarios

### Simple Finite Automaton

```
Task: Visit location A, then location B
Formula: F("p0") & F("p1")
TS: 0 --move--> 1(p0) --move--> 2(p1)
Expected: Path [0,1,2] found within X iterations
Success: Tree size < 20 nodes, no pruning errors
```

### Complex Finite Automaton

```
Task: Visit 4 locations in any order
Formula: F("p0") & F("p1") & F("p2") & F("p3")
TS: Star topology from center to 4 locations
Expected: Any valid permutation of visiting all 4
Success: Solution found, cost reasonable, tree bounded
```

### Simple Infinite Automaton

```
Task: Patrol location forever
Formula: G(F("p0"))
TS: Two states with cycle 0 <--> 1(p0)
Expected: Cycles detected, infinite path recognized
Success: Algorithm terminates with cycles identified
```

### Complex Infinite Automaton

```
Task: Patrol 3 locations forever
Formula: G(F("p0") & F("p1") & F("p2"))
TS: Triangle graph with each location distinct
Expected: Visiting all 3 repeatedly
Success: Visit count tracking works, revisits allowed
```

---

## Debugging Test Failures

### Common Issues and Solutions

**Issue**: Test hangs (infinite loop)
```
Likely cause: Pruning not removing nodes correctly
Solution: Enable pruning debug output, verify removeFrontierNode() called
Check: nodesToRemove vector populated, frontier decreasing
```

**Issue**: Segmentation fault during tree expansion
```
Likely cause: Null pointer or memory corruption
Solution: Add bounds checks on edge access
Check: Edge indices within range, automaton valid
Verify: Parent pointers correct, no circular references
```

**Issue**: Acceptance sets not updating
```
Likely cause: Edge acceptance marks not parsed
Solution: Verify edge label format "p0:{0,1}"
Check: acceptingSets vector modifications in loop
Debug: Print acceptance set before/after each edge
```

**Issue**: Tree grows unbounded despite pruning
```
Likely cause: Node removal code not executing
Solution: Verify nodesToRemove not empty
Check: removeFrontierNode() actually called
Debug: Add assertion that frontier shrinks each iteration
```

### Debug Output Techniques

**Enable Detailed Logging**:
```cpp
#define DEBUG_EDGE_PROCESSING 1
#define DEBUG_ACCEPTANCE_SETS 1
#define DEBUG_PRUNING 1

// In algorithm code:
#if DEBUG_EDGE_PROCESSING
std::cerr << "Processing edge " << i << ": " << edges[i] << std::endl;
#endif
```

**Checkpoint Validation**:
```cpp
// After each major step
assert(!tree->getFrontier().empty() || isGoalReached);
assert(tree->getNodeCount() > 0);
assert(tree->getFrontierSize() <= tree->getNodeCount());
```

**Trace Execution**:
```cpp
// Add to key functions
std::cerr << "TRACE: " << __FUNCTION__ << " at line " 
          << __LINE__ << std::endl;
```

---

## Performance Benchmarking

### Metrics to Track

```cpp
struct TestMetrics {
    double compile_time;          // Build time
    double run_time;              // Execution time
    uint32_t tree_nodes;          // Total nodes created
    uint32_t max_frontier_size;   // Peak frontier size
    double memory_used;           // Memory consumption (MB)
    uint32_t pruning_count;       // Nodes pruned
};
```

### Benchmark Suite

```cpp
void benchmarkFiniteAutomata() {
    for (int i = 1; i <= 5; i++) {
        string formula = generateFiniteFormula(i);  // i tasks
        
        auto start = chrono::high_resolution_clock::now();
        BuchiAutomaton* aut = new BuchiAutomaton();
        aut->generateFromLTLString(formula);
        auto end = chrono::high_resolution_clock::now();
        
        cout << "Formula with " << i << " tasks: "
             << chrono::duration_cast<chrono::milliseconds>(end - start).count()
             << " ms" << endl;
    }
}
```

---

## Continuous Integration

### Test Automation

```bash
#!/bin/bash
# run_all_tests.sh

echo "Building all tests..."
make clean
make test-optimal-path
make TestTaskAllocation
make TestPlanningTree
make TestMultiRobotSystem
make TestIntegration

echo "Running tests..."
./output/OptimalPathTest
./output/TestTaskAllocation
./output/TestPlanningTree
./output/TestMultiRobotSystem
./output/TestIntegration

echo "All tests completed!"
```

### Regression Test Suite

```cpp
// Keep historical test cases that found bugs
struct RegressionTest {
    string name;
    string formula;
    bool should_pass;
    string reason;  // Why this test was added
};

vector<RegressionTest> getRegressionTests() {
    return {
        {"Bug: AND filtering", "F(\"p0\" & \"p1\")", true, 
         "Ensure AND-connected APs excluded properly"},
        {"Bug: Acceptance removal", "G(F(\"p0\"))", true,
         "Verify acceptance sets removed correctly"},
        // ... more regression tests ...
    };
}
```

---

## Adding New Tests

### Test Template

```cpp
void testNewFeature() {
    // Setup
    Environment* env = createTestEnvironment();
    BuchiAutomaton* aut = new BuchiAutomaton();
    aut->generateFromLTLString("F(\"p0\") & F(\"p1\")");
    Robot* robot = createTestRobot(0);
    
    // Execute
    TaskAllocationAlgorithms allocator;
    vector<uint16_t> accepting = aut->getAcceptingStates();
    allocator.unrelatedTaskSearch(robot, env, aut, accepting);
    
    // Verify
    assert(allocator.foundSolution());
    assert(allocator.getSolutionCost() < MAX_EXPECTED_COST);
    
    // Cleanup
    delete env;
    delete aut;
    delete robot;
}
```

### Checklist for New Tests

- [ ] Test name describes what is being tested
- [ ] Clear setup with all required objects
- [ ] One logical assertion per test
- [ ] Proper cleanup/memory deallocation
- [ ] Documentation of expected behavior
- [ ] Edge cases covered
- [ ] Both success and failure paths tested

---

## Known Test Limitations

1. **Small Workspace**: Tests use tiny TS/grids for speed; may not catch scalability bugs
2. **Simple Automata**: Complex nested operators not thoroughly tested
3. **Single Robot**: Multi-robot coordination minimal in tests
4. **Manual Validation**: Some tests require manual inspection of output

---

## See Also

- [../README.md](../README.md) - Main module documentation
- [../Optimal_Path_Test.cpp](../Optimal_Path_Test.cpp) - Main test implementation
- [../../Automatons/README.md](../../Automatons/README.md) - Automata details
- [Makefile](../Makefile) - Build configuration

---

**Last Updated**: 2026-07-21  
**Status**: Comprehensive test coverage with debugging utilities
