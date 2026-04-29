# Task Batch Planning Decision Tree

A hierarchical tree-based planning module for multi-robot task allocation and decision-making in the HMRTA (Hierarchical Multi-Robot Task Allocation) system.

## Overview

The Planning Decision Tree module provides a framework for:
- **Hierarchical Decision Making**: Build and traverse tree-based search structures for planning paths through task-automaton and transition system states
- **Multi-Robot Task Allocation**: Efficiently allocate tasks to multiple robots with constraint-aware scheduling
- **Batch Task Planning**: Organize and manage robot tasks in batches with progress tracking
- **Environment Navigation**: Integrate with grid-world environments and transition system representations

This module forms a critical component of the thesis project for generating test suites using hierarchical multi-robot task allocation strategies.

## Architecture

### Core Components

#### 1. **PlanningDecisionTree** (`PlanningDecisionTree.h/cpp`)
The main tree structure class that manages the hierarchical decision tree.

**Key Responsibilities:**
- Maintain tree root and node count
- Construct tree by inserting nodes with associated automaton and transition system states
- Manage tree lifecycle (creation, modification, cleanup)

**Key Methods:**
- `insertNode()` - Add a new node to the tree
- `deleteSubtree()` - Remove a subtree rooted at a given node
- `getRoot()` - Access the tree root
- `getNodeCount()` - Query total nodes in tree

#### 2. **Tree_Node** (`Tree_Node.h`)
Represents individual nodes within the planning tree.

**Node State Information:**
- **Automaton State**: Current state in the Büchi automaton (NBA)
- **Transition System State**: Current state in the transition system (TS)
- **Task Allocation**: Boolean vector indicating which robot is allocated to which task
- **Time Estimates**: Estimated completion times for each robot
- **Batch Number**: Groups related tasks
- **Progress Type**: Tracks position in task execution cycle (PRE, TRA, SUF, OTH)

**Progress Enum:**
```cpp
enum class TASK_PROGRESS {
    PRE,  // Pre-condition phase
    TRA,  // Transition phase
    SUF,  // Suffix phase
    OTH   // Other/unspecified
};
```

#### 3. **TaskAllocationAlgorithms** (`TaskAllocationAlgorithms.h/cpp`)
Core algorithms for building planning trees and determining optimal paths.

**System Components:**
- Büchi automaton for LTL specification validation
- Environment representation (transition system + grid world)
- Multi-robot system for coordination

**Key Methods:**
- `buildPlanningTree()` - Construct the complete planning decision tree
- `determineBestPath()` - Select optimal execution path through tree
- Component getters/setters for NBA, Environment, and MultiRobotSystem

#### 4. **Environment** (`Environment.h/cpp`)
Encapsulates the operational environment combining a transition system with a grid world.

**Responsibilities:**
- Bridge between discrete transition systems and continuous grid environments
- State-grid coordinate mapping
- Successor state queries
- Obstacle/free space queries

**Key Methods:**
- `getTransitionSystem()` / `setTransitionSystem()` - Access to discrete TS
- `getGridWorld()` / `setGridWorld()` - Access to 2D grid representation
- `gridToStateId()` - Convert 2D grid point to state ID
- `stateIdToGrid()` - Convert state ID to 2D grid point
- `isObstacle()` / `isFree()` - Query environment properties

#### 5. **GridWorld** (`GridWorld.h/cpp`)
2D grid representation of the physical environment with costmap.

**Grid Properties:**
- **Costmap**: 2D array where:
  - 0 = free space
  - 1-254 = movement cost
  - 255 = obstacle

**Key Methods:**
- `getCost()` / `setCost()` - Query/set cell costs
- `isObstacle()` / `isFree()` - Check cell properties
- `isInBounds()` - Validate coordinates
- `setObstacle()` / `clearCell()` - Modify environment

#### 6. **Point** (`Point.h/cpp`)
Simple 2D coordinate representation for grid-based navigation.

**Operations:**
- Coordinate access (x, y)
- Vector arithmetic (addition, subtraction)
- Distance calculations
- Comparison operators

## File Structure

```
Task Batch Planning Decision Tree/
├── PlanningDecisionTree.h          # Main tree structure
├── PlanningDecisionTree.cpp        # Tree implementation
├── Tree_Node.h                      # Individual node definition
├── TaskAllocationAlgorithms.h       # Core algorithms
├── TaskAllocationAlgorithms.cpp     # Algorithm implementation
├── Environment.h                    # Environment abstraction
├── Environment.cpp                  # Environment implementation
├── GridWorld.h                      # 2D grid representation
├── GridWorld.cpp                    # Grid implementation
├── Point.h                          # 2D point utility
├── Point.cpp                        # Point implementation
├── MultiRobotSystem/                # Multi-robot coordination
└── README.md                        # This file
```

## Key Features

### 1. Hierarchical Planning
- Organize complex task allocations in tree structures
- Support branching strategies for different robot configurations
- Track progress through multi-phase task execution (PRE, TRA, SUF)

### 2. Multi-Robot Coordination
- Allocate tasks to multiple robots simultaneously
- Track individual robot completion times
- Batch related tasks for coherent execution

### 3. Formal Methods Integration
- Integrate with Büchi automata for LTL specifications
- Connect to transition systems for state-based planning
- Ensure formal correctness of task sequences

### 4. Environment Awareness
- Navigate 2D grid environments with obstacles and costs
- Map between discrete transition system states and continuous grid coordinates
- Support cost-aware planning decisions

## Usage Example

```cpp
// 1. Create environment
GeneralTransitionSystem* ts = new GeneralTransitionSystem(...);
GridWorld* grid = new GridWorld(100, 100);
Environment* env = new Environment(ts, grid);

// 2. Initialize multi-robot system
MultiRobotSystem* robots = new MultiRobotSystem(3); // 3 robots

// 3. Create NBA from LTL formula
BuchiAutomaton* nba = new BuchiAutomaton(...);

// 4. Setup task allocation algorithms
TaskAllocationAlgorithms* allocator = new TaskAllocationAlgorithms(nba, env, robots);

// 5. Build planning tree
std::vector<bool> initialAllocation = {true, false, true};
std::vector<uint16_t> times = {10, 15, 12};
PlanningDecisionTree* planTree = allocator->buildPlanningTree(
    0,                                  // root ID
    nba->getInitialState(),            // automaton state
    ts->getInitialState(),             // TS state
    initialAllocation,                 // task allocation
    times,                             // estimated times
    1,                                 // batch number
    Tree_Node::TASK_PROGRESS::PRE      // progress type
);

// 6. Determine best execution path
Tree_Node* bestPath = allocator->determineBestPath(planTree);

// 7. Cleanup
delete planTree;
delete allocator;
delete robots;
delete nba;
delete env;
```

## Integration Points

### With Automatons Module
- Uses `Node` from `Automatons/` for automaton state representation
- Works with `BuchiAutomaton` for LTL specification validation
- Tracks progress through automaton states during planning

### With Transition Systems Module
- Integrates `GeneralTransitionSystem` for discrete state space
- Maps between TS state IDs and grid coordinates
- Queries successor states for planning exploration

### With Multi-Robot System
- Coordinates task allocation across multiple robots
- Tracks individual robot times and capabilities
- Manages robot-task relationships in allocation vectors

## Data Flow

```
LTL Formula
    ↓
[Büchi Automaton] ←── Task Allocation Algorithms ←── [Environment]
    ↓                          ↓                          ↓
Automaton States      Planning Decision Tree      [Transition System]
                              ↓                          ↓
                          Tree_Node                  [GridWorld]
                          (decision points)              ↓
                              ↓                      [Point coordinates]
                        Task Allocation
                        (robot assignments)
```

## Compilation

### With CMake
```bash
cmake -B build
cmake --build build
```

### With g++
```bash
g++ -std=c++20 -I. -L. PlanningDecisionTree.cpp Tree_Node.cpp \
    TaskAllocationAlgorithms.cpp Environment.cpp GridWorld.cpp Point.cpp \
    -o planning_tree -lspot -lbddx
```

## Future Extensions

- **Heuristic Pruning**: Implement A* style heuristics to reduce tree size
- **Dynamic Replanning**: Support re-planning when environment changes
- **Parallelization**: Parallel tree construction for large state spaces
- **Visualization**: Export tree structure for analysis and debugging
- **Optimization**: Minimize makespan and resource usage metrics

## Notes for Developers

1. **Memory Management**: The tree owns Tree_Node pointers. Call `deleteSubtree()` or `clearTree()` to free memory properly.
2. **State Consistency**: Ensure automaton and TS states remain synchronized throughout tree construction.
3. **Batch Semantics**: Use batch numbers to group related task phases (PRE, TRA, SUF) for coherent execution.
4. **Thread Safety**: Current implementation is single-threaded. Add synchronization if using in multi-threaded contexts.

## References

- Related to hierarchical task allocation for multi-robot systems
- Integrates with LTL formal specifications via Büchi automata
- Part of the HMRTA test suite generation framework
