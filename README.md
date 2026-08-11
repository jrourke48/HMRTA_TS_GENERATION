# HMRTA (Heterogenous Multi-Robot Task Allocation) with Multi-Capability Robots

## Project Overview

**HMRTA with Multi-Capability Robots** (Heterogenous Multi-Robot Task Allocation with Multi-Capability Robots) This repository contains the source code and guide to use a comprehensive framework for automatic task allocation to heterogeneous robot teams. The task is encoded into a Linear Temporal Logic (LTL) formula which can be used to create a Non-Deterministic Buchi Automaton (NBA): a form of Directed Graph that shows paths that are consistent with the task specification (LTL formula). Previous research has This thesis research implements formal methods for specifying complex, temporally-constrained tasks and automatically distributing them among robots with different capabilities.

---

## Project Requirements

This document outlines the dependencies required for the HMRTA with Multi-Capability Robots project.

### Spot Library
- **Version**: 2.14.4.dev
- **Location**: `C:/Users/johnn/Spot Library/spot-2.14.4.dev`
- **Libraries**: libspot, libbddx
- **Windows Build Dependencies**:
  - g++ (C++20 or later)
  - MinGW-w64 compiler
  - BDD library (buddy)

#### Building Spot on Linux/Ubuntu (for reference):
```bash
# Install build dependencies
sudo apt update
sudo apt install -y \
  build-essential git pkg-config \
  autoconf automake libtool \
  flex bison \
  libbdd-dev libgmp-dev libboost-all-dev \
  python3-dev swig

# Clone and build
git clone --recurse-submodules https://gitlab.lre.epita.fr/spot/spot.git
cd spot
autoreconf -vfi
./configure --prefix=/usr/local
make -j$(nproc)
sudo make install
sudo ldconfig
```

**Note**: The development version has assertions and debugging code enabled. Use `--disable-devel` flag if benchmarking.

### dstar (D* Lite Algorithm)
- **Location**: `dstar/`
- **Build Tool**: Make
- **C++ Compiler**: g++
- **Graphics Dependencies**:
  - **macOS**: OpenGL framework, GLUT framework
  - **Linux**: libGL, libGLU, libglut
  - **Windows**: OpenGL (typically included), GLUT libraries

### Compiler & Tools
- **C++ Standard**: C++20
- **Compiler**: g++ (MinGW on Windows)
- **Build System**: Make/CMake
- **Compiler Flags**: `-std=c++20 -Wall -Wextra -g3`
- **C++ Compiler Location**: `C:/Users/johnn/Cpp compiler/mingw64/bin/g++.exe`

### Visualization (Required)
Visualization is a core feature of this project and is required for full functionality.

#### raylib
- **Purpose**: Real-time graphics and simulation visualization
- **Installation**:
  - **Windows**: Download from [raylib.com](https://www.raylib.com/) or use vcpkg: `vcpkg install raylib`
  - **Ubuntu/Linux**: `sudo apt install libraylib-dev` or build from source
  - **macOS**: `brew install raylib`
- **Reference**: [Working on GNU/Linux](https://github.com/raysan5/raylib/wiki/Working-on-GNU-Linux)

#### Additional Visualization Tools
- **graphviz** (for automata and formula visualization)
  - Ubuntu/Linux: `sudo apt install graphviz`
  - macOS: `brew install graphviz`
  - Windows: Download from [graphviz.org](https://graphviz.org/)
- **xdot** (for interactive dot file viewer)
  - Ubuntu/Linux: `sudo apt install xdot`
  - macOS: `brew install xdot`

#### Visualization Usage
```bash
# View LTL formula as automaton
ltl2tgba -f "GFa & GFb" --dot | xdot -

# Open PDF example
xdg-open automaton.pdf

# Build and visualize gridworld (raylib)
./test 2> aut.dot
xdot aut.dot
```

### Robotics_Research
- **Location**: `Robotics_Research/`
- **Dependencies**: Inherits Spot library dependencies

---

## Repository Breakdown
 - **Transition_Systems**: Discrete workspace abstractions and state-space representations for formal planning. Provides GeneralTransitionSystem (generic AP-based states) and GridWorldTransitionSystem (2D grid coordinates with atomic propositions). Bridges physical workspace dynamics with symbolic planning framework. See [Transition_Systems/README.md](Transition_Systems/README.md) for detailed architecture.
 - **Automatons**: Polymorphic graph-based automata framework implementing node-edge hierarchies. Base Automaton class with three specialized subclasses: BüchiAutomaton (Encodes Task Specification), ProductAutomaton (synchronous product), and TransitionSystem (workspace discretization). Core synthesis component of MCTB-PDT. See [Automatons/README.md](Automatons/README.md) for detailed architecture.
- **MCTB-PDT (Multi-Capability Task Batch Planning Decision Tree)**: Core thesis contribution addressing state-space explosion in product automaton synthesis (TS × NBA). Implements hierarchical tree-based search encoding NBA states and TS configurations in recursive hierarchy. By growing from root to leaf nodes with memoized pruning strategies (finiteness-differentiated), MCTB-PDT discovers optimal task allocations while exploring only a fraction of the full product automaton, enabling scalable synthesis as robot count and state space complexity increase. See [Task Batch Planning Decision Tree/README.md](Task%20Batch%20Planning%20Decision%20Tree/README.md) for implementation details.

---

## Required Background Knowledge

To understand this thesis project, you should be familiar with:

- **Linear Temporal Logic (LTL)**: Temporal logic formulas (F, G, X, U, R operators) for specifying temporal constraints
- **Büchi Automata**: Finite automata accepting infinite words; generalized Büchi automata (GBA) with acceptance set semantics
- **Transition Systems**: Discrete state-space abstractions with states, transitions, and atomic propositions
- **Product Automaton Construction**: Combining transition systems with Büchi automata into synchronized product spaces (TS × GBA)
- **Graph Algorithms**: Reachability, connectivity, and path-finding in directed graphs
- **Formal Methods**: Model checking, LTL synthesis, and automata theory fundamentals

---

## Directory Structure

### Core Components

- **[Automatons/](Automatons/)** - Automata classes and Spot integration
  - Base automaton hierarchy (Automaton, BuchiAutomaton, TS, ProductAutomaton)
  - DOT file parsing with GBA acceptance set tracking
  - Edge-indexed AP collection with filtering

- **[Task Batch Planning Decision Tree/](Task%20Batch%20Planning%20Decision%20Tree/)** - Main planning algorithms
  - Environment modeling and workspace representation
  - LTL formula parsing and semantic representation
  - Multi-robot system and robot capability management
  - Planning decision tree structure with acceptance tracking
  - Task allocation algorithms with pruning strategies
  - Comprehensive test suite for validation

### Supporting Directories

- **[Transition_Systems/](Transition_Systems/)** - General transition system implementations
  - Base TS implementation with generic properties
  - Grid world-based transition systems for spatial domains

- **[dstar/](dstar/)** - D* Lite path planning algorithm
  - Optimal replanning for dynamic environments
  - Cost-based trajectory computation

- **[Learning_Spot/](Learning_Spot/)** - Spot library learning and examples
  - LTL parsing and printing examples
  - Formula relabeling demonstrations
  - Automata equivalence testing

- **[Robotics_Research/](Robotics_Research/)** - Research materials and references
  - LTL formula resources
  - Academic papers and documentation
  - Spot library reference materials

---

## Building the Project

### Prerequisites

- **C++20 Compiler** (g++ recommended)
- **Spot Model Checking Library** (v2.14.4.dev)

### Build Commands

```bash
# Build all task allocation tests
make test-optimal-path

# Build the gridworld visualization tests
make vis

# Build individual components
make clean
make
```

### Build Tasks (VS Code)

1. **"build (script)"** - Uses `.vscode/build.bat` for active file compilation
2. **"g++ build active file (quoted)"** - Direct g++ compilation with Spot library linking

### Makefile Options

The makefile has `sim` and `test` flags:
- **test**: Verifies Spot library is working correctly
- **sim**: Grid world simulation using raylib visualization

---

## Core Concepts

### Linear Temporal Logic (LTL)

LTL formulas specify temporal constraints on tasks:
- **F φ** (Finally): φ must eventually become true
- **G φ** (Globally): φ must always remain true
- **X φ** (Next): φ must be true in the next state
- **φ U ψ** (Until): φ holds until ψ becomes true
- **φ R ψ** (Release): ψ is released when φ becomes true (or always if φ never becomes true)
- **φ & ψ**: Conjunction (AND)

**Example Task**: "Visit location A, then visit location B, then stay at B forever"
```
F(at_A) & F(at_B) & G(at_B | moving_to_B)
```

### Büchi Automata

A Büchi automaton is a finite automaton that accepts infinite words satisfying specific acceptance conditions. This project uses **Generalized Büchi Automata (GBA)** where:
- **States** represent control locations in task execution
- **Transitions** labeled with propositions (e.g., "at_location_A & holding_object")
- **Acceptance Marks** track which acceptance sets each edge satisfies
- **Progress Phases** (PRE → TRA → SUF → OTH) manage acceptance set traversal

### Transition Systems

A discrete abstraction of the robot workspace:
- **States** represent workspace regions or configurations
- **Edges** represent feasible transitions between regions
- **Atomic Propositions** label states (e.g., "location_safe", "near_obstacle")
- **Costs** associated with transitions for optimization

### Product Automaton

Combines TS and GBA into a single state space:
- **States** = (TS_state, Buchi_state) pairs
- **Accepting Paths** = paths in product automaton satisfying both TS motion and GBA acceptance conditions

### Planning Decision Tree

Hierarchical search structure for finding valid task allocations:
- **Root** = initial state with no tasks assigned
- **Nodes** = (task, robot, state) tuples representing partial allocations
- **Branches** = alternative allocations or state transitions
- **Leaves** = complete task assignments with success/failure status
- **Pruning** removes duplicate (state, progress) pairs and infeasible branches

### Finite vs. Infinite Automata

#### Finite Automata
- No globally (G) operators in LTL formula
- All tasks must eventually complete
- Aggressive pruning: removes duplicate (state, progress) pairs after first traversal
- Terminal states represent successful completion

**Example**: `F("pick_up") & F("deliver")`

#### Infinite Automata
- Contains globally (G) operators for perpetual constraints
- Tasks require repeated/cyclic behavior
- Relaxed pruning: allows multiple traversals of same (state, progress) pairs
- Requires infinite edge cycles for acceptance

**Example**: `G(F("monitor")) & G(F("adjust"))`

---

## SPOT Automata Format Differences: Standard vs Generalized Büchi

### Overview

SPOT uses fundamentally different representations for finite and infinite LTL formulas:
- **Finite formulas** (no `G` operator): Standard Büchi Automata with **state-based acceptance**
- **Infinite formulas** (with `G` operator): Generalized Büchi Automata with **transition-based acceptance**

This difference impacts how we detect accepting paths and increment progress through the planning tree.

### Standard Büchi Automata (Finite Tests)

#### Definition
- **Acceptance**: Achieved when visiting an **accepting STATE** at least once
- **Representation**: Accepting states marked in DOT with `peripheries=2` (double circle)
- **Example DOT Label**: (No "Inf" prefix)
  ```
  label="[Büchi]"
  ```

#### Visual Representation
```
State 0 → State 1 → State 2 (accepting: peripheries=2)
                       ↑
                       └─────────┘
```

#### Algorithm Implication
```cpp
if (isAcceptingState(stateId)) {
    incrementProgress();  // PRE → TRA
}
```

#### When Accepting
- Visit any accepting state once → Path satisfies formula (finite)
- Multiple cycles possible but not required

### Generalized Büchi Automata (Infinite Tests)

#### Definition
- **Acceptance**: Achieved when visiting ALL required **acceptance SETS infinitely often**
- **Representation**: Acceptance sets marked on **edges** in DOT with `{0}`, `{1}`, `{2}`, etc.
- **Example DOT Label**: 
  ```
  label="Inf(0)&Inf(1)&Inf(2)\n[gen. Büchi 3]"
  ```
  This means: Must visit transitions in set {0} infinitely AND set {1} infinitely AND set {2} infinitely

#### Visual Representation
```
Edge 0→1: label="p0 & p1\n{0}"        (belongs to acceptance set {0})
Edge 1→2: label="p2 & p3\n{0,1}"      (belongs to sets {0} AND {1})
Edge 2→0: label="p4\n{2}"              (belongs to acceptance set {2})

Required: Visit {0} infinitely, {1} infinitely, AND {2} infinitely
```

#### Algorithm Implication
```cpp
// Track which acceptance sets have been visited
node.visitedSets = {0, 1, 2}  // All required sets visited once

if (allRequiredSetsVisited(node.visitedSets)) {
    incrementProgress();  // PRE → TRA on first complete cycle
    node.visitedSets.clear();
    
    if (allRequiredSetsVisited(node.visitedSets)) {
        acceptingPath();  // TRA complete on second cycle
    }
}
```

#### When Accepting
- Visit transitions containing {0}, {1}, {2} once each → Progress to TRA
- Visit transitions containing {0}, {1}, {2} once more → Path is accepting

### Key Differences Summary

| Feature | Standard Büchi | Generalized Büchi |
|---------|---|---|
| **Acceptance Mechanism** | Visiting accepting state | Visiting all acceptance sets infinitely |
| **Marked On** | Nodes (states) | Edges (transitions) |
| **DOT Marker** | `peripheries=2` on nodes | `{0}`, `{1}`, `{2}` on edges |
| **DOT Label** | No "Inf" prefix | `Inf(0)&Inf(1)&Inf(2)` format |
| **Cycles Required** | 1+ visit to accepting state | Multiple cycles to visit all sets |
| **Tracking** | Node IDs in `acceptingStates` | Edge labels + set membership |
| **Use Case** | Finite task sequences | Infinite patrol/monitoring tasks |

### SPOT Implementation Details

#### How to Detect Format
```cpp
// In DOT generation by SPOT:

// Standard Büchi - finite formula
label="[Büchi]"
label="[Büchi 1]"

// Generalized Büchi - infinite formula  
label="Inf(0)&Inf(1)&Inf(2)\n[gen. Büchi 3]"
label="Inf(0)&Inf(1)\n[gen. Büchi 2]"
```

The presence of `Inf(` in the label indicates generalized format.

#### Parsing Required Sets
```
From: label="Inf(0)&Inf(1)&Inf(2)\n[gen. Büchi 3]"
Extract: {0, 1, 2}

From: label="[Büchi]"
Extract: {} (standard, no sets)
```

### Algorithm Updates Required

#### Current Progress Incrementation (Standard Büchi)
```cpp
// TaskAllocationAlgorithms.cpp - Line 309-320
if (nba->isAccepting(newNode->getAutomatonState()->getId())) {
    // Reached accepting state once
    newNode->setProgress(TRA);  // Mark as satisfying
}

// Path is accepting when reaching TRA state
```

#### New Progress Incrementation (Generalized Büchi)
```cpp
// For each edge transition, check acceptance sets
std::set<int> edgeSets = extractAcceptanceSets(edge.getLabel());
node->addVisitedSets(edgeSets);

if (node->hasAllRequiredSets()) {
    // Completed first cycle through all sets
    if (node->progress == PRE) {
        node->setProgress(TRA);
        node->clearVisitedSets();  // Reset for second cycle
    } else if (node->progress == TRA) {
        // Completed second cycle through all sets
        acceptingPath = true;
        break;
    }
}
```

### Data Structure Changes Needed

#### BuchiAutomaton Class
```cpp
class BuchiAutomaton {
    // Existing
    bool isInfinite;  // Cached from DOT label check
    
    // New for GBA support
    std::set<int> requiredAcceptanceSets;  // {0}, {1}, {2} from Inf()
    
    // New method
    std::set<int> getRequiredAcceptanceSets() const;
    // Parses "Inf(0)&Inf(1)&Inf(2)" to extract {0, 1, 2}
}
```

#### Tree_Node Class  
```cpp
class Tree_Node {
    // Existing
    TASK_PROGRESS progress;
    
    // New for GBA support (only used when automaton is infinite)
    std::set<int> visitedAcceptanceSets;  // Sets visited in current cycle
    
    // New methods
    void addVisitedSet(int setId);
    bool hasAllRequiredSets(const std::set<int>& required) const;
    void clearVisitedSets();
}
```

### Examples by Test Type

#### Example 1: Finite Test (Standard Büchi)
**Formula**: `(F"p0" & F"p1")`  
**DOT Label**: `[Büchi]`  
**Algorithm**: Reach any accepting state → mark PRE, reach again → mark TRA → done

#### Example 2: Infinite Test (Generalized Büchi)  
**Formula**: `G(F"p0" & F"p1")`  
**DOT Label**: `Inf(0)&Inf(1)`  
**Required Sets**: {0, 1}  
**Algorithm**:
1. Take transition with {0} → visitedSets = {0}
2. Take transition with {1} → visitedSets = {0, 1}
3. All required sets visited → Progress to TRA, clear visitedSets
4. Take transition with {0} again → visitedSets = {0}
5. Take transition with {1} again → visitedSets = {0, 1}
6. All required sets visited again → Path is ACCEPTING

### Integration Strategy

1. ✅ **Phase 1** (Complete): Detect automaton type via `isInfinite` field
2. **Phase 2** (Next): Extract required acceptance sets from DOT label
3. **Phase 3** (Next): Modify Tree_Node to track visitedAcceptanceSets
4. **Phase 4** (Next): Update TaskAllocationAlgorithms progress logic
5. **Phase 5** (Next): Test with infinite test suite

---

## Key Algorithms

### Edge-Indexed AP Collection with GBA Support

Extracts atomic propositions from automaton edge labels while respecting GBA semantics:
- **Format**: "p0 & p1:{0,1}" (APs separated from acceptance marks by `:`)
- **Filtering**: Edges with 2+ true APs connected by AND are excluded (incompatible with sequential execution)
- **Acceptance Marks**: Parsed from {x,y,z} format and tracked per edge
- **Output**: Indexed collections: `edgeAPIds[edge_index] = {AP_IDs}`, `edgeAcceptanceSets[edge_index] = {set_IDs}`

### Progress Tracking with Acceptance Sets

Manages task completion phases in the context of GBA acceptance:
- **Initial**: All required acceptance sets in `acceptingSets` vector
- **Traversal**: Each edge removes satisfied acceptance sets from the vector
- **Progress Increment**: When `acceptingSets` becomes empty, advance to next phase (PRE→TRA→SUF→OTH)
- **Infinite Automata**: Cycles back to PRE when all sets satisfied

### Pruning Strategy

**For Finite Automata**:
- Rule 1: Add OTH nodes to traversedTree
- Rule 2: Remove duplicate (state, progress) pairs
- Rule 3: Keep only minimum-cost node per (state, progress)

**For Infinite Automata**:
- Rule 1: Add OTH nodes to traversedTree (no removal)
- Rules 2-3: Skipped (allow revisits for cycle support)
- Visit-count pruning: Limit revisits to MAX_REVISITS to prevent infinite growth

---

## Main API

### Task Allocation Algorithm

```cpp
// Main algorithm entry point
void TaskAllocationAlgorithms::unrelatedTaskSearch(
    const Robot* robot,
    Environment* env,
    const BuchiAutomaton* nba,
    vector<uint16_t> acceptingSets  // Current acceptance sets to visit
);

// Edge extraction with GBA support
void collectAPsFromEdgesByIndexWithAcceptance(
    const vector<string>& edges,
    vector<set<uint16_t>>& outEdgeAPIds,       // Output: AP IDs per edge
    vector<set<uint16_t>>& outEdgeAcceptanceSets // Output: acceptance marks per edge
);

// Subtree pruning with finite/infinite differentiation
void pruneSubtree(
    Tree_Node* root,
    Tree_Node* nodeToRemove,
    TraversedTree* traversedTree
);
```

### Automaton Generation

```cpp
// Parse LTL and generate Büchi automaton
BuchiAutomaton* ba = new BuchiAutomaton();
ba->generateFromLTLString("F(\"p0\") & F(\"p1\")");
bool isInfinite = ba->isInfinite();
```

### Environment Setup

```cpp
// Create environment with TS and grid world
Environment* env = new Environment(ts, gridWorld);
uint32_t initialState = env->getInitialState();
vector<uint32_t> successors = env->getSuccessorStates(initialState);
```

---

## Testing

### Test Suite

The project includes comprehensive tests in [Task Batch Planning Decision Tree/Testing/](Task%20Batch%20Planning%20Decision%20Tree/Testing/):

- **Optimal_Path_Test.cpp** - Main test suite for finite and infinite automata
- **TestTaskAllocationAlgorithms.cpp** - Unit tests for allocation algorithms
- **TestMultiRobotSystem.cpp** - Multi-robot system functionality tests
- **TestPlanningTree.cpp** - Decision tree structure tests
- **TestIntegration.cpp** - End-to-end integration tests

### Running Tests

```bash
# Run optimal path tests
./output/OptimalPathTest

# Run specific test
./output/TestTaskAllocation
```

---

## File Formats

### DOT Format Output

Automata and product spaces are exported in DOT format for visualization:
- **Node format**: `state_id [label="state_description"]`
- **Edge format**: `state_id -> next_id [label="p0 & p1:{0,1}"]`
- **GBA marks**: Acceptance sets appended with `:` separator: `:{acceptance_set_ids}`

### Spot Automaton Labels

- **Simple**: "p0" (single proposition)
- **Complex**: "p0 & p1 | p2" (boolean expressions)
- **With Acceptance**: "p0 & p1:{0,1,2}" (propositions with GBA acceptance marks)

---

## Usage Example

```cpp
// 1. Create environment
GridWorld* grid = new GridWorld(100, 100, 50); // 100x100 grid, 50x50 cells
TS* ts = new TS();
ts->addState(0); // initial state
ts->addState(1);
ts->addEdge(0, 1, "at_location_A");
Environment* env = new Environment(ts, grid);

// 2. Define task as LTL formula
string taskFormula = "F(\"location_A\") & F(\"location_B\")";

// 3. Generate Büchi automaton
BuchiAutomaton* gba = new BuchiAutomaton();
gba->generateFromLTLString(taskFormula);

// 4. Run task allocation
TaskAllocationAlgorithms allocator;
Robot* robot = new Robot(0, "Robot_1");
vector<uint16_t> initialAcceptingSets = gba->getAcceptingStates();
allocator.unrelatedTaskSearch(robot, env, gba, initialAcceptingSets);

// 5. Extract and execute solution path
// (Implementation depends on specific tree structure)
```

---

## Performance Characteristics

- **Time Complexity**: Depends on TS size and GBA complexity; generally O(|TS| × |GBA| × branching_factor)
- **Space Complexity**: O(|TS| × |GBA|) for product automaton; additional memory for decision tree nodes
- **Pruning Impact**: Reduces search space significantly for finite automata; minimal impact for infinite automata (as intended)

---

## Known Limitations

1. **AND-connected APs**: Edges with multiple true APs connected by AND are excluded; decompose into sequential tasks
2. **Discrete Workspace**: Transition system must be discretized; continuous spaces require pre-processing
3. **Single Robot Focus**: Current implementation optimized for single-robot task allocation; multi-robot coordination in development
4. **Finite vs Infinite**: Different pruning strategies required; may need tuning for specific automata types

---

## Future Enhancements

- [ ] Multi-robot coordination with task dependencies
- [ ] Continuous state space support with approximation
- [ ] Dynamic environment adaptation with replanning
- [ ] GPU-accelerated acceptance set tracking
- [ ] Weighted propositions for probabilistic planning
- [ ] Reactive synthesis for real-time task assignment

---

## Contributing

This is a thesis research project. For questions or contributions, please contact the project maintainer.

---

## References

- Spot Library: [https://spot.lrde.epita.fr/](https://spot.lrde.epita.fr/)
- LTL Specification: [https://en.wikipedia.org/wiki/Linear_temporal_logic](https://en.wikipedia.org/wiki/Linear_temporal_logic)
- Büchi Automata: [https://en.wikipedia.org/wiki/Büchi_automaton](https://en.wikipedia.org/wiki/B%C3%BCchi_automaton)
- Formal Methods in Robotics: Various ICRA, RSS, and IROS proceedings
- Raylib Graphics: [https://www.raylib.com/](https://www.raylib.com/)

---

## License

Internal Research Project - Cal Poly

---

**Last Updated**: 2026-08-10  
**Project Status**: Active Development - Algorithm Testing Phase
