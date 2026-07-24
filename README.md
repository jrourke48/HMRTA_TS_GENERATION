# HMRTA (Heterogenous Multi-Robot Task Allocation) with Multi-Capability Robots

## Project Overview

**HMRTA_TS_GENERATION** (Hierarchical Multi-Robot Task Allocation with Transition Systems Generation) is a comprehensive system for automatic task allocation to heterogeneous robot teams based on Linear Temporal Logic (LTL) specifications. This thesis research implements formal methods for specifying complex, temporally-constrained tasks and automatically distributing them among robots with different capabilities.

### Key Features

- **LTL-based Task Specification**: Define complex task requirements using Linear Temporal Logic formulas
- **Generalized Büchi Automata (GBA)**: Automatic conversion of LTL formulas to accepting automata
- **Transition System (TS) Modeling**: Discrete workspace representation with cost-based motion planning
- **Product Automaton Construction**: Synthesis of product spaces between TS and GBA for planning
- **Multi-Robot Planning**: Hierarchical decision tree-based allocation for heterogeneous robot teams
- **Spot Library Integration**: Leverages the Spot model checking library for automata generation and analysis
- **GBA Semantics Support**: Full support for both finite and infinite acceptance conditions with edge-indexed tracking

### System Architecture

```
┌─────────────────────────────────────────────────────────┐
│          LTL Task Specifications (Formulas)             │
└─────────────────┬───────────────────────────────────────┘
                  │ (Parse & Convert)
                  ▼
┌─────────────────────────────────────────────────────────┐
│     Generalized Büchi Automata (GBA) Generation        │
│              (via Spot Library)                         │
└─────────────────┬───────────────────────────────────────┘
                  │
        ┌─────────┴─────────┐
        ▼                   ▼
┌───────────────┐  ┌──────────────────┐
│ Transition    │  │ Büchi Automaton  │
│ System (TS)   │  │ States/Edges     │
└───────┬───────┘  └────────┬─────────┘
        │                   │
        └─────────┬─────────┘
                  ▼
┌─────────────────────────────────────────────────────────┐
│          Product Automaton (TS × GBA)                   │
│       Combined State Space with Acceptance Marks        │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│      Planning Decision Tree (Task Allocation)           │
│     Hierarchical Search with Pruning Strategies         │
└─────────────────┬───────────────────────────────────────┘
                  │
                  ▼
┌─────────────────────────────────────────────────────────┐
│    Allocated Task Sequences for Robot Team             │
│         (Paths satisfying LTL specifications)           │
└─────────────────────────────────────────────────────────┘
```

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

## Building the Project

### Prerequisites

- **C++20 Compiler** (g++ recommended)
  - Located at: `C:/Users/johnn/Cpp compiler/mingw64/bin/g++.exe`
  - Flags: `-std=c++20 -Wall -Wextra -g3`

- **Spot Model Checking Library** (v2.14.4.dev)
  - Location: `C:/Users/johnn/Spot Library/spot-2.14.4.dev/`
  - Headers: `spot/twa/twagraph.hh`, `spot/twaalgos/dot.hh`, `spot/tl/parse.hh`
  - Linking: `-lspot -lbddx`

### Build Commands

```bash
# Build all task allocation tests
make test-optimal-path

# Build individual components
make clean
make
```

### Build Tasks (VS Code)

1. **"build (script)"** - Uses `.vscode/build.bat` for active file compilation
2. **"g++ build active file (quoted)"** - Direct g++ compilation with Spot library linking

## Core Concepts

### Linear Temporal Logic (LTL)

LTL formulas specify temporal constraints on tasks:
- **F φ** (Finally): φ must eventually become true
- **G φ** (Globally): φ must always remain true
- **X φ** (Next): φ must be true in the next state
- **φ U ψ** (Until): φ holds until ψ becomes true
- **φ R ψ** (Release): ψ is released when φ becomes true (or always if φ never becomes true)

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

## Finite vs. Infinite Automata

### Finite Automata
- No globally (G) operators in LTL formula
- All tasks must eventually complete
- Aggressive pruning: removes duplicate (state, progress) pairs after first traversal
- Terminal states represent successful completion

**Example**: `F("pick_up") & F("deliver")`

### Infinite Automata
- Contains globally (G) operators for perpetual constraints
- Tasks require repeated/cyclic behavior
- Relaxed pruning: allows multiple traversals of same (state, progress) pairs
- Requires infinite edge cycles for acceptance

**Example**: `G(F("monitor")) & G(F("adjust"))`

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

## Documentation

For detailed information on specific components:

- [Automatons/README.md](Automatons/README.md) - Automata classes and architecture
- [Task Batch Planning Decision Tree/README.md](Task%20Batch%20Planning%20Decision%20Tree/README.md) - Planning algorithms and decision tree
- [AUTOMATONS_README.md](AUTOMATONS_README.md) - Comprehensive automata format documentation
- [SPOT_AUTOMATA_FORMATS.md](SPOT_AUTOMATA_FORMATS.md) - Spot library format specifications

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

## Performance Characteristics

- **Time Complexity**: Depends on TS size and GBA complexity; generally O(|TS| × |GBA| × branching_factor)
- **Space Complexity**: O(|TS| × |GBA|) for product automaton; additional memory for decision tree nodes
- **Pruning Impact**: Reduces search space significantly for finite automata; minimal impact for infinite automata (as intended)

## Known Limitations

1. **AND-connected APs**: Edges with multiple true APs connected by AND are excluded; decompose into sequential tasks
2. **Discrete Workspace**: Transition system must be discretized; continuous spaces require pre-processing
3. **Single Robot Focus**: Current implementation optimized for single-robot task allocation; multi-robot coordination in development
4. **Finite vs Infinite**: Different pruning strategies required; may need tuning for specific automata types

## Future Enhancements

- [ ] Multi-robot coordination with task dependencies
- [ ] Continuous state space support with approximation
- [ ] Dynamic environment adaptation with replanning
- [ ] GPU-accelerated acceptance set tracking
- [ ] Weighted propositions for probabilistic planning
- [ ] Reactive synthesis for real-time task assignment

## Contributing

This is a thesis research project. For questions or contributions, please contact the project maintainer.

## References

- Spot Library: [https://spot.lrde.epita.fr/](https://spot.lrde.epita.fr/)
- LTL Specification: [https://en.wikipedia.org/wiki/Linear_temporal_logic](https://en.wikipedia.org/wiki/Linear_temporal_logic)
- Büchi Automata: [https://en.wikipedia.org/wiki/Büchi_automaton](https://en.wikipedia.org/wiki/B%C3%BCchi_automaton)
- Formal Methods in Robotics: Various ICRA, RSS, and IROS proceedings

## License

Internal Research Project - Cal Poly

---

**Last Updated**: 2026-07-21  
**Project Status**: Active Development - GBA Semantics Implementation Phase
