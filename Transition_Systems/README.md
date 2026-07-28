# Transition Systems Module

Discrete workspace abstractions and state-space representations for formal task planning.

## Overview

The Transition Systems module provides implementations for discretizing continuous robot environments into abstract state spaces suitable for formal synthesis. It bridges the gap between physical workspace dynamics and the symbolic automata-based planning framework.

**Core Responsibilities**:
1. **Workspace Discretization** - Convert continuous/complex environments to discrete state spaces
2. **State Representation** - Model workspace regions as states with atomic propositions
3. **Transition Modeling** - Capture feasible robot motions as transitions with costs
4. **Query Interface** - Provide efficient adjacency and reachability queries for planning algorithms
5. **Multiple Abstraction Levels** - Support generic and grid-world-specific transition systems

## Architecture

```
TransitionSystem (Abstract Concept)
├── GeneralTransitionSystem (Generic Implementation)
│   ├── State (Atomic proposition-based)
│   ├── Transition (Edges with costs)
│   └── Methods: addState, addAdjacency, getReachable, etc.
│
└── GridWorldTransitionSystem (2D Grid Implementation)
    ├── State (Grid coordinates + atomic propositions)
    ├── Transition (Movement actions: UP, DOWN, LEFT, RIGHT)
    ├── AtomicProposition (Position labels)
    └── Methods: successors, StateTransition, label, etc.
```

## Core Classes

### 1. GeneralTransitionSystem

**File**: `GeneralTransitionSystem.h/cpp`

A base class providing a generic interface for transition systems with arbitrary state representations.

#### Key Concepts

- **States** are identified by unique uint16_t IDs corresponding to atomic propositions
- **Transitions** represent edges between states with associated costs
- Designed for flexibility: support any discretization scheme

#### State Class

```cpp
class State {
private:
    uint16_t AP;                          // Atomic proposition ID (unique state identifier)
    std::vector<Transition> adjecency;    // Outgoing transitions
    uint16_t numadj;                      // Number of adjacent states
    std::string Name;                     // Optional state name/label

public:
    // Accessors
    uint16_t getAP() const;               // Get AP ID
    const std::vector<Transition>& getAdjacency() const;
    uint16_t getNumAdjacency() const;
    const std::string& getName() const;
    
    // Mutation
    void addTransition(const Transition& t);
    bool isAdjacent(uint16_t dstId) const;
};
```

**Usage**:
```cpp
// Create states
State* regionA = new State(0, "RegionA");
State* regionB = new State(1, "RegionB");

// Add transition
Transition t1(regionB, 5);  // cost = 5
regionA->addTransition(t1);

// Query adjacency
if (regionA->isAdjacent(1)) {
    std::cout << "Can reach RegionB\n";
}
```

#### Transition Class

```cpp
class Transition {
public:
    State* next;          // Destination state
    uint16_t cost;        // Transition cost (default: 1)
    
    // Constructors
    Transition(State* nextState = nullptr);
    Transition(State* nextState, uint16_t transitionCost);
};
```

#### GeneralTransitionSystem Methods

**State Management**:
```cpp
void addState(State* state);                    // Add state to system
void removeState(uint16_t stateId);             // Remove state by ID
State* getState(uint16_t stateId) const;        // Retrieve state by AP ID
State* findStateByAP(uint16_t ap) const;        // Find state by atomic proposition
bool hasState(uint16_t stateId) const;          // Check state existence
const std::vector<State*>& getAllStates() const; // Get all states
```

**Adjacency/Edge Management**:
```cpp
void addAdjecency(uint16_t srcId, uint16_t dstId, uint16_t cost = 1);
bool isAdjacent(uint16_t srcId, uint16_t dstId) const;
std::vector<Transition> getTransitionsFrom(uint16_t stateId) const;
std::vector<Transition> getTransitionsTo(uint16_t stateId) const;
std::vector<State*> getAdjacency(uint16_t stateId) const;
std::vector<uint16_t> getAdjacencyIds(uint16_t stateId) const;
```

**Cost Management**:
```cpp
uint16_t getTransitionCost(uint16_t srcId, uint16_t dstId) const;
void setTransitionCost(uint16_t srcId, uint16_t dstId, uint16_t cost);
```

**Query Methods**:
```cpp
uint16_t getNumStates() const;
uint16_t getNumEdges() const;
std::vector<uint16_t> getReachableStates(uint16_t startStateId) const;
bool isConnected() const;
```

**Initial State Management**:
```cpp
void addInitialState(uint16_t stateId);
void removeInitialState(uint16_t stateId);
const std::vector<uint16_t>& getInitialStates() const;
bool isInitialState(uint16_t stateId) const;
void clearInitialStates();
```

**Utility**:
```cpp
void clear();
void printStates() const;
void printTransitions() const;
```

#### Usage Example

```cpp
#include "Transition_Systems/GeneralTransitionSystem.h"

// Create transition system
GeneralTransitionSystem* ts = new GeneralTransitionSystem();

// Add states
State* s0 = new State(0, "Initial");
State* s1 = new State(1, "RegionA");
State* s2 = new State(2, "RegionB");
ts->addState(s0);
ts->addState(s1);
ts->addState(s2);

// Add transitions (workspace connectivity)
ts->addAdjecency(0, 1, 5);   // s0 → s1 with cost 5
ts->addAdjecency(1, 2, 3);   // s1 → s2 with cost 3
ts->addAdjecency(2, 1, 3);   // s2 → s1 with cost 3

// Set initial state
ts->addInitialState(0);

// Query
std::cout << "States: " << ts->getNumStates() << "\n";
std::cout << "Is connected: " << (ts->isConnected() ? "yes" : "no") << "\n";

// Find successors
std::vector<uint16_t> reachable = ts->getReachableStates(0);
```

---

### 2. GridWorldTransitionSystem (TransitionSystem)

**File**: `GridWorldTransitionSystem.h/cpp`

A specialized transition system for 2D grid-based environments (common in robotics).

#### Key Concepts

- **Grid Coordinates**: States identified by (x, y) positions in a 2D grid
- **Atomic Propositions**: One AP per cell, encoding position information
- **Actions**: Four primary movement actions: UP, DOWN, LEFT, RIGHT
- **Cost-Based Motion**: Transitions have associated costs (e.g., distance, energy)
- **Spot Integration**: States and propositions compatible with Spot automata

#### Action Enum

```cpp
enum class Action : uint8_t { UP, DOWN, LEFT, RIGHT };
```

#### AtomicProposition Class

```cpp
class AtomicProposition {
private:
    long id;           // Unique identifier (often cell ID)
    std::string name;  // Human-readable name (e.g., "cell_5")

public:
    AtomicProposition(long i, std::string n) : id(i), name(n) {}
    
    std::string getName() const;
    long getId() const;
    void setId(long new_id);
};
```

#### State Class (Grid Version)

```cpp
class State {
public:
    // Set of atomic propositions true in this state
    std::unordered_set<AtomicProposition, AtomicProposition_Hash> props;
    
    bool operator==(const State& other) const;
};
```

**Properties**:
- Each state contains a set of atomic propositions (typically 1 per state: current cell)
- Can represent multi-cell propositions if needed
- Hashable for use in unordered containers

#### Transition Class (Grid Version)

```cpp
class Transition {
public:
    State next;        // Destination state
    Action action;     // Movement action
    double cost;       // Transition cost (default: 1.0)
};
```

#### TransitionSystem Methods

**Constructors**:
```cpp
// Create grid with default initial state at (0, 0)
TransitionSystem(int width, int height);

// Create grid with specified initial states
TransitionSystem(int width, int height, 
                 const std::unordered_set<State, StateHash>& init_states);
```

**Coordinate Utilities**:
```cpp
int cellId(int x, int y) const;      // Convert (x,y) to flat cell ID
int cellX(int id) const;              // Extract x from cell ID
int cellY(int id) const;              // Extract y from cell ID
```

**State Operations**:
```cpp
State createState(int x, int y) const;     // Create state from coordinates
int getX(const State& s) const;             // Get x coordinate from state
int getY(const State& s) const;             // Get y coordinate from state
bool isValid(const State& s) const;         // Check if state is in bounds
State getCurrentState() const;              // Get current robot position
void StateTransition(const Action& a);      // Update current state via action
```

**Successor Generation**:
```cpp
// Generate all valid successor states from given state
std::vector<Transition> successors(const State& s) const;

// Display successors for debugging
static void display_successors(const std::vector<Transition>& succs);
```

**Query Methods**:
```cpp
int32_t numAPs() const;             // Number of atomic propositions
int32_t numStates() const;          // Total number of states
std::vector<std::string> getAPNames() const;  // Get sorted AP names
std::unordered_set<std::string> getLabels(const State& s) const;
```

**State-ID Conversion**:
```cpp
// Convert between state and integer representation
int stateToId(const State& s) const;
State idToState(int id) const;
```

**Labeling for Automata**:
```cpp
// Generate label string for state (position encoded)
std::string label(const State& s);

// Action string conversion
static std::string actionToString(Action a);
```

**Export**:
```cpp
// Save automaton to DOT format for visualization
static void exportAutomatonDot(const spot::twa_graph_ptr& aut, 
                               const std::string& filename);
```

#### Grid Coordinate System

- **Origin**: (0, 0) at top-left
- **X-axis**: increases right
- **Y-axis**: increases down
- **Cell ID**: `y * grid_width + x`

#### Usage Example

```cpp
#include "Transition_Systems/GridWorldTransitionSystem.h"

// Create 10×10 grid
TransitionSystem grid(10, 10);  // Initial state: (0,0)

// Query grid properties
std::cout << "Grid size: " << grid.grid_width << " × " << grid.grid_height << "\n";
std::cout << "States: " << grid.numStates() << "\n";
std::cout << "Atomic propositions: " << grid.numAPs() << "\n";

// Get successors from current state
State current = grid.getCurrentState();
std::vector<Transition> successors = grid.successors(current);

std::cout << "Successors from (" << grid.getX(current) << "," 
          << grid.getY(current) << "):\n";
for (const auto& trans : successors) {
    std::cout << "  Action: " << grid.actionToString(trans.action)
              << ", Cost: " << trans.cost << "\n";
}

// Move robot
grid.StateTransition(Action::RIGHT);  // Move right
std::cout << "After move: (" << grid.getX(grid.getCurrentState()) << "," 
          << grid.getY(grid.getCurrentState()) << ")\n";

// Get labels (true APs) at current state
auto labels = grid.getLabels(grid.getCurrentState());
for (const auto& label : labels) {
    std::cout << "At position: " << label << "\n";
}
```

---

## Integration with Planning Framework

### Connection to Automatons Module

Transition Systems are used to construct **Product Automata** with Büchi automata:

```cpp
// 1. Create TS representation
GeneralTransitionSystem* ts = new GeneralTransitionSystem();
// ... add states and transitions ...

// 2. Create Büchi automaton from LTL
BuchiAutomaton* gba = new BuchiAutomaton();
gba->generateFromLTLString("F(p0) & F(p1)");

// 3. Build product automaton
ProductAutomaton* product = new ProductAutomaton(gba);
// ... construct product state space ...
```

The product automaton combines:
- **TS Component**: Feasible robot motions
- **Büchi Component**: Task progress tracking
- **Product States**: (ts_state, buchi_state) pairs

### Connection to Task Allocation

Transition systems provide the workspace model for planning:
- **States** map to regions/configurations
- **Transitions** map to feasible motions
- **Costs** guide optimal task allocation
- **Reachability** constrains valid paths

---

## Comparison: GeneralTransitionSystem vs GridWorldTransitionSystem

| Feature | General TS | Grid TS |
|---------|-----------|---------|
| **State Representation** | Arbitrary (AP-based) | 2D Coordinates + AP |
| **Transitions** | Generic edges | 4-directional movement |
| **Cost Type** | uint16_t | double |
| **Use Case** | Any discretization | Grid-based environments |
| **Spot Integration** | Manual mapping | Built-in |
| **Complexity** | O(states × edges) | O(width × height) |

**When to use GeneralTransitionSystem**:
- Non-grid workspaces (e.g., graphs, trees)
- Irregular discretizations
- Custom state representations

**When to use GridWorldTransitionSystem**:
- 2D grid environments
- Regular discretization
- Robot navigation simulations

---

## Complete Example: Multi-Robot Task Allocation

```cpp
#include "Transition_Systems/GridWorldTransitionSystem.h"
#include "Automatons/BuchiAutomaton.h"
#include "Automatons/ProductAutomaton.h"

int main() {
    // ========== 1. Create Grid World ==========
    TransitionSystem grid(20, 20);  // 20×20 grid
    std::cout << "Grid: " << grid.numStates() << " states\n";
    
    // ========== 2. Define Task as LTL Formula ==========
    std::string taskFormula = "F(p5) & F(p50)";  // Visit cell 5, then cell 50
    
    // ========== 3. Generate Büchi Automaton ==========
    BuchiAutomaton* gba = new BuchiAutomaton();
    gba->generateFromLTLString(taskFormula);
    std::cout << "NBA: " << gba->getnumStates() << " states\n";
    
    // ========== 4. Query Grid Properties ==========
    State start = grid.getCurrentState();
    std::vector<Transition> succs = grid.successors(start);
    
    std::cout << "Successors from start:\n";
    for (const auto& trans : succs) {
        int dst_x = grid.getX(trans.next);
        int dst_y = grid.getY(trans.next);
        std::cout << "  → (" << dst_x << "," << dst_y << ") "
                  << "[" << grid.actionToString(trans.action) << "]\n";
    }
    
    // ========== 5. Build Product Automaton ==========
    ProductAutomaton* product = new ProductAutomaton();
    // ... product construction ...
    
    std::cout << "Product: " << product->getnumStates() << " states\n";
    
    return 0;
}
```

---

## Building and Compilation

### Compilation Flags

```bash
-std=c++20          # C++20 standard
-I"spot/include"    # Spot library headers (for grid world export)
-L"spot/lib"        # Spot library linking
```

### Compiling a Grid World Program

```bash
g++ -std=c++20 -Wall -Wextra -g3 \
    -I"Transition_Systems" \
    -I"Spot Library/spot-2.14.4.dev" \
    my_program.cpp GridWorldTransitionSystem.cpp -o my_program \
    -lspot -lbddx
```

### Compiling a Generic TS Program

```bash
g++ -std=c++20 -Wall -Wextra -g3 \
    -I"Transition_Systems" \
    my_program.cpp GeneralTransitionSystem.cpp -o my_program
```

---

## Performance Characteristics

### GeneralTransitionSystem

- **State Lookup**: O(n) where n = number of states
- **Adjacency Check**: O(d) where d = degree of state
- **Reachability**: O(n + e) BFS/DFS (n = states, e = edges)
- **Memory**: O(n + e) for state and edge storage

### GridWorldTransitionSystem

- **State Creation**: O(1)
- **Successor Generation**: O(1) - at most 4 actions
- **Coordinate Conversion**: O(1)
- **Label Lookup**: O(1)
- **Memory**: O(width × height) for grid

---

## Usage Tips

### Working with General Transition Systems

1. **Ensure unique AP IDs** - Each state must have a unique AP identifier
2. **Add transitions carefully** - Verify source and destination states exist
3. **Initialize properly** - Set at least one initial state before planning
4. **Check connectivity** - Use `isConnected()` to verify workspace connectivity

### Working with Grid Worlds

1. **Understand cell indexing** - Cell (x, y) has ID = y × width + x
2. **Movement constraints** - Actions are bounded by grid boundaries
3. **AP naming** - AP names correspond to cell IDs (useful for Spot integration)
4. **State equality** - Two states are equal if they have the same AP set

---

## Known Limitations

1. **GeneralTransitionSystem**: O(n) state lookup; consider hash maps for large systems
2. **GridWorldTransitionSystem**: Limited to regular 2D grids; no diagonal movement
3. **Single Robot**: Both designed for single robot workspace; multi-robot requires duplication
4. **Static Environments**: No dynamic obstacle support in base implementation

---

## Future Enhancements

- [ ] Hash-based state lookup for GeneralTransitionSystem
- [ ] Diagonal movement support for GridWorldTransitionSystem
- [ ] 3D grid world variant
- [ ] Dynamic obstacle avoidance
- [ ] Continuous state space approximation
- [ ] Multi-robot workspace composition
- [ ] Graph-based transition system variant

---

## References

- Parent: [Automatons/README.md](../Automatons/README.md)
- Related: [Task Batch Planning Decision Tree/README.md](../Task%20Batch%20Planning%20Decision%20Tree/README.md)
- Paper: Büchi automata and LTL semantics (see main README)
