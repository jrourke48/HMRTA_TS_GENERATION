# Environment Module

## Overview

The Environment module provides the abstraction layer between discrete transition systems and continuous workspace representations. It enables the planning algorithms to reason about both symbolic state transitions and geometric robot motion.

### Key Responsibilities

1. **Workspace Abstraction** - Bridge between TS and grid worlds
2. **State-Grid Mapping** - Bidirectional mapping between symbolic states and coordinates
3. **Reachability Queries** - Determine feasible transitions and successors
4. **Atomic Proposition Checking** - Evaluate task conditions at states
5. **Cost Tracking** - Support for weighted motion planning

## Components

### Environment

**Files**: `Environment.h`, `Environment.cpp`

Central coordinator connecting transition systems and grid-based environments.

#### Key Members

```cpp
TS* transitionSystem;                           // Discrete state graph
GridWorld* gridWorld;                           // 2D costmap
std::unordered_map<uint16_t, stateGridMapping> // State-to-grid mappings
    stateIdToGridMap;
```

#### State-Grid Mapping Structure

```cpp
struct stateGridMapping {
    Point center;        // Center coordinate of this state
    uint16_t width;      // Cell width in grid units
    uint16_t height;     // Cell height in grid units
};
```

#### Key Methods

```cpp
// Getters
TS* getTransitionSystem() const;
GridWorld* getGridWorld() const;
uint32_t getInitialState() const;
std::vector<uint32_t> getSuccessorStates(uint32_t stateId) const;

// Coordinate conversion
uint32_t gridToTSStateId(Point p) const;
Point TSStateIdToGridCenter(uint32_t stateId) const;

// Queries
bool isObstacle(Point p) const;
bool isReachable(uint32_t fromState, uint32_t toState) const;
double getTransitionCost(uint32_t fromState, uint32_t toState) const;

// State labeling with APs
std::vector<uint16_t> getAtomicPropositions(uint32_t stateId) const;
bool hasAtomicProposition(uint32_t stateId, uint16_t apId) const;
```

#### Usage Example

```cpp
// Create environment
TS* ts = new TS();
ts->addState(0, "");
ts->addState(1, "location_A");
ts->addTransition(0, 1, "move", 5.0);

GridWorld* grid = new GridWorld(100, 100, 50);  // 100x100 grid, 50x50 cells
Environment* env = new Environment(ts, grid);

// Query
uint32_t initial = env->getInitialState();
vector<uint32_t> successors = env->getSuccessorStates(initial);

// Get coordinates
Point p = env->TSStateIdToGridCenter(1);
```

---

### GridWorld

**Files**: `GridWorld.h`, `GridWorld.cpp`

2D occupancy grid with costmap for obstacle representation.

#### Grid Representation

```cpp
// Costmap values
// 0       = free space
// 1-254   = movement cost (higher = harder to traverse)
// 255     = obstacle (impassable)
```

#### Key Members

```cpp
std::vector<std::vector<uint8_t>> costmap;
uint16_t width;   // Total grid width in cells
uint16_t height;  // Total grid height in cells
```

#### Key Methods

```cpp
// Accessors
uint8_t getCost(uint16_t x, uint16_t y) const;
void setCost(uint16_t x, uint16_t y, uint8_t cost);

// Queries
bool isObstacle(uint16_t x, uint16_t y) const;
bool isFree(uint16_t x, uint16_t y) const;
bool isInBounds(uint16_t x, uint16_t y) const;

// Modification
void setObstacle(uint16_t x, uint16_t y);
void clearCell(uint16_t x, uint16_t y);
void clearAll();

// Dimensions
uint16_t getWidth() const;
uint16_t getHeight() const;
```

#### Cost Semantics

- **0 (Free)**: Easily traversable, no cost penalty
- **1-127 (Traversable)**: Slight cost increase (steep slopes, sandy areas)
- **128-254 (Difficult)**: Significant cost increase (mud, water crossing)
- **255 (Obstacle)**: Cannot traverse (walls, cliffs)

#### Usage Example

```cpp
// Create 100x100 grid with 50x50 cells
GridWorld* grid = new GridWorld(100, 100, 50);

// Set some obstacles
grid->setObstacle(25, 25);
grid->setObstacle(25, 26);

// Check feasibility
if (grid->isObstacle(25, 25)) {
    cout << "Cannot go there!" << endl;
}

// Set cost for traversable but difficult area
grid->setCost(30, 30, 150);  // High cost, but traversable
```

---

### Point

**Files**: `Point.h`, `Point.cpp`

Simple 2D coordinate representation for grid-based operations.

#### Members

```cpp
int16_t x;  // X coordinate
int16_t y;  // Y coordinate
```

#### Key Methods

```cpp
// Accessors
int16_t getX() const;
int16_t getY() const;
void setX(int16_t x);
void setY(int16_t y);

// Vector operations
Point operator+(const Point& other) const;  // Addition
Point operator-(const Point& other) const;  // Subtraction
Point operator*(int16_t scalar) const;      // Scaling

// Distance calculations
double distance(const Point& other) const;
double manhattanDistance(const Point& other) const;

// Comparison
bool operator==(const Point& other) const;
bool operator!=(const Point& other) const;
```

#### Usage Example

```cpp
Point p1(10, 20);
Point p2(15, 25);

// Distance
double dist = p1.distance(p2);

// Arithmetic
Point p3 = p1 + p2;  // (25, 45)
Point p4 = p1 * 2;   // (20, 40)

// Comparison
if (p1 == p2) {
    cout << "Same location" << endl;
}
```

---

## Data Flow in Planning

### State Query Process

```
TaskAllocationAlgorithms
    ↓ (needs successors)
    → Environment::getSuccessorStates(current_state)
        ↓
        → TS::getSuccessors()  (check TS edges)
        ↓
        → GridWorld::isFree()  (check obstacle-free path)
        ↓
    ← [list of reachable next_states]
```

### AP Evaluation Process

```
Edge Label: "p0 & p1:{0}"
    ↓
TaskAllocationAlgorithms::collectAPsFromEdgesByIndexWithAcceptance()
    ↓ (need to check if p0 is true)
    → Environment::hasAtomicProposition(current_state, 0)
        ↓
        → Check TS state labels
        ↓
    ← true/false
    ↓
If true: can traverse this edge
```

---

## Integration Patterns

### With Transition System

```cpp
// TS provides abstract states
ts->addState(0, "initial");
ts->addState(1, "location_A");
ts->addTransition(0, 1, "move", 5.0);

// Environment bridges to grid
env->setTransitionSystem(ts);
Point center = env->TSStateIdToGridCenter(1);  // Get grid coordinates
```

### With Planning Algorithms

```cpp
// Planning needs reachability information
vector<uint32_t> successors = env->getSuccessorStates(current);

// Planning needs AP evaluation
bool canExecute = env->hasAtomicProposition(state, ap_id);

// Planning needs costs
double cost = env->getTransitionCost(from, to);
```

### With Visualization

```cpp
// Export grid for visualization
for (int y = 0; y < grid->getHeight(); y++) {
    for (int x = 0; x < grid->getWidth(); x++) {
        uint8_t cost = grid->getCost(x, y);
        if (grid->isObstacle(x, y)) {
            cout << "#";  // Obstacle
        } else if (cost > 200) {
            cout << "~";  // Difficult
        } else if (cost > 100) {
            cout << ".";  // Traversable
        } else {
            cout << " ";  // Free
        }
    }
    cout << endl;
}
```

---

## Coordinate Systems

### Grid Coordinates (GridWorld)

- **Origin**: Top-left corner (0, 0)
- **X-axis**: Increases to the right
- **Y-axis**: Increases downward
- **Range**: [0, width-1] × [0, height-1]

### State IDs (Transition System)

- **Symbolic identifiers**: Arbitrary uint32_t values
- **Semantics**: Defined by application
- **Mapping**: Via `stateIdToGridMap` in Environment

### Conversion

```cpp
// Grid point → TS state
Point p(25, 30);
uint32_t state = env->gridToTSStateId(p);

// TS state → Grid point
Point p_back = env->TSStateIdToGridCenter(state);
```

---

## Common Workflows

### Setting Up a 2D Workspace

```cpp
// 1. Create grid environment (100x100 physical units, 50x50 cells)
GridWorld* grid = new GridWorld(100, 100, 50);

// 2. Mark obstacles
for (int i = 20; i < 30; i++) {
    grid->setObstacle(i, 25);  // Vertical wall
}

// 3. Set difficult areas
for (int x = 40; x < 50; x++) {
    for (int y = 40; y < 50; y++) {
        grid->setCost(x, y, 200);  // High cost region
    }
}

// 4. Create TS
TS* ts = new TS();
ts->addState(0, "");              // Initial
ts->addState(1, "location_A");
ts->addState(2, "location_B");
ts->addTransition(0, 1, "move_A", 5.0);
ts->addTransition(1, 2, "move_B", 8.0);

// 5. Create environment
Environment* env = new Environment(ts, grid);
```

### Querying Environment

```cpp
// Check if path is valid
vector<uint32_t> path = {0, 1, 2};
bool valid = true;
for (int i = 0; i < path.size() - 1; i++) {
    if (!env->isReachable(path[i], path[i+1])) {
        valid = false;
        break;
    }
}

// Get all feasible next moves
vector<uint32_t> options = env->getSuccessorStates(current_state);

// Check if current state satisfies requirement
if (env->hasAtomicProposition(current_state, LOCATION_A)) {
    cout << "At location A!" << endl;
}
```

---

## Performance Considerations

### Time Complexity

- **State query**: O(|TS edges|) for successor computation
- **Grid lookup**: O(1) for cell cost/obstacle check
- **Reachability**: O(1) if precomputed, O(|path|) if checking step-by-step

### Space Complexity

- **GridWorld**: O(width × height) for costmap
- **TS**: O(|states| + |edges|)
- **State-grid mapping**: O(|states|)

### Optimization Tips

1. **Precompute Reachability**: Build reachability matrix if queried frequently
2. **Lazy Grid Loading**: Load grid regions on-demand for large environments
3. **Cache State Lookups**: Memoize frequently queried states
4. **Hierarchical Grids**: Use coarser grids for high-level planning

---

## Debugging Tips

### Verify State-Grid Mapping

```cpp
// Check bidirectional consistency
for (uint32_t state = 0; state < num_states; state++) {
    Point p = env->TSStateIdToGridCenter(state);
    uint32_t state_back = env->gridToTSStateId(p);
    assert(state == state_back);
}
```

### Visualize Reachability

```cpp
// Mark reachable states from starting position
vector<vector<char>> reachable_map(grid->getHeight(), 
                                   vector<char>(grid->getWidth(), '.'));

function<void(uint32_t)> dfs = [&](uint32_t state) {
    Point p = env->TSStateIdToGridCenter(state);
    if (p.getX() >= 0 && p.getX() < grid->getWidth() &&
        p.getY() >= 0 && p.getY() < grid->getHeight()) {
        reachable_map[p.getY()][p.getX()] = 'R';
        for (uint32_t next : env->getSuccessorStates(state)) {
            dfs(next);
        }
    }
};

dfs(initial_state);

// Print map
for (auto& row : reachable_map) {
    for (char c : row) cout << c;
    cout << endl;
}
```

### Check AP Coverage

```cpp
// Verify all required APs are present somewhere
set<uint16_t> present_aps;
for (uint32_t state = 0; state < num_states; state++) {
    auto aps = env->getAtomicPropositions(state);
    for (auto ap : aps) {
        present_aps.insert(ap);
    }
}

for (uint16_t required_ap : required_aps) {
    if (present_aps.find(required_ap) == present_aps.end()) {
        cerr << "Warning: AP " << required_ap << " not in any state!" << endl;
    }
}
```

---

## See Also

- [../README.md](../README.md) - Main module overview
- [../LTLFormula/README.md](../LTLFormula/README.md) - AP naming conventions
- [../MultiRobotSystem/README.md](../MultiRobotSystem/README.md) - Robot capabilities
- [../../Automatons/README.md](../../Automatons/README.md) - TS structure details

---

**Last Updated**: 2026-07-21  
**Status**: Complete with grid and point utilities
