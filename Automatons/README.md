# Automatons Module

## Overview

The Automatons module provides a unified hierarchy of automaton classes for formal task planning with LTL specifications. It serves as the bridge between high-level temporal logic formulas and low-level planning algorithms.

### Core Responsibilities

1. **Abstract Automaton Interface** - Common methods and properties across all automata types
2. **Büchi Automaton Generation** - LTL-to-automaton conversion via Spot library integration
3. **Transition System Representation** - Discrete workspace modeling with state and edge abstractions
4. **Product Automaton Construction** - Synthesis of combined (TS × Büchi) state spaces
5. **Graph Data Structures** - Node and edge abstractions with labeling and properties
6. **GBA Semantics** - Edge-indexed acceptance set tracking and parsing

## Class Hierarchy

```
Automaton (abstract base)
├── BuchiAutomaton
│   └── Generalized Büchi Automata with edge-indexed acceptance
├── TS (TransitionSystem)
│   └── Discrete workspace regions with motion model
└── ProductAutomaton
    └── Product of TS and Büchi with combined state space
```

## Classes and Components

### 1. Automaton (Abstract Base Class)

**File**: `Automaton.h`

Defines the abstract interface all automaton types must implement.

#### Key Members

```cpp
protected:
    uint32_t numNodes;                           // Total number of states
    uint32_t numEdges;                           // Total number of transitions
    std::map<uint32_t, Node*> nodeMap;          // Maps node IDs to node pointers
```

#### Key Methods

```cpp
// Pure virtual - must be implemented by subclasses
virtual void add_Node(Node* node) = 0;
virtual bool isAdjacent(uint32_t srcId, uint32_t dstId) const = 0;

// Common interface
uint32_t getnumStates() const;
uint32_t getnumEdges() const;
Node* getNode(uint32_t nodeId) const;
void add_Edge(Edge* edge);
```

#### Properties

- **Extensible**: Designed for adding new automaton types
- **Type-safe**: Uses `Node*` pointers with virtual methods for polymorphism
- **Memory Managed**: Subclasses responsible for node/edge cleanup

---

### 2. BuchiAutomaton

**Files**: `BuchiAutomaton.h`, `BuchiAutomaton.cpp`

Implements Generalized Büchi Automata (GBA) with full LTL support via Spot library.

#### Features

- **LTL Parsing**: Converts LTL formulas to automata using Spot
- **GBA Semantics**: Edge-indexed acceptance set tracking
- **DOT Format**: Imports and exports automata in DOT graph format
- **Finite/Infinite Detection**: Classifies automata based on acceptance conditions
- **Edge Label Parsing**: Extracts APs and acceptance marks from DOT edge labels

#### Key Methods

**Automaton Generation**
```cpp
// Generate from LTL formula string
void generateFromLTLString(const string& ltlFormula);

// Parse DOT format content
void parseDotFormat(const string& dotContent);
```

**Acceptance Set Handling**
```cpp
// Parse GBA acceptance set numbers from "Inf(0)&Inf(1)" format
vector<uint16_t> parseglobalAcceptanceSets(const string& label);

// Determine if automaton is finite (no cyclic acceptance) or infinite
bool isInfinite() const { return _isInfinite; }

// Check if automaton contains GBA elements
bool checkIsInfinite(const string& dotContent);
```

**Edge Label Processing**
```cpp
// Extract label from DOT edge with acceptance marks
// Format: "p0 & p1:{0,1}" where ':' separates APs from acceptance sets
string extractLabelFromDotBrackets(const string& content);
```

#### Key Members

```cpp
vector<uint16_t> acceptingStates;    // GBA acceptance set IDs (parsed from labels)
bool _isFinite;                      // Whether automaton is finite
std::vector<Edge_Node*> edges;       // All edges with acceptance information
```

#### Label Format

**Edge labels in DOT format**:
- Simple: `"p0"` - single atomic proposition
- Complex: `"p0 & p1 | p2"` - boolean expressions
- With Acceptance: `"p0 & p1:{0,1,2}"` - APs and GBA acceptance marks

**Parsing**:
1. Split by `:` to separate AP section from acceptance marks
2. Parse AP section for atomic propositions and boolean structure
3. Parse acceptance marks from `{x,y,z}` format using comma delimiter

#### GBA Edge Filtering Rules

During edge extraction, edges are filtered based on AP structure:

1. **Single AP or OR-connected APs**: Included (can be satisfied individually)
   - `"p0"` ✓
   - `"p0 | p1"` ✓

2. **Multiple AND-connected APs**: **Completely Excluded** (require simultaneous execution)
   - `"p0 & p1"` ✗ (both true simultaneously)
   - `"p0 & p1 & p2"` ✗

3. **Negated APs**: Always skipped
   - `"!p0"` ✗

**Rationale**: Robot capabilities are checked per state; edges requiring multiple APs simultaneously cannot be decomposed.

#### Spot Integration

```cpp
spot::formula spotFormula;           // Internal Spot representation
spot::twa_graph_ptr spotTWA;         // Spot automaton graph
```

Uses Spot's conversion from LTL to Büchi automata:
- Handles complex temporal operators (F, G, X, U, R, W, etc.)
- Generates minimized automata
- Provides DOT export functionality

---

### 3. TS (Transition System)

**Files**: `TS.h`, `TS.cpp`

Represents the discrete workspace as a state transition graph.

#### Purpose

Abstracts continuous or complex environments into discrete states and transitions suitable for formal planning.

#### Key Members

```cpp
std::map<uint32_t, Node*> nodeMap;        // State ID to state object mapping
std::vector<Edge_Node*> edgeList;         // All transitions in the system
uint32_t initialState;                    // Entry point for planning
```

#### Key Methods

```cpp
// State management
void addState(uint32_t id, const string& label = "");
Node* getState(uint32_t id) const;

// Transition management
void addTransition(uint32_t fromState, uint32_t toState, 
                   const string& label, double cost = 1.0);
vector<uint32_t> getSuccessors(uint32_t state) const;
```

#### Atomic Propositions

States are labeled with atomic propositions that become true when the transition system reaches those states:

```cpp
// Example TS with propositions
TS ts;
ts.addState(0, "");              // Initial state
ts.addState(1, "at_location_A");
ts.addState(2, "at_location_B");
ts.addTransition(0, 1, "move_to_A", 5.0);
ts.addTransition(1, 2, "move_to_B", 8.0);
```

#### Workspace Mapping

TS can be paired with grid-based or continuous workspace representations (see Environment module).

---

### 4. ProductAutomaton

**Files**: `ProductAutomaton.h`, `ProductAutomaton.cpp`

Combines a TS and Büchi automaton into a single unified state space.

#### Concept

The product automaton merges two automaton spaces:
- **TS Component**: Represents feasible robot motions
- **Büchi Component**: Represents task progress tracking

**Product State**: `(ts_state, buchi_state)` pair

#### Key Members

```cpp
TS* transitionSystem;              // Reference to workspace model
BuchiAutomaton* buchiAutomaton;    // Reference to task automaton
std::map<pair<uint32_t, uint32_t>, uint32_t> productStates; // Product state mapping
```

#### Key Methods

```cpp
// Construction
void constructProduct();
uint32_t getProductStateId(uint32_t tsState, uint32_t buchiState) const;

// Queries
vector<uint32_t> getProductSuccessors(uint32_t productState) const;
pair<uint32_t, uint32_t> decomposeProductState(uint32_t productState) const;

// Acceptance
bool isAcceptingState(uint32_t productState) const;
```

#### Product Space Growth

- **Finite TS** with n states
- **Büchi automaton** with m states  
- **Product space**: Up to n × m states

The product automaton is the actual planning domain:
- **States** = valid (workspace_region, task_phase) combinations
- **Transitions** = motions that advance both workspace and task
- **Accepting paths** = complete task execution satisfying both TS and Büchi conditions

---

### 5. Node and Edge Classes

**Files**: `Edge_Node.h`

#### Node (State Representation)

```cpp
class Node {
    uint32_t nodeId;
    std::string label;           // State description / atomic propositions
    std::vector<Edge_Node*> edges; // Outgoing transitions
};
```

**Properties**:
- Unique identifier (nodeId)
- Textual label (APs or state name)
- Collection of outgoing edges

#### Edge_Node (Transition Representation)

```cpp
class Edge_Node {
    uint32_t srcNodeId;
    uint32_t dstNodeId;
    std::string label;           // Atomic propositions / Acceptance marks
    double cost;                 // Transition cost
    uint32_t edgeIndex;          // Position in edge collection (for indexing)
    std::set<uint16_t> acceptanceMarks; // GBA acceptance sets (for GBA edges)
};
```

**Properties**:
- Source and destination nodes
- Label with APs and acceptance marks
- Cost for weighted planning
- Edge index for efficient AP collection

---

## Edge-Indexed AP Collection with GBA Support

### Algorithm: `collectAPsFromEdgesByIndexWithAcceptance()`

Extracts atomic propositions from automaton edges while respecting GBA acceptance semantics.

**Location**: `TaskAllocationAlgorithms.cpp` (lines 800-870)

**Input**:
- `edges`: Vector of edge labels (strings)

**Output**:
- `outEdgeAPIds`: Vector of sets; `outEdgeAPIds[i]` = AP IDs for edge i
- `outEdgeAcceptanceSets`: Vector of sets; `outEdgeAcceptanceSets[i]` = acceptance set IDs for edge i

**Process**:

1. **Split by Colon**: Separate APs from acceptance marks
   ```
   "p0 & p1:{0,1}" → AP part: "p0 & p1", Acceptance part: "{0,1}"
   ```

2. **Parse AP Part**: Extract individual APs while checking for 2+ AND-ed APs
   - OR clauses are split by `|`
   - AND clauses within each OR are checked
   - If any AND clause has 2+ true APs: **skip entire edge** (use `continue`)

3. **Parse Acceptance Part**: Extract set IDs from `{x,y,z}` format
   - Split by `,`
   - Convert each to uint16_t
   - Store in set for deduplication

4. **Populate Outputs**: Add extracted data to output vectors at correct edge index

**Example**:

```
Edge 0: "p0 & p1:{0,1}"     → SKIPPED (2 AND-ed APs)
Edge 1: "p0 | p1:{0}"       → AP IDs: {0, 1}, Acceptance: {0}
Edge 2: "p2:{1,2}"          → AP IDs: {2}, Acceptance: {1, 2}
Edge 3: "!p3:{0}"           → SKIPPED (negated AP)
```

**Output Vectors**:
```cpp
outEdgeAPIds[1] = {0, 1}     // OR clause, both included
outEdgeAPIds[2] = {2}
outEdgeAcceptanceSets[1] = {0}
outEdgeAcceptanceSets[2] = {1, 2}
```

---

## GBA Semantics - Acceptance Set Tracking

### Concept

In GBA, acceptance conditions track which "acceptance sets" each edge belongs to. A path is accepting if it visits all required acceptance sets infinitely often (for infinite automata) or at least once (for finite automata).

### Implementation

**Per-Edge Tracking**:
```cpp
edgeAcceptanceSets[edgeIdx] = {0, 1}  // Edge belongs to acceptance sets 0 and 1
```

**Global Acceptance State**:
```cpp
vector<uint16_t> acceptingSets = gba->getAcceptingStates(); // e.g., {0, 1, 2}
```

### Progress Increment Logic

**When Edge is Traversed**:
1. Retrieve acceptance marks from edge: `{0, 1}`
2. Remove from global acceptingSets: `{0, 1, 2}` → `{2}`
3. When acceptingSets becomes empty: increment progress phase

```cpp
if (nba->isInfinite()) {
    std::set<uint16_t> curracceptingset = edgeAcceptanceSets[edgeIdx];
    for (uint16_t setNum : curracceptingset) {
        auto it = std::find(acceptingSets.begin(), acceptingSets.end(), setNum);
        if (it != acceptingSets.end()) {
            acceptingSets.erase(it);
        }
    }
}

if (acceptingSets.empty()) {
    int newProgressInt = static_cast<int>(currentNode->getProgress()) + 1;
    Tree_Node::TASK_PROGRESS newProgress = static_cast<Tree_Node::TASK_PROGRESS>(newProgressInt);
    newNode->setProgress(newProgress);
}
```

---

## Finite vs. Infinite Automata

### Detection

```cpp
bool isInfinite = ba->isInfinite();  // Automatically detected based on DOT content
```

**Finite**: No "Inf(" patterns in edge labels (all acceptance conditions are one-time events)

**Infinite**: Contains "Inf(" in edge labels (acceptance conditions must be satisfied repeatedly)

### Impact on Planning

**Finite Automata**:
- All acceptance sets must be visited exactly once
- Progress terminates at OTH state
- Aggressive pruning: removes duplicate (state, progress) pairs

**Infinite Automata**:
- Acceptance sets must be revisited infinitely often
- Progress may cycle: PRE→TRA→SUF→OTH→PRE
- Relaxed pruning: allows multiple traversals of same (state, progress) pairs

---

## Building and Compiling

### Compilation Flags

```bash
-std=c++20          # C++20 standard (for modern features)
-Wall -Wextra       # Warnings
-g3                 # Debug symbols
-I"spot/include"    # Spot library headers
-L"spot/lib"        # Spot library linking
-lspot -lbddx       # Link Spot and BDD libraries
```

### Example Build

```bash
g++ -std=c++20 -Wall -Wextra -g3 \
    -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev" \
    -I"C:/Users/johnn/Spot Library/spot-2.14.4.dev/buddy/src" \
    -L"C:/Users/johnn/Spot Library/spot-2.14.4.dev/lib" \
    BuchiAutomaton.cpp -o BuchiAutomaton.o -lspot -lbddx
```

---

## Usage Examples

### Creating a Büchi Automaton from LTL

```cpp
#include "Automatons/BuchiAutomaton.h"

BuchiAutomaton* gba = new BuchiAutomaton();

// Generate from LTL formula
gba->generateFromLTLString("F(\"p0\") & F(\"p1\")");

// Check if infinite
if (gba->isInfinite()) {
    cout << "Infinite automaton (requires cyclic execution)" << endl;
}

// Get acceptance sets
vector<uint16_t> acceptingSets = gba->getAcceptingStates();
cout << "Acceptance sets to visit: ";
for (auto s : acceptingSets) cout << s << " ";
cout << endl;
```

### Working with Transition Systems

```cpp
#include "Automatons/TS.h"

TS ts;

// Add states
ts.addState(0, "");               // Initial
ts.addState(1, "at_location_A");
ts.addState(2, "at_location_B");

// Add transitions
ts.addTransition(0, 1, "move", 5.0);
ts.addTransition(1, 2, "move", 8.0);
ts.addTransition(2, 1, "backtrack", 3.0);

// Query
vector<uint32_t> successors = ts.getSuccessors(0);  // Get states reachable from state 0
```

### Creating a Product Automaton

```cpp
#include "Automatons/ProductAutomaton.h"

TS* ts = new TS();
// ... populate TS ...

BuchiAutomaton* gba = new BuchiAutomaton();
gba->generateFromLTLString("F(\"p0\") & F(\"p1\")");

ProductAutomaton* product = new ProductAutomaton(ts, gba);
product->constructProduct();

// Query product space
vector<uint32_t> productSuccessors = product->getProductSuccessors(0);
```

---

## File Reference

| File | Purpose |
|------|---------|
| `Automaton.h` | Abstract base class for all automata |
| `BuchiAutomaton.h` / `.cpp` | GBA implementation with Spot integration |
| `TS.h` / `.cpp` | Transition system for workspace modeling |
| `ProductAutomaton.h` / `.cpp` | Product space of TS and Büchi |
| `Edge_Node.h` | Node and edge data structures |

---

## Debugging Tips

### Checking Automaton Structure

```cpp
// Print number of states/edges
cout << "States: " << gba->getnumStates() << endl;
cout << "Edges: " << gba->getnumEdges() << endl;

// Export DOT for visualization
gba->exportDot("automaton.dot");
```

### Verifying Edge Labels

Enable debug output in `collectAPsFromEdgesByIndexWithAcceptance()`:
```cpp
std::cerr << "Edge " << edgeIdx << ": " << edges[edgeIdx] << std::endl;
std::cerr << "  APs: "; 
for (auto ap : edgeAPIds[edgeIdx]) std::cerr << ap << " ";
std::cerr << "  Acceptance: ";
for (auto acc : edgeAcceptanceSets[edgeIdx]) std::cerr << acc << " ";
std::cerr << std::endl;
```

### Spot Library Integration

If Spot compilation fails:
1. Verify Spot library path is correct
2. Check include directories: `spot/twa/twagraph.hh`, `spot/tl/parse.hh`
3. Link libraries: `-lspot -lbddx`
4. Use C++20 standard: `-std=c++20`

---

## See Also

- [Task Batch Planning Decision Tree/README.md](../Task%20Batch%20Planning%20Decision%20Tree/README.md) - How automata are used in planning
- [AUTOMATONS_README.md](../AUTOMATONS_README.md) - Detailed automaton format specifications
- [SPOT_AUTOMATA_FORMATS.md](../SPOT_AUTOMATA_FORMATS.md) - Spot library format reference

---

**Last Updated**: 2026-07-21  
**Module Status**: GBA Semantics Implementation Complete
