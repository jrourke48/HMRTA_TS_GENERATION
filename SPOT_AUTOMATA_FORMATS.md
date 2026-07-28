# SPOT Automata Format Differences: Standard vs Generalized Büchi

## Overview

SPOT uses fundamentally different representations for finite and infinite LTL formulas:
- **Finite formulas** (no `G` operator): Standard Büchi Automata with **state-based acceptance**
- **Infinite formulas** (with `G` operator): Generalized Büchi Automata with **transition-based acceptance**

This difference impacts how we detect accepting paths and increment progress through the planning tree.

---

## Standard Büchi Automata (Finite Tests)

### Definition
- **Acceptance**: Achieved when visiting an **accepting STATE** at least once
- **Representation**: Accepting states marked in DOT with `peripheries=2` (double circle)
- **Example DOT Label**: (No "Inf" prefix)
  ```
  label="[Büchi]"
  ```

### Visual Representation
```
State 0 → State 1 → State 2 (accepting: peripheries=2)
                       ↑
                       └─────────┘
```

### Algorithm Implication
```cpp
if (isAcceptingState(stateId)) {
    incrementProgress();  // PRE → TRA
}
```

### When Accepting
- Visit any accepting state once → Path satisfies formula (finite)
- Multiple cycles possible but not required

---

## Generalized Büchi Automata (Infinite Tests)

### Definition
- **Acceptance**: Achieved when visiting ALL required **acceptance SETS infinitely often**
- **Representation**: Acceptance sets marked on **edges** in DOT with `{0}`, `{1}`, `{2}`, etc.
- **Example DOT Label**: 
  ```
  label="Inf(0)&Inf(1)&Inf(2)\n[gen. Büchi 3]"
  ```
  This means: Must visit transitions in set {0} infinitely AND set {1} infinitely AND set {2} infinitely

### Visual Representation
```
Edge 0→1: label="p0 & p1\n{0}"        (belongs to acceptance set {0})
Edge 1→2: label="p2 & p3\n{0,1}"      (belongs to sets {0} AND {1})
Edge 2→0: label="p4\n{2}"              (belongs to acceptance set {2})

Required: Visit {0} infinitely, {1} infinitely, AND {2} infinitely
```

### Algorithm Implication
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

### When Accepting
- Visit transitions containing {0}, {1}, {2} once each → Progress to TRA
- Visit transitions containing {0}, {1}, {2} once more → Path is accepting

---

## Key Differences Summary

| Feature | Standard Büchi | Generalized Büchi |
|---------|---|---|
| **Acceptance Mechanism** | Visiting accepting state | Visiting all acceptance sets infinitely |
| **Marked On** | Nodes (states) | Edges (transitions) |
| **DOT Marker** | `peripheries=2` on nodes | `{0}`, `{1}`, `{2}` on edges |
| **DOT Label** | No "Inf" prefix | `Inf(0)&Inf(1)&Inf(2)` format |
| **Cycles Required** | 1+ visit to accepting state | Multiple cycles to visit all sets |
| **Tracking** | Node IDs in `acceptingStates` | Edge labels + set membership |
| **Use Case** | Finite task sequences | Infinite patrol/monitoring tasks |

---

## SPOT Implementation Details

### How to Detect Format
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

### Parsing Required Sets
```
From: label="Inf(0)&Inf(1)&Inf(2)\n[gen. Büchi 3]"
Extract: {0, 1, 2}

From: label="[Büchi]"
Extract: {} (standard, no sets)
```

---

## Algorithm Updates Required

### Current Progress Incrementation (Standard Büchi)
```cpp
// TaskAllocationAlgorithms.cpp - Line 309-320
if (nba->isAccepting(newNode->getAutomatonState()->getId())) {
    // Reached accepting state once
    newNode->setProgress(TRA);  // Mark as satisfying
}

// Path is accepting when reaching TRA state
```

### New Progress Incrementation (Generalized Büchi)
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

---

## Data Structure Changes Needed

### BuchiAutomaton Class
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

### Tree_Node Class  
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

---

## Examples by Test Type

### Example 1: Finite Test (Standard Büchi)
**Formula**: `(F"p0" & F"p1")`  
**DOT Label**: `[Büchi]`  
**Algorithm**: Reach any accepting state → mark PRE, reach again → mark TRA → done

### Example 2: Infinite Test (Generalized Büchi)  
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

---

## Integration Strategy

1. ✅ **Phase 1** (Complete): Detect automaton type via `isInfinite` field
2. **Phase 2** (Next): Extract required acceptance sets from DOT label
3. **Phase 3** (Next): Modify Tree_Node to track visitedAcceptanceSets
4. **Phase 4** (Next): Update TaskAllocationAlgorithms progress logic
5. **Phase 5** (Next): Test with infinite test suite

---

## References

- SPOT Documentation: Acceptance Conditions
- Generalized Büchi Automata: Multiple acceptance conditions
- LTL Semantics: `G` operator requires infinite repetition
