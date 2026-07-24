# Tree Module

## Overview

The Tree module provides the core data structures for decision tree-based planning. It represents task allocations and planning states in a hierarchical tree structure suitable for search algorithms.

### Key Responsibilities

1. **Tree Structure** - Hierarchical node organization
2. **Node Representation** - Individual planning states with automaton and TS information
3. **Progress Tracking** - Task execution phases (PRE, TRA, SUF, OTH)
4. **Path Management** - Backtrace from leaves to root for solution extraction
5. **Frontier Management** - Track expandable nodes during search

## Components

### PlanningDecisionTree

**Files**: `PlanningDecisionTree.h`, `PlanningDecisionTree.cpp`

Main container for the hierarchical decision tree structure.

#### Key Members

```cpp
Tree_Node* root;                        // Root of the tree
uint32_t nodeCount;                     // Total nodes in tree
std::vector<Tree_Node*> frontier;       // Expandable nodes
std::vector<Tree_Node*> allNodes;       // All nodes for cleanup
```

#### Key Methods

```cpp
// Construction
PlanningDecisionTree();
~PlanningDecisionTree();

// Tree operations
void insertNode(Tree_Node* node);
void removeFrontierNode(Tree_Node* node);
void deleteSubtree(Tree_Node* root);
void clearTree();

// Queries
Tree_Node* getRoot() const;
std::vector<Tree_Node*> getFrontier() const;
std::vector<Tree_Node*> getAllNodes() const;
uint32_t getNodeCount() const;
uint32_t getFrontierSize() const;

// Export
void exportToDot(const std::string& filename) const;
void printTreeStructure() const;
```

#### Tree Operations

**Insertion**:
```cpp
Tree_Node* newNode = new Tree_Node(...);
tree->insertNode(newNode);
// Node added to frontier for expansion
```

**Removal**:
```cpp
tree->removeFrontierNode(nodePtr);
// Node no longer in frontier but remains in tree
```

**Subtree Deletion**:
```cpp
tree->deleteSubtree(nodePtr);
// Deletes nodePtr and all descendants recursively
```

#### Frontier Management

The frontier represents nodes available for expansion:
- **New nodes**: Added to frontier automatically
- **Expanded nodes**: Removed after generating children
- **Pruned nodes**: May be removed during pruning operations
- **Termination**: When frontier is empty, search terminates

#### Usage Example

```cpp
#include "PlanningDecisionTree.h"

// Create tree
PlanningDecisionTree* tree = new PlanningDecisionTree();

// Root is created on first insertion
Tree_Node* root = new Tree_Node(0, 0, 0);  // IDs and initial state
root->setProgress(Tree_Node::TASK_PROGRESS::PRE);
tree->insertNode(root);

// Expand frontier
while (!tree->getFrontier().empty()) {
    Tree_Node* current = tree->getFrontier().front();
    
    // Generate children...
    for (uint32_t next_state : successor_states) {
        Tree_Node* child = new Tree_Node(
            next_state,
            next_automaton_state,
            current->getPathCost() + edge_cost
        );
        child->parent = current;
        tree->insertNode(child);
    }
    
    // Remove from frontier after expansion
    tree->removeFrontierNode(current);
}

// Extract solution path
vector<uint32_t> path = root->getPathToRoot();
```

---

### Tree_Node

**Files**: `Tree_Node.h`, `Tree_Node.cpp`

Individual node representation in the planning tree.

#### Node Structure

```cpp
class Tree_Node {
    // Identification
    uint32_t nodeId;
    Tree_Node* parentNode;
    std::vector<Tree_Node*> childNodes;
    
    // State Information
    uint32_t automatonState;      // Current state in Büchi automaton
    uint32_t tsState;             // Current state in TS (workspace location)
    
    // Progress Tracking (for acceptance set management)
    enum TASK_PROGRESS { PRE, TRA, SUF, OTH };
    TASK_PROGRESS progress;
    
    // Cost and Quality
    double pathCost;              // Cumulative cost from root
    uint16_t visitCount;          // Times this (automaton_state, progress) pair visited
};
```

#### Progress Phases

For infinite automata, progress cycles through acceptance set rounds:

- **PRE** (Pre-progress): Initial phase, starting first acceptance set traversal
- **TRA** (Transition): In middle of acceptance set traversal
- **SUF** (Suffix): Completed first full traversal of all acceptance sets
- **OTH** (Other): Additional cycles completed

For finite automata:
- Start at **PRE**
- Progress through **TRA** → **SUF** → **OTH**
- **OTH** indicates task completion

#### Key Methods

**Accessors**:
```cpp
uint32_t getAutomatonState() const;
uint32_t getTSState() const;
TASK_PROGRESS getProgress() const;
double getPathCost() const;
uint16_t getVisitCount() const;
Tree_Node* getParent() const;
std::vector<Tree_Node*> getChildren() const;
```

**Mutators**:
```cpp
void setProgress(TASK_PROGRESS p);
void setPathCost(double c);
void incrementVisitCount();
void addChild(Tree_Node* child);
```

**Path Operations**:
```cpp
// Backtrace from current node to root
std::vector<uint32_t> getPathToRoot() const;  // Automaton state sequence
std::vector<uint32_t> getTSPath() const;      // Workspace state sequence

// Reconstruct solution
std::vector<Tree_Node*> getNodePath() const;  // Full node path
```

**Comparison**:
```cpp
bool isSameState(const Tree_Node* other) const;
bool hasLowerCost(const Tree_Node* other) const;
```

#### Usage Example

```cpp
#include "Tree_Node.h"

// Create node
Tree_Node* node = new Tree_Node(
    automaton_state_id,    // State in Büchi automaton
    ts_state_id,          // State in transition system
    0.0                   // Initial path cost
);

// Set properties
node->setProgress(Tree_Node::TASK_PROGRESS::PRE);
node->setPathCost(5.0);

// Track acceptance visits
node->incrementVisitCount();

// Add child
Tree_Node* child = new Tree_Node(next_auto, next_ts, 8.0);
child->parent = node;
node->addChild(child);

// Extract solution
vector<uint32_t> path = child->getTSPath();  // Sequence of TS states
vector<uint32_t> automaton_path = child->getPathToRoot();
```

---

## Tree Construction Patterns

### Breadth-First Search (BFS)

```cpp
queue<Tree_Node*> frontier;
Tree_Node* root = new Tree_Node(initial_auto, initial_ts, 0.0);
frontier.push(root);

while (!frontier.empty()) {
    Tree_Node* current = frontier.front();
    frontier.pop();
    
    // Generate successors
    for (const auto& successor : getSuccessors(current)) {
        Tree_Node* child = new Tree_Node(
            successor.automaton_state,
            successor.ts_state,
            current->getPathCost() + successor.cost
        );
        child->parent = current;
        current->addChild(child);
        
        // Check if goal reached
        if (isGoal(child)) {
            return child->getTSPath();
        }
        
        frontier.push(child);
    }
}
```

### Depth-First Search (DFS)

```cpp
stack<Tree_Node*> frontier;
Tree_Node* root = new Tree_Node(initial_auto, initial_ts, 0.0);
frontier.push(root);

while (!frontier.empty()) {
    Tree_Node* current = frontier.top();
    frontier.pop();
    
    if (isGoal(current)) {
        return current->getTSPath();
    }
    
    // Generate successors in reverse order (LIFO)
    auto successors = getSuccessors(current);
    for (auto it = successors.rbegin(); it != successors.rend(); ++it) {
        Tree_Node* child = new Tree_Node(
            it->automaton_state,
            it->ts_state,
            current->getPathCost() + it->cost
        );
        child->parent = current;
        current->addChild(child);
        frontier.push(child);
    }
}
```

### A* Search (Cost-Aware)

```cpp
priority_queue<pair<double, Tree_Node*>,
               vector<pair<double, Tree_Node*>>,
               greater<pair<double, Tree_Node*>>> frontier;

Tree_Node* root = new Tree_Node(initial_auto, initial_ts, 0.0);
frontier.push({heuristic(root), root});

while (!frontier.empty()) {
    auto [_, current] = frontier.top();
    frontier.pop();
    
    if (isGoal(current)) {
        return current->getTSPath();
    }
    
    for (const auto& successor : getSuccessors(current)) {
        double g_cost = current->getPathCost() + successor.cost;
        double f_cost = g_cost + heuristic(successor);
        
        Tree_Node* child = new Tree_Node(
            successor.automaton_state,
            successor.ts_state,
            g_cost
        );
        child->parent = current;
        current->addChild(child);
        frontier.push({f_cost, child});
    }
}
```

---

## Solution Extraction

### Path Reconstruction

```cpp
// From goal node, trace back to root
vector<uint32_t> reconstructPath(Tree_Node* goal_node) {
    vector<uint32_t> path;
    Tree_Node* current = goal_node;
    
    while (current != nullptr) {
        path.push_back(current->getTSState());
        current = current->getParent();
    }
    
    reverse(path.begin(), path.end());
    return path;
}
```

### Solution Validation

```cpp
bool validateSolution(const vector<uint32_t>& path,
                      const BuchiAutomaton* automaton,
                      const Environment* env) {
    // Check that path visits all required states
    // Check that transitions are valid in automaton
    // Check that workspace states form connected path
    
    for (int i = 0; i < path.size() - 1; i++) {
        if (!env->isReachable(path[i], path[i+1])) {
            return false;  // Invalid transition
        }
    }
    
    return true;
}
```

### Cost Minimization

```cpp
// Among multiple solutions, select lowest cost
Tree_Node* selectBestSolution(const vector<Tree_Node*>& candidates) {
    return *min_element(
        candidates.begin(),
        candidates.end(),
        [](const Tree_Node* a, const Tree_Node* b) {
            return a->getPathCost() < b->getPathCost();
        }
    );
}
```

---

## Progress Phase Management

### Progress Increment Logic

```cpp
void incrementProgress(Tree_Node* node, 
                      const std::vector<uint16_t>& remaining_acceptance_sets) {
    if (remaining_acceptance_sets.empty()) {
        // All acceptance sets visited, advance progress
        int current = static_cast<int>(node->getProgress());
        if (current < static_cast<int>(Tree_Node::TASK_PROGRESS::OTH)) {
            Tree_Node::TASK_PROGRESS next = 
                static_cast<Tree_Node::TASK_PROGRESS>(current + 1);
            node->setProgress(next);
        }
    }
}
```

### Finite Task Termination

```cpp
bool isTaskComplete(const Tree_Node* node) {
    return node->getProgress() == Tree_Node::TASK_PROGRESS::OTH
        && acceptingSets.empty();
}
```

### Infinite Task Cycling

```cpp
void cycleProgressForInfinite(Tree_Node* node) {
    if (node->getProgress() == Tree_Node::TASK_PROGRESS::OTH) {
        // Reset to PRE for another cycle
        node->setProgress(Tree_Node::TASK_PROGRESS::PRE);
        node->incrementVisitCount();
    }
}
```

---

## Memory Management

### Cleanup Pattern

```cpp
void cleanupTree(PlanningDecisionTree* tree) {
    for (Tree_Node* node : tree->getAllNodes()) {
        delete node;
    }
    delete tree;
}
```

### Subtree Deletion

```cpp
void deleteSubtreeHelper(Tree_Node* node) {
    if (node == nullptr) return;
    
    for (Tree_Node* child : node->getChildren()) {
        deleteSubtreeHelper(child);
    }
    
    delete node;
}
```

---

## Tree Visualization

### DOT Export

```cpp
void exportTreeToDot(PlanningDecisionTree* tree, const string& filename) {
    ofstream out(filename);
    out << "digraph DecisionTree {" << endl;
    
    function<void(Tree_Node*)> visit = [&](Tree_Node* node) {
        if (node == nullptr) return;
        
        string progress_str;
        switch (node->getProgress()) {
            case Tree_Node::TASK_PROGRESS::PRE: progress_str = "PRE"; break;
            case Tree_Node::TASK_PROGRESS::TRA: progress_str = "TRA"; break;
            case Tree_Node::TASK_PROGRESS::SUF: progress_str = "SUF"; break;
            case Tree_Node::TASK_PROGRESS::OTH: progress_str = "OTH"; break;
        }
        
        out << "Node_" << node->getNodeId() << " [label=\"" 
            << node->getTSState() << "\\n" << progress_str << "\\n"
            << fixed << setprecision(1) << node->getPathCost() << "\"];" << endl;
        
        for (Tree_Node* child : node->getChildren()) {
            out << "Node_" << node->getNodeId() << " -> Node_" 
                << child->getNodeId() << ";" << endl;
            visit(child);
        }
    };
    
    visit(tree->getRoot());
    out << "}" << endl;
    out.close();
}
```

### Text Tree Display

```cpp
void printTree(Tree_Node* node, int depth = 0) {
    if (node == nullptr) return;
    
    string indent(depth * 2, ' ');
    cout << indent << "TS=" << node->getTSState()
         << " Auto=" << node->getAutomatonState()
         << " Cost=" << node->getPathCost()
         << " Visited=" << node->getVisitCount() << endl;
    
    for (Tree_Node* child : node->getChildren()) {
        printTree(child, depth + 1);
    }
}
```

---

## Performance Analysis

### Space Complexity

- **Per node**: O(1) for data members
- **Total**: O(n) where n = number of nodes in tree
- **Practical**: Depends on branching factor and depth

### Time Complexity

- **Node insertion**: O(1)
- **Node deletion**: O(subtree_size)
- **Path extraction**: O(tree_depth)
- **Tree export**: O(n) for all nodes

### Memory Optimization Tips

1. **Prune aggressively**: Remove duplicate states early
2. **Depth limits**: Cap search depth to prevent memory explosion
3. **Lazy child creation**: Generate children on-demand
4. **Memory pools**: Pre-allocate nodes to reduce fragmentation

---

## Debugging Utilities

### Tree Statistics

```cpp
void printTreeStatistics(const PlanningDecisionTree* tree) {
    int max_depth = 0;
    int leaf_count = 0;
    
    function<void(Tree_Node*, int)> analyze = 
        [&](Tree_Node* node, int depth) {
        max_depth = max(max_depth, depth);
        if (node->getChildren().empty()) {
            leaf_count++;
        }
        for (Tree_Node* child : node->getChildren()) {
            analyze(child, depth + 1);
        }
    };
    
    analyze(tree->getRoot(), 0);
    
    cout << "Tree Statistics:" << endl;
    cout << "  Nodes: " << tree->getNodeCount() << endl;
    cout << "  Max Depth: " << max_depth << endl;
    cout << "  Leaf Nodes: " << leaf_count << endl;
    cout << "  Frontier Size: " << tree->getFrontierSize() << endl;
}
```

---

## See Also

- [../README.md](../README.md) - Main module documentation
- [../Testing/README.md](../Testing/README.md) - Tree testing examples
- [../../Automatons/README.md](../../Automatons/README.md) - Automaton state representation
- [../Environment/README.md](../Environment/README.md) - TS state concepts

---

**Last Updated**: 2026-07-21  
**Status**: Complete with progress tracking
