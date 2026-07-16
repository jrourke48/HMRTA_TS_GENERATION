#include "BuchiAutomaton.h"
#include <algorithm>
#include <stack>
#include <set>

BuchiAutomaton::~BuchiAutomaton() {
    // Clean up dynamically allocated nodes
    // Note: Node pointers should be managed by a smart pointer in a production system
}

void BuchiAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool BuchiAutomaton::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}
std::vector<std::string> BuchiAutomaton::getEdgeLabels(uint16_t srcId, uint16_t dstId) const {
    std::vector<std::string> labels;
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return labels;
    
    Node* srcNode = it->second;
    std::vector<Edge> edges = srcNode->getEdges();
    
    for (const auto& edge : edges) {
        if (edge.getDstId() == dstId) {
            labels.push_back(edge.getLabel());
        }
    }
    return labels;
}

// A Büchi automaton is finite if there are no cycles in the from accepting state to accepting state.
// we need to do a depth-first search (DFS) or breadth-first search (BFS) to detect cycles in the graph.

bool BuchiAutomaton::isFinite() const {
    //cycle through all accepting states to see if there is a cycle from any accepting state to any other accepting state
    for (const uint16_t acceptingstate : acceptingStates) {
        std::set<uint16_t> visited;
        std::stack<uint16_t> stack;
        stack.push(acceptingstate);
        
        // Perform DFS to detect cycles
        while (!stack.empty()) {
            uint16_t current = stack.top();
            stack.pop();
            
            if (visited.find(current) != visited.end()) {
                // Cycle detected is it accepting state to accepting state?
                if (isAcceptingState(current)) {
                    return false;  // Cycle detected from accepting state to accepting state
                }
                continue;  // Already visited, skip non-accepting states
            }
            visited.insert(current);
            
            // Explore neighbors
            auto it = nodeMap.find(current);
            if (it != nodeMap.end()) {
                Node* currentNode = it->second;
                // Check all outgoing edges from the current node
                for (const Edge& edge : currentNode->getEdges()) {
                    uint16_t neighborId = edge.getDstId();
                        stack.push(neighborId);
                }
            }
        }
    }
    // No cycles detected from any accepting state
    return true;
}
