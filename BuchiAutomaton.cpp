#include "BuchiAutomaton.h"
#include <algorithm>

BuchiAutomaton::BuchiAutomaton() {
}

BuchiAutomaton::~BuchiAutomaton() {
    // Clean up dynamically allocated nodes
    // Note: Node pointers should be managed by a smart pointer in a production system
}
BuchiAutomaton::BuchiAutomaton(spot::twa_graph_ptr spotAutomaton) {
    if (!spotAutomaton) return;
    
    // Clear existing data
    nodeMap.clear();
    acceptingStates.clear();
    numNodes = 0;
    numEdges = 0;
    
    // Extract number of states from Spot automaton
    unsigned numStates = spotAutomaton->num_states();
    
    // Create nodes for each state
    for (unsigned i = 0; i < numStates; ++i) {
        Node* node = new Node(i);
        add_Node(node);
    }
    
    // Extract edges from Spot automaton
    for (const auto& edge : spotAutomaton->edges()) {
        unsigned srcState = edge.src;
        unsigned dstState = edge.dst;
        
        // Get source node and add edge
        Node* srcNode = getNode(srcState);
        if (srcNode != nullptr) {
            Edge e(dstState);
            srcNode->addEdge(e);
            numEdges++;
        }
    }
    
    // Mark accepting states (states in Büchi acceptance sets)
    for (unsigned i = 0; i < numStates; ++i) {
        if (spotAutomaton->state_is_accepting(i)) {
            setAccepting(i);
        }
    }
}

void BuchiAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint32_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool BuchiAutomaton::isAdjacent(uint32_t srcId, uint32_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}
