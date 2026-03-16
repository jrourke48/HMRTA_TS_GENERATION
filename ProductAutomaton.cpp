#include "ProductAutomaton.h"

ProductAutomaton::ProductAutomaton() {
}

ProductAutomaton::ProductAutomaton(spot::twa_graph_ptr spotAutomaton) {
    if (!spotAutomaton) return;
    
    //Extract number of states from Spot automaton
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
            // Extract edge label/condition if available
            // For now, create unweighted edge
            Edge e(dstState);
            srcNode->addEdge(e);
        }
    }
}

ProductAutomaton::~ProductAutomaton() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void ProductAutomaton::setAccepting(uint32_t stateId) {
    // Add to accepting states if not already present
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    if (it == acceptingStates.end()) {
        acceptingStates.push_back(stateId);
    }
}

bool ProductAutomaton::isAccepting(uint32_t stateId) const {
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    return it != acceptingStates.end();
}

const std::vector<uint32_t>& ProductAutomaton::getAcceptingStates() const {
    return acceptingStates;
}

void ProductAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint32_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool ProductAutomaton::isAdjacent(uint32_t srcId, uint32_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}

void ProductAutomaton::addStateMapping(uint32_t productState, uint32_t tsState, uint32_t automataState) {
    // Ensure stateMapping is large enough
    if (productState >= stateMapping.size()) {
        stateMapping.resize(productState + 1);
    }
    
    stateMapping[productState] = std::make_pair(tsState, automataState);
}

std::pair<uint32_t, uint32_t> ProductAutomaton::getStateMapping(uint32_t productState) const {
    if (productState < stateMapping.size()) {
        return stateMapping[productState];
    }
    
    // Return invalid pair if state not found
    return std::make_pair(UINT32_MAX, UINT32_MAX);
}


