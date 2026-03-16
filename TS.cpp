#include "TS.h"
#include "Transition_Systems/GridWorldTransitionSystem.h"
#include <algorithm>

TS::TS() {
}

TS::TS(TransitionSystem& ts) {
    // Convert states to integer IDs
    std::unordered_map<std::string, uint32_t> stateIdMap;
    uint32_t stateCounter = 0;
    
    // Create nodes for each state
    for (const auto& state : ts.states) {
        Node* node = new Node(stateCounter);
        add_Node(node);
        
        // Store mapping from state to node ID
        std::string stateKey = std::to_string(ts.stateToId(state));
        stateIdMap[stateKey] = stateCounter;
        
        stateCounter++;
    }
    
    // Mark initial states
    for (const auto& initialState : ts.initial_states) {
        int stateId = ts.stateToId(initialState);
        if (stateIdMap.find(std::to_string(stateId)) != stateIdMap.end()) {
            uint32_t nodeId = stateIdMap[std::to_string(stateId)];
            setInitial(nodeId);
        }
    }
    
    // Add edges based on transitions
    stateCounter = 0;
    for (const auto& state : ts.states) {
        // Get successors for this state
        auto successors = ts.successors(state);
        
        for (const auto& transition : successors) {
            int destStateId = ts.stateToId(transition.next);
            uint32_t destNodeId = stateIdMap[std::to_string(destStateId)];
            
            // Add edge with cost as weight
            Edge edge(destNodeId, static_cast<uint32_t>(transition.cost));
            
            // Get the source node and add edge
            Node* srcNode = getNode(stateCounter);
            if (srcNode != nullptr) {
                srcNode->addEdge(edge);
                numEdges++;
            }
        }
        
        stateCounter++;
    }
}

TS::~TS() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void TS::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint32_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool TS::isAdjacent(uint32_t srcId, uint32_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}

void TS::setInitial(uint32_t stateId) {
    // Add to initial states if not already present
    auto it = std::find(initialStates.begin(), initialStates.end(), stateId);
    if (it == initialStates.end()) {
        initialStates.push_back(stateId);
    }
}

bool TS::isInitial(uint32_t stateId) const {
    auto it = std::find(initialStates.begin(), initialStates.end(), stateId);
    return it != initialStates.end();
}

const std::vector<uint32_t>& TS::getInitialStates() const {
    return initialStates;
}

spot::twa_graph_ptr TS::toSpotAutomaton() const {
    // Create a new Spot automaton
    spot::twa_graph_ptr aut = spot::make_twa_graph();
    
    // Create states in the Spot automaton (one per node)
    for (uint32_t i = 0; i < numNodes; ++i) {
        aut->new_state();
    }
    
    // Set initial state(s)
    if (!initialStates.empty()) {
        aut->set_init_state(initialStates[0]);
    }
    
    // Add edges from the TS graph
    for (const auto& nodePair : nodeMap) {
        uint32_t srcId = nodePair.first;
        Node* srcNode = nodePair.second;
        
        if (srcNode == nullptr) continue;
        
        // Iterate through outgoing edges
        for (const auto& edge : srcNode->getEdges()) {
            uint32_t dstId = edge.getDstId();
            aut->new_edge(srcId, dstId);
        }
    }
    
    return aut;
}

