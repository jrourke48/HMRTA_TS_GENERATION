#include "TS.h"
#include "../Transition_Systems/GridWorldTransitionSystem.h"
#include <algorithm>
#include <iostream>

TS::TS() {
}

TS::TS(TransitionSystem* ts) {
    if (!ts) return;
    
    // Create nodes using the actual state IDs from GridWorld
    for (const auto& state : ts->states) {
        uint32_t stateId = ts->stateToId(state);
        Node* node = new Node(stateId);
        add_Node(node);
    }
    
    // Mark initial states based on the GridWorld's initial states
    if (ts->initial_states.size() > 0) {
        for (const auto& initState : ts->initial_states) {
            uint32_t initStateId = ts->stateToId(initState);
            setInitial(initStateId);
        }
    }
    
    // Add edges based on transitions
    for (const auto& state : ts->states) {
        uint32_t srcStateId = ts->stateToId(state);
        // Get successors for this state
        try {
            auto successors = ts->successors(state);
        
            for (const auto& transition : successors) {
                uint32_t destStateId = ts->stateToId(transition.next);
                
                // Add edge with cost as weight
                Edge edge(destStateId, static_cast<uint32_t>(transition.cost));
                
                // Get the source node using actual state ID
                Node* srcNode = getNode(srcStateId);
                if (srcNode != nullptr) {
                    srcNode->addEdge(edge);
                    numEdges++;
                }
            }
        } catch (const std::exception& e) {
            // Handle exception silently
        }
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

spot::twa_graph_ptr TS::toSpotAutomaton(spot::bdd_dict_ptr dict) const {
    // Use provided dictionary or create a new one
    if (!dict) {
        dict = spot::make_bdd_dict();
    }
    
    // Create a new Spot automaton with the dictionary
    spot::twa_graph_ptr aut = spot::make_twa_graph(dict);
    
    // Create a mapping from node ID to Spot state index
    std::unordered_map<uint32_t, unsigned> nodeIdToSpotState;
    unsigned spotStateIndex = 0;
    
    // Create states in the Spot automaton (one per node)
    for (uint32_t i = 0; i < numNodes; ++i) {
        aut->new_state();
    }
    
    // Build the mapping from node IDs to Spot state indices
    for (const auto& nodePair : nodeMap) {
        uint32_t nodeId = nodePair.first;
        nodeIdToSpotState[nodeId] = spotStateIndex++;
    }
    
    // Set initial state(s) using the mapping
    if (!initialStates.empty()) {
        uint32_t initNodeId = initialStates[0];
        if (nodeIdToSpotState.find(initNodeId) != nodeIdToSpotState.end()) {
            aut->set_init_state(nodeIdToSpotState[initNodeId]);
        }
    }
    
    // Add edges from the TS graph
    for (const auto& nodePair : nodeMap) {
        uint32_t srcId = nodePair.first;
        Node* srcNode = nodePair.second;
        
        if (srcNode == nullptr) continue;
        
        unsigned srcSpotState = nodeIdToSpotState[srcId];
        
        // Iterate through outgoing edges
        for (const auto& edge : srcNode->getEdges()) {
            uint32_t dstId = edge.getDstId();
            unsigned dstSpotState = nodeIdToSpotState[dstId];
            // Add edge with bddtrue (true transition) and no acceptance mark
            aut->new_edge(srcSpotState, dstSpotState, bddtrue, {0});
        }
    }
    
    return aut;
}

