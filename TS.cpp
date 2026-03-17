#include "TS.h"
#include "Transition_Systems/GridWorldTransitionSystem.h"
#include <algorithm>
#include <iostream>

TS::TS() {
}

TS::TS(TransitionSystem* ts) {
    if (!ts) return;
    std::cerr << "DEBUG: Starting TS constructor\n";
    std::cerr << "DEBUG: ts->states.size() = " << ts->states.size() << "\n";
    
    // Convert states to integer IDs
    std::unordered_map<std::string, uint32_t> stateIdMap;
    uint32_t stateCounter = 0;
    
    // Create nodes for each state
    std::cerr << "DEBUG: Creating nodes...\n";
    for (const auto& state : ts->states) {
        std::cerr << "DEBUG: Creating node " << stateCounter << "\n";
        Node* node = new Node(stateCounter);
        add_Node(node);
        
        // Store mapping from state to node ID
        std::string stateKey = std::to_string(ts->stateToId(state));
        stateIdMap[stateKey] = stateCounter;
        
        stateCounter++;
    }
    
    std::cerr << "DEBUG: Nodes created. Initial states count: " << ts->initial_states.size() << "\n";
    // Mark initial states - mark state 0 as initial for now
    // (the initial state marking will be handled differently)
    if (ts->initial_states.size() > 0) {
        std::cerr << "DEBUG: Marking first state as initial\n";
        setInitial(0);  // Mark the first created node as initial
    }
    
    std::cerr << "DEBUG: Adding edges...\n";
    // Add edges based on transitions
    stateCounter = 0;
    for (const auto& state : ts->states) {
        std::cerr << "DEBUG: Processing state " << stateCounter << "\n";
        std::cerr << "DEBUG: State has " << state.props.size() << " props\n";
        // Get successors for this state
        try {
            std::cerr << "DEBUG: About to call successors...\n";
            std::cerr << "DEBUG: State ID = " << ts->stateToId(state) << "\n";
            auto successors = ts->successors(state);
            std::cerr << "DEBUG: Got " << successors.size() << " successors\n";
        
            for (const auto& transition : successors) {
                int destStateId = ts->stateToId(transition.next);
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
        } catch (const std::exception& e) {
            std::cerr << "DEBUG: Exception in successors: " << e.what() << "\n";
        }
        
        stateCounter++;
    }
    std::cerr << "DEBUG: TS constructor complete\n";
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
    // Create a BDD dictionary for the automaton
    spot::bdd_dict_ptr dict = spot::make_bdd_dict();
    
    // Create a new Spot automaton with the dictionary
    spot::twa_graph_ptr aut = spot::make_twa_graph(dict);
    
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
            // Add edge with bddtrue (true transition) and no acceptance mark
            aut->new_edge(srcId, dstId, bddtrue, {0});
        }
    }
    
    return aut;
}

