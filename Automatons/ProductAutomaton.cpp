#include "ProductAutomaton.h"

ProductAutomaton::ProductAutomaton() {
}

ProductAutomaton::ProductAutomaton(spot::twa_graph_ptr spotAutomaton) {
    if (!spotAutomaton) return;
    
    //Extract number of states from Spot automaton
    unsigned numStates = spotAutomaton->num_states();
    
    // Create nodes for each state
    for (unsigned i = 0; i < numStates && i <= UINT16_MAX; ++i) {
        Node* node = new Node(static_cast<uint16_t>(i));
        add_Node(node);
    }
    
    // Extract edges from Spot automaton
    // For transition-based acceptance: mark states that are targets of accepting transitions
    for (const auto& edge : spotAutomaton->edges()) {
        unsigned srcState = edge.src;
        unsigned dstState = edge.dst;
        
        // Get source node and add edge
        Node* srcNode = getNode(static_cast<uint16_t>(srcState));
        if (srcNode != nullptr) {
            // Extract edge label/condition if available
            // For now, create unweighted edge
            Edge e(static_cast<uint16_t>(dstState));
            srcNode->addEdge(e);
            numEdges++;
        }
        
        // Check if this edge has acceptance marks (transition-based acceptance)
        // Mark destination state as accepting if the transition is accepting
        if (edge.acc != spot::acc_cond::mark_t()) {
            setAccepting(static_cast<uint16_t>(dstState));
        }
    }
}
ProductAutomaton::ProductAutomaton(const TS& ts, const MultiRobotSystem& mrs, const BuchiAutomaton& buchiAutomaton) {
    // Initialize product automaton based on the individual components
    // This is a placeholder implementation and should be replaced with actual logic
    if (buchiAutomaton.getNumStates() > 100 || ts.getNumStates() > 20 || mrs.getNumRobots() > 10) return;
    
    // Get the number of robots in the multi-robot system 
    uint8_t numRobots = mrs.getNumRobots();
    
    // Convert the Buchi automaton to a Spot automaton
    spot::twa_graph_ptr buchiSpot = buchiAutomaton.getSpotAutomaton();
    
    // Convert the transition system to a Spot automaton
    spot::twa_graph_ptr tsSpot = ts.toSpotAutomaton(buchiSpot->get_dict());

    // Initialize the product automaton as the transition system Spot automaton
    spot::twa_graph_ptr productSpot = tsSpot;
    // Create synchronized product of each TS for each robot
    for (uint8_t i = 0; i < numRobots; ++i) {
        productSpot = spot::product(tsSpot, productSpot);
    }
    // do the final synchronization of the product automaton
    productSpot = spot::product(buchiSpot, productSpot);
    // At this point, productSpot contains the synchronized product of all TS and the Buchi automaton so call the other constructor to convert it into the internal representation of ProductAutomaton
    *this = ProductAutomaton(productSpot);
}

ProductAutomaton::~ProductAutomaton() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void ProductAutomaton::setAccepting(uint16_t stateId) {
    // Add to accepting states if not already present
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    if (it == acceptingStates.end()) {
        acceptingStates.push_back(stateId);
    }
}

bool ProductAutomaton::isAccepting(uint16_t stateId) const {
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    return it != acceptingStates.end();
}

const std::vector<uint16_t>& ProductAutomaton::getAcceptingStates() const {
    return acceptingStates;
}

void ProductAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool ProductAutomaton::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}

void ProductAutomaton::addStateMapping(uint16_t productState, uint16_t tsState, uint16_t automataState) {
    // Ensure stateMapping is large enough
    if (productState >= stateMapping.size()) {
        stateMapping.resize(productState + 1);
    }
    
    stateMapping[productState] = std::make_pair(tsState, automataState);
}

std::pair<uint16_t, uint16_t> ProductAutomaton::getStateMapping(uint16_t productState) const {
    if (productState < stateMapping.size()) {
        return stateMapping[productState];
    }
    
    // Return invalid pair if state not found
    return std::make_pair(UINT32_MAX, UINT32_MAX);
}


