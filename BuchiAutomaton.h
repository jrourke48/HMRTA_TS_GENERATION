#ifndef BUCHI_AUTOMATON_H
#define BUCHI_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <algorithm>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>

class BuchiAutomaton : public Automaton {
private:
    std::vector<uint32_t> acceptingStates;  // Set of accepting states (Büchi accepting states)
public:
    BuchiAutomaton();
    ~BuchiAutomaton() override;
    BuchiAutomaton(spot::twa_graph_ptr spotAutomaton);

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;

    // Büchi-specific methods
    void setAccepting(uint32_t stateId) {
        // Add to accepting states if not already present
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        if (it == acceptingStates.end()) {
            acceptingStates.push_back(stateId);
        }
    };
    void fromSpotAutomaton(spot::twa_graph_ptr spotAutomaton) {
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
    };
    bool isAccepting(uint32_t stateId) const {
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        return it != acceptingStates.end();
    };
    const std::vector<uint32_t>& getAcceptingStates() const {
        return acceptingStates;
    };

};

#endif
