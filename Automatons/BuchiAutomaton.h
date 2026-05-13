#ifndef BUCHI_AUTOMATON_H
#define BUCHI_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <string>
#include <memory>
#include <algorithm>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>
#include "../Task Batch Planning Decision Tree/LTLFormula/LTLFormula.h"
#include "../Task Batch Planning Decision Tree/LTLFormula/BatchAtomicProposition.h"

class BuchiAutomaton : public Automaton {
private:
    std::vector<uint32_t> acceptingStates;  // Set of accepting states (Büchi accepting states)
    LTLFormula ltlFormula;  // LTL formula associated with the Büchi automaton

public:
    BuchiAutomaton();
    ~BuchiAutomaton() override;
    BuchiAutomaton(spot::twa_graph_ptr spotAutomaton);

    // Constructor that takes an LTL formula string and converts it to Büchi automaton
    BuchiAutomaton(const LTLFormula& ltlFormula) {
        try {
            this->ltlFormula = ltlFormula.getSpotFormula();
            // construct spot buchi
            spot::translator trans;
            spot::twa_graph_ptr buchiAut = trans.run(ltlFormula);
            // convert spot buchi to our buchi automaton
            fromSpotAutomaton(buchiAut);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to create Buchi automaton from formula: " + std::string(e.what()));
        }
    }

    // Constructor that takes an LTL formula object
    BuchiAutomaton(const LTLFormula& formula) {
        try {
            this->ltlFormula = formula.getSpotFormula();
            // construct spot buchi
            spot::translator trans;
            spot::twa_graph_ptr buchiAut = trans.run(ltlFormula);

            // convert spot buchi to our buchi automaton
            fromSpotAutomaton(buchiAut);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to create Buchi automaton from formula: " + std::string(e.what()));
        }
    }
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
        // For Büchi: mark states that are targets of accepting transitions
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
            
            // Check if this edge has acceptance marks (transition-based acceptance)
            // Mark destination state as accepting if the transition is accepting
            if (edge.acc != spot::acc_cond::mark_t()) {
                setAccepting(dstState);
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

    // Method to get the Spot dictionary for accessing atomic propositions
    spot::formula get_ltl_formula() const {
        return ltlFormula;
    };

};

#endif
