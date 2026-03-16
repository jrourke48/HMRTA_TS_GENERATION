#ifndef BUCHI_AUTOMATON_H
#define BUCHI_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>

class BuchiAutomaton : public Automaton {
private:
    std::vector<uint32_t> acceptingStates;  // Set of accepting states (Büchi accepting states)

public:
    BuchiAutomaton();
    ~BuchiAutomaton() override;


    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;

    // Büchi-specific methods
    void setAccepting(uint32_t stateId);
    bool isAccepting(uint32_t stateId) const;
    const std::vector<uint32_t>& getAcceptingStates() const;

    // Convert from Spot automaton
    void fromSpotAutomaton(spot::twa_graph_ptr spotAutomaton);
};

#endif
