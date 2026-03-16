#ifndef PRODUCT_AUTOMATON_H
#define PRODUCT_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <spot/twa/twagraph.hh>

class ProductAutomaton : public Automaton {
private:
    std::vector<std::pair<uint32_t, uint32_t>> stateMapping;  // Maps product states to (ts_state, automaton_state)
    std::vector<uint32_t> acceptingStates;  // Set of accepting states
public:
    ProductAutomaton();
    
    // Constructor from Spot twa_graph
    ProductAutomaton(spot::twa_graph_ptr spotAutomaton);
    
    ~ProductAutomaton() override;

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;
    
    void setAccepting(uint32_t stateId);
    bool isAccepting(uint32_t stateId) const;
    const std::vector<uint32_t>& getAcceptingStates() const;

    // Product-specific methods
    void addStateMapping(uint32_t productState, uint32_t tsState, uint32_t automataState);
    std::pair<uint32_t, uint32_t> getStateMapping(uint32_t productState) const;
};

#endif
