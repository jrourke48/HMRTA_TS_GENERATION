#ifndef PRODUCT_AUTOMATON_H
#define PRODUCT_AUTOMATON_H

#include "Automaton.h"
#include "TS.h"
#include "BuchiAutomaton.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <spot/twa/twagraph.hh>

class ProductAutomaton : public Automaton {
private:
    std::vector<std::pair<uint16_t, uint16_t>> stateMapping;  // Maps product states to (ts_state, automaton_state)
    std::vector<uint16_t> acceptingStates;  // Set of accepting states
public:
    ProductAutomaton();
    
    // Constructor from Spot twa_graph
    ProductAutomaton(spot::twa_graph_ptr spotAutomaton);

    //Constructor from individual components (TS, Mult-Robot System, and automaton states)
    ProductAutomaton(const TS& ts, const MultiRobotSystem& mrs, const BuchiAutomaton& buchiAutomaton);
    
    ~ProductAutomaton() override;

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint16_t srcId, uint16_t dstId) const override;
    
    void setAccepting(uint16_t stateId);
    bool isAccepting(uint16_t stateId) const;
    const std::vector<uint16_t>& getAcceptingStates() const;

    // Product-specific methods
    void addStateMapping(uint16_t productState, uint16_t tsState, uint16_t automataState);
    std::pair<uint16_t, uint16_t> getStateMapping(uint16_t productState) const;
};

#endif
