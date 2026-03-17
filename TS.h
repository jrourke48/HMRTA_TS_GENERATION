#ifndef TS_H
#define TS_H

#include "Automaton.h"
#include "Edge_Node.h"
#include <cstdint>
#include <vector>
#include <memory>
#include <unordered_map>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>

// Forward declaration
class TransitionSystem;

class TS : public Automaton {
private:
    std::vector<uint32_t> initialStates;  // Track which states are initial states
    std::unordered_map<uint32_t, uint32_t> stateToNodeId;  // Map from state index to node ID

public:
    TS();
    
    // Constructor from GridWorldTransitionSystem
    TS(TransitionSystem* ts);
    
    ~TS() override;

    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint32_t srcId, uint32_t dstId) const override;

    // TS-specific methods
    void setInitial(uint32_t stateId);
    bool isInitial(uint32_t stateId) const;
    const std::vector<uint32_t>& getInitialStates() const;

    // Convert TS to Spot automaton
    spot::twa_graph_ptr toSpotAutomaton() const;
};

#endif
