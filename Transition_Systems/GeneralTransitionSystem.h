#ifndef GENERAL_TRANSITION_SYSTEM_H
#define GENERAL_TRANSITION_SYSTEM_H

#include "../Automatons/Edge_Node.h"
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>

/**
 * GeneralTransitionSystem - Base class for transition systems
 * Assumptions: 
 * each state is represented by a unique uint16_t ID that is the states respective single true atomic proposition
 * and transitions are represented as edges between states.
 */

// Forward declaration
class State;

// Transition represents a transition with an action and cost
class Transition {
public:
    State* next;  // Use pointer to avoid circular dependency
    uint16_t cost{ 1 };
        
    Transition(State* nextState = nullptr) : next(nextState) {}
    Transition(State* nextState, uint16_t transitionCost) : next(nextState), cost(transitionCost) {}
    bool operator==(const Transition& other) const {
        return next == other.next && cost == other.cost;
    }
    std::string toString() const;
};

// State represents a position on a 2D grid
class State {
private:
    // Set of atomic propositions true in this state
    uint16_t AP; // For simplicity, we can represent the state as a single atomic proposition ID (since each state corresponds to one true AP)
    std::vector<Transition> adjecency; // adjacency list of transitions from this state
    uint16_t numadj = 0; // number of adjacent states (transitions)
    std::string Name; 
public:
    State(uint16_t ap, const std::string& name = "") : AP(ap), Name(name) {}
    State(const State& other) : AP(other.AP), adjecency(other.adjecency), numadj(other.numadj), Name(other.Name) {}
    uint16_t getAP() const { return AP; }
    const std::vector<Transition>& getAdjacency() const { return adjecency; }
    uint16_t getNumAdjacency() const { return numadj; }
    const std::string& getName() const { return Name; }
    void addTransition(const Transition& t) {
        adjecency.push_back(t);
        numadj++;
    }
    bool isAdjacent(uint16_t dstId) const {
        for (size_t i = 0; i < numadj; i++)
        {
            if (adjecency[i].next != nullptr && adjecency[i].next->getAP() == dstId) {
                return true;
            }
        }
        return false;
    }
    bool operator==(const State& other) const {
        return AP == other.AP;
    }

};

class GeneralTransitionSystem {
protected:
    std::vector<State*> states;  // Collection of all states
    uint16_t numStates;
    std::vector<uint16_t> initialStates;
    
public:
    // Constructor and Destructor
    GeneralTransitionSystem();
    virtual ~GeneralTransitionSystem();
    
    // State management
    virtual void addState(State* state);
    virtual void removeState(uint16_t stateId);
    State* getState(uint16_t stateId) const;
    State* findStateByAP(uint16_t ap) const;
    bool hasState(uint16_t stateId) const;
    const std::vector<State*>& getAllStates() const { return states; }
    
    // Adjacency/Edge management
    virtual void addAdjecency(uint16_t srcId, uint16_t dstId, uint16_t cost = 1);
    virtual bool isAdjacent(uint16_t srcId, uint16_t dstId) const;
    virtual std::vector<Transition> getTransitionsFrom(uint16_t stateId) const;
    virtual std::vector<Transition> getTransitionsTo(uint16_t stateId) const;
    
    // Query methods
    uint16_t getNumStates() const { return numStates; }
    uint16_t getNumEdges() const;
    virtual std::vector<State*> getAdjacency(uint16_t stateId) const;
    virtual std::vector<uint16_t> getAdjacencyIds(uint16_t stateId) const;
    
    // Initial state management
    void addInitialState(uint16_t stateId);
    void removeInitialState(uint16_t stateId);
    const std::vector<uint16_t>& getInitialStates() const { return initialStates; }
    bool isInitialState(uint16_t stateId) const;
    void clearInitialStates() { initialStates.clear(); }
    
    // Utility methods
    virtual void clear();
    uint16_t getTransitionCost(uint16_t srcId, uint16_t dstId) const;
    void setTransitionCost(uint16_t srcId, uint16_t dstId, uint16_t cost);
    void printStates() const;
    void printTransitions() const;
    bool isConnected() const;
    std::vector<uint16_t> getReachableStates(uint16_t startStateId) const;
};

#endif // GENERAL_TRANSITION_SYSTEM_H
