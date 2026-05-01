#include "GeneralTransitionSystem.h"
#include <algorithm>
#include <iostream>
#include <queue>

/**
 * GeneralTransitionSystem - Constructor
 */
GeneralTransitionSystem::GeneralTransitionSystem()
    : numStates(0) {
}

/**
 * GeneralTransitionSystem - Destructor
 */
GeneralTransitionSystem::~GeneralTransitionSystem() {
    clear();
}

/**
 * addState - Add a state to the transition system
 */
void GeneralTransitionSystem::addState(State* state) {
    if (!state) return;
    
    // Check if state already exists
    if (findStateByAP(state->getAP())) {
        return;
    }
    
    states.push_back(state);
    numStates++;
}

/**
 * removeState - Remove a state by ID
 */
void GeneralTransitionSystem::removeState(uint16_t stateId) {
    auto it = std::find_if(states.begin(), states.end(),
        [stateId](State* s) { return s->getAP() == stateId; });
    
    if (it != states.end()) {
        delete *it;
        states.erase(it);
        numStates--;
    }
}

/**
 * getState - Get a state by ID
 */
State* GeneralTransitionSystem::getState(uint16_t stateId) const {
    for (auto* state : states) {
        if (state->getAP() == stateId) {
            return state;
        }
    }
    return nullptr;
}

/**
 * findStateByAP - Find state by atomic proposition
 */
State* GeneralTransitionSystem::findStateByAP(uint16_t ap) const {
    for (auto* state : states) {
        if (state->getAP() == ap) {
            return state;
        }
    }
    return nullptr;
}

/**
 * hasState - Check if a state exists
 */
bool GeneralTransitionSystem::hasState(uint16_t stateId) const {
    return getState(stateId) != nullptr;
}

/**
 * addAdjecency - Add a transition between two states
 */
void GeneralTransitionSystem::addAdjecency(uint16_t srcId, uint16_t dstId, uint16_t cost) {
    State* srcState = getState(srcId);
    State* dstState = getState(dstId);
    
    if (!srcState || !dstState) return;
    
    // Check if this adjacency already exists
    if (srcState->isAdjacent(dstId)) {
        return;  // Already exists
    }
    
    Transition trans(dstState, cost);
    srcState->addTransition(trans);
}

/**
 * isAdjacent - Check if two states are adjacent
 */
bool GeneralTransitionSystem::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    State* srcState = getState(srcId);
    if (!srcState) return false;
    
    return srcState->isAdjacent(dstId);
}

/**
 * getTransitionsFrom - Get all transitions leaving a state
 */
std::vector<Transition> GeneralTransitionSystem::getTransitionsFrom(uint16_t stateId) const {
    State* state = getState(stateId);
    if (!state) return std::vector<Transition>();
    return state->getAdjacency();
}

/**
 * getTransitionsTo - Get all transitions entering a state
 */
std::vector<Transition> GeneralTransitionSystem::getTransitionsTo(uint16_t stateId) const {
    std::vector<Transition> result;
    for (auto* state : states) {
        for (const auto& trans : state->getAdjacency()) {
            if (trans.next->getAP() == stateId) {
                result.push_back(trans);
            }
        }
    }
    return result;
}

/**
 * getNumEdges - Count total number of transitions
 */
uint16_t GeneralTransitionSystem::getNumEdges() const {
    uint16_t count = 0;
    for (auto* state : states) {
        count += state->getNumAdjacency();
    }
    return count;
}

/**
 * getAdjacency - Get all adjacent state pointers
 */
std::vector<State*> GeneralTransitionSystem::getAdjacency(uint16_t stateId) const {
    std::vector<State*> result;
    State* state = getState(stateId);
    if (!state) return result;
    
    for (const auto& trans : state->getAdjacency()) {
        State* adj = getState(trans.next->getAP());
        if (adj) {
            result.push_back(adj);
        }
    }
    return result;
}

/**
 * getAdjacencyIds - Get IDs of all adjacent states
 */
std::vector<uint16_t> GeneralTransitionSystem::getAdjacencyIds(uint16_t stateId) const {
    std::vector<uint16_t> result;
    State* state = getState(stateId);
    if (!state) return result;
    
    for (const auto& trans : state->getAdjacency()) {
        result.push_back(trans.next->getAP());
    }
    return result;
}

/**
 * addInitialState - Add an initial state
 */
void GeneralTransitionSystem::addInitialState(uint16_t stateId) {
    if (!isInitialState(stateId)) {
        initialStates.push_back(stateId);
    }
}

/**
 * removeInitialState - Remove an initial state
 */
void GeneralTransitionSystem::removeInitialState(uint16_t stateId) {
    auto it = std::find(initialStates.begin(), initialStates.end(), stateId);
    if (it != initialStates.end()) {
        initialStates.erase(it);
    }
}

/**
 * isInitialState - Check if a state is initial
 */
bool GeneralTransitionSystem::isInitialState(uint16_t stateId) const {
    return std::find(initialStates.begin(), initialStates.end(), stateId) != initialStates.end();
}

/**
 * getTransitionCost - Get cost of a transition
 */
uint16_t GeneralTransitionSystem::getTransitionCost(uint16_t srcId, uint16_t dstId) const {
    State* srcState = getState(srcId);
    if (!srcState) return 0;
    
    for (const auto& trans : srcState->getAdjacency()) {
        if (trans.next->getAP() == dstId) {
            return trans.cost;
        }
    }
    return 0;
}

/**
 * setTransitionCost - Set cost of a transition
 */
void GeneralTransitionSystem::setTransitionCost(uint16_t srcId, uint16_t dstId, uint16_t cost) {
    State* srcState = getState(srcId);
    if (!srcState) return;
    
    for (const auto& trans : srcState->getAdjacency()) {
        if (trans.next->getAP() == dstId) {
            // Note: Cannot modify const transition from getAdjacency()
            // Cost is fixed at construction time
            return;
        }
    }
}

/**
 * clear - Clear all states and transitions
 */
void GeneralTransitionSystem::clear() {
    for (auto* state : states) {
        delete state;
    }
    states.clear();
    initialStates.clear();
    numStates = 0;
}

/**
 * printStates - Print all states
 */
void GeneralTransitionSystem::printStates() const {
    std::cout << "States (" << numStates << "):\n";
    for (auto* state : states) {
        std::cout << "  State AP=" << state->getAP() << " Name=\"" << state->getName() << "\"\n";
    }
}

/**
 * printTransitions - Print all transitions
 */
void GeneralTransitionSystem::printTransitions() const {
    std::cout << "Transitions (" << getNumEdges() << "):\n";
    for (auto* state : states) {
        for (const auto& trans : state->getAdjacency()) {
            std::cout << "  " << state->getAP() << " -> " << trans.next->getAP() 
                      << " (cost=" << trans.cost << ")\n";
        }
    }
}

/**
 * isConnected - Check if graph is connected using BFS from first initial state
 */
bool GeneralTransitionSystem::isConnected() const {
    if (states.empty() || initialStates.empty()) return false;
    
    std::queue<uint16_t> q;
    std::vector<bool> visited(65536, false);
    
    q.push(initialStates[0]);
    visited[initialStates[0]] = true;
    uint16_t visitedCount = 1;
    
    while (!q.empty()) {
        uint16_t current = q.front();
        q.pop();
        
        std::vector<uint16_t> successors = getAdjacencyIds(current);
        for (uint16_t successor : successors) {
            if (!visited[successor]) {
                visited[successor] = true;
                visitedCount++;
                q.push(successor);
            }
        }
    }
    
    return visitedCount == numStates;
}

/**
 * getReachableStates - Get all states reachable from a starting state using BFS
 */
std::vector<uint16_t> GeneralTransitionSystem::getReachableStates(uint16_t startStateId) const {
    std::vector<uint16_t> result;
    std::queue<uint16_t> q;
    std::vector<bool> visited(65536, false);
    
    if (!hasState(startStateId)) return result;
    
    q.push(startStateId);
    visited[startStateId] = true;
    result.push_back(startStateId);
    
    while (!q.empty()) {
        uint16_t current = q.front();
        q.pop();
        
        std::vector<uint16_t> successors = getAdjacencyIds(current);
        for (uint16_t successor : successors) {
            if (!visited[successor]) {
                visited[successor] = true;
                result.push_back(successor);
                q.push(successor);
            }
        }
    }
    
    return result;
}

/**
 * Transition::toString - Convert transition to string representation
 */
std::string Transition::toString() const {
    return "Transition to AP=" + std::to_string(next->getAP()) + " (cost=" + std::to_string(cost) + ")";
}