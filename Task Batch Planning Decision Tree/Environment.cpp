#include "Environment.h"

/**
 * Environment - Constructor
 */
Environment::Environment(GeneralTransitionSystem* ts, GridWorld* grid)
    : transitionSystem(ts), gridWorld(grid), environmentName("Environment") {
}

/**
 * Environment - Destructor
 */
Environment::~Environment() {
    // Note: We don't delete the pointers as they're managed externally
}

/**
 * getInitialState - Get the initial state of the transition system
 */
uint32_t Environment::getInitialState() const {
    if (!transitionSystem) {
        return 0;
    }
    return transitionSystem->getInitialState();
}

/**
 * getSuccessorStates - Get successor states for a given state
 */
std::vector<uint32_t> Environment::getSuccessorStates(uint32_t stateId) const {
    if (!transitionSystem) {
        return std::vector<uint32_t>();
    }
    return transitionSystem->getSuccessorIds(stateId);
}

/**
 * isObstacle - Check if a grid cell is an obstacle
 */
bool Environment::isObstacle(uint32_t x, uint32_t y) const {
    if (!gridWorld) {
        return false;
    }
    return gridWorld->isObstacle(x, y);
}

/**
 * isFree - Check if a grid cell is free
 */
bool Environment::isFree(uint32_t x, uint32_t y) const {
    if (!gridWorld) {
        return true;
    }
    return gridWorld->isFree(x, y);
}

/**
 * isValidState - Check if a state is valid in the transition system
 */
bool Environment::isValidState(uint32_t stateId) const {
    if (!transitionSystem) {
        return false;
    }
    return transitionSystem->getNode(stateId) != nullptr;
}

/**
 * getNumStates - Get the number of states in the transition system
 */
uint32_t Environment::getNumStates() const {
    if (!transitionSystem) {
        return 0;
    }
    return transitionSystem->getNumNodes();
}
