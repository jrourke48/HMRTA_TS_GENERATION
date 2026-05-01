#include "Environment.h"
#include "Point.h"

/**
 * Environment - Constructor
 */
Environment::Environment(GeneralTransitionSystem* ts, GridWorld* grid)
    : transitionSystem(ts), gridWorld(grid) {
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
uint16_t Environment::getInitialState() const {
    if (!transitionSystem || transitionSystem->getInitialStates().empty()) {
        return 0;
    }
    return transitionSystem->getInitialStates()[0];
}

/**
 * getSuccessorStates - Get successor states for a given state
 */
std::vector<uint16_t> Environment::getSuccessorStates(uint16_t stateId) const {
    if (!transitionSystem) {
        return std::vector<uint16_t>();
    }
    return transitionSystem->getAdjacencyIds(stateId);
}

/**
 * isObstacle - Check if a grid cell is an obstacle
 */
bool Environment::isObstacle(Point p) const {
    if (!gridWorld) {
        return false;
    }
    return gridWorld->isObstacle(p);
}

/**
 * isFree - Check if a grid cell is free
 */
bool Environment::isFree(Point p) const {
    if (!gridWorld) {
        return true;
    }
    return gridWorld->isFree(p);
}

/**
 * isValidState - Check if a state is valid in the transition system
 */
bool Environment::isValidState(uint16_t stateId) const {
    if (!transitionSystem) {
        return false;
    }
    return transitionSystem->hasState(stateId);
}

/**
 * getNumStates - Get the number of states in the transition system
 */
uint16_t Environment::getNumStates() const {
    if (!transitionSystem) {
        return 0;
    }
    return transitionSystem->getNumStates();
}

/**
 * gridToStateId - Convert grid coordinates to state ID
 * Encoding: stateId = y * gridWidth + x
 */
uint16_t Environment::gridToStateId(Point p) const {
    if (!gridWorld) {
        return 0;
    }
    return p.getY() * gridWorld->getWidth() + p.getX();
}

/**
 * stateIdToGrid - Convert state ID to grid coordinates
 * Decoding: x = stateId % gridWidth, y = stateId / gridWidth
 */
Point Environment::stateIdToGrid(uint16_t stateId) const {
    if (!gridWorld || gridWorld->getWidth() == 0) {
        return Point(0, 0);
    }
    uint32_t x = stateId % gridWorld->getWidth();
    uint32_t y = stateId / gridWorld->getWidth();
    return Point(x, y);
}
