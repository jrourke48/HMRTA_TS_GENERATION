#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "../Transition_Systems/GeneralTransitionSystem.h"
#include "GridWorld.h"
#include <cstdint>
#include <vector>
#include <string>

/**
 * Environment - Represents the environment/transition system
 * Encapsulates a GeneralTransitionSystem instance and a 2D GridWorld costmap
 */
class Environment {
private:
    GeneralTransitionSystem* transitionSystem;
    GridWorld* gridWorld;  // 2D global costmap
    
public:
    // Constructor
    Environment(GeneralTransitionSystem* ts = nullptr, GridWorld* grid = nullptr);
    
    // Destructor
    ~Environment();
    
    // Getters
    GeneralTransitionSystem* getTransitionSystem() const { return transitionSystem; }
    GridWorld* getGridWorld() const { return gridWorld; }
    
    // Setters
    void setTransitionSystem(GeneralTransitionSystem* ts) { transitionSystem = ts; }
    void setGridWorld(GridWorld* grid) { gridWorld = grid; }
    
    // State queries
    uint16_t getInitialState() const;
    std::vector<uint16_t> getSuccessorStates(uint16_t stateId) const;
    
    // Bidirectional mapping between GridWorld and TransitionSystem
    uint16_t gridToStateId(Point p) const;      // Convert grid coordinates to state ID
    Point stateIdToGrid(uint16_t stateId) const; // Convert state ID to grid coordinates
    
    // Grid queries
    bool isObstacle(Point p) const;
    bool isFree(Point p) const;
    
    // Utility methods
    bool isValidState(uint16_t stateId) const;
    uint16_t getNumStates() const;
};

#endif // ENVIRONMENT_H
