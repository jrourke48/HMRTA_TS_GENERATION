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
    std::string environmentName;
    
public:
    // Constructor
    Environment(GeneralTransitionSystem* ts = nullptr, GridWorld* grid = nullptr);
    
    // Destructor
    ~Environment();
    
    // Getters
    GeneralTransitionSystem* getTransitionSystem() const { return transitionSystem; }
    GridWorld* getGridWorld() const { return gridWorld; }
    std::string getEnvironmentName() const { return environmentName; }
    
    // Setters
    void setTransitionSystem(GeneralTransitionSystem* ts) { transitionSystem = ts; }
    void setGridWorld(GridWorld* grid) { gridWorld = grid; }
    void setEnvironmentName(const std::string& name) { environmentName = name; }
    
    // State queries
    uint32_t getInitialState() const;
    std::vector<uint32_t> getSuccessorStates(uint32_t stateId) const;
    
    // Grid queries
    bool isObstacle(uint32_t x, uint32_t y) const;
    bool isFree(uint32_t x, uint32_t y) const;
    
    // Utility methods
    bool isValidState(uint32_t stateId) const;
    uint32_t getNumStates() const;
};

#endif // ENVIRONMENT_H
