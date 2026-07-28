#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include "../Automatons/TS.h"
#include "GridWorld.h"
#include "Point.h"
#include <cstdint>
#include <vector>
#include <string>
#include <unordered_map>

/**
 * Environment - Represents the environment/transition system
 * Encapsulates a GeneralTransitionSystem instance and a 2D GridWorld costmap
 */
class Environment {
public:
    struct stateGridMapping {
        Point center; // Center point of the grid cell corresponding to this state
        uint16_t width;    // Width of the grid cell
        uint16_t height;   // Height of the grid cell
    };
private:
    TS* transitionSystem;
    GridWorld* gridWorld;  // 2D global costmap
    std::unordered_map<uint16_t, stateGridMapping> stateIdToGridMap; // Map from transition system state ID to grid coordinates
    
public:
    // Constructor
    Environment(TS* ts = nullptr, GridWorld* grid = nullptr);
    
    // Destructor
    ~Environment();
    
    // Getters
    TS* getTransitionSystem() const { return transitionSystem; }
    GridWorld* getGridWorld() const { return gridWorld; }
    
    // Setters
    void setTransitionSystem(TS* ts) { transitionSystem = ts; }
    void setGridWorld(GridWorld* grid) { gridWorld = grid; }
    
    // State queries
    uint32_t getInitialState() const;
    std::vector<uint32_t> getSuccessorStates(uint32_t stateId) const;
    
    // Bidirectional mapping between GridWorld and TransitionSystem
    uint32_t gridToTSStateId(Point p) const;      // Convert grid coordinates to state ID
    Point TSStateIdToGridCenter(uint32_t stateId) const {
        auto id = stateIdToGridMap.find(stateId);
        if (id != stateIdToGridMap.end()) {
                return id->second.center;
            }
        return Point(0, 0); // Default if not found
    } // Convert state ID to grid coordinates
    void mapTSStateToGrid(uint32_t stateId, Point center, uint16_t width, uint16_t height); 
    // Map a state ID to a grid area
    
    // Obstacle and free space queries
    bool isObstacle(Point p) const;
    bool isFree(Point p) const;
    //add obstacle
    void addObstacle(Point p);
    //clear obstacle
    void clearObstacle(Point p);
    
    // Utility methods
    bool isValidState(uint32_t stateId) const;
    uint32_t getNumStates() const;
    void print_Environment() const;
};

#endif // ENVIRONMENT_H
