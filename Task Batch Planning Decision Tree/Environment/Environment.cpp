#include "Environment.h"
#include "Point.h"
#include <iostream>
#include <iomanip>

/**
 * Environment - Constructor
 */
Environment::Environment(TS* ts, GridWorld* grid)
    : transitionSystem(ts), gridWorld(grid) {
}

/**
 * Environment - Destructor
 */
Environment::~Environment() {
    // don't delete the pointers as they're managed externally
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
std::vector<uint32_t> Environment::getSuccessorStates(uint32_t stateId) const {
    if (!transitionSystem) {
        return std::vector<uint32_t>();
    }
    return transitionSystem->getAdjacent(stateId);
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
bool Environment::isValidState(uint32_t stateId) const {
    if (!transitionSystem) {
        return false;
    }
    // Check if state exists in the state-to-grid mapping
    return stateIdToGridMap.find(stateId) != stateIdToGridMap.end();
}

/**
 * getNumStates - Get the number of states in the transition system
 */
uint32_t Environment::getNumStates() const {
    if (!transitionSystem) {
        return 0;
    }
    return transitionSystem->getNumStates();
}

// Bidirectional mapping between GridWorld and TransitionSystem
uint32_t Environment::gridToTSStateId(Point p) const {
    // Find which state's region contains this grid point
    for (const auto& mapping : stateIdToGridMap) {
        const stateGridMapping& region = mapping.second;
        uint16_t half_width = region.width / 2;
        uint16_t half_height = region.height / 2;
        
        if (p.getX() >= region.center.getX() - half_width && p.getX() < region.center.getX() + half_width &&
            p.getY() >= region.center.getY() - half_height && p.getY() < region.center.getY() + half_height) {
            return mapping.first;
        }
    }
    return 0; // Default to state 0 if not found
}

void Environment::mapTSStateToGrid(uint16_t stateId, Point center, uint16_t width, uint16_t height) {
    stateIdToGridMap[stateId] = {center, width, height};
}
void Environment::print_Environment() const {
    if (!gridWorld || !transitionSystem) {
        std::cout << "Cannot print environment: GridWorld or TransitionSystem is null" << std::endl;
        return;
    }
    
    uint32_t width = gridWorld->getWidth();
    uint32_t height = gridWorld->getHeight();
    
    std::cout << "\n========== ASCII Environment Visualization ==========" << std::endl;
    std::cout << "Grid: " << width << "x" << height << " | States: " << transitionSystem->getNumStates() << std::endl;
    std::cout << "Legend: . = free, # = obstacle, [0-9] = state center\n" << std::endl;
    
    // Print column headers
    std::cout << "    ";
    for (uint32_t x = 0; x < width; x++) {
        if (x % 10 == 0) std::cout << (x / 10) % 10;
        else std::cout << " ";
    }
    std::cout << std::endl;
    
    std::cout << "    ";
    for (uint32_t x = 0; x < width; x++) {
        std::cout << (x % 10);
    }
    std::cout << std::endl;
    
    // Print grid
    for (uint32_t y = 0; y < height; y++) {
        std::cout << std::setw(3) << y << "|";
        
        for (uint32_t x = 0; x < width; x++) {
            Point p(x, y);
            char cell = '.';
            
            // Check if obstacle
            if (gridWorld->isObstacle(p)) {
                cell = '#';
            }
            
            // Check if state center is here
            for (const auto& mapping : stateIdToGridMap) {
                if (mapping.second.center.getX() == x && mapping.second.center.getY() == y) {
                    uint16_t stateId = mapping.first;
                    if (stateId < 10) {
                        cell = '0' + stateId;
                    } else {
                        cell = 'A' + (stateId - 10);
                    }
                    break;
                }
            }
            
            std::cout << cell;
        }
        
        std::cout << "|" << std::endl;
    }
    
    // Print row indicators
    std::cout << "    ";
    for (uint32_t x = 0; x < width; x++) {
        std::cout << (x % 10);
    }
    std::cout << std::endl;
    std::cout << "====================================================\n" << std::endl;
}
