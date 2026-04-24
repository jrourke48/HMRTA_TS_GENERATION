#ifndef MULTI_ROBOT_SYSTEM_H
#define MULTI_ROBOT_SYSTEM_H

#include "Robot.h"
#include <vector>
#include <cstdint>
#include <memory>

/**
 * MultiRobotSystem - Manages multiple robots and their interactions
 */
class MultiRobotSystem {
private:
    std::vector<Robot*> robots;
    uint32_t numRobots;
    std::vector<std::vector<bool>> taskAllocationMatrix;  // Task allocation for all robots
    
public:
    // Constructor
    MultiRobotSystem(uint32_t numRobots = 0);
    
    // Destructor
    ~MultiRobotSystem();
    
    // Robot management
    void addRobot(Robot* robot);
    Robot* getRobot(uint32_t robotId) const;
    const std::vector<Robot*>& getRobots() const { return robots; }
    uint32_t getNumRobots() const { return numRobots; }
    
    // Task allocation
    void setTaskAllocation(uint32_t robotId, const std::vector<bool>& allocation);
    const std::vector<bool>& getTaskAllocation(uint32_t robotId) const;
    const std::vector<std::vector<bool>>& getTaskAllocationMatrix() const { return taskAllocationMatrix; }
    
    // State management
    void updateRobotState(uint32_t robotId, uint32_t newState);
    std::vector<uint32_t> getAllRobotStates() const;
    
    // System queries
    bool isTaskAllocated(uint32_t robotId, uint32_t taskId) const;
    std::vector<uint32_t> getRobotsForTask(uint32_t taskId) const;
    
    // Utility
    void clear();
};

#endif // MULTI_ROBOT_SYSTEM_H
