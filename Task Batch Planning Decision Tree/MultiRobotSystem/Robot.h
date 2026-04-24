#ifndef ROBOT_H
#define ROBOT_H

#include <cstdint>
#include <vector>
#include <string>

/**
 * Robot - Represents a single robot in the multi-robot system
 */
class Robot {
private:
    uint32_t robotId;
    std::string robotName;
    uint32_t currentState;      // Current state in the transition system
    std::vector<bool> taskAllocation;  // Which tasks this robot is allocated to
    std::vector<uint16_t> taskTimes;   // Estimated time to complete each task
    
public:
    // Constructor
    Robot(uint32_t id, const std::string& name = "");
    
    // Destructor
    ~Robot();
    
    // Getters
    uint32_t getRobotId() const { return robotId; }
    std::string getRobotName() const { return robotName; }
    uint32_t getCurrentState() const { return currentState; }
    const std::vector<bool>& getTaskAllocation() const { return taskAllocation; }
    const std::vector<uint16_t>& getTaskTimes() const { return taskTimes; }
    
    // Setters
    void setCurrentState(uint32_t stateId) { currentState = stateId; }
    void setTaskAllocation(const std::vector<bool>& allocation) { taskAllocation = allocation; }
    void setTaskTimes(const std::vector<uint16_t>& times) { taskTimes = times; }
    
    // Task management
    bool isAllocatedTask(uint32_t taskId) const;
    uint16_t getTaskTime(uint32_t taskId) const;
    void allocateTask(uint32_t taskId);
    void deallocateTask(uint32_t taskId);
};

#endif // ROBOT_H
