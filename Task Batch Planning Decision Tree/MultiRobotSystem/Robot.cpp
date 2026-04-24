#include "MultiRobotSystem/Robot.h"

/**
 * Robot - Constructor
 */
Robot::Robot(uint32_t id, const std::string& name)
    : robotId(id), robotName(name), currentState(0) {
}

/**
 * Robot - Destructor
 */
Robot::~Robot() {
}

/**
 * isAllocatedTask - Check if a task is allocated to this robot
 */
bool Robot::isAllocatedTask(uint32_t taskId) const {
    if (taskId >= taskAllocation.size()) {
        return false;
    }
    return taskAllocation[taskId];
}

/**
 * getTaskTime - Get the time for a specific task
 */
uint16_t Robot::getTaskTime(uint32_t taskId) const {
    if (taskId >= taskTimes.size()) {
        return 0;
    }
    return taskTimes[taskId];
}

/**
 * allocateTask - Allocate a task to this robot
 */
void Robot::allocateTask(uint32_t taskId) {
    if (taskId >= taskAllocation.size()) {
        taskAllocation.resize(taskId + 1, false);
    }
    taskAllocation[taskId] = true;
}

/**
 * deallocateTask - Deallocate a task from this robot
 */
void Robot::deallocateTask(uint32_t taskId) {
    if (taskId >= taskAllocation.size()) {
        return;
    }
    taskAllocation[taskId] = false;
}
