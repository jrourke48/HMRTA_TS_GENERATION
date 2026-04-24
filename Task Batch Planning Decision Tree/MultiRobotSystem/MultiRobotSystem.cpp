#include "MultiRobotSystem/MultiRobotSystem.h"

/**
 * MultiRobotSystem - Constructor
 */
MultiRobotSystem::MultiRobotSystem(uint32_t numRobots)
    : numRobots(numRobots) {
    taskAllocationMatrix.resize(numRobots);
}

/**
 * MultiRobotSystem - Destructor
 */
MultiRobotSystem::~MultiRobotSystem() {
    for (auto* robot : robots) {
        delete robot;
    }
    robots.clear();
}

/**
 * addRobot - Add a robot to the system
 */
void MultiRobotSystem::addRobot(Robot* robot) {
    if (!robot) return;
    robots.push_back(robot);
}

/**
 * getRobot - Get a robot by ID
 */
Robot* MultiRobotSystem::getRobot(uint32_t robotId) const {
    for (auto* robot : robots) {
        if (robot->getRobotId() == robotId) {
            return robot;
        }
    }
    return nullptr;
}

/**
 * setTaskAllocation - Set task allocation for a robot
 */
void MultiRobotSystem::setTaskAllocation(uint32_t robotId, const std::vector<bool>& allocation) {
    for (size_t i = 0; i < robots.size(); ++i) {
        if (robots[i]->getRobotId() == robotId) {
            robots[i]->setTaskAllocation(allocation);
            if (i < taskAllocationMatrix.size()) {
                taskAllocationMatrix[i] = allocation;
            }
            return;
        }
    }
}

/**
 * getTaskAllocation - Get task allocation for a robot
 */
const std::vector<bool>& MultiRobotSystem::getTaskAllocation(uint32_t robotId) const {
    for (const auto* robot : robots) {
        if (robot->getRobotId() == robotId) {
            return robot->getRoboTaskAllocation();
        }
    }
    static const std::vector<bool> emptyAllocation;
    return emptyAllocation;
}

/**
 * updateRobotState - Update a robot's current state
 */
void MultiRobotSystem::updateRobotState(uint32_t robotId, uint32_t newState) {
    Robot* robot = getRobot(robotId);
    if (robot) {
        robot->setCurrentState(newState);
    }
}

/**
 * getAllRobotStates - Get current states of all robots
 */
std::vector<uint32_t> MultiRobotSystem::getAllRobotStates() const {
    std::vector<uint32_t> states;
    for (const auto* robot : robots) {
        states.push_back(robot->getCurrentState());
    }
    return states;
}

/**
 * isTaskAllocated - Check if a task is allocated to a robot
 */
bool MultiRobotSystem::isTaskAllocated(uint32_t robotId, uint32_t taskId) const {
    Robot* robot = getRobot(robotId);
    if (robot) {
        return robot->isAllocatedTask(taskId);
    }
    return false;
}

/**
 * getRobotsForTask - Get all robots allocated to a specific task
 */
std::vector<uint32_t> MultiRobotSystem::getRobotsForTask(uint32_t taskId) const {
    std::vector<uint32_t> robotIds;
    for (const auto* robot : robots) {
        if (robot->isAllocatedTask(taskId)) {
            robotIds.push_back(robot->getRobotId());
        }
    }
    return robotIds;
}

/**
 * clear - Clear all robots
 */
void MultiRobotSystem::clear() {
    for (auto* robot : robots) {
        delete robot;
    }
    robots.clear();
    taskAllocationMatrix.clear();
}
