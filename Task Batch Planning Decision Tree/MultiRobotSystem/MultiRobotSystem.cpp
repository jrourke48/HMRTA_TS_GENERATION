#include "MultiRobotSystem.h"
#include "RobotCapabilities.h"
#include <sstream>

/**
 * MultiRobotSystem - Constructor
 */
MultiRobotSystem::MultiRobotSystem(uint32_t numRobots)
    : numRobots(numRobots) {
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
 * clear - Clear all robots
 */
void MultiRobotSystem::clear() {
    for (auto* robot : robots) {
        delete robot;
    }
    robots.clear();
}

/**
 * getRobotsWithCapability - Get all robots that have a specific capability
 */
std::vector<Robot*> MultiRobotSystem::getRobotsWithCapability(RobotCapability cap) const {
    std::vector<Robot*> result;
    for (auto* robot : robots) {
        if (robot->hasCapability(cap)) {
            result.push_back(robot);
        }
    }
    return result;
}

/**
 * getRobotIdsWithCapability - Get IDs of all robots with a specific capability
 */
std::vector<uint32_t> MultiRobotSystem::getRobotIdsWithCapability(RobotCapability cap) const {
    std::vector<uint32_t> result;
    for (auto* robot : robots) {
        if (robot->hasCapability(cap)) {
            result.push_back(robot->getRobotId());
        }
    }
    return result;
}

/**
 * hasRobotWithCapability - Check if any robot has a specific capability
 */
bool MultiRobotSystem::hasRobotWithCapability(RobotCapability cap) const {
    for (auto* robot : robots) {
        if (robot->hasCapability(cap)) {
            return true;
        }
    }
    return false;
}

/**
 * countRobotsWithCapability - Count how many robots have a specific capability
 */
uint32_t MultiRobotSystem::countRobotsWithCapability(RobotCapability cap) const {
    uint32_t count = 0;
    for (auto* robot : robots) {
        if (robot->hasCapability(cap)) {
            count++;
        }
    }
    return count;
}

/**
 * getRobotsWithAllCapabilities - Get robots that have ALL specified capabilities
 */
std::vector<Robot*> MultiRobotSystem::getRobotsWithAllCapabilities(const std::vector<RobotCapability>& caps) const {
    std::vector<Robot*> result;
    for (auto* robot : robots) {
        bool hasAll = true;
        for (RobotCapability cap : caps) {
            if (!robot->hasCapability(cap)) {
                hasAll = false;
                break;
            }
        }
        if (hasAll) {
            result.push_back(robot);
        }
    }
    return result;
}

/**
 * getRobotsWithAnyCapability - Get robots that have ANY of the specified capabilities
 */
std::vector<Robot*> MultiRobotSystem::getRobotsWithAnyCapability(const std::vector<RobotCapability>& caps) const {
    std::vector<Robot*> result;
    for (auto* robot : robots) {
        for (RobotCapability cap : caps) {
            if (robot->hasCapability(cap)) {
                result.push_back(robot);
                break;
            }
        }
    }
    return result;
}

/**
 * updateAllRobotTimes - Update the times for all robots to reach a target position
 */
std::vector<uint16_t> MultiRobotSystem::updateAllRobotTimes(const std::vector<uint16_t>& currentTimes, const Point& targetPosition) {
    std::vector<uint16_t> updatedTimes;
    for (size_t i = 0; i < robots.size(); ++i) {
        if (i < currentTimes.size()) {
            updatedTimes.push_back(robots[i]->getUpdatedTime(currentTimes[i], targetPosition));
        } else {
            updatedTimes.push_back(0); // Default time if not enough currentTimes provided
        }
    }
    return updatedTimes;
}

/**
 * getEmptyV - Get a vector of false values corresponding to the number of robots
 */
std::vector<bool> MultiRobotSystem::getEmptyV() const {
    std::vector<bool> emptyV(robots.size(), false);
    return emptyV;
}

/**
 * to_string - Return a formatted table of robots and their capabilities
 */
std::string MultiRobotSystem::to_string() const {
    std::stringstream ss;
    
    // Header
    ss << "MultiRobotSystem contains " << robots.size() << " robots\n";
    ss << "================================================================================\n";
    ss << "| Robot ID | Robot Name        | Capabilities\n";
    ss << "================================================================================\n";
    
    // Robot entries
    for (const auto* robot : robots) {
        ss << "| " << robot->getRobotId() << "        | " << robot->getName();
        
        // Pad robot name to fixed width
        int nameLen = robot->getName().length();
        for (int i = 0; i < 17 - nameLen; i++) {
            ss << " ";
        }
        ss << "| ";
        
        // List capabilities
        bool firstCap = true;
        for (size_t i = 0; i < robot->getCapabilities().size(); i++) {
            if (robot->getCapabilities()[i]) {
                if (!firstCap) {
                    ss << ", ";
                }
                ss << capabilityToString(static_cast<RobotCapability>(i));
                firstCap = false;
            }
        }
        ss << "\n";
    }
    
    ss << "================================================================================\n";
    
    return ss.str();
}

