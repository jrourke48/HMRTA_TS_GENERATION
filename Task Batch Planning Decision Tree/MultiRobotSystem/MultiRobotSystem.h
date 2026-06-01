#ifndef MULTI_ROBOT_SYSTEM_H
#define MULTI_ROBOT_SYSTEM_H

#include "Robot.h"
#include <vector>
#include <cstdint>
#include <memory>

/**
 * MultiRobotSystem - Manages multiple robots
 */
class MultiRobotSystem {
private:
    std::vector<Robot*> robots;
    uint32_t numRobots;
    
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
    
    // Capability queries
    std::vector<Robot*> getRobotsWithCapability(RobotCapability cap) const;
    std::vector<uint32_t> getRobotIdsWithCapability(RobotCapability cap) const;
    bool hasRobotWithCapability(RobotCapability cap) const;
    uint32_t countRobotsWithCapability(RobotCapability cap) const;
    std::vector<Robot*> getRobotsWithAllCapabilities(const std::vector<RobotCapability>& caps) const;
    std::vector<Robot*> getRobotsWithAnyCapability(const std::vector<RobotCapability>& caps) const;
    std::string to_string() const;
    static std::vector<uint16_t> updateAllRobotTimes(const std::vector<Robot*>& robots, const std::vector<uint16_t>& currentTimes, const Point& targetPosition);
    //getemptyV vector
    std::vector<bool> getEmptyV() const;
    
    // Utility
    void clear();
};

#endif // MULTI_ROBOT_SYSTEM_H
