#ifndef ROBOT_H
#define ROBOT_H

#include "RobotCapabilities.h"
#include "../Point.h"
#include <cstdint>
#include <vector>
#include <string>
#include <algorithm>

/**
 * Robot - Represents a single robot in the multi-robot system
 */
class Robot {
private:
    uint32_t robotId;
    Point startPosition;
    Point currentPosition;
    std::vector<bool> capabilities;  // Index corresponds to RobotCapability enum value
    
public:
    // Constructor
    Robot(uint32_t id, const Point& startPos = Point());
    
    // Destructor
    ~Robot();
    
    // Getters
    uint32_t getRobotId() const { return robotId; }
    const Point& getStartPosition() const { return startPosition; }
    const Point& getCurrentPosition() const { return currentPosition; }
    const std::vector<bool>& getCapabilities() const { return capabilities; }
    
    // Setters
    void setCurrentPosition(const Point& position) { currentPosition = position; }
    void initializeCapabilities(size_t numCapabilities);
    
    // Capability management
    bool hasCapability(RobotCapability cap) const;
    void setCapability(RobotCapability cap, bool enabled);
    void enableCapability(RobotCapability cap);
    void disableCapability(RobotCapability cap);
    void clearCapabilities();
};

#endif // ROBOT_H
