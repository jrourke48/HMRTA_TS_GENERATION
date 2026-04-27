#ifndef ROBOT_CAPABILITIES_H
#define ROBOT_CAPABILITIES_H

#include <cstdint>
#include <string>

/**
 * RobotCapabilities - Enum defining possible robot capabilities
 */
enum class RobotCapability : uint8_t {
    // Movement Capabilities
    MOVEMENT_GROUND      = 0,   // Can move on ground/wheels
    MOVEMENT_AERIAL      = 1,   // Can fly (UAV/drone)
    MOVEMENT_AQUATIC     = 2,   // Can move in water
    
    // Sensing Capabilities
    SENSOR_CAMERA        = 3,   // Has camera/vision
    SENSOR_LIDAR         = 4,   // Has LIDAR
    SENSOR_GPS           = 5,   // Has GPS
    SENSOR_IMU           = 6,   // Has IMU/accelerometer
    SENSOR_PROXIMITY     = 7,   // Has proximity/distance sensors
    
    // Manipulation Capabilities
    MANIPULATION_ARM     = 8,   // Has robotic arm
    MANIPULATION_GRIPPER = 9,   // Has gripper/manipulator
    MANIPULATION_TOOL    = 10,  // Can use tools
    
    // Communication Capabilities
    COMMUNICATION_WIFI   = 11,  // Can communicate via WiFi
    COMMUNICATION_4G     = 12,  // Can communicate via 4G/LTE
    COMMUNICATION_RADIO  = 13,  // Can communicate via radio
    
    // Special Capabilities
    CAPABILITY_CHARGING  = 14,  // Can charge other robots
    CAPABILITY_MAPPING   = 15,  // Can create maps
    CAPABILITY_PAYLOAD   = 16,  // Can carry payload
};

/**
 * Helper function to convert RobotCapability to string
 */
inline std::string capabilityToString(RobotCapability cap) {
    switch (cap) {
        case RobotCapability::MOVEMENT_GROUND:      return "Movement_Ground";
        case RobotCapability::MOVEMENT_AERIAL:      return "Movement_Aerial";
        case RobotCapability::MOVEMENT_AQUATIC:     return "Movement_Aquatic";
        case RobotCapability::SENSOR_CAMERA:        return "Sensor_Camera";
        case RobotCapability::SENSOR_LIDAR:         return "Sensor_LIDAR";
        case RobotCapability::SENSOR_GPS:           return "Sensor_GPS";
        case RobotCapability::SENSOR_IMU:           return "Sensor_IMU";
        case RobotCapability::SENSOR_PROXIMITY:     return "Sensor_Proximity";
        case RobotCapability::MANIPULATION_ARM:     return "Manipulation_Arm";
        case RobotCapability::MANIPULATION_GRIPPER: return "Manipulation_Gripper";
        case RobotCapability::MANIPULATION_TOOL:    return "Manipulation_Tool";
        case RobotCapability::COMMUNICATION_WIFI:   return "Communication_WiFi";
        case RobotCapability::COMMUNICATION_4G:     return "Communication_4G";
        case RobotCapability::COMMUNICATION_RADIO:  return "Communication_Radio";
        case RobotCapability::CAPABILITY_CHARGING:  return "Capability_Charging";
        case RobotCapability::CAPABILITY_MAPPING:   return "Capability_Mapping";
        case RobotCapability::CAPABILITY_PAYLOAD:   return "Capability_Payload";
        default:                                     return "Unknown";
    }
}

#endif // ROBOT_CAPABILITIES_H
