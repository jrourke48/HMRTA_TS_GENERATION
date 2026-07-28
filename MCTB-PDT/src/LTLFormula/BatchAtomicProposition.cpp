#include "LTLFormula/BatchAtomicProposition.h"

std::string BatchAtomicProposition::capabilityToString(RobotCapability cap) const {
    switch (cap) {
        case RobotCapability::MOVEMENT_GROUND:
            return "MOVEMENT_GROUND";
        case RobotCapability::MOVEMENT_AERIAL:
            return "MOVEMENT_AERIAL";
        case RobotCapability::MOVEMENT_AQUATIC:
            return "MOVEMENT_AQUATIC";
        case RobotCapability::SENSOR_GPS:
            return "SENSOR_GPS";
        case RobotCapability::SENSOR_LIDAR:
            return "SENSOR_LIDAR";
        case RobotCapability::SENSOR_CAMERA:
            return "SENSOR_CAMERA";
        case RobotCapability::SENSOR_IMU:
            return "SENSOR_IMU";
        case RobotCapability::SENSOR_PROXIMITY:
            return "SENSOR_PROXIMITY";
        case RobotCapability::COMMUNICATION_WIFI:
            return "COMMUNICATION_WIFI";
        case RobotCapability::COMMUNICATION_4G:
            return "COMMUNICATION_4G";
        case RobotCapability::MANIPULATION_GRIPPER:
            return "MANIPULATION_GRIPPER";
        case RobotCapability::MANIPULATION_TOOL:
            return "MANIPULATION_TOOL";
        case RobotCapability::CAPABILITY_PAYLOAD:
            return "CAPABILITY_PAYLOAD";
        default:
            return "UNKNOWN";
    }
}

