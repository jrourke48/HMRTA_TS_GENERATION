#include "LTLFormula/BatchAtomicProposition.h"

// Constructors
BatchAtomicProposition::BatchAtomicProposition() 
    : apId(0), ap(0), batch(0) {
}

BatchAtomicProposition::BatchAtomicProposition(uint16_t apId, uint16_t ap, 
                                             const std::vector<bool>& capabilities, int8_t batch)
    : apId(apId), ap(ap), capabilityRequirements(capabilities), batch(batch) {
}

// Destructor
BatchAtomicProposition::~BatchAtomicProposition() {
}

std::string BatchAtomicProposition::capabilityToString(int cap) const {
    switch (static_cast<int>(cap)) {
        case 0:  // MOVEMENT_GROUND
            return "MOVEMENT_GROUND";
        case 1:  // MOVEMENT_AERIAL
            return "MOVEMENT_AERIAL";
        case 2:  // MOVEMENT_AQUATIC
            return "MOVEMENT_AQUATIC";
        case 5:  // SENSOR_GPS
            return "SENSOR_GPS";
        case 4:  // SENSOR_LIDAR
            return "SENSOR_LIDAR";
        case 3:  // SENSOR_CAMERA
            return "SENSOR_CAMERA";
        case 6:  // SENSOR_IMU
            return "SENSOR_IMU";
        case 7:  // SENSOR_PROXIMITY
            return "SENSOR_PROXIMITY";
        case 10:  // COMMUNICATION_WIFI
            return "COMMUNICATION_WIFI";
        case 11:  // COMMUNICATION_4G
            return "COMMUNICATION_4G";
        case 8:  // MANIPULATION_GRIPPER
            return "MANIPULATION_GRIPPER";
        case 9:  // MANIPULATION_TOOL
            return "MANIPULATION_TOOL";
        case 12:  // CAPABILITY_PAYLOAD
            return "CAPABILITY_PAYLOAD";
        default:
            return "UNKNOWN";
    }
}

std::string BatchAtomicProposition::toString() const {
    std::string result = "AP[" + std::to_string(apId) + "] ";
    result += "(TS state " + std::to_string(ap) + ") ";
    result += "Batch: " + std::to_string(batch) + " | ";
    result += "Capabilities: ";
    bool hasCapability = false;
    for (size_t i = 0; i < capabilityRequirements.size(); ++i) {
        if (capabilityRequirements[i]) {
            if (hasCapability) result += ", ";
            result += capabilityToString(i);
            hasCapability = true;
        }
    }
    if (!hasCapability) result += "None";
    return result;
}

