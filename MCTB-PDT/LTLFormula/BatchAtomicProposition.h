#ifndef BATCH_ATOMIC_PROPOSITION_H
#define BATCH_ATOMIC_PROPOSITION_H

#include <string>
#include <vector>
#include "MultiRobotSystem/RobotCapabilities.h"

class BatchAtomicProposition {
private:
    uint16_t ap_id; // Unique identifier for the atomic proposition (AP index)
    uint16_t ap;
    std::vector<bool> capabilities; // Indices of bools representing the presence of capabilities necessary
    // to complete this task
    int8_t batch; // Batch number this atomic proposition belongs to 
    std::string name; // name for the LTL formula (e.g., "R0", "R1", etc.)
public:
    BatchAtomicProposition(const uint16_t id, const uint16_t ap, const std::vector<bool>& capabilities, int8_t batch)
        : ap_id(id), ap(ap), capabilities(capabilities), batch(batch), name("R" + std::to_string(ap)) {}
    uint16_t getAP() const { return ap; }
    uint16_t getAPId() const { return ap_id; }
    const std::vector<bool>& getCapabilities() const { return capabilities; }
    int8_t getBatch() const { return batch; }
    bool requiresCapability(RobotCapability cap) const {


        uint8_t index = static_cast<uint8_t>(cap);
        if (index >= capabilities.size()) {
            return false; // Capability index out of bounds
        }
        return capabilities[index];
    }
    std::string toString() const {
        std::string result = "AP: " + std::to_string(ap) + ", Capabilities: [";
        for (size_t i = 0; i < capabilities.size(); ++i) {
            if (capabilities[i]) {
                result += capabilityToString(static_cast<RobotCapability>(i)) + ", ";
            }
        }
        if (!capabilities.empty()) {
            result.pop_back(); // Remove last space
            result.pop_back(); // Remove last comma
        }
        result += "], Batch: " + std::to_string(batch);
        return result;
    }

private:
    std::string capabilityToString(RobotCapability cap) const;
};

#endif // BATCH_ATOMIC_PROPOSITION_H