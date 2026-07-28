#ifndef BATCH_ATOMIC_PROPOSITION_H
#define BATCH_ATOMIC_PROPOSITION_H

#include <cstdint>
#include <string>
#include <vector>

/**
 * BatchAtomicProposition - Represents an atomic proposition in LTL formulas
 * with associated robot capabilities
 */
class BatchAtomicProposition {
private:
    uint16_t apId;                           // Atomic proposition ID
    uint16_t ap;                             // TS state (AP value)
    std::vector<bool> capabilityRequirements; // 13 boolean flags for robot capabilities
    int8_t batch;                            // Batch value: >0 compatible, <0 exclusive, 0 unrelated

public:
    // Constructors
    BatchAtomicProposition();
    BatchAtomicProposition(uint16_t apId, uint16_t ap, const std::vector<bool>& capabilities, int8_t batch);
    
    // Destructor
    ~BatchAtomicProposition();
    
    // Getters
    uint16_t getAPId() const { return apId; }
    uint16_t getAP() const { return ap; }
    int8_t getBatch() const { return batch; }
    const std::vector<bool>& getCapabilities() const { return capabilityRequirements; }
    
    // Setters
    void setAPId(uint16_t id) { apId = id; }
    void setAP(uint16_t apVal) { ap = apVal; }
    void setBatch(int8_t batchVal) { batch = batchVal; }
    void setCapabilities(const std::vector<bool>& caps) { capabilityRequirements = caps; }
    
    // Utility methods
    std::string capabilityToString(int cap) const;
    std::string toString() const;
    bool hasCapability(int cap) const;
};

#endif // BATCH_ATOMIC_PROPOSITION_H
