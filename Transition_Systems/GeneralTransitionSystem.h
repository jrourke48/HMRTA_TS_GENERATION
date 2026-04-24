#ifndef GENERAL_TRANSITION_SYSTEM_H
#define GENERAL_TRANSITION_SYSTEM_H

#include "../Automatons/Edge_Node.h"
#include <cstdint>
#include <vector>
#include <unordered_map>
#include <string>

/**
 * GeneralTransitionSystem - Base class for transition systems
 * Represents a state space with transitions between states
 */
class GeneralTransitionSystem {
protected:
    std::unordered_map<uint32_t, Node*> nodeMap;  // Map of state ID to Node
    uint32_t numNodes;
    uint32_t numEdges;
    uint32_t initialState;
    
public:
    // Constructor and Destructor
    GeneralTransitionSystem();
    virtual ~GeneralTransitionSystem();
    
    // Virtual methods for subclasses
    virtual void addNode(Node* node);
    virtual void addEdge(uint32_t srcId, uint32_t dstId, const std::string& label = "");
    virtual bool isAdjacent(uint32_t srcId, uint32_t dstId) const;
    
    // Getters
    Node* getNode(uint32_t nodeId) const;
    const std::unordered_map<uint32_t, Node*>& getNodeMap() const { return nodeMap; }
    uint32_t getNumNodes() const { return numNodes; }
    uint32_t getNumEdges() const { return numEdges; }
    uint32_t getInitialState() const { return initialState; }
    
    // Setters
    void setInitialState(uint32_t stateId) { initialState = stateId; }
    
    // Utility methods
    virtual void clear();
    virtual std::vector<Node*> getSuccessors(uint32_t nodeId) const;
    virtual std::vector<uint32_t> getSuccessorIds(uint32_t nodeId) const;
};

#endif // GENERAL_TRANSITION_SYSTEM_H
