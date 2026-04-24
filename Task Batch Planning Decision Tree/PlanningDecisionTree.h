#ifndef PLANNING_DECISION_TREE_H
#define PLANNING_DECISION_TREE_H

#include "Tree_Node.h"
#include <vector>
#include <memory>

class PlanningDecisionTree {
    private:
        Tree_Node* root; // Root node of the tree
        size_t nodeCount; // Total number of nodes in the tree
        
    public:
        // Constructor
        PlanningDecisionTree(uint32_t rootId, Node* automatonState, Node* tsState, 
                             std::vector<bool> taskAllocation, std::vector<uint16_t> times, 
                             int8_t batch, Tree_Node::TASK_PROGRESS prog);
        
        // Destructor
        ~PlanningDecisionTree();
        
        // Tree construction methods
        Tree_Node* insertNode(Tree_Node* parent, uint32_t nodeId, Node* automatonState, Node* tsState,
                              std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                              int8_t batch, Tree_Node::TASK_PROGRESS prog);
        void deleteSubtree(Tree_Node* node);
        
        // Getter methods
        Tree_Node* getRoot() const;
        size_t getNodeCount() const;
        
        // Utility methods
        void clearTree();
        bool isEmpty() const;
};

#endif // PLANNING_DECISION_TREE_H
