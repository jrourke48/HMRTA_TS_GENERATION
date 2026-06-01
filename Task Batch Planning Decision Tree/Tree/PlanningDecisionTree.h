#ifndef PLANNING_DECISION_TREE_H
#define PLANNING_DECISION_TREE_H

#include "Tree_Node.h"
#include <vector>
#include <set>
#include <memory>

class PlanningDecisionTree {
    private:
        Tree_Node* root; // Root node of the tree
        size_t nodeCount; // Total number of nodes in the tree
        std::vector<Tree_Node*> frontierNodes; // Current frontier/leaf nodes for tree traversal
        
    public:
        // Constructor
        PlanningDecisionTree(Node* automatonState, Node* tsState, 
                             std::vector<bool> taskAllocation, std::vector<uint16_t> times, 
                             int8_t batch, Tree_Node::TASK_PROGRESS prog);
        
        // Empty constructor
        PlanningDecisionTree();
        
        // Destructor
        ~PlanningDecisionTree();
        
        // Tree construction methods
        Tree_Node* insertNode(Tree_Node* parent, Node* automatonState, Node* tsState,
                              std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                              int8_t batch, Tree_Node::TASK_PROGRESS prog);
        Tree_Node* insertNode(Tree_Node* newNode);
        
        /**
         * insertSubtree - Attach an entire subtree as a child of an existing node
         * This method attaches subtree's root as a child of parentNode
         * Updates frontier nodes to include all new leaf nodes from the subtree
         * All nodes in the subtree are connected through parent pointers
         */
        std::vector<Tree_Node*> getAllNodes();
        Tree_Node* insertSubtree(Tree_Node* parentNode, PlanningDecisionTree* subtree);
        
        // Frontier nodes management
        void addFrontierNode(Tree_Node* node);
        void removeFrontierNode(Tree_Node* node);
        const std::vector<Tree_Node*>& getFrontierNodes() const;
        void clearFrontierNodes();
        
        void deleteSubtree(Tree_Node* node);
        
        // Getter methods
        Tree_Node* getRoot() const;
        size_t getNodeCount() const;
        
        // Utility methods
        void clearTree();
        bool isEmpty() const;
};

#endif // PLANNING_DECISION_TREE_H
