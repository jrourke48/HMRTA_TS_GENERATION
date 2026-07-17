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
                             std::vector<Point> robotPositions,
                             int8_t batch, Tree_Node::TASK_PROGRESS prog);
        
        // Empty constructor
        PlanningDecisionTree();
        
        // Constructor with existing node
        PlanningDecisionTree(Tree_Node* existingNode);
        
        // Destructor
        ~PlanningDecisionTree();
        
        // Tree construction methods
        Tree_Node* insertNode(Tree_Node* parent, Node* automatonState, Node* tsState,
                              std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                              std::vector<Point> positions, int8_t batch, Tree_Node::TASK_PROGRESS prog);
        Tree_Node* insertNode(Tree_Node* newNode);
        Tree_Node* insertSubtree(Tree_Node* parentNode, PlanningDecisionTree* subtree);
        
        // Tree traversal
        std::vector<Tree_Node*> getAllNodes();
        std::vector<Tree_Node*> getLeafNodes();
        
        // Frontier management
        void addFrontierNode(Tree_Node* node);
        void removeFrontierNode(Tree_Node* node);
        const std::vector<Tree_Node*>& getFrontierNodes() const;
        void clearFrontierNodes();
        Tree_Node* getOptimalFrontierNode(bool finite_nba) const; // Returns the frontier node with the lowest max time (heuristic)
        std::vector<Tree_Node*> getPathtoFrontierNode(Tree_Node* frontierNode);

        // Tree manipulation
        void deleteSubtree(Tree_Node* node);
        
        // Getter methods
        Tree_Node* getRoot() const;
        size_t getNodeCount() const;
        
        // Utility methods
        void clearTree();
        bool isEmpty() const;
        void reassignNodeIds();  // Reassign IDs in tree hierarchy order (BFS from root)
};

#endif // PLANNING_DECISION_TREE_H
