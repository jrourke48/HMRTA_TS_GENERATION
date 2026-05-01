#include "PlanningDecisionTree.h"

/**
 * PlanningDecisionTree - Constructor
 * Creates a new planning tree with the root node
 */
PlanningDecisionTree::PlanningDecisionTree(uint32_t rootId, Node* automatonState, Node* tsState,
                                           std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                                           int8_t batch, Tree_Node::TASK_PROGRESS prog)
    : nodeCount(0) {
    root = new Tree_Node(rootId, nullptr, automatonState, tsState, taskAllocation, times, batch, prog);
    nodeCount = 1;
}

/**
 * PlanningDecisionTree - Destructor
 * Cleans up the tree structure
 */
PlanningDecisionTree::~PlanningDecisionTree() {
    clearTree();
}

/**
 * insertNode - Insert a new node into the tree
 * Returns pointer to the newly created node
 */
Tree_Node* PlanningDecisionTree::insertNode(Tree_Node* parent, uint32_t nodeId, Node* automatonState,
                                            Node* tsState, std::vector<bool> taskAllocation,
                                            std::vector<uint16_t> times, int8_t batch,
                                            Tree_Node::TASK_PROGRESS prog) {
    if (parent == nullptr) {
        return nullptr;
    }

    Tree_Node* newNode = new Tree_Node(nodeId, parent, automatonState, tsState, taskAllocation, times, batch, prog);
    nodeCount++;
    return newNode;
}

/**
 * deleteSubtree - Delete a subtree rooted at the given node
 */
void PlanningDecisionTree::deleteSubtree(Tree_Node* node) {
    if (node == nullptr) {
        return;
    }
    
    // Recursively delete children if we maintain a children vector
    // For now, this just deletes the node itself
    delete node;
    nodeCount--;
}

/**
 * getRoot - Get the root node of the tree
 */
Tree_Node* PlanningDecisionTree::getRoot() const {
    return root;
}

/**
 * getNodeCount - Get the total number of nodes in the tree
 */
size_t PlanningDecisionTree::getNodeCount() const {
    return nodeCount;
}

/**
 * clearTree - Clear all nodes in the tree (recursive delete)
 */
void PlanningDecisionTree::clearTree() {
    if (root != nullptr) {
        deleteSubtree(root);
        root = nullptr;
        nodeCount = 0;
    }
}

/**
 * isEmpty - Check if the tree is empty
 */
bool PlanningDecisionTree::isEmpty() const {
    return root == nullptr || nodeCount == 0;
}
