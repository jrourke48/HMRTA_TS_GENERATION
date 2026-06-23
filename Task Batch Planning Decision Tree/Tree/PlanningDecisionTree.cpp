#include "PlanningDecisionTree.h"
#include <algorithm>
#include <iostream>

/**
 * PlanningDecisionTree - Constructor
 * Creates a new planning tree with the root node
 * Root node is automatically assigned ID 0
 */
PlanningDecisionTree::PlanningDecisionTree(Node* automatonState, Node* tsState,
                                           std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                                           int8_t batch, Tree_Node::TASK_PROGRESS prog)
    : nodeCount(0) {
    // Root node always gets ID 0
    root = new Tree_Node(0, nullptr, automatonState, tsState, taskAllocation, times, batch, prog);
    nodeCount = 1;
    frontierNodes.push_back(root);  // Root is initially the frontier
}
/**
 * PlanningDecisionTree - Empty Constructor
 */
PlanningDecisionTree::PlanningDecisionTree() : root(nullptr), nodeCount(0), frontierNodes() {}

/**
 * PlanningDecisionTree - Constructor with existing node
 */
PlanningDecisionTree::PlanningDecisionTree(Tree_Node* existingNode) 
    : root(existingNode), nodeCount(1), frontierNodes() {
    if (existingNode != nullptr) {
        frontierNodes.push_back(existingNode);
    }
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
 * Automatically assigns nodeId based on current node count
 * Manages frontier: removes parent from frontier (no longer a leaf) and adds newNode to frontier
 * Returns pointer to the newly created node
 */
Tree_Node* PlanningDecisionTree::insertNode(Tree_Node* parent, Node* automatonState,
                                            Node* tsState, std::vector<bool> taskAllocation,
                                            std::vector<uint16_t> times, int8_t batch,
                                            Tree_Node::TASK_PROGRESS prog) {
    if (parent == nullptr) {
        return nullptr;
    }

    // Auto-assign nodeId based on current node count
    uint32_t nodeId = static_cast<uint32_t>(nodeCount);
    Tree_Node* newNode = new Tree_Node(nodeId, parent, automatonState, tsState, taskAllocation, times, batch, prog);
    nodeCount++;
    
    // Frontier management: parent is no longer a leaf
    removeFrontierNode(parent);
    // New node becomes the new frontier node
    addFrontierNode(newNode);
    
    return newNode;
}

/**
 * insertNode - Insert an existing node into the tree
 * This overload takes a pre-constructed Tree_Node and adds it to the tree
 * Automatically reassigns nodeId based on current node count
 * Manages frontier: removes parent from frontier and adds newNode to frontier
 * Returns pointer to the inserted node
 */
Tree_Node* PlanningDecisionTree::insertNode(Tree_Node* newNode) {
    if (newNode == nullptr) {
        return nullptr;
    }
    
    // Auto-assign nodeId based on current node count
    uint32_t nodeId = static_cast<uint32_t>(nodeCount);
    newNode->setId(nodeId);
    nodeCount++;
    
    // If tree is empty and node has no parent, this node becomes the root
    if (root == nullptr && newNode->getParent() == nullptr) {
        root = newNode;
    }
    
    // Frontier management: if node has a parent, remove parent from frontier
    Tree_Node* parent = newNode->getParent();
    if (parent != nullptr) {
        removeFrontierNode(parent);
    }
    
    // New node becomes a frontier node
    addFrontierNode(newNode);
    
    return newNode;
}

/**
 * insertSubtree - Attach an entire subtree as a child of an existing node
 * This method attaches subtreeRoot as a child of parentNode
 * Manages frontier: removes parentNode from frontier and adds all subtree frontier nodes
 * All nodes in the subtree are incorporated into this tree
 * Special case: if subtreeRoot == parentNode, node is already in tree; only add new descendants
 */
Tree_Node* PlanningDecisionTree::insertSubtree(Tree_Node* parentNode, PlanningDecisionTree* subtree) {
    if (parentNode == nullptr || subtree == nullptr) {
        return nullptr;
    }
    
    Tree_Node* subtreeRoot = subtree->getRoot();
    if (subtreeRoot == nullptr) {
        return nullptr;
    }
    
    // Special case: if subtreeRoot == parentNode, the node is already in our tree
    // Only add the count of new children (don't double-count the node itself)
    if (subtreeRoot != parentNode) {
        // Normal case: attach a separate subtree
        subtreeRoot->setParent(parentNode);
        nodeCount += subtree->getNodeCount();
    } else {
        // Special case: parentNode is the root of the subtree being attached
        // Add only the NEW nodes (subtree children), not the root itself
        size_t newNodeCount = subtree->getNodeCount() - 1;  // Exclude root from count
        nodeCount += newNodeCount;
    }
    
    // Add all frontier nodes from the subtree to our frontier (except parentNode itself)
    std::vector<Tree_Node*> newFrontierNodes;
    for (Tree_Node* frontierNode : subtree->getFrontierNodes()) {
        if (frontierNode != parentNode) {
            newFrontierNodes.push_back(frontierNode);
            addFrontierNode(frontierNode);
        }
    }
    
    // Only remove parentNode from frontier if it has new children
    // If all children were pruned, keep parent in frontier (it's a dead-end leaf)
    if (!newFrontierNodes.empty()) {
        removeFrontierNode(parentNode);
    }
    
    return subtreeRoot;
}

/**
 * deleteSubtree - Delete a subtree rooted at the given node
 * Removes the node from frontier and deletes it and all descendants from memory
 */
void PlanningDecisionTree::deleteSubtree(Tree_Node* node) {
    if (node == nullptr) {
        return;
    }
    
    // Remove from frontier if present
    removeFrontierNode(node);
    
    // Collect all descendants by backward traversal from frontier
    std::vector<Tree_Node*> toDelete;
    std::set<Tree_Node*> visited;
    
    // Start from frontier nodes that are descendants of 'node'
    for (Tree_Node* frontierNode : frontierNodes) {
        Tree_Node* current = frontierNode;
        while (current != nullptr) {
            if (current == node) {
                // This frontier node is a descendant of 'node', collect path
                Tree_Node* temp = frontierNode;
                while (temp != node && temp != nullptr) {
                    if (visited.find(temp) == visited.end()) {
                        toDelete.push_back(temp);
                        visited.insert(temp);
                    }
                    temp = temp->getParent();
                }
                break;
            }
            current = current->getParent();
        }
    }
    
    // Add the node itself
    if (visited.find(node) == visited.end()) {
        toDelete.push_back(node);
        visited.insert(node);
    }
    
    // Delete all collected nodes
    for (Tree_Node* deleteNode : toDelete) {
        removeFrontierNode(deleteNode);
        delete deleteNode;
        nodeCount--;
    }
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
        clearFrontierNodes();
    }
}

/**
 * isEmpty - Check if the tree is empty
 */
bool PlanningDecisionTree::isEmpty() const {
    return root == nullptr || nodeCount == 0;
}

/**
 * getAllNodes - Get all nodes in the tree by traversing backward from frontier nodes
 * Traverses from each frontier node back to root, collecting all unique nodes
 * Defensive: also uses frontier nodes if root is somehow still null
 */
std::vector<Tree_Node*> PlanningDecisionTree::getAllNodes() {
    std::vector<Tree_Node*> allNodes;
    std::set<Tree_Node*> visited;
    
    // If tree is empty (no root and no frontier), return empty vector
    if (root == nullptr && frontierNodes.empty()) {
        return allNodes;
    }
    
    // Determine starting nodes: use frontier if available, otherwise use root
    // This handles edge cases and ensures we get all nodes even if root somehow isn't set
    std::vector<Tree_Node*> startNodes;
    if (!frontierNodes.empty()) {
        startNodes = frontierNodes;
    } else if (root != nullptr) {
        startNodes = {root};
    } else {
        return allNodes;  // Truly empty
    }
    
    // Traverse backward from each frontier node to root
    for (Tree_Node* leaf : startNodes) {
        Tree_Node* current = leaf;
        while (current != nullptr && visited.find(current) == visited.end()) {
            allNodes.push_back(current);
            visited.insert(current);
            current = current->getParent();
        }
    }
    
    return allNodes;
}

/**
 * addFrontierNode - Add a node to the frontier
 */
void PlanningDecisionTree::addFrontierNode(Tree_Node* node) {
    if (node != nullptr) {
        frontierNodes.push_back(node);
    }
}

/**
 * removeFrontierNode - Remove a node from the frontier
 */
void PlanningDecisionTree::removeFrontierNode(Tree_Node* node) {
    if (node == nullptr) {
        return;
    }
    
    auto it = std::find(frontierNodes.begin(), frontierNodes.end(), node);
    if (it != frontierNodes.end()) {
        frontierNodes.erase(it);
    }
}

/**
 * getFrontierNodes - Get the current frontier nodes
 */
const std::vector<Tree_Node*>& PlanningDecisionTree::getFrontierNodes() const {
    return frontierNodes;
}

/**
 * clearFrontierNodes - Clear all frontier nodes
 */
void PlanningDecisionTree::clearFrontierNodes() {
    frontierNodes.clear();
}

/**
 * getLeafNodes - Get all leaf nodes (frontier nodes) in the tree
 * In frontier-based representation, frontier nodes ARE the leaf nodes
 * Returns vector of nodes that are currently frontier nodes
 */
std::vector<Tree_Node*> PlanningDecisionTree::getLeafNodes() {
    return frontierNodes;
}

/**
 * getOptimalFrontierNode - Get the frontier node with the lowest max time (heuristic for optimality)
 * Only considers frontier nodes at TRANSITION progress (TRA)
 * Returns the frontier node with the best cost for expansion
 */
Tree_Node* PlanningDecisionTree::getOptimalFrontierNode() const {
    if (frontierNodes.empty()) {
        return nullptr;
    }
    
    // Filter frontier nodes to only those at TRANSITION progress
    std::vector<Tree_Node*> transitionNodes;
    for (Tree_Node* node : frontierNodes) {
        if (node->getProgress() == Tree_Node::TASK_PROGRESS::TRA) {
            transitionNodes.push_back(node);
        }
    }
    
    // If no transition nodes, return null
    if (transitionNodes.empty()) {
        return nullptr;
    }
    
    Tree_Node* optimal = transitionNodes[0];
    uint16_t minMaxTime = optimal->getMaxTime();

    for (Tree_Node* node : transitionNodes) {
        uint16_t nodeMaxTime = node->getMaxTime();
        std::cout << "Max Time for Node NBA: node " << node->getId() << " = " << nodeMaxTime << std::endl;
        if (nodeMaxTime < minMaxTime) {
            minMaxTime = nodeMaxTime;
            optimal = node;
        }
    }
    
    return optimal;
}

/**
 * reassignNodeIds - Reassign all node IDs in tree hierarchy order
 * This ensures root has ID 0, and IDs increment in order
 * Useful after tree modifications (insertSubtree, etc.)
 */
void PlanningDecisionTree::reassignNodeIds() {
    if (root == nullptr) {
        return;
    }
    
    // Get all nodes via backward traversal
    std::vector<Tree_Node*> allNodes = getAllNodes();
    
    // Assign IDs in order
    uint32_t nextId = 0;
    for (Tree_Node* node : allNodes) {
        node->setId(nextId++);
    }
}



std::vector<Tree_Node*> PlanningDecisionTree::getPathtoFrontierNode(Tree_Node* frontierNode) {
    std::vector<Tree_Node*> path;
    if (frontierNode == nullptr) {
        return path;
    }
    
    Tree_Node* current = frontierNode;
    while (current != nullptr) {
        path.push_back(current);
        current = current->getParent();
    }
    
    // Reverse to get path from root to frontier node
    std::reverse(path.begin(), path.end());
    return path;
}