#include "Tree/PlanningDecisionTree.h"
#include <algorithm>
#include <iostream>
#include <functional>

/**
 * PlanningDecisionTree - Constructor
 * Creates a new planning tree with the root node
 * Root node is automatically assigned ID 0
 */
PlanningDecisionTree::PlanningDecisionTree(Node* automatonState, Node* tsState,
                                           std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                                           std::vector<Point> robotPositions,
                                           int8_t batch, Tree_Node::TASK_PROGRESS prog)
    : nodeCount(0) {
    // Root node always gets ID 0
    root = new Tree_Node(0, nullptr, automatonState, tsState, taskAllocation, times, robotPositions, batch, prog);
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
                                            std::vector<uint16_t> times, std::vector<Point> positions, int8_t batch,
                                            Tree_Node::TASK_PROGRESS prog) {
    if (parent == nullptr) {
        return nullptr;
    }

    // Auto-assign nodeId based on current node count
    uint32_t nodeId = static_cast<uint32_t>(nodeCount);
    Tree_Node* newNode = new Tree_Node(nodeId, parent, automatonState, tsState, taskAllocation, times, positions, batch, prog);
    nodeCount++;
    
    // Add new node to parent's children
    parent->addChild(newNode);
    
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
    } else if (newNode->getParent() != nullptr) {
        // Add node to parent's children
        newNode->getParent()->addChild(newNode);
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
 * insertNodes - Insert multiple nodes into the tree
 * This method takes a vector of pre-constructed Tree_Node pointers and adds them to the tree
 * Each node is assigned a new ID based on the current node count
 * Manages frontier: removes parent from frontier and adds new nodes to frontier
 * Returns a vector of pointers to the inserted nodes
 */
std::vector<Tree_Node*> PlanningDecisionTree::insertNodes(Tree_Node* parentNode, const std::vector<Tree_Node*>& childrenNodes) {
    std::vector<Tree_Node*> insertedNodes;
    
    if (parentNode == nullptr) {
        std::cerr << "[ERROR] insertNodes called with null parentNode" << std::endl;
        return insertedNodes;  // Can't insert without a valid parent
    }
    
    if (childrenNodes.empty()) {
        return insertedNodes;  // Nothing to insert
    }
    
    // Validate all children before proceeding
    for (const auto* node : childrenNodes) {
        if (node == nullptr) {
            std::cerr << "[ERROR] insertNodes received null child node" << std::endl;
            return insertedNodes;  // Fail if any child is null
        }
    }
    
    // Remove parent from frontier once (optimization - we'll add it back later if needed)
    removeFrontierNode(parentNode);
    
    for (Tree_Node* newNode : childrenNodes) {
        if (newNode != nullptr) {
            // Set parent relationship
            newNode->setParent(parentNode);
            
            // Add to parent's children
            parentNode->addChild(newNode);
            
            // Auto-assign ID
            uint32_t nodeId = static_cast<uint32_t>(nodeCount);
            newNode->setId(nodeId);
            nodeCount++;
            
            // Add to frontier
            addFrontierNode(newNode);
            
            insertedNodes.push_back(newNode);
        }
    }
    
    // Only add parent to frontier if it still has no children (shouldn't happen)
    if (parentNode->getChildren().empty()) {
        addFrontierNode(parentNode);
    }
    
    return insertedNodes;
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
        parentNode->addChild(subtreeRoot);  // Add to parent's children vector for traversal
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
 * Recursively deletes the node and all its descendants using children pointers
 * Removes all affected nodes from frontier
 * Updates nodeCount based on deleted nodes
 */
void PlanningDecisionTree::deleteSubtree(Tree_Node* node) {
    if (node == nullptr) {
        return;
    }
    
    Tree_Node* nodeToDelete = node;  // Mark which node is being deleted
    
    // Helper function to count and delete nodes recursively
    std::function<void(Tree_Node*)> deleteRecursive = [&](Tree_Node* current) {
        if (current == nullptr) return;
        
        // Remove from frontier FIRST
        removeFrontierNode(current);
        
        // Recursively delete all children BEFORE anything else
        // Make a copy of children list since deleteRecursive modifies things
        std::vector<Tree_Node*> childrenToDelete;
        const auto& children = current->getChildren();
        for (Tree_Node* child : children) {
            if (child != nullptr) {
                childrenToDelete.push_back(child);
            }
        }
        
        // Delete all children first
        for (Tree_Node* child : childrenToDelete) {
            deleteRecursive(child);
        }
        
        // AFTER deleting all children, remove this node from its parent's children vector
        Tree_Node* parent = current->getParent();
        if (parent != nullptr) {
            parent->removeChild(current);
            // Only re-add parent to frontier if parent is NOT being deleted
            if (parent != nodeToDelete && parent->getChildren().empty()) {
                // If parent now has no more children, it becomes a frontier node
                addFrontierNode(parent);
            }
        }
        
        // Finally delete this node
        nodeCount--;
        delete current;
    };
    
    // Start deletion from the target node
    deleteRecursive(node);
    
    // If root was deleted, clear the tree
    if (root == node) {
        root = nullptr;
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
    return static_cast<size_t>(nodeCount);
}

/**
 * getNumNodes - Get the total number of nodes in the tree
 */
long long PlanningDecisionTree::getNumNodes() const {
    return static_cast<long long>(nodeCount);
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
 * getAllNodes - Get all nodes in the tree by DFS traversal from root
 * Uses children pointers for proper tree traversal, independent of frontier status
 * Returns all nodes reachable from root following parent-child relationships
 */
std::vector<Tree_Node*> PlanningDecisionTree::getAllNodes() const {
    std::vector<Tree_Node*> allNodes;
    
    if (root == nullptr) {
        return allNodes;  // Empty tree
    }
    
    // DFS from root using children pointers
    std::vector<Tree_Node*> stack;
    stack.push_back(root);
    
    while (!stack.empty()) {
        Tree_Node* current = stack.back();
        stack.pop_back();
        
        allNodes.push_back(current);
        
        // Add all children to stack (in reverse order for consistent traversal)
        const auto& children = current->getChildren();
        for (auto it = children.rbegin(); it != children.rend(); ++it) {
            stack.push_back(*it);
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
Tree_Node* PlanningDecisionTree::getOptimalFrontierNode(bool finite_nba) const {
    if (frontierNodes.empty()) {
        return nullptr;
    }
    //list of nodes that have completed the progress through the nba
    std::vector<Tree_Node*> completeNodes;
    //first check if the nba is finite, if it is not finite.
    if (finite_nba) {
        // Filter frontier nodes to only those at TRANSITION progress
        for (Tree_Node* node : frontierNodes) {
            if (node->getProgress() == Tree_Node::TASK_PROGRESS::TRA) {
                completeNodes.push_back(node);
            }
        }
    }
    else {
        // if the nba is infinte Filter frontier nodes to only those at Other progress
        for (Tree_Node* node : frontierNodes) {
            if (node->getProgress() == Tree_Node::TASK_PROGRESS::OTH) {
                completeNodes.push_back(node);
            }
        }
    }
        
    // If no transition nodes or oth nodes, return null
    if (completeNodes.empty()) {
        return nullptr;
    }
    
    //else return the node with the lowest max time
    Tree_Node* optimal = completeNodes[0];
    uint16_t minMaxTime = optimal->getMaxTime();

    for (Tree_Node* node : completeNodes) {
        uint16_t nodeMaxTime = node->getMaxTime();
        if (nodeMaxTime < minMaxTime) {
            minMaxTime = nodeMaxTime;
            optimal = node;
        }
    }
    
    return optimal;
}
/**
 * getAllocationfromBatchValue - Get task allocation vector from batch value
 * Searches the tree for a node with matching batch value and returns its allocation
 * Returns empty vector if no matching node found
 */
std::vector<bool> PlanningDecisionTree::getAllocationfromBatchValue(uint16_t batchValue) const {
    // Get all nodes in the tree
    std::vector<Tree_Node*> allNodes = const_cast<PlanningDecisionTree*>(this)->getAllNodes();
    
    // Search for a node with matching batch value
    for (Tree_Node* node : allNodes) {
        if (node != nullptr && node->getBatch() == static_cast<int8_t>(batchValue)) {
            // Found matching batch - return its allocation
            return node->getRoboTaskAllocation();
        }
    }
    
    // No matching batch value found, return empty vector
    return std::vector<bool>();
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