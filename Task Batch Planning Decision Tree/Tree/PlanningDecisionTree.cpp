#include "PlanningDecisionTree.h"
#include <algorithm>

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
PlanningDecisionTree::PlanningDecisionTree() : root(nullptr), nodeCount(0), frontierNodes() {};

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
 */
Tree_Node* PlanningDecisionTree::insertSubtree(Tree_Node* parentNode, PlanningDecisionTree* subtree) {
    if (parentNode == nullptr || subtree == nullptr) {
        return nullptr;
    }
    
    Tree_Node* subtreeRoot = subtree->getRoot();
    if (subtreeRoot == nullptr) {
        return nullptr;
    }
    
    // Update the subtree root's parent to point to parentNode
    subtreeRoot->setParent(parentNode);
    
    // Update node count to include all nodes from the subtree
    nodeCount += subtree->getNodeCount();
    
    // Frontier management: parentNode is no longer a leaf
    removeFrontierNode(parentNode);
    
    // Add all frontier nodes from the subtree to our frontier
    for (Tree_Node* frontierNode : subtree->getFrontierNodes()) {
        addFrontierNode(frontierNode);
    }
    
    return subtreeRoot;
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
 */
std::vector<Tree_Node*> PlanningDecisionTree::getAllNodes() {
    std::vector<Tree_Node*> allNodes;
    std::set<Tree_Node*> visited;
    
    // If tree is empty, return empty vector
    if (root == nullptr) {
        return allNodes;
    }
    
    // If frontier is empty, start from root
    std::vector<Tree_Node*> startNodes = frontierNodes.empty() ? std::vector<Tree_Node*>{root} : frontierNodes;
    
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
