#include "TaskAllocationAlgorithms.h"
#include <algorithm>
#include <queue>
#include <limits>

/**
 * TaskAllocationAlgorithms - Constructor
 * Initializes the algorithm with references to NBA, Environment, and MultiRobotSystem
 */
TaskAllocationAlgorithms::TaskAllocationAlgorithms(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr)
    : nba(nbaPtr), environment(envPtr), multiRobotSystem(robotSysPtr) {
}

/**
 * TaskAllocationAlgorithms - Destructor
 */
TaskAllocationAlgorithms::~TaskAllocationAlgorithms() {
    // Note: We don't delete the pointers here as they are managed externally
}

/**
 * setNBA - Set the NBA reference
 */
void TaskAllocationAlgorithms::setNBA(BuchiAutomaton* nbaPtr) {
    nba = nbaPtr;
}

/**
 * setEnvironment - Set the Environment reference
 */
void TaskAllocationAlgorithms::setEnvironment(Environment* envPtr) {
    environment = envPtr;
}

/**
 * setMultiRobotSystem - Set the MultiRobotSystem reference
 */
void TaskAllocationAlgorithms::setMultiRobotSystem(MultiRobotSystem* robotSysPtr) {
    multiRobotSystem = robotSysPtr;
}

/**
 * getNBA - Get the NBA reference
 */
BuchiAutomaton* TaskAllocationAlgorithms::getNBA() const {
    return nba;
}

/**
 * getEnvironment - Get the Environment reference
 */
Environment* TaskAllocationAlgorithms::getEnvironment() const {
    return environment;
}

/**
 * getMultiRobotSystem - Get the MultiRobotSystem reference
 */
MultiRobotSystem* TaskAllocationAlgorithms::getMultiRobotSystem() const {
    return multiRobotSystem;
}

/**
 * buildPlanningTree - Build a planning tree starting from the root node
 * This method creates a tree by expanding nodes based on the current state
 * It builds the path as it goes down the tree
 */
PlanningDecisionTree* TaskAllocationAlgorithms::buildPlanningTree(
    uint32_t rootId, Node* automatonState, Node* tsState,
    std::vector<bool> taskAllocation, std::vector<uint16_t> times,
    int8_t batch, Tree_Node::TASK_PROGRESS prog) {
    
    // Create the tree with root node
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        rootId, automatonState, tsState, taskAllocation, times, batch, prog
    );
    
    if (tree->isEmpty()) {
        return tree;
    }
    
    // Use BFS to expand the tree level by level
    std::queue<Tree_Node*> nodeQueue;
    nodeQueue.push(tree->getRoot());
    
    // TODO: Implement tree expansion logic
    // - Pop node from queue
    // - Get successors from NBA and Environment
    // - Create child nodes for valid successors
    // - Push child nodes to queue
    
    return tree;
}

/**
 * determineBestPath - Determine the best path through the tree
 * This method traverses the tree and evaluates different paths to find the optimal one
 */
Tree_Node* TaskAllocationAlgorithms::determineBestPath(PlanningDecisionTree* tree) {
    if (!tree || tree->isEmpty()) {
        return nullptr;
    }
    
    Tree_Node* bestNode = tree->getRoot();
    double bestCost = std::numeric_limits<double>::max();
    
    // TODO: Implement path evaluation logic
    // - Traverse tree (BFS or DFS)
    // - Evaluate cost of each leaf node path
    // - Return the node representing the best path
    
    return bestNode;
}
