#include "TaskAllocationAlgorithms.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>

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
 * addVisitedNode - Add a node to the visited nodes collection
 */
void TaskAllocationAlgorithms::addVisitedNode(Tree_Node* node) {
    if (node) {
        visitedNodes.push_back(node);
    }
}

/**
 * isNodeVisited - Check if a node has been visited
 */
bool TaskAllocationAlgorithms::isNodeVisited(Tree_Node* node) const {
    if (!node) return false;
    return std::find(visitedNodes.begin(), visitedNodes.end(), node) != visitedNodes.end();
}

/**
 * getVisitedNodes - Get the collection of visited nodes
 */
std::vector<Tree_Node*>& TaskAllocationAlgorithms::getVisitedNodes() {
    return visitedNodes;
}

/**
 * clearVisitedNodes - Clear all visited nodes
 */
void TaskAllocationAlgorithms::clearVisitedNodes() {
    visitedNodes.clear();
}

/**
 * addVisitedAutomatonState - Add an automaton state to the visited states collection
 */
void TaskAllocationAlgorithms::addVisitedAutomatonState(uint32_t state) {
    visitedAutomatonStates.push_back(state);
}

/**
 * isAutomatonStateVisited - Check if an automaton state has been visited
 */
bool TaskAllocationAlgorithms::isAutomatonStateVisited(uint32_t state) const {
    return std::find(visitedAutomatonStates.begin(), visitedAutomatonStates.end(), state) != visitedAutomatonStates.end();
}

/**
 * getVisitedAutomatonStates - Get the collection of visited automaton states
 */
std::vector<uint32_t>& TaskAllocationAlgorithms::getVisitedAutomatonStates() {
    return visitedAutomatonStates;
}

/**
 * clearVisitedAutomatonStates - Clear all visited automaton states
 */
void TaskAllocationAlgorithms::clearVisitedAutomatonStates() {
    visitedAutomatonStates.clear();
}

/**
 * Algorithm 1: Intensive Inter-Task Relationship Tree Search
 * Converts LTL formula to Büchi automaton and initializes tree planning
 */
Tree_Node* TaskAllocationAlgorithms::intensiveInterTaskRelationshipTreeSearch(
    BuchiAutomaton* nbaPtr,
    Environment* envPtr,
    MultiRobotSystem* multiRobotSystemPtr) {
    
    std::cout << "[DEBUG] Starting intensiveInterTaskRelationshipTreeSearch" << std::endl;
    
    // Get initial automaton state (typically state 0)
    Node* initialAutomatonState = nbaPtr->getNode(0);
    // Get environment initial state ID
    uint16_t initialEnvStateId = envPtr->getInitialState();
    
    // Create a placeholder node for environment (use stack allocation instead of heap)
    std::cout << "[DEBUG] Creating initial environment node with ID: " << initialEnvStateId << std::endl;
    Node initialEnvNode(initialEnvStateId);
    Node* initialEnvNodePtr = &initialEnvNode;
    
    // Create initial task allocation and times vectors
    std::vector<bool> initialAllocation;
    std::vector<uint16_t> initialTimes;
    if (multiRobotSystemPtr) {
        for (size_t i = 0; i < multiRobotSystemPtr->getRobots().size(); ++i) {
            initialAllocation.push_back(false);
            initialTimes.push_back(0);
        }
    }
    
    // Initialize planning tree with initial state
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        0,                              // rootId
        initialAutomatonState,          // automaton state
        initialEnvNodePtr,              // ts state node (using pointer to stack-allocated node)
        initialAllocation,              // task allocation
        initialTimes,                   // times
        0,                              // batch
        Tree_Node::TASK_PROGRESS::PRE   // progress
    );
    // Add initial node and automaton state to visited collections
    addVisitedNode(tree->getRoot());
    addVisitedAutomatonState(initialAutomatonState->getId());
    
    // BFS expansion - limit depth to prevent infinite loops
    std::queue<Tree_Node*> nodeQueue;
    nodeQueue.push(tree->getRoot());
    // Initialize planning tree with initial state
    uint32_t nodeIdCounter = 1;
    int maxDepth = 3;  // Limited depth for testing
    int currentDepth = 0;
    
    while (!nodeQueue.empty() && currentDepth < maxDepth) {
        PlanningDecisionTree* tree = new PlanningDecisionTree();
        uint32_t buchisize = nbaPtr->getNumStates();

        for (uint32_t i = 0; i < buchisize; ++i) {
            Node* nbaState = nbaPtr->getNode(i);
            if (!isAutomatonStateVisited(nbaState->getId())) {
                // Only process if this state has NOT been visited in this task stage
                addVisitedAutomatonState(nbaState->getId());
                addVisitedNode(tree->getRoot());
                nodeQueue.pop();
            // Get current automaton state
            Node* automatonState = currentNode->getAutomatonState();
            std::cout << "[DEBUG] Got automaton state pointer: " << (void*)automatonState << std::endl;
            
            if (!automatonState) {
                std::cout << "[DEBUG] Automaton state is null, skipping" << std::endl;
                continue;
            }
            
            std::cout << "[DEBUG] About to call getEdges()" << std::endl;
            std::cout.flush();
            
            std::vector<Edge> edges = automatonState->getEdges();
            
            std::cout << "[DEBUG] Successfully called getEdges()" << std::endl;
            std::cout << "[DEBUG] Automaton state has " << edges.size() << " edges" << std::endl;
            
            for (size_t edgeIdx = 0; edgeIdx < edges.size(); ++edgeIdx) {
                const auto& edge = edges[edgeIdx];
                uint32_t nextAutomatonStateId = edge.getDstId();
                
                Node* nextAutomatonState = nbaPtr->getNode(nextAutomatonStateId);
                if (!nextAutomatonState) continue;
                
                // Reuse the same environment node
                Node* tsNode = currentNode->getTSState();
                if (!tsNode) continue;
                
                std::cout << "[DEBUG] About to insert child node " << nodeIdCounter << std::endl;
                
                Tree_Node* childNode = tree->insertNode(
                    currentNode,
                    nodeIdCounter++,
                    nextAutomatonState,
                    tsNode,
                    currentNode->getRoboTaskAllocation(),
                    currentNode->getTimes(),
                    currentNode->getBatch(),
                    Tree_Node::TASK_PROGRESS::TRA
                );
                
                std::cout << "[DEBUG] Child node inserted: " << (childNode ? "success" : "failed") << std::endl;
                
                if (childNode) {
                    nodeQueue.push(childNode);
                }
            }
        }
        currentDepth++;
    }
    
    std::cout << "[DEBUG] BFS expansion complete, returning tree" << std::endl;
    return tree;
}

/**
 * Algorithm 2: Unrelated-Task Search (US)
 * Searches for unrelated tasks that can be executed independently
 */
Tree_Node* TaskAllocationAlgorithms::unrelatedTaskSearch(
    uint32_t /*q*/,
    uint32_t /*ds*/,
    spot::formula /*ltlFormula*/) {
    
    // Initialize cost calculation
    // ρ_pos(ds) = cost to reach ds
    double rho_pos = 1.0; // Placeholder for actual cost calculation
    
    // ρ_prox(ds) = proximity cost from current robot position
    double rho_prox = 0.5; // Placeholder for actual proximity calculation
    
    // Calculate base cost: u_ds = ρ_pos + (1/V_m) * |ρ_pos^-1 - ρ_prox|
    // where V_m is the maximum velocity (set to 1 for normalization)
    double V_m = 1.0;
    // double u_ds = rho_pos + (1.0 / V_m) * std::abs((1.0 / rho_pos) - rho_prox);
    
    // Sort by cost T according to sorted order based on (d_T, V, u_ds)
    // Return best unrelated task tree node
    // TODO: Create and return appropriate tree node
    return nullptr;
}

/**
 * Algorithm 3: Compatible-Task Search (CS)
 * Searches for compatible tasks considering both sub-tasks and main tasks
 */
Tree_Node* TaskAllocationAlgorithms::compatibleTaskSearch(
    uint32_t q,
    uint32_t /*ds*/,
    const std::vector<uint16_t>& subTasks,
    const std::vector<uint16_t>& /*tasks*/) {
    
    // Initialize cost: t_cs = t_us^-1 + (1/V_m) * |ρ_ds^-1 - ρ_prox|
    double t_us = 1.0; // From previous US result
    double rho_ds = 1.0; // Distance to task
    double rho_prox = 0.5; // Proximity to robot
    double V_m = 1.0; // Max velocity
    
    double t_cs = (1.0 / t_us) + (1.0 / V_m) * std::abs((1.0 / rho_ds) - rho_prox);
    
    // Determine cost t_cs based on automaton state
    // If automaton state V(n) == 1: highest priority
    bool isAcceptingState = nba && nba->isAccepting(q);
    
    if (isAcceptingState) {
        // Higher priority for accepting states
        t_cs = std::max(t_cs, 0.9);
    } else {
        // Lower priority for non-accepting states
        t_cs = std::min(t_cs, 0.5);
    }
    
    // Return best compatible task tree node
    // TODO: Create and return appropriate tree node
    return nullptr;
}

/**
 * Algorithm 4: Exclusive-task Search (ES)
 * Searches for exclusive tasks that have conflicting batches
 * Input: d_s (robot state), T^sub_B (sub-tasks), T_B (tasks)
 */
Tree_Node* TaskAllocationAlgorithms::exclusiveTaskSearch(
    uint32_t ds,
    const std::vector<uint16_t>& subTasks,
    const std::vector<uint16_t>& tasks) {
    
    if (!nba || !multiRobotSystem) {
        return 0;
    }
    
    // Step 1: Check if there exist exclusive robots in T_B
    // Find tasks where d_s.bat = -d_s.bat (conflicting batches)
    std::vector<uint16_t> exclusiveRobots;
    
    for (uint16_t task : tasks) {
        // Check for batch conflicts with sub-tasks
        for (uint16_t subTask : subTasks) {
            // If batches are conflicting (opposite signs or mutually exclusive)
            // Add to exclusive robots set
            if (task != subTask) {
                exclusiveRobots.push_back(task);
                break;
            }
        }
    }
    
    // Step 2: If exclusive robots exist and automaton is in accepting state
    std::vector<Robot*> exclusiveSet;  // A^-
    
    // Get all robots and filter for exclusives
    if (!exclusiveRobots.empty() && nba) {
        // Check current automaton states - for all nodes, check if V(n) == 1
        bool hasAcceptingState = false;
        for (uint32_t i = 0; i < nba->getNumStates(); ++i) {
            if (nba->isAccepting(i)) {
                hasAcceptingState = true;
                break;
            }
        }
        
        if (hasAcceptingState) {
            // Add exclusive robots to A^- set
            for (auto robot : multiRobotSystem->getRobots()) {
                if (robot) {
                    exclusiveSet.push_back(robot);
                }
            }
        }
    }
    
    // Step 3: Generate new multi-robot system A_new = A \ A^-
    // This represents available robots after removing exclusive ones
    
    // Step 4: Determine t_es cost
    // ρ_pos = cost to reach ds
    // ρ_prox = proximity cost
    double rho_pos = 1.0;
    double rho_prox = 0.5;
    double V_m = 1.0;
    
    // Cost formula: t_es = t_cs^-1 + (1/V_m) * |ρ_ds^-1 - ρ_prox|
    double t_cs = 1.0;  // From previous CS result
    // double t_es = (1.0 / t_cs) + (1.0 / V_m) * std::abs((1.0 / rho_pos) - rho_prox);
    
    // Step 5: Set d^sub according to sorted d, V based on d_s, V and z_cs
    uint16_t best_d_sub = 0;
    if (!subTasks.empty()) {
        best_d_sub = subTasks[0];
    }
    
    // Step 6: If d^sub.V(n) == 1, set t^i and p^i_cs
    bool isAcceptingTask = false;
    if (nba && best_d_sub < nba->getNumStates()) {
        isAcceptingTask = nba->isAccepting(best_d_sub);
    }
    
    if (isAcceptingTask) {
        // Set maximum t_i
        // double t_i = 1.0;
        // Set proximity p^i_cs
        // double p_i_cs = 0.8;
    }
    
    // Step 7: Set d^sub_a based on t_i (task assignment)
    // This would involve assigning the task to an available robot
    
    // Step 8: Return exclusive task tree node
    // TODO: Create and return appropriate tree node
    return nullptr;
}
