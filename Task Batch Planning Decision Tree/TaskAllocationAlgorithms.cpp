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
    : nba(nbaPtr), environment(envPtr), planningTree(nullptr), traversedTree(nullptr), multiRobotSystem(robotSysPtr) {
}

/**
 * TaskAllocationAlgorithms - Destructor
 */
TaskAllocationAlgorithms::~TaskAllocationAlgorithms() {
    // Note: We don't delete the pointers here as they are managed externally
}

//  * Algorithm 1: Intensive Inter-Task Relationship Tree Search
//  * Converts LTL formula to Büchi automaton and initializes tree planning
//  */


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
 * setPlanningTree - Set the planning tree reference
 */
void TaskAllocationAlgorithms::setPlanningTree(PlanningDecisionTree* tree) {
    planningTree = tree;
}

/**
 * getPlanningTree - Get the planning tree reference
 */
PlanningDecisionTree* TaskAllocationAlgorithms::getPlanningTree() const {
    return planningTree;
}

/**
 * setTraversedTree - Set the traversed tree reference
 */
void TaskAllocationAlgorithms::setTraversedTree(PlanningDecisionTree* tree) {
    traversedTree = tree;
}

/**
 * getTraversedTree - Get the traversed tree reference
 */
PlanningDecisionTree* TaskAllocationAlgorithms::getTraversedTree() const {
    return traversedTree;
}

Tree_Node* TaskAllocationAlgorithms::intensiveInterTaskRelationshipTreeSearch(
    BuchiAutomaton* nbaPtr,
    Environment* envPtr,
    MultiRobotSystem* multiRobotSystemPtr) {
    
    // Get initial automaton state (typically state 0)
    Node* initialAutomatonState = nbaPtr->getNode(0);
    // Get environment initial state ID
    uint16_t initialEnvStateId = envPtr->getInitialState();

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
    setPlanningTree(tree);
    setTraversedTree(new PlanningDecisionTree());
    
    // Initialize untraversed queue with root node
    addUntraversedPlanningNode(tree->getRoot());
    
    // ID counter for new nodes
    uint32_t nodeIdCounter = 1;
    //add root to the visited nodes and visited automaton states
    addVisitedNode(tree->getRoot());
    addVisitedAutomatonState(initialAutomatonState->getId());
    
    // Process nodes from untraversed queue until empty
    Tree_Node* currentNode = nullptr;
    while ((currentNode = getNextUntraversedNode()) != nullptr) {
        
        // Create subtree for current node
        PlanningDecisionTree* subtree = new PlanningDecisionTree();
        uint16_t buchisize = nbaPtr->getNumStates();
        
        // Iterate through all automaton states
        for (uint16_t i = 0; i < buchisize; ++i) {
            Node* nbaState = nbaPtr->getNode(i);
            uint16_t nbaId = nbaState->getId();
            
            if (!isAutomatonStateVisited(nbaId)) {
                // Only process if this automaton state has NOT been visited in this task stage
                // Get edge labels from current automaton state to this state
                std::vector<std::string> edges = nbaPtr->getEdgeLabels(nbaId, currentNode->getAutomatonState()->getId());
                std::vector<uint16_t> apIds = collectUniqueAPsFromEdges(edges);
                
                for (uint16_t apId : apIds) {
                    int8_t batchVal = nbaPtr->getLTLFormula()->getBatchVal(apId);
                    Node* TSState = envPtr->getTSStateForAP(apId);
                    
                    // Create new tree node with automaton state and task state
                    nodeIdCounter++;
                    Tree_Node* newNode = new Tree_Node(nodeIdCounter, currentNode, nbaState, 
                                                       TSState, batchVal);
                    
                    // Route based on batch value
                    if (!isBatchValueInTree(batchVal) || batchVal == 0) {
                        unrelatedTaskSearch(newNode, TSState, currentNode);
                    }
                    else if (batchVal > 0) {
                        // TODO: compatible task search needs to consider sub tasks and main tasks
                        compatibleTaskSearch(newNode, TSState, currentNode);
                    }
                    else {
                        // TODO: exclusive task search needs to consider sub tasks and main tasks
                        exclusiveTaskSearch(newNode, TSState, currentNode);
                    }
                    
                    // Track visited automaton states based on progress
                    if (currentNode->prog == newNode->prog) {
                        addVisitedAutomatonState(nbaState->getId());
                    }
                    else {
                        clearVisitedAutomatonStates();
                    }
                    
                    // Add to subtree
                    subtree->insertNode(newNode);
                }
            }
        }
        
        // Prune subtree (TODO: implement pruning logic)
        // ...
        
        // After pruning, add all remaining nodes in subtree to untraversed queue (except currentNode)
        std::vector<Tree_Node*> remainingNodes = subtree->getAllNodes();  // Assumes this method exists
        for (Tree_Node* node : remainingNodes) {
            if (node != currentNode) {
                addUntraversedPlanningNode(node);
            }
        }
        
        // Attach subtree to planning tree
        if (currentNode->getParent()) {
            planningTree->insertSubtree(currentNode->getParent(), subtree);
        }
        else {
            // If currentNode is root, insert subtree differently
            planningTree->insertSubtree(currentNode, subtree);
        }
        
        // Add currentNode to traversed tree
        traversedTree->addNode(currentNode);
    }
    
    return tree->getRoot();
}
/**
 * Algorithm 2: Unrelated-Task Search (US)
 * Searches for unrelated tasks that can be executed independently
 * Mutates newNode with search results
 */
void TaskAllocationAlgorithms::unrelatedTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode) {
    
    if (!newNode || !TSState || !multiRobotSystem || !nba || !environment) {
        return;
    }
    
    uint16_t tsStateId = TSState->getId();
    Point taskLocation = environment->TSStatetoGridCenter(tsStateId);
    
    // Update all robot times based on current positions and travel to task location
    std::vector<uint16_t> curtimes = newNode->getTimes();
    std::vector<uint16_t> updatedTimes = multiRobotSystem->updateAllRobotTimes(curtimes, taskLocation);

    //get the sort of the times vector and get the corresponding robot indices to find the best robot assignment
    std::vector<std::pair<uint16_t, uint16_t>> sortedTimes = newNode->getSortedTimes();
    
    // Get required capabilities from the BatchAtomicProposition
    BatchAtomicProposition batchAP = nba->getLTLFormula()->getAP(tsStateId);
    std::vector<bool> requiredCapabilities = batchAP.getCapabilities();
    
    // Find all permutations of robots that satisfy all required capabilities
    std::vector<Robot*>& allRobots = multiRobotSystem->getRobots();
    std::vector<bool> V = 
        findMinRoboTimes(allRobots, requiredCapabilities, sortedTimes);
    
    if (V.empty()) {
        // No valid robot combinations found that satisfy all capabilities
        return;
    }
    
    // Calculate time for each valid permutation and select minimum
    uint16_t minTime = std::numeric_limits<uint16_t>::max();
    std::vector<Robot*> bestPermutation;
    
        
/**
 * Algorithm 3: Compatible-Task Search (CS)
 * Searches for compatible tasks considering both sub-tasks and main tasks
 * Mutates newNode with search results
 */
void TaskAllocationAlgorithms::compatibleTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode) {
    
    if (!newNode) return;
    
    // Initialize cost: t_cs = t_us^-1 + (1/V_m) * |ρ_ds^-1 - ρ_prox|
    double t_us = 1.0; // From previous US result
    double rho_ds = 1.0; // Distance to task
    double rho_prox = 0.5; // Proximity to robot
    double V_m = 1.0; // Max velocity
    
    double t_cs = (1.0 / t_us) + (1.0 / V_m) * std::abs((1.0 / rho_ds) - rho_prox);
    
    // Determine cost t_cs based on automaton state
    // If automaton state V(n) == 1: highest priority
    if (newNode->getAutomatonState() && nba) {
        bool isAcceptingState = nba->isAccepting(newNode->getAutomatonState()->getId());
        
        if (isAcceptingState) {
            // Higher priority for accepting states
            t_cs = std::max(t_cs, 0.9);
        } else {
            // Lower priority for non-accepting states
            t_cs = std::min(t_cs, 0.5);
        }
    }
    
    // TODO: Mutate newNode with appropriate cost calculations and task assignment
}

/**
 * Algorithm 4: Exclusive-task Search (ES)
 * Searches for exclusive tasks that have conflicting batches
 * Mutates newNode with search results
 */
void TaskAllocationAlgorithms::exclusiveTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode) {
    
    if (!newNode || !nba || !multiRobotSystem) {
        return;
    }
    
    // Step 1: Check if there exist exclusive robots
    // Find robots with conflicting batch values
    std::vector<Robot*> exclusiveRobots;
    
    // Step 2: If exclusive robots exist and automaton is in accepting state
    std::vector<Robot*> exclusiveSet;  // A^-
    
    // Check current automaton states - for all nodes, check if V(n) == 1
    bool hasAcceptingState = false;
    for (uint16_t i = 0; i < nba->getNumStates(); ++i) {
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
    
    // TODO: Mutate newNode with appropriate cost calculations and task assignment
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
void TaskAllocationAlgorithms::addVisitedAutomatonState(uint16_t state) {
    visitedAutomatonStates.push_back(state);
}

/**
 * isAutomatonStateVisited - Check if an automaton state has been visited
 */
bool TaskAllocationAlgorithms::isAutomatonStateVisited(uint16_t state) const {
    return std::find(visitedAutomatonStates.begin(), visitedAutomatonStates.end(), state) != visitedAutomatonStates.end();
}

/**
 * getVisitedAutomatonStates - Get the collection of visited automaton states
 */
std::vector<uint16_t>& TaskAllocationAlgorithms::getVisitedAutomatonStates() {
    return visitedAutomatonStates;
}

/**
 * clearVisitedAutomatonStates - Clear all visited automaton states
 */
void TaskAllocationAlgorithms::clearVisitedAutomatonStates() {
    visitedAutomatonStates.clear();
}

/**
 * addBatchValue - Add a batch value to the collection
 */
void TaskAllocationAlgorithms::addBatchValue(uint8_t batchValue) {
    treebatchvals.push_back(batchValue);
}

/**
 * isBatchValueInTree - Check if a batch value is in the tree
 * If not found, adds the absolute value of the batch (only if positive)
 */
bool TaskAllocationAlgorithms::isBatchValueInTree(int8_t batchValue) {
    uint8_t absBatchValue = static_cast<uint8_t>(std::abs(batchValue));
    bool found = std::find(treebatchvals.begin(), treebatchvals.end(), absBatchValue) != treebatchvals.end();
    if (!found) {
        if (absBatchValue > 0) {
            treebatchvals.push_back(absBatchValue);
        }
    }
    return found;
}

/**
 * getBatchValues - Get the collection of batch values in the tree
 */
std::vector<uint8_t>& TaskAllocationAlgorithms::getBatchValues() {
    return treebatchvals;
}

/**
 * clearBatchValues - Clear all batch values
 */
void TaskAllocationAlgorithms::clearBatchValues() {
    treebatchvals.clear();
}

/**
 * addUntraversedPlanningNode - Add a node to the untraversed queue
 */
void TaskAllocationAlgorithms::addUntraversedPlanningNode(Tree_Node* node) {
    if (node) {
        untraversedPlanningQueue.push(node);
    }
}

/**
 * getNextUntraversedNode - Get the next untraversed node from the queue
 */
Tree_Node* TaskAllocationAlgorithms::getNextUntraversedNode() {
    if (!untraversedPlanningQueue.empty()) {
        Tree_Node* node = untraversedPlanningQueue.front();
        untraversedPlanningQueue.pop();
        return node;
    }
    return nullptr;
}

/**
 * clearUntraversedQueue - Clear all untraversed nodes from the queue
 */
void TaskAllocationAlgorithms::clearUntraversedQueue() {
    while (!untraversedPlanningQueue.empty()) {
        untraversedPlanningQueue.pop();
    }
}

std::vector<uint16_t> TaskAllocationAlgorithms::parseEdgeLabel(const std::string& label) const {
    std::vector<uint16_t> apIds;
    
    // Remove acceptance marks {0}
    std::string cleaned = label;
    size_t pos = cleaned.find("{0}");
    if (pos != std::string::npos) {
        cleaned.erase(pos, 3);
    }
    
    // Replace | with & to have single delimiter
    for (size_t i = 0; i < cleaned.length(); ++i) {
        if (cleaned[i] == '|') {
            cleaned[i] = '&';
        }
    }
    
    // Split by &
    size_t start = 0;
    size_t end = cleaned.find('&');
    
    while (start < cleaned.length()) {
        // Extract token
        std::string token = (end == std::string::npos) ? 
                           cleaned.substr(start) : 
                           cleaned.substr(start, end - start);
        
        // Trim whitespace and quotes
        token.erase(0, token.find_first_not_of(" \t\n\r\""));
        token.erase(token.find_last_not_of(" \t\n\r\"") + 1);
        
        // Skip if empty or negated (starts with !)
        if (!token.empty() && token[0] != '!') {
            try {
                // Extract number from "p0", "p1", etc. (skip 'p' prefix)
                if (token[0] == 'p' && token.length() > 1) {
                    uint16_t apId = static_cast<uint16_t>(std::stoul(token.substr(1)));
                    apIds.push_back(apId);
                }
            } catch (const std::exception& e) {
                std::cerr << "Error parsing AP: " << token << std::endl;
            }
        }
        
        // Move to next token
        if (end == std::string::npos) break;
        start = end + 1;
        end = cleaned.find('&', start);
    }
    
    return apIds;
}

std::vector<uint16_t> TaskAllocationAlgorithms::collectUniqueAPsFromEdges(const std::vector<std::string>& edges) const {
    std::vector<uint16_t> allApIds;
    
    for (const auto& edgeLabel : edges) {
        std::vector<uint16_t> apIds = parseEdgeLabel(edgeLabel);
        // Insert unique elements
        for (uint16_t apId : apIds) {
            if (std::find(allApIds.begin(), allApIds.end(), apId) == allApIds.end()) {
                allApIds.push_back(apId);
            }
        }
    }
    
    return allApIds;
}

/**
 * findMinRoboTimes
 * Finds all combinations of robots whose combined capabilities satisfy all required capabilities
 * Uses bitmask iteration to generate all possible combinations (2^n)
 * Filters to only valid combinations where all required capabilities are satisfied
 */
std::vector<bool> TaskAllocationAlgorithms::getTaskAllocation(
    std::vector<Robot*>& robots,
    const std::vector<bool>& requiredCapabilities, const std::vector<std::pair<uint16_t, uint16_t>>& sortedTimes) {
    
    std::vector<bool> bestVector = MultiRobotSystem::getEmptyV();  // This will store the best combination of robots (as a boolean vector)
    
    if (robots.empty() || requiredCapabilities.empty()) {
        return bestVector;  // No robots or no capabilities means we can't satisfy requirements
    }
    // Number of robots and capabilities
    std::vector<bool> curcapabilities = MultiRobotSystemptr->getRobot(sortedTimes[0].first)->getCapabilities();  
    // This will store the capabilities of the current combination of robots
    uint8_t i = 0;
    while () {
        uint16_t maxtime = sortedTimes[i].second;  // Get the max time (second element of pair)
        
        for (size_t j = 0; j < requiredCapabilities.size(); ++j) {
            if (requiredCapabilities[i]) {
                curcapabilities.push_back(static_cast<RobotCapability>(i));
            }
            i++;
        }
    }
}
