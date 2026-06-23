#include "TaskAllocationAlgorithms.h"
#include <algorithm>
#include <queue>
#include <limits>
#include <cmath>
#include <map>

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

PlanningDecisionTree* TaskAllocationAlgorithms::intensiveInterTaskRelationshipTreeSearch(
    BuchiAutomaton* nbaPtr,
    Environment* envPtr,
    MultiRobotSystem* multiRobotSystemPtr) {
    
    // Get initial automaton state from the Büchi automaton
    Node* initialAutomatonState = nbaPtr->getNode(nbaPtr->getInitialState());
    // Get environment initial state ID
    uint16_t initialEnvStateId = envPtr->getInitialState();

    // Allocate on heap to avoid dangling pointer
    Node* initialEnvNodePtr = new Node(initialEnvStateId);
    
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
    PlanningDecisionTree* tree = new PlanningDecisionTree(                             // rootId
        initialAutomatonState,          // automaton state
        initialEnvNodePtr,              // ts state node (heap-allocated)
        initialAllocation,              // task allocation
        initialTimes,                   // times
        0,                              // batch
        Tree_Node::TASK_PROGRESS::PRE   // progress
    );
    setPlanningTree(tree);
    setTraversedTree(new PlanningDecisionTree());
    
    
    // Initialize untraversed queue with root node
    addUntraversedPlanningNode(tree->getRoot());
    
    //add root to the visited nodes and visited automaton states
    addVisitedNode(tree->getRoot());
    addVisitedAutomatonState(initialAutomatonState->getId());

    // Process nodes from untraversed queue until empty
    Tree_Node* currentNode = nullptr;
    int iterationCount = 0;
    while ((currentNode = getNextUntraversedNode()) != nullptr) {
        iterationCount++;
        std::cout << "\n=== ITERATION " << iterationCount << " ===" << std::endl;
        std::cout << "[DEBUG] Current node - NBA state: " << currentNode->getAutomatonState()->getId() 
                  << ", TS state: " << currentNode->getTSState()->getId()
                  << ", Progress: " << static_cast<int>(currentNode->getProgress()) << std::endl;
        
        // Create subtree for current node
        PlanningDecisionTree* subtree = new PlanningDecisionTree(currentNode);
        uint16_t buchisize = nbaPtr->getNumStates();
        std::cout << "[DEBUG] Processing " << buchisize << " automaton states from current node (state " 
                  << currentNode->getAutomatonState()->getId() << ")" << std::endl;
        
        // Iterate through all automaton states
        for (uint16_t i = 0; i < buchisize; ++i) {
            Node* nbaState = nbaPtr->getNode(i);
            uint16_t nbaId = nbaState->getId();
            
            if (!isAutomatonStateVisited(nbaId)) {
                std::cout << "  [DEBUG] Automaton state " << nbaId << " not visited, processing..." << std::endl;
                // Only process if this automaton state has NOT been visited in this task stage
                // Get edge labels from current automaton state to this state
                std::vector<std::string> edges = nbaPtr->getEdgeLabels(currentNode->getAutomatonState()->getId(), nbaId);
                std::cout << "    [DEBUG] Found " << edges.size() << " edge(s)" << std::endl;
                std::vector<uint16_t> apIds = collectUniqueAPsFromEdges(edges);
                std::cout << "    [DEBUG] Collected " << apIds.size() << " unique AP(s): ";
                for (auto apId : apIds) std::cout << apId << " ";
                std::cout << std::endl;
                
                for (uint16_t apId : apIds) {
                    int8_t batchVal = nbaPtr->getLTLFormula()->getBatchVal(apId);
                    std::cout << "      [DEBUG] AP " << apId << " has batch value " << static_cast<int>(batchVal) << std::endl;
                    Node* TSState = envPtr->getTransitionSystem()->getNode(apId);
                    
                    // Create new tree node with automaton state and task state
                    // Note: nodeId is a placeholder; insertNode will auto-assign the correct ID
                    Tree_Node* newNode = new Tree_Node(0, currentNode, nbaState, 
                                                       TSState, batchVal);
                    
                    // Route based on batch value
                    if (!isBatchValueInTree(batchVal) || batchVal == 0) {
                        std::cout << "        [DEBUG] Routing to unrelatedTaskSearch" << std::endl;
                        unrelatedTaskSearch(newNode, TSState, currentNode);
                    }
                    else if (batchVal > 0) {
                        std::cout << "        [DEBUG] Routing to compatibleTaskSearch" << std::endl;
                        // TODO: compatible task search needs to consider sub tasks and main tasks
                        compatibleTaskSearch(newNode, TSState, currentNode);
                    }
                    else {
                        std::cout << "        [DEBUG] Routing to exclusiveTaskSearch" << std::endl;
                        // TODO: exclusive task search needs to consider sub tasks and main tasks
                        exclusiveTaskSearch(newNode, TSState, currentNode);
                    }
                    
                    // Track visited automaton states based on progress
                    if (currentNode->getProgress() == newNode->getProgress()) {
                        addVisitedAutomatonState(nbaState->getId());
                    }
                    else {
                        clearVisitedAutomatonStates();
                    }
                    if (currentNode->getParent() == nullptr && nba->isAccepting(currentNode->getAutomatonState()->getId())) {
                        std::cout << "        [DEBUG] Current node is root and accepting, clearing visited automaton states for new batch..." << std::endl;
                        clearVisitedAutomatonStates();
                    }
                    
                    // Add to subtree (insertNode will auto-assign the correct nodeId)
                    subtree->insertNode(newNode);
                    visitedNodes.push_back(newNode); // Mark new node as visited
                    std::cout << "        [DEBUG] Added node to subtree (NBA: " << nbaId << ")" << std::endl;
                }
            }
        }
        
        // Show subtree state before pruning
        std::vector<Tree_Node*> subtreeNodesBefore = subtree->getAllNodes();
        std::cout << "[DEBUG] Subtree before pruning: " << subtreeNodesBefore.size() << " nodes" << std::endl;
        for (const auto& node : subtreeNodesBefore) {
            std::cout << "  - NBA: " << node->getAutomatonState()->getId() 
                      << ", Progress: " << static_cast<int>(node->getProgress()) << std::endl;
        }
        
        // Prune subtree
        std::cout << "[DEBUG] Calling pruneSubtree..." << std::endl;
        PlanningDecisionTree* pruningResult = pruneSubtree(subtree);
        
        // Show tree state after pruning
        std::vector<Tree_Node*> subtreeNodesAfter = pruningResult->getAllNodes();
        std::cout << "[DEBUG] Subtree after pruning: " << subtreeNodesAfter.size() << " nodes" << std::endl;
        for (const auto& node : subtreeNodesAfter) {
            std::cout << "  - NBA: " << node->getAutomatonState()->getId() 
                      << ", Progress: " << static_cast<int>(node->getProgress()) << std::endl;
        }
        
        // After pruning, add all remaining nodes in subtree to untraversed queue (except currentNode)
        std::cout << "[DEBUG] Adding nodes to untraversed queue..." << std::endl;
        std::vector<Tree_Node*> remainingNodes = pruningResult->getAllNodes();
        for (Tree_Node* node : remainingNodes) {
            if (node != currentNode) {
                addUntraversedPlanningNode(node);
                std::cout << "  [DEBUG] Added to untraversed: NBA " << node->getAutomatonState()->getId() << std::endl;
            }
        }
        
        // Show planning tree state before inserting subtree
        std::vector<Tree_Node*> planningTreeNodesBefore = planningTree->getAllNodes();
        std::cout << "[DEBUG] Planning tree before insert: " << planningTreeNodesBefore.size() << " nodes" << std::endl;
        // Attach subtree to planning tree
        // currentNode is the root of the subtree, so attach pruned subtree as children of currentNode
        planningTree->insertSubtree(currentNode, pruningResult);
        // Show planning tree state after inserting subtree
        std::vector<Tree_Node*> planningTreeNodesAfter = planningTree->getAllNodes();
        std::cout << "[DEBUG] Planning tree after insert: " << planningTreeNodesAfter.size() << " nodes" << std::endl;
        // Add currentNode to traversed tree
        traversedTree->insertNode(currentNode);
        std::cout << "[DEBUG] Added current node to traversed tree" << std::endl;
        std::cout << "[DEBUG] Untraversed queue size: " << std::endl;
        // Peek at queue size (we can't directly access it, but we can note this for observation)
    }
    std::cout << "\n=== SEARCH COMPLETE ===" << std::endl;
    
    Tree_Node* finalOptimalNode = planningTree->getOptimalFrontierNode(); // Final optimal node after search completion
    if (finalOptimalNode) {
        uint16_t cost = finalOptimalNode->getMaxTime(); 
        std::cout << "Optimal frontier node found with cost (max time): " << cost << std::endl;
        std::cout << "Optimal frontier node - NBA state: " << finalOptimalNode->
        getAutomatonState()->getId() << ", TS state: " << finalOptimalNode->getTSState()->getId() 
                  << ", Progress: " << static_cast<int>(finalOptimalNode->getProgress()) << std::endl;
        
        
        // Manually traverse from frontier node to root to avoid infinite loop issues
        std::cout << "Path to optimal node:" << std::endl;
        std::vector<Tree_Node*> path;
        Tree_Node* current = finalOptimalNode;
        size_t maxDepth = 1000;  // Safety check to prevent infinite loops
        size_t depth = 0;
        while (current != nullptr && depth < maxDepth) {
            path.push_back(current);
            Tree_Node* parent = current->getParent();
            if (parent == current) {
                std::cout << "  WARNING: Circular parent reference detected at node!" << std::endl;
                break;
            }
            current = parent;
            depth++;
        }
        
        // Print path from root to frontier
        std::reverse(path.begin(), path.end());
        for (Tree_Node* node : path) {
            std::cout << "  - NBA: " << node->getAutomatonState()->getId() 
                      << ", TS: " << node->getTSState()->getId() 
                      << ", Progress: " << static_cast<int>(node->getProgress());  
            std::cout << "Task Allocation Result: [";
            for (size_t i = 0; i < node->getRoboTaskAllocation().size(); ++i) {
                std::cout << (node->getRoboTaskAllocation()[i] ? "1" : "0");
                if (i < node->getRoboTaskAllocation().size() - 1) std::cout << ", ";
            }
            std::cout << "]" << std::endl;
        }
    }
    
    // Final tree state
    // Reassign node IDs to ensure proper hierarchy order (root = 0, then by depth)
    tree->reassignNodeIds();
    
    return tree;
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
    
    // current task state information
    uint16_t tsStateId = TSState->getId();
    // Get task location from environment mapping p_an
    Point taskLocation = environment->TSStateIdToGridCenter(tsStateId);
    std::vector<uint16_t> updatedtimes = multiRobotSystem->updateRobotTimesToGoal(currentNode->getTimes(), taskLocation);

    // Get the sort of the times vector and get the corresponding robot indices to find the best robot assignment
    std::vector<std::pair<uint16_t, uint16_t>> sortedTimes = Tree_Node::getSortedTimes(updatedtimes);
    
    // Get required capabilities from the BatchAtomicProposition
    BatchAtomicProposition batchAP = nba->getLTLFormula()->getAP(tsStateId);
    std::vector<bool> requiredCapabilities = batchAP.getCapabilities();
    
    // Find all permutations of robots that satisfy all required capabilities
    const std::vector<Robot*>& allRobots = multiRobotSystem->getRobots();
    
    auto [taskAllocation, maxTime] = getTaskAllocation(allRobots, requiredCapabilities, sortedTimes);
    
    std::cout << "Task Allocation Result: [";
    for (size_t i = 0; i < taskAllocation.size(); ++i) {
        std::cout << (taskAllocation[i] ? "1" : "0");
        if (i < taskAllocation.size() - 1) std::cout << ", ";
    }
    std::cout << "], Max Time: " << maxTime << std::endl;
    
    // Check if we found a valid allocation
    bool hasAllocation = false;
    for (bool allocated : taskAllocation) {
        if (allocated) {
            hasAllocation = true;
            break;
        }
    }
    
    if (!hasAllocation) {
        std::cout << "No valid task allocation found for this node." << std::endl;
        return;
    }
    std::vector<uint16_t> updatedTimes = currentNode->getTimes();
    // Set all allocated robots to the max time of allocated robots, leave others unchanged
    for (size_t i = 0; i < taskAllocation.size() && i < updatedTimes.size(); ++i) {
        if (taskAllocation[i]) {
            updatedTimes[i] = maxTime;
        }
    }
    
    // Update newNode with the task allocation and updated times
    newNode->setRoboTaskAllocation(taskAllocation);
    newNode->setTimes(updatedTimes);
    
    // Check if the new node is an accepting state (increment progress when entering accepting)
    if (newNode && nba) {
        bool acceptingState = nba->isAccepting(newNode->getAutomatonState()->getId());
        std::cout << "Automaton State " << newNode->getAutomatonState()->getId() 
                  << " which is " << (acceptingState ? "ACCEPTING" : "NON-ACCEPTING") << std::endl;
        if (acceptingState) {
            // Increment progress when entering accepting state
            int newProgressInt = static_cast<int>(currentNode->getProgress()) + 1;
            Tree_Node::TASK_PROGRESS newProgress = static_cast<Tree_Node::TASK_PROGRESS>(newProgressInt);
            newNode->setProgress(newProgress);
            std::cout << "Progress incremented to " << static_cast<int>(newProgress) << " (entering accepting)" << std::endl;
        } else {
            // Not entering accepting: keep progress the same
            newNode->setProgress(currentNode->getProgress());
        }
    }
    
    std::cout << "unrelatedTaskSearch completed successfully" << std::endl;
 }
/**
 * Algorithm 3: Compatible-Task Search (CS)
 * Searches for compatible tasks considering both sub-tasks and main tasks
 * Mutates newNode with search results
 * 
 * TODO: Implement cost calculation:
 *   - t_cs = t_us^-1 + (1/V_m) * |ρ_ds^-1 - ρ_prox|
 *   - Adjust costs based on accepting/non-accepting states
 *   - Implement task allocation using greedy robot selection
 *   - Update newNode with allocated robots and updated times
 */
void TaskAllocationAlgorithms::compatibleTaskSearch(
    Tree_Node* newNode,
    [[maybe_unused]] Node* TSState,
    [[maybe_unused]] Tree_Node* currentNode) {
    
    if (!newNode) return;
    
    // Priority adjustment based on automaton state
    if (newNode->getAutomatonState() && nba) {
        bool isAcceptingState = nba->isAccepting(newNode->getAutomatonState()->getId());
        
        // Accepting states get higher priority in cost calculation
        // Non-accepting states get lower priority
        (void)isAcceptingState;  // Mark variable as intentionally used for future implementation
    }
    
    // Implementation will be added in future iterations
}

/**
 * Algorithm 4: Exclusive-task Search (ES)
 * Searches for exclusive tasks that have conflicting batches
 * Mutates newNode with search results
 * 
 * TODO: Implement exclusive task search:
 *   - Step 1: Identify exclusive robots (conflicting batch values)
 *   - Step 2: Build exclusive robot set A^- if automaton has accepting states
 *   - Step 3: Create reduced multi-robot system A_new = A \ A^-
 *   - Step 4: Calculate cost t_es = t_cs^-1 + (1/V_m) * |ρ_ds^-1 - ρ_prox|
 *   - Step 5: Allocate tasks using greedy selection on reduced system
 *   - Step 6: Update newNode with allocation and times
 */
void TaskAllocationAlgorithms::exclusiveTaskSearch(
    Tree_Node* newNode,
    Node* TSState,
    Tree_Node* currentNode) {
    
    if (!newNode || !nba || !multiRobotSystem) {
        return;
    }
    
    // Check for accepting states in automaton
    bool hasAcceptingState = false;
    for (uint16_t i = 0; i < nba->getNumStates(); ++i) {
        if (nba->isAccepting(i)) {
            hasAcceptingState = true;
            break;
        }
    }
    
    // Mark parameters as intentionally used for future implementation
    (void)TSState;
    (void)currentNode;
    (void)hasAcceptingState;
    
    // Implementation will be added in future iterations
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
    if (std::find(treebatchvals.begin(), treebatchvals.end(), batchValue) == treebatchvals.end()) {
        treebatchvals.push_back(batchValue);
    }
}

/**
 * isBatchValueInTree - Check if a batch value is in the tree
 * If not found, adds the absolute value of the batch (only if positive)
 */
bool TaskAllocationAlgorithms::isBatchValueInTree(int8_t batchValue) {
    uint8_t absBatchValue = static_cast<uint8_t>(std::abs(batchValue));
    bool found = std::find(treebatchvals.begin(), treebatchvals.end(), absBatchValue) != treebatchvals.end();
    if (!found && absBatchValue > 0) {
        treebatchvals.push_back(absBatchValue);
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
                    std::string numStr = token.substr(1);
                    uint16_t apId = static_cast<uint16_t>(std::stoul(numStr));
                    apIds.push_back(apId);
                }
            } catch (const std::exception& e) {
                // Silent catch
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
    
    // Process each edge label string
    for (const auto& edgeLabel : edges) {
        // Remove acceptance marks {0}
        std::string cleaned = edgeLabel;
        size_t pos = cleaned.find("{0}");
        if (pos != std::string::npos) {
            cleaned.erase(pos, 3);
        }
        
        // Check if there are 2+ true APs connected by AND (not OR)
        // Split by OR first to get independent clauses
        bool hasMultipleTrueAPsInAndClause = false;
        
        size_t orStart = 0;
        size_t orEnd = cleaned.find('|');
        
        while (orStart < cleaned.length() && !hasMultipleTrueAPsInAndClause) {
            // Extract OR-separated clause
            std::string orClause = (orEnd == std::string::npos) ? 
                                   cleaned.substr(orStart) : 
                                   cleaned.substr(orStart, orEnd - orStart);
            
            // Count true APs in this AND-clause (don't replace | with &)
            int trueApCount = 0;
            size_t andStart = 0;
            size_t andEnd = orClause.find('&');
            
            while (andStart < orClause.length()) {
                // Extract token
                std::string token = (andEnd == std::string::npos) ? 
                                   orClause.substr(andStart) : 
                                   orClause.substr(andStart, andEnd - andStart);
                
                // Trim whitespace and quotes
                token.erase(0, token.find_first_not_of(" \t\n\r\""));
                token.erase(token.find_last_not_of(" \t\n\r\"") + 1);
                
                // Count if this is a true (non-negated) AP
                if (!token.empty() && token[0] != '!') {
                    trueApCount++;
                }
                
                // Move to next token
                if (andEnd == std::string::npos) break;
                andStart = andEnd + 1;
                andEnd = orClause.find('&', andStart);
            }
            
            // If 2+ true APs AND-ed together, skip this edge
            if (trueApCount >= 2) {
                hasMultipleTrueAPsInAndClause = true;
            }
            
            // Move to next OR clause
            if (orEnd == std::string::npos) break;
            orStart = orEnd + 1;
            orEnd = cleaned.find('|', orStart);
        }
        
        // Skip edges with 2+ true APs connected by AND
        if (hasMultipleTrueAPsInAndClause) {
            continue;
        }
        
        // Parse the edge label to extract AP IDs
        std::vector<uint16_t> apIds = parseEdgeLabel(edgeLabel);
        
        // Merge parsed AP IDs into the result, avoiding duplicates
        for (uint16_t apId : apIds) {
            // Check if this AP ID is already in the collection
            auto it = std::find(allApIds.begin(), allApIds.end(), apId);
            if (it == allApIds.end()) {
                // Add only if not already present
                allApIds.push_back(apId);
            }
        }
    }
    
    return allApIds;
}

/**
 * getTaskAllocation
 * Greedily selects minimum set of robots from sorted times that satisfy all required capabilities
 * Iterates through robots in ascending time order and accumulates only required capabilities
 * Returns (task allocation vector, max time of selected robots)
 */
std::pair<std::vector<bool>, uint16_t> TaskAllocationAlgorithms::getTaskAllocation(
    const std::vector<Robot*>& robots,
    const std::vector<bool>& requiredCapabilities, 
    const std::vector<std::pair<uint16_t, uint16_t>>& sortedTimes) {
    
    std::vector<bool> taskAllocation(robots.size(), false);
    std::vector<bool> accumulatedCapabilities(requiredCapabilities.size(), false);
    uint16_t maxTime = 0;
    
    if (robots.empty() || requiredCapabilities.empty()) {
        return {taskAllocation, maxTime};
    }
    
    // Iterate through robots in ascending time order (best times first)
    // Exit as soon as all required capabilities are satisfied
    auto it = sortedTimes.begin();
    while (it != sortedTimes.end() && accumulatedCapabilities != requiredCapabilities) {
        const auto& [robotIdx, time] = *it;
        
        if (robotIdx < robots.size()) {
            Robot* robot = robots[robotIdx];
            if (robot) {
                std::vector<bool> robotCapabilities = robot->getCapabilities();
                bool robotContributesCapability = false;
                
                // Check if this robot contributes any NEW required capabilities (not already accumulated)
                for (size_t j = 0; j < requiredCapabilities.size() && j < robotCapabilities.size(); ++j) {
                    if (requiredCapabilities[j] && robotCapabilities[j] && !accumulatedCapabilities[j]) {
                        accumulatedCapabilities[j] = true;
                        robotContributesCapability = true;
                    }
                }
                
                // Only mark robot as selected if it contributes at least one required capability
                if (robotContributesCapability) {
                    taskAllocation[robotIdx] = true;
                    maxTime = std::max(maxTime, time);  // Update max time of selected robots
                }
            }
        }
        
        ++it;
    }

    return {taskAllocation, maxTime};
}
//helper method to prune subtree
// Implements three boundary rules:
// Rule 1: Remove nodes with OTH progress (terminal nodes)
// Rule 2: Remove nodes with (automaton state, progress) pairs already in traversedTree
// Rule 3: For nodes with same automaton state and progress, keep only minimum cost
PlanningDecisionTree* TaskAllocationAlgorithms::pruneSubtree(PlanningDecisionTree* subtree) {
    std::vector<Tree_Node*> nodesToRemove;
    std::vector<Tree_Node*> subtreeNodes = subtree->getAllNodes();
    
    std::cout << "    [PRUNE] Starting pruneSubtree with " << subtreeNodes.size() << " nodes" << std::endl;
    
    // Rule 1: Remove nodes with OTH progress (terminal nodes)
    std::cout << "    [PRUNE] Rule 1: Checking for OTH progress nodes..." << std::endl;
    for (Tree_Node* node : subtreeNodes) {
        if (node->getProgress() == Tree_Node::TASK_PROGRESS::OTH) {
            std::cout << "      [PRUNE] Marking for removal: NBA " << node->getAutomatonState()->getId() 
                      << " (OTH progress)" << std::endl;
            nodesToRemove.push_back(node);
        }
    }
    
    // Rule 2: Remove nodes with traversed (automaton state, progress) pairs
    // If we've already visited (state S, progress P) in traversedTree, don't sample it again
    std::vector<Tree_Node*> traversedNodes = traversedTree->getAllNodes();
    std::cout << "    [PRUNE] Rule 2: Checking traversed tree (" << traversedNodes.size() << " nodes)..." << std::endl;
    
    for (Tree_Node* node : subtreeNodes) {
        // Skip if already marked for removal
        if (std::find(nodesToRemove.begin(), nodesToRemove.end(), node) != nodesToRemove.end()) {
            continue;
        }
        
        // Check if this (automaton state, progress) pair exists in traversedTree
        bool foundInTraversed = false;
        for (Tree_Node* traversedNode : traversedNodes) {
            if (node->getAutomatonState() && traversedNode->getAutomatonState() &&
                node->getAutomatonState()->getId() == traversedNode->getAutomatonState()->getId() &&
                node->getProgress() == traversedNode->getProgress()) {
                foundInTraversed = true;
                std::cout << "      [PRUNE] Found duplicate in traversed tree: NBA " << node->getAutomatonState()->getId() 
                          << " with progress " << static_cast<int>(node->getProgress()) << std::endl;
                break;
            }
        }
        
        if (foundInTraversed) {
            std::cout << "      [PRUNE] Marking for removal: NBA " << node->getAutomatonState()->getId() 
                      << " (duplicate in traversed)" << std::endl;
            nodesToRemove.push_back(node);
        }
    }
    
    // Rule 3: For nodes with same automaton state and progress, keep only minimum cost
    // Cost is calculated as sum of all robot execution times
    std::cout << "    [PRUNE] Rule 3: Checking for duplicate states (same NBA + Progress)..." << std::endl;
    std::map<std::pair<uint32_t, int>, std::vector<Tree_Node*>> nodeGroups;
    
    // Group remaining nodes by (automaton state ID, progress)
    for (Tree_Node* node : subtreeNodes) {
        if (std::find(nodesToRemove.begin(), nodesToRemove.end(), node) != nodesToRemove.end()) {
            continue;  // Skip nodes already marked for removal
        }
        
        if (node->getAutomatonState()) {
            auto key = std::make_pair(node->getAutomatonState()->getId(), 
                                     static_cast<int>(node->getProgress()));
            nodeGroups[key].push_back(node);
        }
    }
    
    // Within each group with duplicates, keep only the minimum cost node
    for (auto& [key, nodes] : nodeGroups) {
        if (nodes.size() > 1) {
            std::cout << "      [PRUNE] Found " << nodes.size() << " nodes with same NBA " << key.first 
                      << " and progress " << key.second << " - selecting min cost" << std::endl;
            
            // Calculate cost for each node (sum of all robot times)
            Tree_Node* minCostNode = nodes[0];
            uint32_t minCost = 0;
            for (uint16_t time : minCostNode->getTimes()) {
                minCost += time;
            }
            std::cout << "        Initial min cost: " << minCost << std::endl;
            
            // Find minimum cost node and mark all others for removal
            for (size_t i = 1; i < nodes.size(); ++i) {
                uint32_t nodeCost = 0;
                for (uint16_t time : nodes[i]->getTimes()) {
                    nodeCost += time;
                }
                
                std::cout << "        Node " << i << " cost: " << nodeCost << std::endl;
                
                if (nodeCost < minCost) {
                    // Current node is better, mark previous min for removal
                    std::cout << "          -> Marking previous min for removal" << std::endl;
                    nodesToRemove.push_back(minCostNode);
                    minCostNode = nodes[i];
                    minCost = nodeCost;
                } else {
                    // Current node is worse or equal, mark for removal
                    std::cout << "          -> Marking for removal (higher cost)" << std::endl;
                    nodesToRemove.push_back(nodes[i]);
                }
            }
            std::cout << "        Keeping node with cost: " << minCost << std::endl;
        }
    }
    
    std::cout << "    [PRUNE] Total nodes marked for removal: " << nodesToRemove.size() << std::endl;
    std::cout << "    [PRUNE] Nodes remaining in subtree: " << (subtreeNodes.size() - nodesToRemove.size()) << std::endl;
    
    // Remove all marked nodes from subtree and add to traversedTree
    // Transfer nodes from subtree to traversedTree (don't delete, just move)
    for (Tree_Node* node : nodesToRemove) {
        // Add to traversedTree first (while node still exists in memory)
        traversedTree->insertNode(node);
        // Remove from subtree's frontier (transfers ownership semantically)
        subtree->removeFrontierNode(node);
    } 
    
    return subtree;
}