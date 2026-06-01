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
    PlanningDecisionTree* tree = new PlanningDecisionTree(                             // rootId
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
    
    //add root to the visited nodes and visited automaton states
    addVisitedNode(tree->getRoot());
    addVisitedAutomatonState(initialAutomatonState->getId());
    
    // Process nodes from untraversed queue until empty
    Tree_Node* currentNode = nullptr;
    while ((currentNode = getNextUntraversedNode()) != nullptr) {
        
        // Create subtree for current node
        PlanningDecisionTree* subtree = new PlanningDecisionTree();
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
                    
                    // Add to subtree (insertNode will auto-assign the correct nodeId)
                    subtree->insertNode(newNode);
                }
            }
        }
        
        // Prune subtree (TODO: implement pruning logic)
        PlanningDecisionTree* pruningResult = pruneSubtree(subtree);
        
        // After pruning, add all remaining nodes in subtree to untraversed queue (except currentNode)
        std::vector<Tree_Node*> remainingNodes = pruningResult->getAllNodes();  // Assumes this method exists
        for (Tree_Node* node : remainingNodes) {
            if (node != currentNode) {
                addUntraversedPlanningNode(node);
            }
        }
        
        // Attach subtree to planning tree
        if (currentNode->getParent()) {
            planningTree->insertSubtree(currentNode->getParent(), pruningResult);
        }
        else {
            // If currentNode is root, insert subtree differently
            planningTree->insertSubtree(currentNode, pruningResult);
        }
        
        // Add currentNode to traversed tree
        traversedTree->insertNode(currentNode);
    }
    
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
    std::vector<uint16_t> updatedtimes = MultiRobotSystem::updateAllRobotTimes(multiRobotSystem->getRobots(), currentNode->getTimes(), taskLocation);

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
        return;
    }

    // Update all robot times based on current positions and travel to task location
    std::vector<uint16_t> curtimes = newNode->getTimes();
    
    std::vector<uint16_t> updatedTimes = MultiRobotSystem::updateAllRobotTimes(multiRobotSystem->getRobots(), curtimes, taskLocation);
    
    // Set all allocated robots to the max time of allocated robots, leave others unchanged
    for (size_t i = 0; i < taskAllocation.size() && i < updatedTimes.size(); ++i) {
        if (taskAllocation[i]) {
            updatedTimes[i] = maxTime;
        }
    }
    
    // Update newNode with the task allocation and updated times
    newNode->setRoboTaskAllocation(taskAllocation);
    newNode->setTimes(updatedTimes);
    if (TSState && nba) {
        bool isAcceptingState = nba->isAccepting(newNode->getAutomatonState()->getId());
        std::cout << "Automaton State " << newNode->getAutomatonState()->getId() 
                  << " is " << (isAcceptingState ? "ACCEPTING" : "NON-ACCEPTING") << std::endl;
        if (isAcceptingState) {
            newNode->setProgress(currentNode->getProgress());    // If accepting state, increment progress
            std::cout << "Progress updated" << std::endl;
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
    
    std::cout << "[parseEdgeLabel] ===== ENTRY =====" << std::endl;
    std::cout << "[parseEdgeLabel] Input label: \"" << label << "\"" << std::endl;
    std::cout << "[parseEdgeLabel] Label length: " << label.length() << " characters" << std::endl;
    
    // Remove acceptance marks {0}
    std::string cleaned = label;
    size_t pos = cleaned.find("{0}");
    if (pos != std::string::npos) {
        std::cout << "[parseEdgeLabel] Found acceptance mark {0} at position " << pos << ", removing..." << std::endl;
        cleaned.erase(pos, 3);
        std::cout << "[parseEdgeLabel] After removing {0}: \"" << cleaned << "\"" << std::endl;
    } else {
        std::cout << "[parseEdgeLabel] No acceptance marks {0} found" << std::endl;
    }
    
    // Replace | with & to have single delimiter
    int pipeCount = 0;
    for (size_t i = 0; i < cleaned.length(); ++i) {
        if (cleaned[i] == '|') {
            pipeCount++;
            cleaned[i] = '&';
        }
    }
    if (pipeCount > 0) {
        std::cout << "[parseEdgeLabel] Replaced " << pipeCount << " pipe(s) (|) with ampersand(&)" << std::endl;
        std::cout << "[parseEdgeLabel] After replacement: \"" << cleaned << "\"" << std::endl;
    } else {
        std::cout << "[parseEdgeLabel] No pipes (|) to replace" << std::endl;
    }
    
    // Split by &
    size_t start = 0;
    size_t end = cleaned.find('&');
    int tokenCount = 0;
    
    std::cout << "[parseEdgeLabel] Starting tokenization..." << std::endl;
    
    while (start < cleaned.length()) {
        // Extract token
        std::string token = (end == std::string::npos) ? 
                           cleaned.substr(start) : 
                           cleaned.substr(start, end - start);
        
        std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] (raw): \"" << token << "\"" << std::endl;
        
        // Trim whitespace and quotes
        token.erase(0, token.find_first_not_of(" \t\n\r\""));
        token.erase(token.find_last_not_of(" \t\n\r\"") + 1);
        
        std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] (trimmed): \"" << token << "\"" << std::endl;
        
        // Skip if empty or negated (starts with !)
        if (token.empty()) {
            std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] - SKIPPED (empty)" << std::endl;
        } else if (token[0] == '!') {
            std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] - SKIPPED (negated, starts with !)" << std::endl;
        } else {
            try {
                // Extract number from "p0", "p1", etc. (skip 'p' prefix)
                if (token[0] == 'p' && token.length() > 1) {
                    std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] - Format OK (starts with 'p')" << std::endl;
                    std::string numStr = token.substr(1);
                    std::cout << "[parseEdgeLabel]   Extracting number from: \"" << numStr << "\"" << std::endl;
                    uint16_t apId = static_cast<uint16_t>(std::stoul(numStr));
                    std::cout << "[parseEdgeLabel]   Parsed AP ID: " << apId << std::endl;
                    apIds.push_back(apId);
                } else if (token[0] != 'p') {
                    std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] - SKIPPED (doesn't start with 'p')" << std::endl;
                } else {
                    std::cout << "[parseEdgeLabel]   Token[" << tokenCount << "] - SKIPPED (starts with 'p' but no number)" << std::endl;
                }
            } catch (const std::exception& e) {
                std::cerr << "[parseEdgeLabel] ERROR parsing AP from token \"" << token << "\": " << e.what() << std::endl;
            }
        }
        
        // Move to next token
        if (end == std::string::npos) break;
        start = end + 1;
        end = cleaned.find('&', start);
        tokenCount++;
    }
    
    std::cout << "[parseEdgeLabel] Final Result: " << apIds.size() << " AP ID(s) extracted: [";
    if (apIds.empty()) {
        std::cout << "(empty)";
    } else {
        for (size_t i = 0; i < apIds.size(); ++i) {
            std::cout << apIds[i];
            if (i < apIds.size() - 1) std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    std::cout << "[parseEdgeLabel] ===== EXIT =====" << std::endl;
    
    return apIds;
}

std::vector<uint16_t> TaskAllocationAlgorithms::collectUniqueAPsFromEdges(const std::vector<std::string>& edges) const {
    std::vector<uint16_t> allApIds;
    
    std::cout << "[collectUniqueAPsFromEdges] ===== ENTRY =====" << std::endl;
    std::cout << "[collectUniqueAPsFromEdges] Input: " << edges.size() << " edge(s)" << std::endl;
    
    // Log all input edges
    for (size_t i = 0; i < edges.size(); ++i) {
        std::cout << "[collectUniqueAPsFromEdges]   Edge[" << i << "]: \"" << edges[i] << "\"" << std::endl;
    }
    
    // Process each edge label string
    for (size_t edgeIdx = 0; edgeIdx < edges.size(); ++edgeIdx) {
        const auto& edgeLabel = edges[edgeIdx];
        std::cout << "[collectUniqueAPsFromEdges] Processing Edge[" << edgeIdx << "]: \"" << edgeLabel << "\"" << std::endl;
        
        // Parse the edge label to extract AP IDs
        // Edge labels are typically formatted like: "p0", "p0 | p1", "p0 & p1", etc.
        std::vector<uint16_t> apIds = parseEdgeLabel(edgeLabel);
        std::cout << "[collectUniqueAPsFromEdges]   Parsed " << apIds.size() << " AP(s) from this edge: ";
        if (apIds.empty()) {
            std::cout << "(empty)" << std::endl;
        } else {
            for (size_t i = 0; i < apIds.size(); ++i) {
                std::cout << apIds[i];
                if (i < apIds.size() - 1) std::cout << ", ";
            }
            std::cout << std::endl;
        }
        
        // Merge parsed AP IDs into the result, avoiding duplicates
        // This ensures each unique AP ID appears only once in the final result
        for (uint16_t apId : apIds) {
            // Check if this AP ID is already in the collection
            auto it = std::find(allApIds.begin(), allApIds.end(), apId);
            if (it == allApIds.end()) {
                // Add only if not already present
                std::cout << "[collectUniqueAPsFromEdges]     Adding AP " << apId << " to collection (NEW)" << std::endl;
                allApIds.push_back(apId);
            } else {
                // Already exists, skip
                std::cout << "[collectUniqueAPsFromEdges]     AP " << apId << " already in collection (DUPLICATE, skipped)" << std::endl;
            }
        }
    }
    
    std::cout << "[collectUniqueAPsFromEdges] Final Result:" << std::endl;
    std::cout << "[collectUniqueAPsFromEdges]   Total unique AP(s): " << allApIds.size() << std::endl;
    std::cout << "[collectUniqueAPsFromEdges]   Collection: [";
    if (allApIds.empty()) {
        std::cout << "(empty)";
    } else {
        for (size_t i = 0; i < allApIds.size(); ++i) {
            std::cout << allApIds[i];
            if (i < allApIds.size() - 1) std::cout << ", ";
        }
    }
    std::cout << "]" << std::endl;
    std::cout << "[collectUniqueAPsFromEdges] ===== EXIT =====" << std::endl;
    
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
                
                // Check if this robot contributes any required capabilities
                for (size_t j = 0; j < requiredCapabilities.size() && j < robotCapabilities.size(); ++j) {
                    if (requiredCapabilities[j] && robotCapabilities[j]) {
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
    
    // Rule 1: Remove nodes with OTH progress (terminal nodes)
    for (Tree_Node* node : subtreeNodes) {
        if (node->getProgress() == Tree_Node::TASK_PROGRESS::OTH) {
            nodesToRemove.push_back(node);
        }
    }
    
    // Rule 2: Remove nodes with traversed (automaton state, progress) pairs
    // If we've already visited (state S, progress P) in traversedTree, don't sample it again
    std::vector<Tree_Node*> traversedNodes = traversedTree->getAllNodes();
    
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
                break;
            }
        }
        
        if (foundInTraversed) {
            nodesToRemove.push_back(node);
        }
    }
    
    // Rule 3: For nodes with same automaton state and progress, keep only minimum cost
    // Cost is calculated as sum of all robot execution times
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
            // Calculate cost for each node (sum of all robot times)
            Tree_Node* minCostNode = nodes[0];
            uint32_t minCost = 0;
            for (uint16_t time : minCostNode->getTimes()) {
                minCost += time;
            }
            
            // Find minimum cost node and mark all others for removal
            for (size_t i = 1; i < nodes.size(); ++i) {
                uint32_t nodeCost = 0;
                for (uint16_t time : nodes[i]->getTimes()) {
                    nodeCost += time;
                }
                
                if (nodeCost < minCost) {
                    // Current node is better, mark previous min for removal
                    nodesToRemove.push_back(minCostNode);
                    minCostNode = nodes[i];
                    minCost = nodeCost;
                } else {
                    // Current node is worse or equal, mark for removal
                    nodesToRemove.push_back(nodes[i]);
                }
            }
        }
    }
    
    // Remove all marked nodes from subtree and add to traversedTree
    // Note: PlanningDecisionTree doesn't have removeNode/addNode methods
    // For now, we skip the actual removal. TODO: implement proper node pruning
    // for (Tree_Node* node : nodesToRemove) {
    //     subtree->deleteSubtree(node);
    //     traversedTree->insertNode(node);
    // }
    
    return subtree;
}