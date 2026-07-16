#ifndef TASK_ALLOCATION_ALGORITHMS_H
#define TASK_ALLOCATION_ALGORITHMS_H

#include "Tree/PlanningDecisionTree.h"
#include "Environment/Environment.h"
#include "../Automatons/BuchiAutomaton.h"
#include "../Transition_Systems/GeneralTransitionSystem.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include <vector>
#include <queue>
#include <memory>
#include <string>
#include <spot/tl/formula.hh>

class TaskAllocationAlgorithms {
    //TODO implement the prog function and the the unrelated task search
    // implement the preliminary tests of the algorithms and the overall tree search to see if we can get a working
    // version of the algorithms before optimizing and adding the batch value tracking and batch-based search routing
    private:
        BuchiAutomaton* nba;
        Environment* environment;
        PlanningDecisionTree* planningTree;
        PlanningDecisionTree* traversedTree; // To keep track of the tree being traversed during search
        MultiRobotSystem* multiRobotSystem;
        std::vector<Tree_Node*> visitedNodes; // To keep track of visited nodes during search
        std::vector<std::pair<uint16_t, uint16_t>> visitedStateAPPairs; // Track (automatonStateId, apId) pairs
        std::vector<uint8_t> treebatchvals; // To keep track of batch values in the tree (if needed for cost calculations)
        std::queue<Tree_Node*> untraversedPlanningQueue; // Queue of nodes in planning tree not yet traversed
    
        
    public:
        // Constructor
        TaskAllocationAlgorithms(BuchiAutomaton* nbaPtr, Environment* envPtr, MultiRobotSystem* robotSysPtr);
        
        // Destructor
        ~TaskAllocationAlgorithms();
        
        // System component setters
        void setNBA(BuchiAutomaton* nbaPtr);
        void setEnvironment(Environment* envPtr);
        void setMultiRobotSystem(MultiRobotSystem* robotSysPtr);
        
        // System component getters
        BuchiAutomaton* getNBA() const;
        Environment* getEnvironment() const;
        MultiRobotSystem* getMultiRobotSystem() const;

        // Planning tree methods
        void setPlanningTree(PlanningDecisionTree* tree);
        PlanningDecisionTree* getPlanningTree() const;
        
        // Traversed tree methods
        void setTraversedTree(PlanningDecisionTree* tree);
        PlanningDecisionTree* getTraversedTree() const;

        // Visited nodes methods
        void addVisitedNode(Tree_Node* node);
        bool isNodeVisited(Tree_Node* node) const;
        std::vector<Tree_Node*>& getVisitedNodes();
        Tree_Node* getLastVisitedNode() const{
            if (visitedNodes.empty()) return nullptr;
            return visitedNodes.back();
        };
        Tree_Node* getFirstVisitedNode() const{
            if (visitedNodes.empty()) return nullptr;
            return visitedNodes.front();
        };
        void clearVisitedNodes();

        // Visited (automatonState, AP) pairs methods - Solution 1
        void addVisitedStateAPPair(uint16_t automatonStateId, uint16_t apId);
        bool isStateAPPairVisited(uint16_t automatonStateId, uint16_t apId) const;
        std::vector<std::pair<uint16_t, uint16_t>>& getVisitedStateAPPairs();
        void clearVisitedStateAPPairs();  // Solution 2: Clear when progress changes

        // Legacy methods (kept for compatibility, will use state-AP pairs internally)
        void addVisitedAutomatonState(uint16_t state);
        bool isAutomatonStateVisited(uint16_t state) const;
        std::vector<uint16_t>& getVisitedAutomatonStates();
        void clearVisitedAutomatonStates();

        // Batch values in tree methods
        void addBatchValue(uint8_t batchValue);
        bool isBatchValueInTree(int8_t batchValue);
        std::vector<uint8_t>& getBatchValues();
        void clearBatchValues();

        // Untraversed planning queue methods
        void addUntraversedPlanningNode(Tree_Node* node);
        Tree_Node* getNextUntraversedNode();
        void clearUntraversedQueue();

        // Parse edge labels to extract AP IDs
        // Removes !, &, | symbols and returns vector of AP IDs that are TRUE (not negated)
        std::vector<uint16_t> parseEdgeLabel(const std::string& label) const;
        
        // Collect and merge AP IDs from multiple edges, removing duplicates
        std::vector<uint16_t> collectUniqueAPsFromEdges(const std::vector<std::string>& edges) const;
        /**
         * Algorithm 1: Intensive Inter-Task Relationship Tree Search
         * Builds a planning tree considering inter-task relationships
         */
        PlanningDecisionTree* intensiveInterTaskRelationshipTreeSearch(
            BuchiAutomaton* nba,
            Environment* env,
            MultiRobotSystem* multiRobotSystem);
        
        /**
         * Algorithm 2: Unrelated-Task Search (US)
         * Searches for unrelated tasks that can be executed
         * Mutates newNode with search results
         */
        void unrelatedTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode,
            uint16_t apId);
        
        /**
         * Algorithm 3: Compatible-Task Search (CS)
         * Searches for compatible tasks considering constraints
         * Mutates newNode with search results
         */
        void compatibleTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode);
        
        /**
         * Algorithm 4: Exclusive-task Search (ES)
         * Searches for exclusive tasks that have conflicting batches
         * Mutates newNode with search results
         */
        void exclusiveTaskSearch(
            Tree_Node* newNode,
            Node* TSState,
            Tree_Node* currentNode);
        PlanningDecisionTree* pruneSubtree(PlanningDecisionTree* subtree);
        
        /**
         * getTaskAllocation
         * Greedily selects minimum set of robots that satisfy all required capabilities
         * Returns (task allocation vector, max time of selected robots)
         */
        std::pair<std::vector<bool>, uint16_t> getTaskAllocation(
            const std::vector<Robot*>& robots,
            const std::vector<bool>& requiredCapabilities,
            const std::vector<std::pair<uint16_t, uint16_t>>& sortedTimes);

        /**
         * visualizeTree
         * Visualizes the planning decision tree using Graphviz 
         */
        void visualizeTree(const std::string& filename) const;
        /** 
         * visualize Optimal Path
         */
        void visualizeOptimalPath(const std::string& filename) const;

private:
};
#endif // TASK_ALLOCATION_ALGORITHMS_H
