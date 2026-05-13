#ifndef TASK_ALLOCATION_ALGORITHMS_H
#define TASK_ALLOCATION_ALGORITHMS_H

#include "Tree/PlanningDecisionTree.h"
#include "Environment/Environment.h"
#include "../Automatons/BuchiAutomaton.h"
#include "../Transition_Systems/GeneralTransitionSystem.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include <vector>
#include <memory>
#include <string>
#include <spot/tl/formula.hh>

class TaskAllocationAlgorithms {
    private:
        BuchiAutomaton* nba;
        Environment* environment;
        MultiRobotSystem* multiRobotSystem;
        std::vector<Tree_Node*> visitedNodes; // To keep track of visited nodes during search
        std::vector<uint32_t> visitedAutomatonStates; // To keep track of visited automaton states during search
        
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

        // Visited nodes methods
        void addVisitedNode(Tree_Node* node);
        bool isNodeVisited(Tree_Node* node) const;
        std::vector<Tree_Node*>& getVisitedNodes();
        void clearVisitedNodes();

        // Visited automaton states methods
        void addVisitedAutomatonState(uint32_t state);
        bool isAutomatonStateVisited(uint32_t state) const;
        std::vector<uint32_t>& getVisitedAutomatonStates();
        void clearVisitedAutomatonStates();

        // Main algorithm methods
        PlanningDecisionTree* buildPlanningTree(uint32_t rootId, Node* automatonState, Node* tsState,
                                        std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                                        int8_t batch, Tree_Node::TASK_PROGRESS prog);

        // Helper algorithm methods based on pseudocode
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
         * Input: q (automaton state), ds (robot state), φ' (LTL formula)
         */
        Tree_Node* unrelatedTaskSearch(
            uint32_t q,
            uint32_t ds,
            spot::formula ltlFormula);
        
        /**
         * Algorithm 3: Compatible-Task Search (CS)
         * Searches for compatible tasks considering constraints
         * Input: q (automaton state), ds (robot state), T^sub_B (sub-tasks), T_B (tasks)
         */
        Tree_Node* compatibleTaskSearch(
            uint32_t q,
            uint32_t ds,
            const std::vector<uint16_t>& subTasks,
            const std::vector<uint16_t>& tasks);
        
        /**
         * Algorithm 4: Exclusive-task Search (ES)
         * Searches for exclusive tasks that have conflicting batches
         * Input: d_s (robot state), T^sub_B (sub-tasks), T_B (tasks)
         */
        Tree_Node* exclusiveTaskSearch(
            uint32_t ds,
            const std::vector<uint16_t>& subTasks,
            const std::vector<uint16_t>& tasks);

private:
};
#endif // TASK_ALLOCATION_ALGORITHMS_H
