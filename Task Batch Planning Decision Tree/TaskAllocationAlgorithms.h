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

class TaskAllocationAlgorithms {
    private:
        BuchiAutomaton* nba;
        Environment* environment;
        MultiRobotSystem* multiRobotSystem;
        
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

        // Main algorithm methods
        PlanningDecisionTree* buildPlanningTree(uint32_t rootId, Node* automatonState, Node* tsState,
                                        std::vector<bool> taskAllocation, std::vector<uint16_t> times,
                                        int8_t batch, Tree_Node::TASK_PROGRESS prog);
        
        Tree_Node* determineBestPath(PlanningDecisionTree* tree);
};

#endif // TASK_ALLOCATION_ALGORITHMS_H
