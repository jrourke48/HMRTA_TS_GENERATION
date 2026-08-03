#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include "../include/Environment/Environment.h"
#include "../include/Environment/GridWorld.h"
#include "../include/Environment/gridvis.h"
#include "../include/MultiRobotSystem/MultiRobotSystem.h"
#include "../include/MultiRobotSystem/Robot.h"
#include "../include/Tree/PlanningDecisionTree.h"
#include "../include/LTLFormula/LTLFormula.h"
#include "../include/LTLFormula/BatchAtomicProposition.h"
#include "../include/TaskAllocationAlgorithms.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/TS.h"

/**
 * Visualization App - Display task plan execution with robot paths
 * 
 * This app creates a simple environment, robots, and task tree,
 * then visualizes the multi-robot paths computed by D* Lite
 */

int main() {
    try {
        std::cout << "=== Multi-Robot Task Plan Visualization ===" << std::endl;
        
    // Allocate GridWorld
        GridWorld* grid = new GridWorld(21, 21);
        std::cout << "✓ GridWorld created (20x20)" << std::endl;
        
        // Allocate Transition System
        TS* ts = new TS();
        
        // Add 6 states 
        Node* node0 = new Node(0, "R0");
        Node* node1 = new Node(1, "R1");
        Node* node2 = new Node(2, "R2");
        Node* node3 = new Node(3, "R3");
        Node* node4 = new Node(4, "R4");
        Node* node5 = new Node(5, "R5");

        //with edges: 0-2 1-2 2-3 2-4 2-5
        node0->addEdge(Edge(2));
        node2->addEdge(Edge(0));
        node1->addEdge(Edge(2));
        node2->addEdge(Edge(1));
        node3->addEdge(Edge(2));
        node2->addEdge(Edge(3));
        node4->addEdge(Edge(2));
        node2->addEdge(Edge(4));
        node5->addEdge(Edge(2));
        node2->addEdge(Edge(5));
        
        // Add nodes to TS
        ts->add_Node(node0);
        ts->add_Node(node1);
        ts->add_Node(node2);
        ts->add_Node(node3);
        ts->add_Node(node4);
        ts->add_Node(node5);
        ts->setInitial(0);
        
        std::cout << "✓ Transition System created" << std::endl;
        std::cout << "  - States: " << ts->getNumStates() << std::endl;
        std::cout << "  - Initial state: 0" << std::endl;
        
        // Allocate Environment
        Environment* env = new Environment(ts, grid);
        std::cout << "✓ Environment created" << std::endl;
        
        // Map states to grid regions
        env->mapTSStateToGrid(0, Point(18, 14), 5, 14);    // State 0 centered at (17,15), 4x6 region
        env->mapTSStateToGrid(1, Point(18, 4), 5, 7);   // State 1 centered at (17,4)
        env->mapTSStateToGrid(2, Point(10, 10), 6, 20);   // State 2 centered at (10,11)
        env->mapTSStateToGrid(3, Point(5, 3), 5, 18);   // State 3 centered at (5,5)
        env->mapTSStateToGrid(4, Point(5, 10), 5, 11);   // State 4 centered at (5,10)
        env->mapTSStateToGrid(5, Point(5, 15), 5, 4);   // State 5 centered at (5,15)
        std::cout << "✓ Mapped 6 states to grid regions" << std::endl;
        
        // Create MultiRobotSystem
        MultiRobotSystem* mrs = new MultiRobotSystem();
        if (!mrs) {
            std::cerr << "Failed to create MultiRobotSystem" << std::endl;
            return 1;
        }
        
        // Position all robots in room 0 (centered at Point(18, 14))
        Robot* r1 = new Robot(1, "Rover_1", Point(18, 14));
        r1->initializeCapabilities(13);
        r1->enableCapability(RobotCapability::SENSOR_GPS); //C
        mrs->addRobot(r1);
        
        Robot* r2 = new Robot(2, "Rover_2", Point(17, 14));
        r2->initializeCapabilities(13);
        r2->enableCapability(RobotCapability::MOVEMENT_GROUND); //A
        mrs->addRobot(r2);
        
        Robot* r3 = new Robot(3, "Rover_3", Point(19, 14));
        r3->initializeCapabilities(13);
        r3->enableCapability(RobotCapability::SENSOR_CAMERA); // B
        mrs->addRobot(r3);
        Robot* r4 = new Robot(4, "Rover_4", Point(18, 13));
        r4->initializeCapabilities(13);
        r4->enableCapability(RobotCapability::SENSOR_GPS); // C
        mrs->addRobot(r4);
        
        Robot* r5 = new Robot(5, "Rover_5", Point(18, 15));
        r5->initializeCapabilities(13);
        r5->enableCapability(RobotCapability::MOVEMENT_GROUND);
        mrs->addRobot(r5);
        
        Robot* r6 = new Robot(6, "Rover_6", Point(17, 15));
        r6->initializeCapabilities(13);
        r6->enableCapability(RobotCapability::SENSOR_CAMERA);
        mrs->addRobot(r6);
    
    std::cout << "✓ MultiRobotSystem created with 6 robots" << std::endl;
        
        // Add some obstacles for interesting pathfinding
        std::cout << "Adding obstacles..." << std::endl;
        for (int x = 10; x < 15; ++x) {
            env->addObstacle(Point(x, 15));  // Horizontal wall
        }
        for (int y = 18; y < 20; ++y) {
            env->addObstacle(Point(12, y));  // Vertical wall
        }
        std::string ltl_str = "(F\"p1\" && F\"p2\")";
    
        std::vector<BatchAtomicProposition> batchAPs;
        batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
        
        LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
        BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
        buchi->visualize("output/buchi_automaton_tree_test");
        std::cout << "✓ BuchiAutomaton created" << std::endl;
        
        TaskAllocationAlgorithms* taa1 = new TaskAllocationAlgorithms(buchi, env, mrs);
        taa1->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
        PlanningDecisionTree* planningTree = taa1->getPlanningTree();
        std::vector<Tree_Node*> optimalPath = planningTree->getPathtoFrontierNode(planningTree->getOptimalFrontierNode(true));
        std::cout << "Computing D* paths for robots..." << std::endl;
        RobotPathMap robotPaths = compute_dstar_paths(*env, *mrs, optimalPath);
        
        std::cout << "Displaying visualization (close window to exit)..." << std::endl;
        std::cout << "  - Green/Blue/Orange regions: TS states" << std::endl;
        std::cout << "  - Dark gray: obstacles" << std::endl;
        std::cout << "  - Colored circles: robots (R0, R1, R2)" << std::endl;
        std::cout << "  - Colored squares: robot paths (D* computed)" << std::endl;
        
        visualize_environment(
            *env,
            *mrs,
            optimalPath,
            robotPaths,
            "Multi-Robot Task Plan Visualization - D* Lite Pathfinding"
        );
        
        std::cout << "Visualization closed." << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
