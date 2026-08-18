#include <iostream>
#include <memory>
#include <vector>
#include <string>
#include <cstdlib>
#include <ctime>
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
        Node* node0 = new Node(0, "S0");
        Node* node1 = new Node(1, "S1");
        Node* node2 = new Node(2, "S2");
        Node* node3 = new Node(3, "S3");
        Node* node4 = new Node(4, "S4");
        Node* node5 = new Node(5, "S5");

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
        env->mapTSStateToGrid(0, Point(17, 15), 8, 12);    // State 0 centered at (17,15), 4x6 region
        env->mapTSStateToGrid(1, Point(17, 4), 8, 8);   // State 1 centered at (17,4)
        env->mapTSStateToGrid(2, Point(10, 10), 6, 22);   // State 2 centered at (10,11)
        env->mapTSStateToGrid(3, Point(3, 3), 8, 6);   // State 3 centered at (5,5)
        env->mapTSStateToGrid(4, Point(3, 10), 8, 8);   // State 4 centered at (5,10)
        env->mapTSStateToGrid(5, Point(3, 17), 8, 8);   // State 5 centered at (5,15)
        std::cout << "✓ Mapped 6 states to grid regions" << std::endl;
        
        // Create MultiRobotSystem
        MultiRobotSystem* mrs = new MultiRobotSystem();
        if (!mrs) {
            std::cerr << "Failed to create MultiRobotSystem" << std::endl;
            return 1;
        }
        
        // Position all robots in room 0 (centered at Point(18, 14))
        Robot* r0 = new Robot(0, "R0", Point(18, 14));
        r0->initializeCapabilities(13);
        r0->enableCapability(RobotCapability::SENSOR_GPS); //C
        mrs->addRobot(r0);
        
        Robot* r1 = new Robot(1, "R1", Point(17, 14));
        r1->initializeCapabilities(13);
        r1->enableCapability(RobotCapability::MOVEMENT_GROUND); //A
        mrs->addRobot(r1);
        
        Robot* r2 = new Robot(2, "R2", Point(19, 14));
        r2->initializeCapabilities(13);
        r2->enableCapability(RobotCapability::SENSOR_CAMERA); // B
        mrs->addRobot(r2);
        Robot* r3 = new Robot(3, "R3", Point(18, 13));
        r3->initializeCapabilities(13);
        r3->enableCapability(RobotCapability::SENSOR_GPS); // C
        mrs->addRobot(r3);
        
        Robot* r4 = new Robot(4, "R4", Point(18, 15));
        r4->initializeCapabilities(13);
        r4->enableCapability(RobotCapability::MOVEMENT_GROUND);
        mrs->addRobot(r4);
        
        Robot* r5 = new Robot(5, "R5", Point(17, 15));
        r5->initializeCapabilities(13);
        r5->enableCapability(RobotCapability::SENSOR_CAMERA);
        mrs->addRobot(r5);
    
    std::cout << "✓ MultiRobotSystem created with 6 robots" << std::endl;
        
        // Add random obstacles for interesting pathfinding
        std::cout << "Adding random obstacles..." << std::endl;
        srand(time(0));
        int numObstacles = 15;
        for (int i = 0; i < numObstacles; i++) {
            int x = rand() % 20;
            int y = rand() % 20;
            env->addObstacle(Point(x, y));
        }
        std::cout << "✓ Added " << numObstacles << " random obstacles" << std::endl;
        std::string ltl_str = "G(F\"p1\") && G(F\"p2\") && G(F\"p3\")";
    
        std::vector<BatchAtomicProposition> batchAPs;
        batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
        
        LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
        BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
        buchi->visualize("output/buchi_automaton_tree_test");
        std::cout << "✓ BuchiAutomaton created" << std::endl;
        
        TaskAllocationAlgorithms* taa1 = new TaskAllocationAlgorithms(buchi, env, mrs);
        taa1->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
        PlanningDecisionTree* planningTree = taa1->getPlanningTree();
        std::vector<Tree_Node*> optimalPath = planningTree->getPathtoFrontierNode(planningTree->getOptimalFrontierNode(false));
        
        // Print metrics after algorithm completes
        std::cout << "\n=== Algorithm Metrics ===" << std::endl;
        taa1->getMetrics().printSummary();
        std::cout << std::endl;
        
        // Visualize planning tree and optimal path
        taa1->visualizeTree("output/planning_tree");
        taa1->visualizeOptimalPath("output/optimal_path");
        std::cout << "✓ Visualizations saved" << std::endl;
        
        std::cout << "Computing D* paths for robots..." << std::endl;
        RobotPathMap robotPaths = compute_dstar_paths(*env, *mrs, optimalPath);
        std::cout << "Displaying visualization (close window to exit)..." << std::endl;
        std::cout << "  - Green/Blue/Orange regions: TS states" << std::endl;
        std::cout << "  - Dark gray: obstacles" << std::endl;
        std::cout << "  - Colored circles: robots (0-5)" << std::endl;
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
