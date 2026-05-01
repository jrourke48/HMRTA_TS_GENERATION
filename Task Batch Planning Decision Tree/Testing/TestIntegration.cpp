#include "../TaskAllocationAlgorithms.h"
#include "../MultiRobotSystem/MultiRobotSystem.h"
#include "../MultiRobotSystem/Robot.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/Edge_Node.h"
#include "../../Transition_Systems/GeneralTransitionSystem.h"
#include <iostream>
#include <cassert>

/**
 * Test 1: Create all system components
 */
void testComponentCreation() {
    std::cout << "\n=== Test 1: System Component Creation ===" << std::endl;
    
    // Create GridWorld
    GridWorld grid(10, 10);
    std::cout << "✓ GridWorld created (10x10)" << std::endl;
    
    // Create Environment (needs a GeneralTransitionSystem)
    GeneralTransitionSystem ts;
    for (uint16_t i = 0; i < 10; i++) {
        State s(i, "state_" + std::to_string(i));
        ts.add_State(s);
    }
    std::cout << "✓ GeneralTransitionSystem created with 10 states" << std::endl;
    
    Environment env(&ts, &grid);
    std::cout << "✓ Environment created" << std::endl;
    
    // Create MultiRobotSystem
    MultiRobotSystem mrs;
    
    Robot* r1 = new Robot(1, "Rover_1");
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r1->enableCapability(RobotCapability::SENSOR_GPS);
    mrs.addRobot(r1);
    
    Robot* r2 = new Robot(2, "Rover_2");
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);
    mrs.addRobot(r2);
    
    std::cout << "✓ MultiRobotSystem created with 2 robots" << std::endl;
    
    // Create BuchiAutomaton
    BuchiAutomaton* buchi = new BuchiAutomaton();
    Node* n1 = new Node(0, "init");
    Node* n2 = new Node(1, "accepting");
    n1->addEdge(Edge(1));
    buchi->add_Node(n1);
    buchi->add_Node(n2);
    buchi->setAccepting(1);
    std::cout << "✓ BuchiAutomaton created with 2 nodes" << std::endl;
    
    // Create TaskAllocationAlgorithms
    TaskAllocationAlgorithms* algo = new TaskAllocationAlgorithms(buchi, &env, &mrs);
    std::cout << "✓ TaskAllocationAlgorithms created" << std::endl;
    
    // Verify getters
    assert(algo->getNBA() == buchi);
    assert(algo->getEnvironment() == &env);
    assert(algo->getMultiRobotSystem() == &mrs);
    std::cout << "✓ All component getters verified" << std::endl;
    
    // Cleanup
    delete algo;
    delete buchi;
}

/**
 * Test 2: Build planning tree integration
 */
void testBuildPlanningTree() {
    std::cout << "\n=== Test 2: Build Planning Tree Integration ===" << std::endl;
    
    // Setup system components
    GridWorld grid(5, 5);
    GeneralTransitionSystem ts;
    
    // Add states to transition system
    for (uint16_t i = 0; i < 5; i++) {
        State s(i, "state_" + std::to_string(i));
        ts.add_State(s);
    }
    
    Environment env(&ts, &grid);
    
    // Create multi-robot system
    MultiRobotSystem mrs;
    Robot* rover = new Robot(1, "Rover");
    rover->initializeCapabilities(13);
    rover->enableCapability(RobotCapability::MOVEMENT_GROUND);
    rover->enableCapability(RobotCapability::SENSOR_CAMERA);
    mrs.addRobot(rover);
    
    // Create Büchi automaton
    BuchiAutomaton* buchi = new BuchiAutomaton();
    Node* n1 = new Node(0, "q0");
    Node* n2 = new Node(1, "q1");
    n1->addEdge(Edge(0));  // Self-loop
    n1->addEdge(Edge(1));  // To accepting
    n2->addEdge(Edge(1));  // Self-loop (accepting)
    buchi->add_Node(n1);
    buchi->add_Node(n2);
    buchi->setAccepting(1);
    
    // Create algorithm
    TaskAllocationAlgorithms algo(buchi, &env, &mrs);
    
    // Prepare parameters for buildPlanningTree
    uint32_t rootId = 0;
    Node* automatonState = n1;
    Node* tsState = ts.getState(0);
    std::vector<bool> taskAlloc(1, false);  // 1 robot, not allocated yet
    std::vector<uint16_t> times(1, 0);      // Initial time 0
    int8_t batch = 0;
    Tree_Node::TASK_PROGRESS progress = Tree_Node::TASK_PROGRESS::PRE;
    
    // Build tree
    PlanningDecisionTree* tree = algo.buildPlanningTree(
        rootId, automatonState, tsState, taskAlloc, times, batch, progress
    );
    
    // Verify tree was created
    assert(tree != nullptr);
    std::cout << "✓ Planning tree built successfully" << std::endl;
    
    // Verify tree has root node
    Tree_Node* root = tree->getRoot();
    assert(root != nullptr);
    std::cout << "✓ Tree has valid root node" << std::endl;
    
    // Verify root properties
    assert(root->getAutomatonStateId() == 0);
    assert(root->getTSStateId() == 0);
    std::cout << "✓ Root node has correct automaton and TS states" << std::endl;
    
    // Cleanup
    delete tree;
    delete buchi;
}

/**
 * Test 3: Full integration with table output
 */
void testFullIntegrationWithOutput() {
    std::cout << "\n=== Test 3: Full Integration with Output ===" << std::endl;
    
    // Create system
    GridWorld grid(8, 8);
    GeneralTransitionSystem ts;
    
    for (uint16_t i = 0; i < 8; i++) {
        State s(i, "s" + std::to_string(i));
        ts.add_State(s);
    }
    
    Environment env(&ts, &grid);
    MultiRobotSystem mrs;
    
    // Add 3 diverse robots
    Robot* r1 = new Robot(1, "GroundRover");
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r1->enableCapability(RobotCapability::SENSOR_GPS);
    r1->enableCapability(RobotCapability::SENSOR_CAMERA);
    mrs.addRobot(r1);
    
    Robot* r2 = new Robot(2, "Drone");
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_AERIAL);
    r2->enableCapability(RobotCapability::SENSOR_LIDAR);
    mrs.addRobot(r2);
    
    Robot* r3 = new Robot(3, "Scout");
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::MOVEMENT_GROUND);
    r3->enableCapability(RobotCapability::COMMUNICATION_WIFI);
    mrs.addRobot(r3);
    
    // Print robot table
    std::cout << mrs.to_string() << std::endl;
    
    // Create Büchi automaton
    BuchiAutomaton* buchi = new BuchiAutomaton();
    Node* q0 = new Node(0, "initial");
    Node* q1 = new Node(1, "accepting");
    q0->addEdge(Edge(1, "task_complete"));
    q1->addEdge(Edge(1, "wait"));
    buchi->add_Node(q0);
    buchi->add_Node(q1);
    buchi->setAccepting(1);
    
    // Create algorithm
    TaskAllocationAlgorithms algo(buchi, &env, &mrs);
    
    // Build tree with multiple robots
    std::vector<bool> taskAlloc = {true, false, true};  // Allocate to robots 1 and 3
    std::vector<uint16_t> times = {10, 0, 15};
    
    PlanningDecisionTree* tree = algo.buildPlanningTree(
        0, q0, ts.getState(0), taskAlloc, times, 1, Tree_Node::TASK_PROGRESS::TRA
    );
    
    assert(tree != nullptr);
    assert(tree->getRoot() != nullptr);
    std::cout << "\n✓ Planning tree built for 3-robot system" << std::endl;
    std::cout << "✓ Tree root automaton state: " << tree->getRoot()->getAutomatonStateId() << std::endl;
    std::cout << "✓ Tree root TS state: " << tree->getRoot()->getTSStateId() << std::endl;
    
    // Cleanup
    delete tree;
    delete buchi;
}

/**
 * Main test runner
 */
int main() {
    std::cout << "==========================================================" << std::endl;
    std::cout << "  Integration Tests: Planning Tree with MultiRobot System" << std::endl;
    std::cout << "==========================================================" << std::endl;
    
    try {
        testComponentCreation();
        testBuildPlanningTree();
        testFullIntegrationWithOutput();
        
        std::cout << "\n==========================================================" << std::endl;
        std::cout << "  All integration tests passed! ✓" << std::endl;
        std::cout << "==========================================================" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Test failed with exception: " << e.what() << std::endl;
        return 1;
    }
}
