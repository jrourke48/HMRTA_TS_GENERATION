#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
#include "../include/TaskAllocationAlgorithms.h"
#include "../include/Environment/gridvis.h"
#include "../include/Tree/PlanningDecisionTree.h"
#include "../include/Tree/Tree_Node.h"
#include "../include/Environment/Environment.h"
#include "../include/MultiRobotSystem/MultiRobotSystem.h"
#include "../include/LTLFormula/LTLFormula.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/ProductAutomaton.h"

using namespace std;

// Test: Number of Automaton States (5-150 states)
// Fixed: 6 robots, 6 regions, 4 different Buchi automata

// Forward declarations
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestInfiniteBuchiAutomaton1(int estimatedStates);
BuchiAutomaton* createTestInfiniteBuchiAutomaton2(int estimatedStates);
BuchiAutomaton* createTestInfiniteBuchiAutomaton3(int estimatedStates);
BuchiAutomaton* createTestInfiniteBuchiAutomaton4(int estimatedStates);
BuchiAutomaton* createTestInfiniteBuchiAutomaton5(int estimatedStates);
BuchiAutomaton* createTestInfiniteBuchiAutomaton6(int estimatedStates);

// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   AUTOMATON SCALING TEST SUITE" << endl;
    cout << "   4 Different Büchi Automata" << endl;
    cout << "   2 Environments: 3-robot and 6-robot" << endl;
    cout << "   Total Tests: 8" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Create array of automaton factory functions
    vector<BuchiAutomaton*(*)(int)> automatonFactories = {
        createTestInfiniteBuchiAutomaton1,
        createTestInfiniteBuchiAutomaton2,
        createTestInfiniteBuchiAutomaton3,
        createTestInfiniteBuchiAutomaton4,
        createTestInfiniteBuchiAutomaton5,
        createTestInfiniteBuchiAutomaton6
    };
    
    vector<int> robotCounts = {3, 6};
    int testNum = 1;
    
    // Run tests for each environment
    for (int robotCount : robotCounts) {
        cout << "\n" << string(80, '=') << endl;
        cout << "   TESTING WITH " << robotCount << "-ROBOT ENVIRONMENT" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        // Create test environment
        TS* ts = nullptr;
        GridWorld* grid = nullptr;
        Environment* env = nullptr;
        MultiRobotSystem* mrs = nullptr;
        
        if (robotCount == 3) {
            createTestEnvironment3(ts, grid, env, mrs);
        } else {
            createTestEnvironment6(ts, grid, env, mrs);
        }
        
        cout << "\n" << string(80, '-') << endl;
        cout << "   RUNNING TESTS" << endl;
        cout << string(80, '-') << "\n" << endl;
        
        // For each of the 4 automata
        for (int automatonId = 1; automatonId <= 6; ++automatonId) {
            cout << "\n  Test " << testNum << " (Automaton " << automatonId << ")... ";
            cout.flush();
            
            try {
                // Create the Buchi automaton
                BuchiAutomaton* buchi = automatonFactories[automatonId - 1](0);
                
                if (!buchi) {
                    cout << "ERROR: Failed to create automaton" << endl;
                    testNum++;
                    continue;
                }
                
                // Create TaskAllocationAlgorithms
                TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
                
                // Measure memory and time
                double memBefore = getMemoryUsageMB();
                //build the planning decision tree
                allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
                if (robotCount == 6 && (automatonId == 5 || automatonId == 6)) {
                    allocAlg->visualizeTree("output/automaton_test_" + to_string(robotCount) + "robots_" + to_string(automatonId) + "_tree.dot");
                    allocAlg->visualizeOptimalPath("output/automaton_test_" + to_string(robotCount) + "robots_" + to_string(automatonId) + "_path.dot");
                }
                
                cout << "✓ Complete\n";
                allocAlg->getMetrics().printSummary();
                
                // Export metrics to CSV in output folder
                string csvFilename = "output/automaton_test_" + to_string(robotCount) + "robots_" + 
                                    to_string(automatonId) + ".csv";
                allocAlg->getMetrics().exportToCSV(csvFilename);
                
                delete allocAlg;
                delete buchi;
                
            } catch (const exception& e) {
                cout << "ERROR: " << e.what() << endl;
            }
            
            testNum++;
        }
        
        cout << "\n" << string(80, '=') << endl;
        cout << "   ENVIRONMENT TESTING COMPLETE" << endl;
        cout << string(80, '=') << "\n" << endl;
        
        // Cleanup
        delete mrs;
        delete env;
        delete grid;
        delete ts;
    }

    cout << "\n" << string(80, '=') << "\n" << endl;
    cout << "✓ All 10 tests completed!" << endl;
    cout << "   - 5 automata tested with 3-robot environment" << endl;
    cout << "   - 5 automata tested with 6-robot environment" << endl;
    cout << "\n✓ CSV Results stored in organizedtests/output/ folder:" << endl;
    cout << "   - output/automaton_test_3robots_1.csv through automaton_test_3robots_4.csv" << endl;
    cout << "   - output/automaton_test_6robots_1.csv through automaton_test_6robots_4.csv" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    return 0;
}

// ============================================================================
//Create test environment with TS and GridWorld with 6 robots and 6 regions
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(21, 21);
    cout << "✓ GridWorld created (21x21)" << endl;
    
    // Allocate Transition System
    ts = new TS();
    
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
    
    cout << "✓ Transition System created" << endl;
    cout << "  - States: " << ts->getNumStates() << endl;
    cout << "  - Initial state: 0" << endl;
    
    // Allocate Environment
    env = new Environment(ts, grid);
    cout << "✓ Environment created" << endl;
    
    // Map states to grid regions
    env->mapTSStateToGrid(0, Point(18, 14), 5, 14);    // State 0 centered at (17,15), 4x6 region
    env->mapTSStateToGrid(1, Point(18, 4), 5, 7);   // State 1 centered at (17,4)
    env->mapTSStateToGrid(2, Point(10, 10), 6, 20);   // State 2 centered at (10,11)
    env->mapTSStateToGrid(3, Point(5, 3), 5, 18);   // State 3 centered at (5,5)
    env->mapTSStateToGrid(4, Point(5, 10), 5, 11);   // State 4 centered at (5,10)
    env->mapTSStateToGrid(5, Point(5, 15), 5, 4);   // State 5 centered at (5,15)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem
    mrs = new MultiRobotSystem();
    
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
    
    cout << "✓ MultiRobotSystem created with 6 robots" << endl;
}

// ============================================================================
//Create test environment with TS and GridWorld with 3 robots and 6 regions
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(21, 21);
    cout << "✓ GridWorld created (21x21)" << endl;
    
    // Allocate Transition System
    ts = new TS();
    
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
    
    cout << "✓ Transition System created" << endl;
    cout << "  - States: " << ts->getNumStates() << endl;
    cout << "  - Initial state: 0" << endl;
    
    // Allocate Environment
    env = new Environment(ts, grid);
    cout << "✓ Environment created" << endl;
    
    // Map states to grid regions
    env->mapTSStateToGrid(0, Point(18, 14), 5, 14);    // State 0 centered at (17,15), 4x6 region
    env->mapTSStateToGrid(1, Point(18, 4), 5, 7);   // State 1 centered at (17,4)
    env->mapTSStateToGrid(2, Point(10, 10), 6, 20);   // State 2 centered at (10,11)
    env->mapTSStateToGrid(3, Point(5, 3), 5, 18);   // State 3 centered at (5,5)
    env->mapTSStateToGrid(4, Point(5, 10), 5, 11);   // State 4 centered at (5,10)
    env->mapTSStateToGrid(5, Point(5, 15), 5, 4);   // State 5 centered at (5,15)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem
    mrs = new MultiRobotSystem();
    
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
    
    
    cout << "✓ MultiRobotSystem created with 3 robots" << endl;
}


// ============================================================================
// INFINITE AUTOMATA TESTS (4)
// ============================================================================


/**
 * ADVANCED INFINITE Test 1:
 * G(F("p0")) & G(F("p2"))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1(int estimatedStates) {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p2\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * ADVANCED INFINITE Test 4: Complex nested temporal operators with sequencing
 * G(F(p0 & X(p1 & X p2))) & G(F(p3 & (F p4)))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4(int estimatedStates) {
    string ltl_str = "G(F(\"p0\" & X(\"p1\" & X\"p2\"))) & G(F(\"p3\" & (F\"p4\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * ADVANCED INFINITE Test 3: Complex temporal formula
 * G(F("p0")) & G(F("p1")) & X("p2") & G(F("p3" U "p4"))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3(int estimatedStates) {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p1\" & X(\"p2\"))) & G(F(\"p3\" U \"p4\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * ADVANCED INFINITE Test 5: Complex nesting with conditional sequencing
 * G((F("p0" & X("p1" U "p2"))) & (G!"p3" | F("p4" & X"p0"))) 
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton5(int estimatedStates) {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}
BuchiAutomaton* createTestInfiniteBuchiAutomaton6(int estimatedStates) {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p101\" & X(\"p202\") & G(F(\"p31\") & X(\"p41\")))))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(101, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p101
    batchAPs.push_back(BatchAtomicProposition(202, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p202
    batchAPs.push_back(BatchAtomicProposition(31, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p31
    batchAPs.push_back(BatchAtomicProposition(41, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p41

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_5.dot");
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton2(int estimatedStates) {
    string ltl_str = "G(F(\"p0\")) & G(F(\"p1\")) & G(F(\"p2\")) & G(F(\"p3\")) & G(F(\"p4\")) & G(F(\"p5\"))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_2.dot");
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton7(int estimatedStates) {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton8(int estimatedStates) {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton9(int estimatedStates) {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\") & G(F(\"p6\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}