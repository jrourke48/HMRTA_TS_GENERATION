#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
#include <map>
#include "../include/TaskAllocationAlgorithms.h"
#include "../include/Environment/gridvis.h"
#include "../include/Tree/PlanningDecisionTree.h"
#include "../include/Tree/Tree_Node.h"
#include "../include/Environment/Environment.h"
#include "../include/MultiRobotSystem/MultiRobotSystem.h"
#include "../include/LTLFormula/LTLFormula.h"
#include "../include/TestRunManager.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/ProductAutomaton.h"

using namespace std;

// Test: Number of Robots (3-20 robots)
// Fixed: 6 regions, 1 Buchi automaton

// Forward declarations
void createTestEnvironmentWithRobots(int numRobots, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestBuchiAutomaton();

// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   NUMBER OF ROBOTS SCALING TEST SUITE" << endl;
    cout << "   Variable: Number of Robots (3-20)" << endl;
    cout << "   Fixed: 6 regions, 1 Buchi automaton" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Initialize TestRunManager for NUMBER_ROBOTS category
    TestRunManager manager(TestRunManager::TestCategory::NUMBER_ROBOTS, ".");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    // Robot counts to test
    vector<int> robotCounts = {3, 6, 8, 10, 12, 15, 18, 20};
    int testNum = 1;
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   RUNNING ROBOT SCALING TESTS" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    // For each robot count
    for (int robotCount : robotCounts) {
        cout << "\n  Test " << testNum << " (" << robotCount << " robots)... ";
        cout.flush();
        
        try {
            // Create test environment with variable number of robots
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            
            createTestEnvironmentWithRobots(robotCount, ts, grid, env, mrs);
            
            // Create the Buchi automaton
            BuchiAutomaton* buchi = createTestBuchiAutomaton();
            
            if (!buchi) {
                cout << "ERROR: Failed to create automaton" << endl;
                delete mrs;
                delete env;
                delete grid;
                delete ts;
                testNum++;
                continue;
            }
            
            // Create TaskAllocationAlgorithms
            TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
            
            // Measure memory and time
            double memBefore = getMemoryUsageMB();
            // Build the planning decision tree
            allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
            double memAfter = getMemoryUsageMB();
            
            cout << "✓ Complete\n";
            allocAlg->getMetrics().printSummary();
            
            // Store run in TestRunManager
            map<string, string> parameters;
            parameters["num_robots"] = to_string(robotCount);
            
            manager.storeRun(
                allocAlg->getMetrics(),
                parameters,
                to_string(robotCount),  // independent variable: group by robot count
                1  // trial number
            );
            
            delete allocAlg;
            delete buchi;
            delete mrs;
            delete env;
            delete grid;
            delete ts;
            
        } catch (const exception& e) {
            cout << "ERROR: " << e.what() << endl;
        }
        
        testNum++;
    }
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   TESTING COMPLETE" << endl;
    cout << string(80, '=') << "\n" << endl;

    cout << "\n✓ All tests completed!" << endl;
    cout << "   - " << robotCounts.size() << " robot configurations tested" << endl;
    
    // Export results from TestRunManager
    cout << "\n✓ Exporting results from TestRunManager..." << endl;
    manager.exportByConfiguration();
    manager.exportStatisticsToCSV("data/statistics.csv");
    manager.exportSummaryReport("data/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ Results stored in data/" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    return 0;
}

/**
 * Create a Buchi automaton for testing
 * Formula: G(F("p0")) & G(F("p1")) & G(F("p2")) & (!p0 U p1)
 */
BuchiAutomaton* createTestBuchiAutomaton() {
    string ltl_str = "G(F(\"p0\")) & G(F(\"p1\")) & G(F(\"p2\")) & (!\"p0\" U \"p1\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Create test environment with variable number of robots
 * Fixed: 6 regions, 210x210 grid
 */
void createTestEnvironmentWithRobots(int numRobots, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    // Allocate GridWorld
    grid = new GridWorld(210, 210);
    
    // Allocate Transition System
    ts = new TS();
    
    // Add 6 states 
    Node* node0 = new Node(0, "R0");
    Node* node1 = new Node(1, "R1");
    Node* node2 = new Node(2, "R2");
    Node* node3 = new Node(3, "R3");
    Node* node4 = new Node(4, "R4");
    Node* node5 = new Node(5, "R5");

    // with edges: 0-2 1-2 2-3 2-4 2-5
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
    
    // Allocate Environment
    env = new Environment(ts, grid);
    
    // Map states to grid regions
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);
    
    // Create MultiRobotSystem with specified number of robots
    mrs = new MultiRobotSystem();
    
    // Calculate grid dimensions for robots
    int cols = (int)ceil(sqrt(numRobots));
    int rows = (int)ceil((double)numRobots / cols);
    
    // Position robots in a grid starting from room 0 center
    for (int i = 1; i <= numRobots; i++) {
        int col = (i - 1) % cols;
        int row = (i - 1) / cols;
        int x = 140 + col;
        int y = 120 + row;
        
        // Rotate capabilities: GPS, MOVEMENT_GROUND, SENSOR_CAMERA
        RobotCapability cap = (i % 3 == 1) ? RobotCapability::SENSOR_GPS : 
                              (i % 3 == 2) ? RobotCapability::MOVEMENT_GROUND : 
                              RobotCapability::SENSOR_CAMERA;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        r->enableCapability(cap);
        mrs->addRobot(r);
    }
}

