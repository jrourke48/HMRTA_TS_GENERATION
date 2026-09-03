#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>
#include <sys/resource.h>
#include <map>
#include <cmath>
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

// Test: Average Robot Capabilities (1-5 capabilities per robot)
// Fixed: 6 robots, 6 regions, 1 Buchi automaton

struct CapabilityTest {
    int testNum;
    double avgCapabilities;
    int totalCapabilities;
};

// Forward declarations
void createTestEnvironmentWithCapabilities(int totalCaps, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestBuchiAutomaton();

// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   AVERAGE ROBOT CAPABILITIES TEST SUITE" << endl;
    cout << "   Variable: Average capabilities per robot (1.0-5.0)" << endl;
    cout << "   Fixed: 6 robots, 6 regions, 1 Buchi automaton" << endl;
    cout << string(80, '=') << "\n" << endl;

    TestRunManager manager(TestRunManager::TestCategory::AVG_CAPABILITIES, ".");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    vector<CapabilityTest> tests = {
        {1, 1.0, 6},
        {2, 1.2, 7},
        {3, 1.5, 9},
        {4, 1.8, 11},
        {5, 2.0, 12},
        {6, 2.5, 15},
        {7, 3.0, 18},
        {8, 3.5, 21},
        {9, 4.0, 24},
        {10, 5.0, 30}
    };
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   RUNNING CAPABILITY CONFIGURATION TESTS" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    for (const auto& test : tests) {
        cout << "\n  Test " << test.testNum << " (" << fixed << setprecision(1) << test.avgCapabilities 
             << " caps/robot, " << test.totalCapabilities << " total)... ";
        cout.flush();
        
        try {
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            
            createTestEnvironmentWithCapabilities(test.totalCapabilities, ts, grid, env, mrs);
            
            BuchiAutomaton* buchi = createTestBuchiAutomaton();
            
            if (!buchi) {
                cout << "ERROR: Failed to create automaton" << endl;
                delete mrs;
                delete env;
                delete grid;
                delete ts;
                continue;
            }
            
            TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
            allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
            
            cout << "✓ Complete\n";
            allocAlg->getMetrics().printSummary();
            
            map<string, string> parameters;
            parameters["avg_capabilities"] = to_string(test.avgCapabilities);
            parameters["total_capabilities"] = to_string(test.totalCapabilities);
            
            manager.storeRun(
                allocAlg->getMetrics(),
                parameters,
                to_string(test.totalCapabilities),
                1
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
    }
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   TESTING COMPLETE" << endl;
    cout << string(80, '=') << "\n" << endl;

    cout << "\n✓ All tests completed!" << endl;
    cout << "   - " << tests.size() << " capability configurations tested" << endl;
    
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
 * Create test environment with variable capability distribution
 * Fixed: 6 robots, 6 regions, 210x210 grid
 * Variable: Total capabilities distributed among robots
 */
void createTestEnvironmentWithCapabilities(int totalCaps, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(210, 210);
    ts = new TS();
    
    Node* node0 = new Node(0, "R0");
    Node* node1 = new Node(1, "R1");
    Node* node2 = new Node(2, "R2");
    Node* node3 = new Node(3, "R3");
    Node* node4 = new Node(4, "R4");
    Node* node5 = new Node(5, "R5");

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
    
    ts->add_Node(node0);
    ts->add_Node(node1);
    ts->add_Node(node2);
    ts->add_Node(node3);
    ts->add_Node(node4);
    ts->add_Node(node5);
    ts->setInitial(0);
    
    env = new Environment(ts, grid);
    
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);
    
    mrs = new MultiRobotSystem();
    
    // Distribute capabilities among 6 robots
    int capsPerRobot = totalCaps / 6;
    int extraCaps = totalCaps % 6;
    
    vector<RobotCapability> capOrder = {
        RobotCapability::SENSOR_GPS,
        RobotCapability::MOVEMENT_GROUND,
        RobotCapability::SENSOR_CAMERA
    };
    
    for (int i = 1; i <= 6; i++) {
        int x = 140 + (i - 1);
        int y = 120;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        
        int numCaps = capsPerRobot + (i <= extraCaps ? 1 : 0);
        for (int j = 0; j < numCaps && j < capOrder.size(); j++) {
            r->enableCapability(capOrder[j]);
        }
        
        mrs->addRobot(r);
    }
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
