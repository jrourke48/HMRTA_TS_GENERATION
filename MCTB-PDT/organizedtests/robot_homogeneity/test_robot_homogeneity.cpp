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

// Test: Robot Fleet Homogeneity (0.2-3.0)
// Fixed: 6 robots, 6 regions, 1 Buchi automaton

struct HomogeneityTest {
    int testNum;
    double homogeneity;
    int independentCapabilities;
    string fleetType;
};

// Forward declarations
void createTestEnvironmentWithHomogeneity(int indepCaps, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestBuchiAutomaton();

// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   ROBOT FLEET HOMOGENEITY TEST SUITE" << endl;
    cout << "   Variable: Fleet homogeneity (0.2-3.0)" << endl;
    cout << "   Fixed: 6 robots, 6 regions, 1 Buchi automaton" << endl;
    cout << string(80, '=') << "\n" << endl;

    TestRunManager manager(TestRunManager::TestCategory::ROBOT_HOMOGENEITY, ".");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    vector<HomogeneityTest> tests = {
        {1, 0.2, 1, "Highly Specialized"},
        {2, 0.4, 2, "Very Specialized"},
        {3, 0.6, 3, "Specialized"},
        {4, 0.8, 4, "Mixed"},
        {5, 1.0, 6, "Balanced"},
        {6, 1.3, 8, "Homogeneous"},
        {7, 1.5, 9, "Homogeneous"},
        {8, 1.8, 11, "Very Homogeneous"},
        {9, 2.3, 14, "Very Homogeneous"},
        {10, 3.0, 18, "Identical Fleet"}
    };
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   RUNNING HOMOGENEITY CONFIGURATION TESTS" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    for (const auto& test : tests) {
        cout << "\n  Test " << test.testNum << " (" << fixed << setprecision(1) << test.homogeneity 
             << " - " << test.fleetType << ")... ";
        cout.flush();
        
        try {
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            
            createTestEnvironmentWithHomogeneity(test.independentCapabilities, ts, grid, env, mrs);
            
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
            parameters["homogeneity"] = to_string(test.homogeneity);
            parameters["independent_capabilities"] = to_string(test.independentCapabilities);
            parameters["fleet_type"] = test.fleetType;
            
            manager.storeRun(
                allocAlg->getMetrics(),
                parameters,
                to_string(test.independentCapabilities),
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
    cout << "   - " << tests.size() << " homogeneity configurations tested" << endl;
    
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
 * Create test environment with variable capability distribution for homogeneity testing
 * Fixed: 6 robots, 6 regions, 210x210 grid
 * Variable: Number of independent capabilities to achieve different homogeneity scores
 */
void createTestEnvironmentWithHomogeneity(int indepCaps, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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
    
    // Capability pool to distribute
    vector<RobotCapability> capPool = {
        RobotCapability::SENSOR_GPS,
        RobotCapability::MOVEMENT_GROUND,
        RobotCapability::SENSOR_CAMERA,
        RobotCapability::SENSOR_GPS,
        RobotCapability::MOVEMENT_GROUND,
        RobotCapability::SENSOR_CAMERA
    };
    
    // Create 6 robots with varying capability distribution
    for (int i = 1; i <= 6; i++) {
        int x = 140 + (i - 1);
        int y = 120;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        
        // Distribute capabilities based on homogeneity
        if (i <= indepCaps % 6) {
            // First robots get more diverse capabilities
            if (i <= indepCaps) {
                r->enableCapability(capPool[i - 1]);
            }
        } else {
            // Remaining robots get repeated/similar capabilities
            r->enableCapability(capPool[(i - 1) % 3]);
        }
        
        mrs->addRobot(r);
    }
}

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
