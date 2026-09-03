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

// Test: Transition System Regions (2-12 regions)
// Fixed: 6 robots, 1 Buchi automaton

// Forward declarations
void createTestEnvironmentWithRegions(int numRegions, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestBuchiAutomaton();

// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   TRANSITION SYSTEM REGIONS TEST SUITE" << endl;
    cout << "   Variable: Number of TS regions (2-12)" << endl;
    cout << "   Fixed: 6 robots, 1 Buchi automaton" << endl;
    cout << string(80, '=') << "\n" << endl;

    TestRunManager manager(TestRunManager::TestCategory::TRANSITION_SYSTEM_REGIONS, "test_results");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    vector<int> regionCounts = {2, 3, 4, 5, 6, 8, 10, 12};
    int testNum = 1;
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   RUNNING TRANSITION SYSTEM SCALING TESTS" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    for (int regionCount : regionCounts) {
        cout << "\n  Test " << testNum << " (" << regionCount << " regions)... ";
        cout.flush();
        
        try {
            TS* ts = nullptr;
            GridWorld* grid = nullptr;
            Environment* env = nullptr;
            MultiRobotSystem* mrs = nullptr;
            
            createTestEnvironmentWithRegions(regionCount, ts, grid, env, mrs);
            
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
            
            TaskAllocationAlgorithms* allocAlg = new TaskAllocationAlgorithms(buchi, env, mrs);
            allocAlg->intensiveInterTaskRelationshipTreeSearch(buchi, env, mrs);
            
            cout << "✓ Complete\n";
            allocAlg->getMetrics().printSummary();
            
            map<string, string> parameters;
            parameters["num_regions"] = to_string(regionCount);
            
            manager.storeRun(
                allocAlg->getMetrics(),
                parameters,
                to_string(regionCount),
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
        
        testNum++;
    }
    
    cout << "\n" << string(80, '=') << endl;
    cout << "   TESTING COMPLETE" << endl;
    cout << string(80, '=') << "\n" << endl;

    cout << "\n✓ All tests completed!" << endl;
    cout << "   - " << regionCounts.size() << " region configurations tested" << endl;
    
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
 * Create test environment with variable number of regions
 * Fixed: 6 robots, 210x210 grid
 * Variable: Number of TS states/regions (2-12)
 */
void createTestEnvironmentWithRegions(int numRegions, TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    grid = new GridWorld(210, 210);
    ts = new TS();
    
    // Create nodes with hub-and-spoke topology (center node connects to all others)
    vector<Node*> nodes;
    for (int i = 0; i < numRegions; i++) {
        Node* n = new Node(i, "R" + to_string(i));
        nodes.push_back(n);
        ts->add_Node(n);
    }
    
    // Create hub-and-spoke connections
    // If only 2 nodes, just connect them bidirectionally
    if (numRegions == 2) {
        nodes[0]->addEdge(Edge(1));
        nodes[1]->addEdge(Edge(0));
    } else {
        // Connect all nodes to center node (index numRegions/2)
        int center = numRegions / 2;
        for (int i = 0; i < numRegions; i++) {
            if (i != center) {
                nodes[i]->addEdge(Edge(center));
                nodes[center]->addEdge(Edge(i));
            }
        }
    }
    
    ts->setInitial(0);
    
    // Create environment and map regions to grid
    env = new Environment(ts, grid);
    
    // Distribute regions across grid
    // Use circular layout with center at (100, 100)
    double angleStep = 2 * M_PI / numRegions;
    int radius = 70;
    
    for (int i = 0; i < numRegions; i++) {
        double angle = i * angleStep;
        int x = 100 + (int)(radius * cos(angle));
        int y = 100 + (int)(radius * sin(angle));
        int width = 30;
        int height = 30;
        
        env->mapTSStateToGrid(i, Point(x, y), width, height);
    }
    
    // Create MultiRobotSystem with 6 robots
    mrs = new MultiRobotSystem();
    
    for (int i = 1; i <= 6; i++) {
        int x = 140 + (i - 1);
        int y = 120;
        
        Robot* r = new Robot(i, "Rover_" + to_string(i), Point(x, y));
        r->initializeCapabilities(13);
        
        // Capability rotation: GPS always enabled, rotate MOVEMENT_GROUND and SENSOR_CAMERA
        r->enableCapability(RobotCapability::SENSOR_GPS);
        if ((i - 1) % 2 == 0) {
            r->enableCapability(RobotCapability::MOVEMENT_GROUND);
        } else {
            r->enableCapability(RobotCapability::SENSOR_CAMERA);
        }
        
        mrs->addRobot(r);
    }
}

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
