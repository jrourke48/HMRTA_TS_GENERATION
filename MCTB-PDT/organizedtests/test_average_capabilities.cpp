#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>

// Test: Average Robot Capabilities (1-5 capabilities per robot)
// Fixed: 6 robots, 6 regions, 15 automaton states, 3 different Buchi automata

struct CapabilityTest {
    int testNum;
    double avgCapabilities;
    int totalCapabilities;
    int robots;
};

int main() {
    std::cout << "=== Test Suite: Average Robot Capabilities ===" << std::endl;
    std::cout << "Variable: Average capabilities per robot" << std::endl;
    std::cout << "Range: 1.0-5.0 capabilities per robot" << std::endl;
    std::cout << "Fixed: 6 robots, 6 regions, 15 automaton states" << std::endl;
    std::cout << "\n";

    // Test configurations (total capabilities / 6 robots)
    std::vector<CapabilityTest> tests = {
        {1, 1.0, 6, 6},
        {2, 1.2, 7, 6},
        {3, 1.5, 9, 6},
        {4, 1.8, 11, 6},
        {5, 2.0, 12, 6},
        {6, 2.5, 15, 6},
        {7, 3.0, 18, 6},
        {8, 3.5, 21, 6},
        {9, 4.0, 24, 6},
        {10, 5.0, 30, 6}
    };
    
    // Data collection
    std::ofstream resultsFile("average_capabilities_results.csv");
    resultsFile << "Test#,AvgCapabilities,TotalCapabilities,Computation_Time_ms,Memory_MB,TaskCoverage_Score,Redundancy_Score,Status\n";
    
    for (const auto& test : tests) {
        std::cout << "Test " << test.testNum << ": " << std::fixed << std::setprecision(1) 
                  << test.avgCapabilities << " capabilities/robot (total: " << test.totalCapabilities << ")... ";
        
        // TODO: Call TestingAutomaton with capability configuration
        // auto start = std::chrono::high_resolution_clock::now();
        // int result = runTestWithCapabilities(15, 6, 6, test.totalCapabilities);
        // auto end = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "PENDING" << std::endl;
        // resultsFile << test.testNum << "," << test.avgCapabilities << "," << test.totalCapabilities 
        //             << ",0,0,0,0,PENDING\n";
    }
    
    resultsFile.close();
    std::cout << "\nResults saved to average_capabilities_results.csv" << std::endl;
    
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
