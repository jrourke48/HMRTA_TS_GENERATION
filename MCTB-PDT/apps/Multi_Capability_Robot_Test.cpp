#include <iostream>
#include <cassert>
#include <vector>
#include <memory>
#include <algorithm>
#include "../include/TaskAllocationAlgorithms.h"
#include "../include/Environment/gridvis.h"
#include "../include/Tree/PlanningDecisionTree.h"
#include "../include/Tree/Tree_Node.h"
#include "../include/Environment/Environment.h"
#include "../include/MultiRobotSystem/MultiRobotSystem.h"
#include "../include/LTLFormula/LTLFormula.h"
#include "../../Automatons/BuchiAutomaton.h"

using namespace std;

/*
 * ============================================================================
 * MULTI-CAPABILITY ROBOT TEST SUITE
 * ============================================================================
 *
 * PURPOSE:
 *   Specialized test suite for validating task allocation algorithms
 *   with multi-capability robots. Focuses on scenarios where robots have
 *   multiple capabilities required for complex task sequences.
 *
 * TEST FOCUS:
 *   - Capability combinations (GPS + CAMERA, GROUND + CAMERA, etc.)
 *   - Task allocation with heterogeneous robot teams
 *   - Inter-task constraints with capability-dependent sequencing
 *   - Robot swapping and re-tasking with shared capabilities
 *
 * ROBOT CAPABILITY BITS (13-bit vector):
 *   Bit 0: MOVEMENT_GROUND       (A)
 *   Bit 1: MOVEMENT_AERIAL       (B)
 *   Bit 2: MOVEMENT_AQUATIC      (C)
 *   Bit 3: SENSOR_GPS            (D)
 *   Bit 4: SENSOR_ACCELEROMETER  (E)
 *   Bit 5: SENSOR_CAMERA         (F)
 *   Bit 6: COMMUNICATION_RELAY   (G)
 *   Bit 7-12: Reserved for future expansion
 *
 * ============================================================================
 */

// Forward declarations
void createMultiCapabilityEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void multiCapabilityFiniteTests(Environment* env, MultiRobotSystem* mrs);
void multiCapabilityInfiniteTests(Environment* env, MultiRobotSystem* mrs);

// Test automaton creation functions
BuchiAutomaton* createMultiCapabilityFiniteTest1();
BuchiAutomaton* createMultiCapabilityFiniteTest2();
BuchiAutomaton* createMultiCapabilityFiniteTest3();
BuchiAutomaton* createMultiCapabilityFiniteTest4();

BuchiAutomaton* createMultiCapabilityInfiniteTest1();
BuchiAutomaton* createMultiCapabilityInfiniteTest2();
BuchiAutomaton* createMultiCapabilityInfiniteTest3();
BuchiAutomaton* createMultiCapabilityInfiniteTest4();

// Global result vectors
vector<PlanningDecisionTree*> multiCapFiniteResults;
vector<PlanningDecisionTree*> multiCapInfiniteResults;

int main() {
    cout << "\n" << string(80, '=') << endl;
    cout << "   MULTI-CAPABILITY ROBOT TEST SUITE" << endl;
    cout << "   Testing intensiveInterTaskRelationshipTreeSearch" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Create test environment
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    MultiRobotSystem* mrs = nullptr;
    createMultiCapabilityEnvironment(ts, grid, env, mrs);

    // User input to choose which tests to run
    cout << "Which tests would you like to run?" << endl;
    cout << "  (1) Finite Multi-Capability Tests" << endl;
    cout << "  (2) Infinite Multi-Capability Tests" << endl;
    cout << "  (3) Both Finite and Infinite Tests" << endl;
    cout << "Enter your choice (1-3): ";
    int choice;
    cin >> choice;

    if (choice == 1) {
        multiCapabilityFiniteTests(env, mrs);
    } else if (choice == 2) {
        multiCapabilityInfiniteTests(env, mrs);
    } else if (choice == 3) {
        multiCapabilityFiniteTests(env, mrs);
        multiCapabilityInfiniteTests(env, mrs);
    } else {
        cout << "Invalid choice. Exiting." << endl;
        return 1;
    }

    // Print summary
    int finiteSuccess = 0, infiniteSuccess = 0;
    for (auto& tree : multiCapFiniteResults) {
        if (tree) finiteSuccess++;
    }
    for (auto& tree : multiCapInfiniteResults) {
        if (tree) infiniteSuccess++;
    }

    cout << "\n" << string(80, '=') << endl;
    cout << "   MULTI-CAPABILITY TEST SUMMARY" << endl;
    cout << string(80, '=') << "\n" << endl;
    cout << "✓ Finite Tests: " << finiteSuccess << "/4 successful" << endl;
    cout << "✓ Infinite Tests: " << infiniteSuccess << "/4 successful" << endl;
    cout << "\nTotal Tests Run: " << finiteSuccess + infiniteSuccess << endl;
    cout << "All tests completed!\n" << endl;

    // Cleanup
    delete mrs;
    delete env;
    delete grid;
    delete ts;

    cout << string(80, '=') << "\n" << endl;
    return 0;
}

// ============================================================================
// TEST ENVIRONMENT SETUP FOR MULTI-CAPABILITY ROBOTS
// ============================================================================

void createMultiCapabilityEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
    // Create GridWorld (21x21)
    grid = new GridWorld(21, 21);
    cout << "✓ GridWorld created (21x21)" << endl;

    // Create Transition System with 8 states for more complex testing
    ts = new TS();

    Node* nodes[8];
    for (int i = 0; i < 8; i++) {
        nodes[i] = new Node(i, "R" + to_string(i));
    }

    // Create connectivity: star topology with node 3 as center
    // Edges: 0-3, 1-3, 2-3, 3-4, 3-5, 3-6, 3-7
    nodes[0]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(0));
    
    nodes[1]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(1));
    
    nodes[2]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(2));
    
    nodes[4]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(4));
    
    nodes[5]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(5));
    
    nodes[6]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(6));
    
    nodes[7]->addEdge(Edge(3));
    nodes[3]->addEdge(Edge(7));

    for (int i = 0; i < 8; i++) {
        ts->add_Node(nodes[i]);
    }
    ts->setInitial(3);

    cout << "✓ Transition System created" << endl;
    cout << "  - States: " << ts->getNumStates() << endl;
    cout << "  - Initial state: 3 (central hub)" << endl;

    // Create Environment
    env = new Environment(ts, grid);
    cout << "✓ Environment created" << endl;

    // Map states to grid regions (hub and 7 peripheral regions)
    env->mapTSStateToGrid(0, Point(3, 3), 4, 4);         // Top-left corner
    env->mapTSStateToGrid(1, Point(3, 18), 4, 4);        // Bottom-left corner
    env->mapTSStateToGrid(2, Point(18, 3), 4, 4);        // Top-right corner
    env->mapTSStateToGrid(3, Point(10, 10), 8, 8);       // Center hub
    env->mapTSStateToGrid(4, Point(18, 18), 4, 4);       // Bottom-right corner
    env->mapTSStateToGrid(5, Point(10, 2), 4, 4);        // Top center
    env->mapTSStateToGrid(6, Point(10, 18), 4, 4);       // Bottom center
    env->mapTSStateToGrid(7, Point(2, 10), 4, 4);        // Left center

    cout << "✓ Mapped 8 states to grid regions" << endl;

    // Create MultiRobotSystem with 8 robots, each with different capability combinations
    mrs = new MultiRobotSystem();

    // Robot 1: MOVEMENT_GROUND + SENSOR_GPS (bits 0, 3)
    Robot* r1 = new Robot(1, "GroundGPS_1", Point(10, 10));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::MOVEMENT_GROUND);      // Bit 0
    r1->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    mrs->addRobot(r1);

    // Robot 2: MOVEMENT_GROUND + SENSOR_CAMERA (bits 0, 5)
    Robot* r2 = new Robot(2, "GroundCamera_1", Point(10, 9));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND);      // Bit 0
    r2->enableCapability(RobotCapability::SENSOR_CAMERA);        // Bit 5
    mrs->addRobot(r2);

    // Robot 3: MOVEMENT_AERIAL + SENSOR_GPS (bits 1, 3)
    Robot* r3 = new Robot(3, "AerialGPS_1", Point(11, 10));
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::MOVEMENT_AERIAL);      // Bit 1
    r3->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    mrs->addRobot(r3);

    // Robot 4: MOVEMENT_GROUND + SENSOR_CAMERA + SENSOR_GPS (bits 0, 3, 5)
    Robot* r4 = new Robot(4, "GroundGPS_Camera_1", Point(11, 9));
    r4->initializeCapabilities(13);
    r4->enableCapability(RobotCapability::MOVEMENT_GROUND);      // Bit 0
    r4->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    r4->enableCapability(RobotCapability::SENSOR_CAMERA);        // Bit 5
    mrs->addRobot(r4);

    // Robot 5: MOVEMENT_AERIAL + SENSOR_CAMERA (bits 1, 5)
    Robot* r5 = new Robot(5, "AerialCamera_1", Point(10, 11));
    r5->initializeCapabilities(13);
    r5->enableCapability(RobotCapability::MOVEMENT_AERIAL);      // Bit 1
    r5->enableCapability(RobotCapability::SENSOR_CAMERA);        // Bit 5
    mrs->addRobot(r5);

    // Robot 6: MOVEMENT_AQUATIC + SENSOR_GPS (bits 2, 3)
    Robot* r6 = new Robot(6, "AquaticGPS_1", Point(9, 10));
    r6->initializeCapabilities(13);
    r6->enableCapability(RobotCapability::MOVEMENT_AQUATIC);     // Bit 2
    r6->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    mrs->addRobot(r6);

    // Robot 7: MOVEMENT_AERIAL + MOVEMENT_GROUND + SENSOR_GPS (bits 0, 1, 3)
    Robot* r7 = new Robot(7, "HybridMover_GPS_1", Point(9, 11));
    r7->initializeCapabilities(13);
    r7->enableCapability(RobotCapability::MOVEMENT_GROUND);      // Bit 0
    r7->enableCapability(RobotCapability::MOVEMENT_AERIAL);      // Bit 1
    r7->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    mrs->addRobot(r7);

    // Robot 8: All major capabilities (bits 0, 1, 3, 5)
    Robot* r8 = new Robot(8, "OmniRobot_1", Point(10, 10));
    r8->initializeCapabilities(13);
    r8->enableCapability(RobotCapability::MOVEMENT_GROUND);      // Bit 0
    r8->enableCapability(RobotCapability::MOVEMENT_AERIAL);      // Bit 1
    r8->enableCapability(RobotCapability::SENSOR_GPS);           // Bit 3
    r8->enableCapability(RobotCapability::SENSOR_CAMERA);        // Bit 5
    mrs->addRobot(r8);

    cout << "✓ MultiRobotSystem created with 8 multi-capability robots" << endl;
    cout << "  - Robot 1: Ground + GPS" << endl;
    cout << "  - Robot 2: Ground + Camera" << endl;
    cout << "  - Robot 3: Aerial + GPS" << endl;
    cout << "  - Robot 4: Ground + GPS + Camera (tri-capable)" << endl;
    cout << "  - Robot 5: Aerial + Camera" << endl;
    cout << "  - Robot 6: Aquatic + GPS" << endl;
    cout << "  - Robot 7: Ground + Aerial + GPS (mobility hybrid)" << endl;
    cout << "  - Robot 8: Ground + Aerial + GPS + Camera (omni-capable)" << endl;
}

// ============================================================================
// MULTI-CAPABILITY FINITE AUTOMATA TESTS (4)
// ============================================================================

/**
 * FINITE Test 1: Dual-Capability Sequential Tasks
 * Requires: GPS capability for tasks p0, p1
 *          CAMERA capability for tasks p2, p3
 * Tests robot selection prioritizing specific capabilities
 */
void multiCapabilityFiniteTests(Environment* env, MultiRobotSystem* mrs) {
    cout << "\n" << string(80, '-') << endl;
    cout << "   MULTI-CAPABILITY FINITE AUTOMATA TESTS" << endl;
    cout << string(80, '-') << "\n" << endl;

    vector<BuchiAutomaton*> finiteAutomata;
    vector<TaskAllocationAlgorithms*> finiteAlgorithms;

    // Test 1: GPS and Camera capability requirements
    BuchiAutomaton* finite1 = createMultiCapabilityFiniteTest1();
    finiteAutomata.push_back(finite1);
    TaskAllocationAlgorithms* taa1 = new TaskAllocationAlgorithms(finite1, env, mrs);
    finiteAlgorithms.push_back(taa1);
    multiCapFiniteResults.push_back(taa1->intensiveInterTaskRelationshipTreeSearch(finite1, env, mrs));
    cout << "  → Multi-Cap Test 1 (GPS+Camera Sequential) Result: " << (multiCapFiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapFiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa1->getMetrics().printSummary();
        taa1->visualizeTree("output/multicap_finite_test_1_tree");
        taa1->visualizeOptimalPath("output/multicap_finite_test_1_path");
    }

    // Test 2: Complex capability switching
    BuchiAutomaton* finite2 = createMultiCapabilityFiniteTest2();
    finiteAutomata.push_back(finite2);
    TaskAllocationAlgorithms* taa2 = new TaskAllocationAlgorithms(finite2, env, mrs);
    finiteAlgorithms.push_back(taa2);
    multiCapFiniteResults.push_back(taa2->intensiveInterTaskRelationshipTreeSearch(finite2, env, mrs));
    cout << "  → Multi-Cap Test 2 (Capability Switching) Result: " << (multiCapFiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapFiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa2->getMetrics().printSummary();
        taa2->visualizeTree("output/multicap_finite_test_2_tree");
        taa2->visualizeOptimalPath("output/multicap_finite_test_2_path");
    }

    // Test 3: Triple capability requirements
    BuchiAutomaton* finite3 = createMultiCapabilityFiniteTest3();
    finiteAutomata.push_back(finite3);
    TaskAllocationAlgorithms* taa3 = new TaskAllocationAlgorithms(finite3, env, mrs);
    finiteAlgorithms.push_back(taa3);
    multiCapFiniteResults.push_back(taa3->intensiveInterTaskRelationshipTreeSearch(finite3, env, mrs));
    cout << "  → Multi-Cap Test 3 (Triple Capability) Result: " << (multiCapFiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapFiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa3->getMetrics().printSummary();
        taa3->visualizeTree("output/multicap_finite_test_3_tree");
        taa3->visualizeOptimalPath("output/multicap_finite_test_3_path");
    }

    // Test 4: Heterogeneous robot team with inter-task constraints
    BuchiAutomaton* finite4 = createMultiCapabilityFiniteTest4();
    finiteAutomata.push_back(finite4);
    TaskAllocationAlgorithms* taa4 = new TaskAllocationAlgorithms(finite4, env, mrs);
    finiteAlgorithms.push_back(taa4);
    multiCapFiniteResults.push_back(taa4->intensiveInterTaskRelationshipTreeSearch(finite4, env, mrs));
    cout << "  → Multi-Cap Test 4 (Heterogeneous Team) Result: " << (multiCapFiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapFiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa4->getMetrics().printSummary();
        taa4->visualizeTree("output/multicap_finite_test_4_tree");
        taa4->visualizeOptimalPath("output/multicap_finite_test_4_path");
    }
}

/**
 * MULTI-CAP FINITE Test 1: GPS and Camera capability sequence
 * Formula: F(p0 with GPS) & F(p1 with GPS) & F(p2 with Camera) & F(p3 with Camera)
 * Tests: Sequential tasks requiring specific capabilities
 */
BuchiAutomaton* createMultiCapabilityFiniteTest1() {
    string ltl_str = "(F\"p0\" & F\"p1\" & F\"p2\" & F\"p3\")";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires GPS (bit 3)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires GPS (bit 3)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p2: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // p3: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_finite_test_1");
    cout << "✓ Multi-Cap Finite Test 1 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP FINITE Test 2: Capability switching with inter-task constraints
 * Formula: (F(p0 with GPS & Aerial) & F(p1 with GPS & Ground)) & F(p2 with Camera)
 * Tests: Robots switching between different capability-dependent tasks
 */
BuchiAutomaton* createMultiCapabilityFiniteTest2() {
    string ltl_str = "((!\"p0\" U \"p1\") & F\"p0\") & F\"p2\" & F\"p3\"";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires Aerial + GPS (bits 1, 3)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {false, true, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires Ground + GPS (bits 0, 3)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p2: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // p3: requires Aquatic + GPS (bits 2, 3)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, true, true, false, false, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_finite_test_2");
    cout << "✓ Multi-Cap Finite Test 2 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP FINITE Test 3: Triple capability requirements
 * Formula: F(p0 with Ground+GPS+Camera) & F(p1 with Aerial+GPS) & F(p2 with Ground) & F(p3 with Camera)
 * Tests: Tasks requiring 3+ capabilities, forcing use of specialized robots
 */
BuchiAutomaton* createMultiCapabilityFiniteTest3() {
    string ltl_str = "(F\"p0\" & F\"p1\") & (F\"p2\" & F\"p3\")";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires Ground + GPS + Camera (bits 0, 3, 5)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // p1: requires Aerial + GPS (bits 1, 3)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, true, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p2: requires Ground (bit 0)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // p3: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_finite_test_3");
    cout << "✓ Multi-Cap Finite Test 3 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP FINITE Test 4: Heterogeneous team with inter-task constraints
 * Formula: Complex sequence with related and exclusive task constraints
 * Tests: Full multi-capability allocation with batch relationships
 */
BuchiAutomaton* createMultiCapabilityFiniteTest4() {
    string ltl_str = "(F\"p0\" & F\"p1\") & F\"p2\" & F\"p3\"";

    vector<BatchAtomicProposition> batchAPs;
    // Batch 1 (related): p0 and p1 are compatible tasks
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, true, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Batch 2: p2 needs aerial movement
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, true, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // Batch 3: p3 needs camera + ground
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_finite_test_4");
    cout << "✓ Multi-Cap Finite Test 4 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

// ============================================================================
// MULTI-CAPABILITY INFINITE AUTOMATA TESTS (4)
// ============================================================================

void multiCapabilityInfiniteTests(Environment* env, MultiRobotSystem* mrs) {
    cout << "\n" << string(80, '-') << endl;
    cout << "   MULTI-CAPABILITY INFINITE AUTOMATA TESTS" << endl;
    cout << string(80, '-') << "\n" << endl;

    vector<BuchiAutomaton*> infiniteAutomata;
    vector<TaskAllocationAlgorithms*> infiniteAlgorithms;

    // Test 1: Repeating capability requirements
    BuchiAutomaton* infinite1 = createMultiCapabilityInfiniteTest1();
    infiniteAutomata.push_back(infinite1);
    TaskAllocationAlgorithms* taa5 = new TaskAllocationAlgorithms(infinite1, env, mrs);
    infiniteAlgorithms.push_back(taa5);
    multiCapInfiniteResults.push_back(taa5->intensiveInterTaskRelationshipTreeSearch(infinite1, env, mrs));
    cout << "  → Multi-Cap Test 1 (Repeating Capabilities) Result: " << (multiCapInfiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapInfiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa5->getMetrics().printSummary();
        taa5->visualizeTree("output/multicap_infinite_test_1_tree");
        taa5->visualizeOptimalPath("output/multicap_infinite_test_1_path");
    }

    // Test 2: Interleaved capabilities
    BuchiAutomaton* infinite2 = createMultiCapabilityInfiniteTest2();
    infiniteAutomata.push_back(infinite2);
    TaskAllocationAlgorithms* taa6 = new TaskAllocationAlgorithms(infinite2, env, mrs);
    infiniteAlgorithms.push_back(taa6);
    multiCapInfiniteResults.push_back(taa6->intensiveInterTaskRelationshipTreeSearch(infinite2, env, mrs));
    cout << "  → Multi-Cap Test 2 (Interleaved Capabilities) Result: " << (multiCapInfiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapInfiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa6->getMetrics().printSummary();
        taa6->visualizeTree("output/multicap_infinite_test_2_tree");
        taa6->visualizeOptimalPath("output/multicap_infinite_test_2_path");
    }

    // Test 3: Complex patrol with capability constraints
    BuchiAutomaton* infinite3 = createMultiCapabilityInfiniteTest3();
    infiniteAutomata.push_back(infinite3);
    TaskAllocationAlgorithms* taa7 = new TaskAllocationAlgorithms(infinite3, env, mrs);
    infiniteAlgorithms.push_back(taa7);
    multiCapInfiniteResults.push_back(taa7->intensiveInterTaskRelationshipTreeSearch(infinite3, env, mrs));
    cout << "  → Multi-Cap Test 3 (Capability Patrol) Result: " << (multiCapInfiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapInfiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa7->getMetrics().printSummary();
        taa7->visualizeTree("output/multicap_infinite_test_3_tree");
        taa7->visualizeOptimalPath("output/multicap_infinite_test_3_path");
    }

    // Test 4: Advanced capability cycling
    BuchiAutomaton* infinite4 = createMultiCapabilityInfiniteTest4();
    infiniteAutomata.push_back(infinite4);
    TaskAllocationAlgorithms* taa8 = new TaskAllocationAlgorithms(infinite4, env, mrs);
    infiniteAlgorithms.push_back(taa8);
    multiCapInfiniteResults.push_back(taa8->intensiveInterTaskRelationshipTreeSearch(infinite4, env, mrs));
    cout << "  → Multi-Cap Test 4 (Advanced Cycling) Result: " << (multiCapInfiniteResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (multiCapInfiniteResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa8->getMetrics().printSummary();
        taa8->visualizeTree("output/multicap_infinite_test_4_tree");
        taa8->visualizeOptimalPath("output/multicap_infinite_test_4_path");
    }

    cout << "\n" << string(80, '=') << endl;
}

/**
 * MULTI-CAP INFINITE Test 1: Repeating GPS patrol
 * Formula: G(F(p0 with GPS) & F(p1 with GPS))
 * Tests: Robots with GPS continuously visiting two locations
 */
BuchiAutomaton* createMultiCapabilityInfiniteTest1() {
    string ltl_str = "G(F\"p0\" & F\"p1\")";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires GPS (bit 3)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires GPS (bit 3)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_infinite_test_1");
    cout << "✓ Multi-Cap Infinite Test 1 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP INFINITE Test 2: Interleaved aerial and ground patrol
 * Formula: G(F(p0 with Aerial) & F(p1 with Ground))
 * Tests: Different robot types continuously visiting different locations
 */
BuchiAutomaton* createMultiCapabilityInfiniteTest2() {
    string ltl_str = "G(F\"p0\" & F\"p1\" & F\"p2\")";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires Aerial (bit 1)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {false, true, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires Ground (bit 0)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // p2: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_infinite_test_2");
    cout << "✓ Multi-Cap Infinite Test 2 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP INFINITE Test 3: Complex capability patrol
 * Formula: G(F(p0 with Ground+GPS) & F(p1 with Aerial+Camera) & F(p2 with Aquatic))
 * Tests: Heterogeneous team with specialized capabilities on patrol
 */
BuchiAutomaton* createMultiCapabilityInfiniteTest3() {
    string ltl_str = "G(F\"p0\" & F\"p1\" & F\"p2\")";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires Ground + GPS (bits 0, 3)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires Aerial + Camera (bits 1, 5)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, true, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // p2: requires Aquatic (bit 2)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, true, false, false, false, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_infinite_test_3");
    cout << "✓ Multi-Cap Infinite Test 3 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * MULTI-CAP INFINITE Test 4: Advanced cycling with multiple capability combinations
 * Formula: G((F(p0 with GPS) & X(F(p1 with Camera))) & (F(p2 with Aerial) & X(F(p3 with Ground))))
 * Tests: Complex sequencing with capability-dependent ordering
 */
BuchiAutomaton* createMultiCapabilityInfiniteTest4() {
    string ltl_str = "G((F\"p0\" & X(F\"p1\")) & (F\"p2\" & X(F\"p3\")))";

    vector<BatchAtomicProposition> batchAPs;
    // p0: requires GPS (bit 3)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    // p1: requires Camera (bit 5)
    batchAPs.push_back(BatchAtomicProposition(1, 1, {false, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // p2: requires Aerial (bit 1)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, true, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // p3: requires Ground (bit 0)
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/multicap_infinite_test_4");
    cout << "✓ Multi-Cap Infinite Test 4 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}
