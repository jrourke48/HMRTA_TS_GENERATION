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
 * LTL FORMULA SYNTAX GUIDE & OPERATOR PRECEDENCE
 * ============================================================================
 *
 * BASIC OPERATORS (in order of precedence, highest to lowest):
 *   1. !    not (highest precedence)
 *   2. &    and
 *   3. |    or
 *   4. ^    xor
 *   5. i    implies
 *   6. e    equivalent (lowest precedence)
 *
 * TEMPORAL OPERATORS (always have lower precedence than logical operators):
 *   X    next (state transitions)
 *   F    eventually
 *   G    globally / always
 *   U    strong until
 *   V    weak release
 *   M    strong release
 *   W    weak until
 *
 * OPERATOR PRECEDENCE RULES:
 *   - Logical operators bind tighter than temporal operators
 *   - Use parentheses for clarity and to override defaults
 *   - F(a & b) ≠ F a & F b
 *   - G(F a) means "infinitely often a"
 *   - F a & F b means "eventually a" AND "eventually b"
 *
 * ============================================================================
 * EXAMPLES FROM OUR TEST SUITE
 * ============================================================================
 *
 * FINITE AUTOMATA (no G, just F - eventuality):
 *
 *   (F"p0" & F"p1")
 *     Eventually visit location p0, and eventually visit location p1.
 *     Once both are satisfied, mission can complete.
 *
 *   (F"p0" & F"p1" & (!\"p2\"U"p0") X "p1")
 *     Eventually p0 and p1, avoid p2 until p0, then next go to p1.
 *     Complex sequence with ordering constraints.
 *
 *   (F"p0" & F"p1" & F"p2" & F"p3")
 *     Visit four locations in any order. Finite - all must eventually occur.
 *
 * INFINITE AUTOMATA (has G - liveness/infinite repetition):
 *
 *   G(F"p0")
 *     Infinitely often visit p0. Must repeatedly return to location p0.
 *     Creates cycles - cannot terminate.
 *
 *   G(F("p0" & "p1"))
 *     Infinitely often satisfy both p0 and p1 together.
 *     Robots must repeatedly visit both locations simultaneously.
 *
 *   G((F"p0") & (F"p1") & (F"p2"))
 *     Always eventually p0, AND always eventually p1, AND always eventually p2.
 *     All three locations must be visited infinitely often (patrol pattern).
 *
 * ============================================================================
 * PRACTICAL TIPS
 * ============================================================================
 *
 * To check if an automaton is finite or infinite:
 *   - No G operator → likely FINITE (eventuality)
 *   - Has G operator → likely INFINITE (liveness requirement)
 *   - isFinite() DFS cycle detection confirms actual behavior
 *
 * For task allocation
 *   - Finite formulas: search completes when all tasks done
 *   - Infinite formulas: search continues, must find repeating path
 *
 * ============================================================================
 */

// ============================================================================
// MAIN TEST RUNNER: can run infinite, finite, or multiple robot capabilities tests 
// ============================================================================

// Forward declarations
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void finiteAutomataTests(Environment* env, MultiRobotSystem* mrs);
void infiniteAutomataTests(Environment* env, MultiRobotSystem* mrs);
BuchiAutomaton* createTestFiniteBuchiAutomaton1();
BuchiAutomaton* createTestFiniteBuchiAutomaton2();
BuchiAutomaton* createTestFiniteBuchiAutomaton3();
BuchiAutomaton* createTestFiniteBuchiAutomaton4();
BuchiAutomaton* createTestInfiniteBuchiAutomaton1();
BuchiAutomaton* createTestInfiniteBuchiAutomaton2();
BuchiAutomaton* createTestInfiniteBuchiAutomaton3();
BuchiAutomaton* createTestInfiniteBuchiAutomaton4();
BuchiAutomaton* createTestInfiniteBuchiAutomaton5();

// Global result vectors (populated by test functions)
vector<PlanningDecisionTree*> finiteSearchResults;
vector<PlanningDecisionTree*> infiniteSearchResults;

int main() {
    cout << "\n" << string(80, '=') << endl;
    cout << "   FINITE & INFINITE BÜCHI AUTOMATON TEST SUITE" << endl;
    cout << "   Testing with intensiveInterTaskRelationshipTreeSearch" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Create test environment once
    TS* ts = nullptr;
    GridWorld* grid = nullptr;
    Environment* env = nullptr;
    MultiRobotSystem* mrs = nullptr;
    createTestEnvironment(ts, grid, env, mrs);

    //user input to choose which tests to run
    cout << "Which tests would run? (1) Finite, (2) Infinite, (3) Both: ";
    int choice;
    cin >> choice;  // Read the user's choice

    if (choice == 1) {
        finiteAutomataTests(env, mrs);
    } else if (choice == 2) {
        infiniteAutomataTests(env, mrs);
    } else if (choice == 3) {
        finiteAutomataTests(env, mrs);
        infiniteAutomataTests(env, mrs);
    } else {
        cout << "Invalid choice. Exiting." << endl;
        return 1;
    }

    int finiteSuccess = 0, infiniteSuccess = 0;
    for (auto& tree : finiteSearchResults) {
        if (tree) finiteSuccess++;
    }
    for (auto& tree : infiniteSearchResults) {
        if (tree) infiniteSuccess++;
    }

    cout << "✓ Finite Automata Tests: " << finiteSuccess << "/4 successful" << endl;
    cout << "✓ Infinite Automata Tests: " << infiniteSuccess << "/4 successful" << endl;
    cout << "\nTotal Tests Run: 8" << endl;
    cout << "All tests completed!\n" << endl;

    // Cleanup - remove pointers created during setup
    delete mrs;
    delete env;
    delete grid;
    delete ts;

    cout << string(80, '=') << "\n" << endl;
    return 0;
}

void finiteAutomataTests(Environment* env, MultiRobotSystem* mrs) {
cout << "\n" << string(80, '-') << endl;
    cout << "   FINITE AUTOMATA TESTS (Expected: isFinite() = YES)" << endl;
    cout << string(80, '-') << "\n" << endl;

    // Run 4 Finite Tests with intensiveInterTaskRelationshipTreeSearch
    vector<BuchiAutomaton*> finiteAutomata;
    vector<TaskAllocationAlgorithms*> finiteAlgorithms;
    
    BuchiAutomaton* finite1 = createTestFiniteBuchiAutomaton1();
    finiteAutomata.push_back(finite1);
    TaskAllocationAlgorithms* taa1 = new TaskAllocationAlgorithms(finite1, env, mrs);
    finiteAlgorithms.push_back(taa1);
    finiteSearchResults.push_back(taa1->intensiveInterTaskRelationshipTreeSearch(finite1, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa1->getMetrics().printSummary();
        taa1->visualizeTree("output/finite_test_1_tree");
        taa1->visualizeOptimalPath("output/finite_test_1_path");
    }
    
    BuchiAutomaton* finite2 = createTestFiniteBuchiAutomaton2();
    finiteAutomata.push_back(finite2);
    TaskAllocationAlgorithms* taa2 = new TaskAllocationAlgorithms(finite2, env, mrs);
    finiteAlgorithms.push_back(taa2);
    finiteSearchResults.push_back(taa2->intensiveInterTaskRelationshipTreeSearch(finite2, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa2->getMetrics().printSummary();
        taa2->visualizeTree("output/finite_test_2_tree");
        taa2->visualizeOptimalPath("output/finite_test_2_path");
    }
    

    BuchiAutomaton* finite3 = createTestFiniteBuchiAutomaton3();
    finiteAutomata.push_back(finite3);
    TaskAllocationAlgorithms* taa3 = new TaskAllocationAlgorithms(finite3, env, mrs);
    finiteAlgorithms.push_back(taa3);
    finiteSearchResults.push_back(taa3->intensiveInterTaskRelationshipTreeSearch(finite3, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa3->getMetrics().printSummary();
        taa3->visualizeTree("output/finite_test_3_tree");
        taa3->visualizeOptimalPath("output/finite_test_3_path");
        
        // PlanningDecisionTree* planningTree = taa3->getPlanningTree();
        // std::vector<Tree_Node*> optimalPath = planningTree->getPathtoFrontierNode(planningTree->getOptimalFrontierNode(true));
        // visualize_environment(
        //     *env,
        //     *mrs,
        //     optimalPath,
        //     compute_dstar_paths(*env, *mrs, optimalPath),
        //     "Multi-Robot Task Plan Visualization - D* Lite Pathfinding"
        // );
    }
    
    BuchiAutomaton* finite4 = createTestFiniteBuchiAutomaton4();
    finiteAutomata.push_back(finite4);
    TaskAllocationAlgorithms* taa4 = new TaskAllocationAlgorithms(finite4, env, mrs);
    finiteAlgorithms.push_back(taa4);
    finiteSearchResults.push_back(taa4->intensiveInterTaskRelationshipTreeSearch(finite4, env, mrs));
    cout << "  → Search Result: " << (finiteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (finiteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa4->getMetrics().printSummary();
        taa4->visualizeTree("output/finite_test_4_tree");
        taa4->visualizeOptimalPath("output/finite_test_4_path");
    }
}

void infiniteAutomataTests(Environment* env, MultiRobotSystem* mrs) {
    cout << "\n" << string(80, '-') << endl;
    cout << "   INFINITE AUTOMATA TESTS (Expected: isFinite() = NO)" << endl;
    cout << string(80, '-') << "\n" << endl;

    // Run 4 Infinite Tests with intensiveInterTaskRelationshipTreeSearch
    vector<BuchiAutomaton*> infiniteAutomata;
    vector<TaskAllocationAlgorithms*> infiniteAlgorithms;
    
    BuchiAutomaton* infinite1 = createTestInfiniteBuchiAutomaton1();
    infiniteAutomata.push_back(infinite1);
    TaskAllocationAlgorithms* taa5 = new TaskAllocationAlgorithms(infinite1, env, mrs);
    infiniteAlgorithms.push_back(taa5);
    infiniteSearchResults.push_back(taa5->intensiveInterTaskRelationshipTreeSearch(infinite1, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa5->getMetrics().printSummary();
        taa5->visualizeTree("output/infinite_test_1_tree");
        taa5->visualizeOptimalPath("output/infinite_test_1_path");
    }
    
    BuchiAutomaton* infinite2 = createTestInfiniteBuchiAutomaton2();
    infiniteAutomata.push_back(infinite2);
    TaskAllocationAlgorithms* taa6 = new TaskAllocationAlgorithms(infinite2, env, mrs);
    infiniteAlgorithms.push_back(taa6);
    infiniteSearchResults.push_back(taa6->intensiveInterTaskRelationshipTreeSearch(infinite2, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa6->getMetrics().printSummary();
        taa6->visualizeTree("output/infinite_test_2_tree");
        taa6->visualizeOptimalPath("output/infinite_test_2_path");
    }
    
    BuchiAutomaton* infinite3 = createTestInfiniteBuchiAutomaton3();
    infiniteAutomata.push_back(infinite3);
    TaskAllocationAlgorithms* taa7 = new TaskAllocationAlgorithms(infinite3, env, mrs);
    infiniteAlgorithms.push_back(taa7);
    infiniteSearchResults.push_back(taa7->intensiveInterTaskRelationshipTreeSearch(infinite3, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa7->getMetrics().printSummary();
        taa7->visualizeTree("output/infinite_test_3_tree");
        taa7->visualizeOptimalPath("output/infinite_test_3_path");
    }
    
    BuchiAutomaton* infinite4 = createTestInfiniteBuchiAutomaton4();
    infiniteAutomata.push_back(infinite4);
    TaskAllocationAlgorithms* taa8 = new TaskAllocationAlgorithms(infinite4, env, mrs);
    infiniteAlgorithms.push_back(taa8);
    infiniteSearchResults.push_back(taa8->intensiveInterTaskRelationshipTreeSearch(infinite4, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa8->getMetrics().printSummary();
        taa8->visualizeTree("output/infinite_test_4_tree");
        taa8->visualizeOptimalPath("output/infinite_test_4_path");
    }

    BuchiAutomaton* infinite5 = createTestInfiniteBuchiAutomaton5();
    infiniteAutomata.push_back(infinite5);
    TaskAllocationAlgorithms* taa9 = new TaskAllocationAlgorithms(infinite5, env, mrs);
    infiniteAlgorithms.push_back(taa9);
    infiniteSearchResults.push_back(taa9->intensiveInterTaskRelationshipTreeSearch(infinite5, env, mrs));
    cout << "  → Search Result: " << (infiniteSearchResults.back() ? "SUCCESS" : "FAILED") << endl;
    if (infiniteSearchResults.back()) {
        cout << "\n  Metrics Summary:" << endl;
        taa9->getMetrics().printSummary();
        taa9->visualizeTree("output/infinite_test_5_tree");
        taa9->visualizeOptimalPath("output/infinite_test_5_path");
    }

    cout << "\n" << string(80, '=') << endl;
    cout << "   TEST SUMMARY" << endl;
    cout << string(80, '=') << "\n" << endl;
}


// ============================================================================
//Create test environment with TS and GridWorld
void createTestEnvironment(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
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

/**
 * Create test Büchi automaton: eventually p1 and eventually p2
 */
BuchiAutomaton* createTestBuchiAutomaton() {
    string ltl_str = "(F\"p1\" && F\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}

/**
 * Create test Büchi automaton: eventually p1 and eventually p4 and eventually p5 and eventually p3
 */
BuchiAutomaton* createTestBuchiAutomaton2() {
    string ltl_str = "(F\"p1\" && F\"p4\" && F\"p5\" && F\"p3\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/buchi_automaton_tree_test");
    cout << "✓ BuchiAutomaton created" << endl;
    return buchi;
}
/** 
 * Create test Büchi automaton: eventually p1 and eventually p4 and eventually p5 and eventually p3
 * with different capabilities for each atomic proposition
*/

// ============================================================================
// FINITE AUTOMATA TESTS (4)
// ============================================================================

/**
 * FINITE Test 1: Simple two-step sequential (F p0 && F p1)
 * Linear progression with no cycles
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton1() {
    string ltl_str = "(((!\"p1\" & !\"p2\") U(\"p4\" & X\"p3\")) & F\"p1\"& F\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_1");
    cout << "✓ Finite Test 1 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * FINITE Test 2: Three sequential tasks (F p0 && F p1 && F p2)
 * Linear progression with different capabilities
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton2() {
    string ltl_str = "((!\"p1\" U \"p4\") & (!\"p2\" U \"p5\")) & F\"p3\" & F(\"p1\"|\"p2\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_2");
    cout << "✓ Finite Test 2 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * ADVANCED FINITE Test 3: Sequential with revisit
 * Tests two batch APs with the same AP value (p1)
 * Two robot groups visit p1 at different stages with different capabilities
 * Example: Robots with GPS visit p1 first, then robots with camera visit p1 later
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton3() {
    string ltl_str = "(!\"p10\" U \"p4\") & F\"p10\" & F(\"p2\" & X\"p11\")";
    
    vector<BatchAtomicProposition> batchAPs;
    // First group: robots with GPS capability visit p1a
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p1a with GPS (ID: 10)
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Second group: different robots with camera capability visit p1b
    batchAPs.push_back(BatchAtomicProposition(11, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p1b with camera (ID: 11)
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_3");
    cout << "✓ Finite Test 3 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * ADVANCED FINITE Test 4: Complex multi-location with shared AP and ordering
 * Tests two batch APs with the same AP value (p1) plus complex constraint ordering
 * Sequence: avoid p1 until p0, visit p1 (GPS), visit p2, avoid p1 until p3, visit p1 again (camera), visit p4
 * Example: First GPS robots visit p1 after checkpoint p0, then later camera robots must visit p1 after checkpoint p3
 */
BuchiAutomaton* createTestFiniteBuchiAutomaton4() {
    string ltl_str = "((!\"p10\" U \"p0\") & F\"p10\") & F\"p2\" & ((!\"p11\" U \"p3\") & F\"p11\") & F\"p4\"";
    
    vector<BatchAtomicProposition> batchAPs;
    // Checkpoint p0
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
    // First p1a visit: robots with GPS capability after p0 (ID: 10)
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // Intermediate location p2
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Checkpoint p3
    batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Second p1b visit: different robots with MOVEMENT_GROUND, SENSOR_CAMERA, and GPS after p3 (ID: 11)
    batchAPs.push_back(BatchAtomicProposition(11, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Final location p4
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/finite_test_4");
    cout << "✓ Finite Test 4 created: " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

// ============================================================================
// INFINITE AUTOMATA TESTS (4)
// ============================================================================

/**
 * ADVANCED INFINITE Test 1: Complex nested temporal operators with sequencing
 * G(F(p0 & X(p1 & X p2))) & G(F(p3 & (F p4)))
 * Must infinitely often satisfy: after p0, next p1, then next p2
 * AND simultaneously infinitely often have p3 followed by eventually p4
 * Tests multi-layer temporal nesting with interlocking constraints
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1() {
    string ltl_str = "G(F(\"p0\" & X(\"p1\" & X\"p2\"))) & G(F(\"p3\" & (F\"p4\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    // Core locations for nested temporal pattern
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // Parallel repeating constraint
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_1");
    cout << "✓ ADVANCED Infinite Test 1 created (nested temporal with sequencing): " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}
BuchiAutomaton* createTestInfiniteBuchiAutomaton5() {
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
    buchi->visualize("output/automaton_test_buchi_5.dot");
    cout << "✓ ADVANCED Infinite Test 5 created (multiple liveness constraints): " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * ADVANCED INFINITE Test 2: Weak until with globally constrained liveness
 * G((F"p0" W "p1") & (F"p2" U "p3")) & G(F("p4" & X"p0"))
 * Complex: p0 must occur unless p1 occurs (weak until), AND eventually p2 until p3,
 * AND infinitely often p4 followed by p0
 * Tests weak until combined with strong until and nested sequencing with 5 locations
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton2() {
    string ltl_str = "G(F(\"p0\" W \"p1\") & F(\"p2\" U \"p3\")) & G(F(\"p4\" & X\"p0\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    // Weak until pattern with p0 and p1
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // Strong until pattern
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    // Nested sequencing back to p0
    batchAPs.push_back(BatchAtomicProposition(4, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_2");
    cout << "✓ ADVANCED Infinite Test 2 created (weak/strong until nesting): " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * ADVANCED INFINITE Test 3: Release operator with disjunctive liveness
 * G(("p0" M ("p1" & F"p2")) | ("p3" U "p4"))
 * Complex: Either (p0 releases from p1 which must eventually have p2) OR (p3 until p4),
 * and both patterns must repeat infinitely
 * Tests strong release operator with disjunctive choices using 5 locations
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3() {
    string ltl_str = "G(F(\"p0\")) ^ G(F(\"p1\" & X (\"p2\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    // Release pattern: p0 releases from (p1 & eventually p2)
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_3");
    cout << "✓ ADVANCED Infinite Test 3 created (release with disjunctive liveness): " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}

/**
 * ADVANCED INFINITE Test 4: Complex nesting with conditional sequencing
 * G((F("p0" & X("p1" U "p2"))) & (G!"p3" | F("p4" & X"p0"))) 
 * Extremely complex: infinitely often (p0 then eventually p1 until p2),
 * AND (always avoid p3 OR infinitely often p4 followed by return to p0)
 * Tests multi-level nesting with mixed safety and liveness properties using 5 locations
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4() {
    string ltl_str = "G((F(\"p0\" & X(\"p1\" U \"p2\")))) & (G(F\"p3\") | G(F(\"p4\" & X(\"p0\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    // Main nested pattern: p0 then until pattern
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    // Safety constraint: avoid p3, or infinitely often p4 then p0
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/infinite_test_4");
    cout << "✓ ADVANCED Infinite Test 4 created (complex nesting with conditional sequencing): " << ltl_str << endl;
    cout << "  - Is Finite: " << (buchi->isFinite() ? "YES" : "NO") << endl;
    return buchi;
}
