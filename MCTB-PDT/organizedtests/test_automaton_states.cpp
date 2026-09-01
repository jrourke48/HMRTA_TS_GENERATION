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
#include "../include/TestRunManager.h"
#include "../../Automatons/BuchiAutomaton.h"
#include "../../Automatons/ProductAutomaton.h"

using namespace std;

// Test: Number of Automaton States (5-150 states)
// Fixed: 6 robots, 6 regions, 4 different Buchi automata

// Forward declarations
void createTestEnvironment3(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment15(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
void createTestEnvironment45(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs);
BuchiAutomaton* createTestInfiniteBuchiAutomaton1();
BuchiAutomaton* createTestInfiniteBuchiAutomaton2();
BuchiAutomaton* createTestInfiniteBuchiAutomaton3();
BuchiAutomaton* createTestInfiniteBuchiAutomaton4();
BuchiAutomaton* createTestInfiniteBuchiAutomaton5();
BuchiAutomaton* createTestInfiniteBuchiAutomaton6();
BuchiAutomaton* createTestInfiniteBuchiAutomaton7();
BuchiAutomaton* createTestInfiniteBuchiAutomaton8();
BuchiAutomaton* createTestInfiniteBuchiAutomaton9();
BuchiAutomaton* createTestInfiniteBuchiAutomaton10();
BuchiAutomaton* createTestInfiniteBuchiAutomaton11();
BuchiAutomaton* createTestInfiniteBuchiAutomaton12();
BuchiAutomaton* createTestInfiniteBuchiAutomaton13();
BuchiAutomaton* createTestInfiniteBuchiAutomaton14();
BuchiAutomaton* createTestInfiniteBuchiAutomaton15();
BuchiAutomaton* createTestInfiniteBuchiAutomaton16();


// Get memory usage in MB
double getMemoryUsageMB() {
    struct rusage r_usage;
    getrusage(RUSAGE_SELF, &r_usage);
    return (double)r_usage.ru_maxrss / 1024.0;  // Convert from KB to MB
}

int main() {
    cout << string(80, '=') << endl;
    cout << "   AUTOMATON SCALING TEST SUITE" << endl;
    cout << "   11 Büchi Automata" << endl;
    cout << "   4 Environments: 3-robot, 6-robot, 15-robot, and 45-robot" << endl;
    cout << "   Total Tests: 44 (11 automata × 4 environments)" << endl;
    cout << string(80, '=') << "\n" << endl;

    // Initialize TestRunManager for AUTOMATON_STATES category
    TestRunManager manager(TestRunManager::TestCategory::AUTOMATON_STATES, "test_results");
    manager.initialize();
    cout << "✓ TestRunManager initialized\n" << endl;

    // Create array of automaton factory functions
    vector<BuchiAutomaton*(*)()> automatonFactories = {
        createTestInfiniteBuchiAutomaton1,
        createTestInfiniteBuchiAutomaton2,
        createTestInfiniteBuchiAutomaton3,
        createTestInfiniteBuchiAutomaton4,
        createTestInfiniteBuchiAutomaton5,
        createTestInfiniteBuchiAutomaton6,
        createTestInfiniteBuchiAutomaton7,
        createTestInfiniteBuchiAutomaton8,
        createTestInfiniteBuchiAutomaton9,
        createTestInfiniteBuchiAutomaton10,
        createTestInfiniteBuchiAutomaton11,
        createTestInfiniteBuchiAutomaton12,
        createTestInfiniteBuchiAutomaton13,
        createTestInfiniteBuchiAutomaton14,
        createTestInfiniteBuchiAutomaton15,
        createTestInfiniteBuchiAutomaton16
    };
    
    vector<int> robotCounts = {3, 6, 15, 45};
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
        } else if (robotCount == 6) {
            createTestEnvironment6(ts, grid, env, mrs);
        } else if (robotCount == 15) {
            createTestEnvironment15(ts, grid, env, mrs);
        } else {
            createTestEnvironment45(ts, grid, env, mrs);
        }
        
        cout << "\n" << string(80, '-') << endl;
        cout << "   RUNNING TESTS" << endl;
        cout << string(80, '-') << "\n" << endl;
        
        // For each of the 16 automata
        for (int automatonId = 1; automatonId <= 16; ++automatonId) {
            cout << "\n  Test " << testNum << " (Automaton " << automatonId << ")... ";
            cout.flush();
            
            try {
                // Create the Buchi automaton
                BuchiAutomaton* buchi = automatonFactories[automatonId - 1]();
                
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
                if (robotCount == 6 && automatonId >= 7) {
                    allocAlg->visualizeTree("output/automaton_test_" + to_string(robotCount) + "robots_" + to_string(automatonId) + "_tree.dot");
                    allocAlg->visualizeOptimalPath("output/automaton_test_" + to_string(robotCount) + "robots_" + to_string(automatonId) + "_path.dot");
                }
                
                cout << "✓ Complete\n";
                allocAlg->getMetrics().printSummary();
                
                // Store run in TestRunManager
                map<string, string> parameters;
                parameters["automaton_id"] = to_string(automatonId);
                parameters["num_robots"] = to_string(robotCount);
                
                manager.storeRun(
                    allocAlg->getMetrics(),
                    parameters,
                    to_string(automatonId),  // independent variable: group by automaton
                    1  // trial number
                );
                
                delete allocAlg;
                delete buchi;
                
            } catch (const exception& e) {
                cout << "ERROR: " << e.what() << endl;
            }
            
            testNum++;
        }
        //export the csv for the test run manager

        
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
    cout << "✓ All tests completed!" << endl;
    cout << "   - 16 automata tested with current robot configuration(s)" << endl;
    cout << "   - When all 4 configs enabled (3, 6, 15, 45 robots): 64 total tests" << endl;
    
    // Export results from TestRunManager
    cout << "\n✓ Exporting results from TestRunManager..." << endl;
    manager.exportByConfiguration();  // Export separate CSV for each configuration
    manager.exportStatisticsToCSV("test_results/automaton_states/statistics.csv");
    manager.exportSummaryReport("test_results/automaton_states/summary_report.txt");
    manager.printTestProgress();
    
    cout << "\n✓ CSV Results stored in test_results/automaton_states/exports/ folder:" << endl;
    cout << "   - automaton_states_num_robots_3.csv" << endl;
    cout << "   - automaton_states_num_robots_6.csv" << endl;
    cout << "   - automaton_states_num_robots_15.csv" << endl;
    cout << "   - automaton_states_num_robots_45.csv" << endl;
    cout << "\n✓ Statistics and summary stored in test_results/automaton_states/" << endl;
    cout << string(80, '=') << "\n" << endl;
    
    return 0;
}


/**
 * Test 1: Basic Conjunctive Liveness
 * Simple conjunction of two infinitely-often conditions
 * Complexity: 2 APs, minimal nesting
 * G(F("p0")) & G(F("p2"))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton1() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p2\")))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_infinite_1.dot");
    return buchi;
}

/**
 * Test 2: Nested Next Operators with Sequencing
 * Combines infinitely-often with chained next operators
 * Complexity: 4 APs, X nesting chain
 * G(F("p0" & X("p1" & X"p2"))) & G(F("p3"))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton2() {
    string ltl_str = "G(F(\"p0\" & X(\"p1\" & X\"p2\"))) & G(F(\"p3\"))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 3: Mixed Next and Until Operators
 * Combines infinitely-often with until (weak until) patterns
 * Complexity: 5 APs, mixed temporal operators
 * G(F("p0")) & G(F("p1" & X("p2"))) & G(F(!"p3" U "p4") & G(F("p3")))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton3() {
    string ltl_str = "(G(F(\"p0\")) & G(F(\"p1\" & X(\"p2\"))) & G(F(!\"p3\" U \"p4\") & G(F(\"p3\"))))";
    
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
 * Test 4: Until with Disjunctive Branching
 * Introduces disjunction at top level with complex nested structure
 * Complexity: 5 APs, disjunctive branching, until nesting
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) | G(F("p4" & X("p0"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton4() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) | G(F(\"p4\" & X(\"p0\"))))";
    
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
 * Test 5: Multiple Sequential Until Conditions
 * Deep nesting of until operators with complex boolean combinations
 * Complexity: 10 APs, multiple until chains, high nesting depth
 * G((F(!"p0" U ("p1" & F("p2"))) & G(F("p0")) & G(F("p3")) & F(!"p3" U ("p4" & F("p5"))) & F("p3") & F("p6" & X("p7")) & G(F("p8")) & G(F(!"p8" U "p9"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton5() {
    string ltl_str = "(G((F(!\"p0\" U (\"p1\" & F(\"p2\"))) & G(F(\"p0\")) & G(F(\"p3\")) & F(!\"p3\" U (\"p4\" & F(\"p5\"))) & F(\"p3\") & F(\"p6\" & X(\"p7\")) & G(F(\"p8\")) & G(F(!\"p8\" U \"p9\"))))";
    
    vector<BatchAtomicProposition> batchAPs;
    for (int i = 0; i < 10; i++) {
        uint16_t tsState = i % 6;
        bool hasGPS = (i % 2 == 0);
        vector<bool> caps(13, false);
        if (hasGPS) caps[5] = true;
        if (i % 3 == 1) caps[0] = true;
        caps[5] = true;  // All have GPS
        
        batchAPs.push_back(BatchAtomicProposition(i, tsState, caps, 0));
    }
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 7: Extended Formula with Infinitely-Often and Next Operators
 * Enhances Test 6 pattern with additional temporal constraints (p8, p9)
 * Complexity: 10 APs, includes G(F(p8)) for infinitely-often p8, X(p9) for next operator
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) & G(F("p8")) & X("p9") | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton6() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p8\") & X(\"p9\")))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(8, 3, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p8
    batchAPs.push_back(BatchAtomicProposition(9, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p9

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 8: Standardized High-Complexity Formula (Disjunctive Pattern)
 * 18 APs with until-based liveness properties and multi-level next chaining
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern throughout, disjunctive top-level
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton7() {
    string ltl_str = "(G(F(!\"p0\" U \"p1\")) & G(F(\"p0\")) & G(F(\"p2\")) & G(F(!\"p2\" U \"p3\")) & G(F(!\"p4\" U \"p5\")) & G(F(!\"p6\" U \"p7\")) & G(F(!\"p8\" U \"p9\")) & G(F(!\"p10\" U \"p11\") | F(!\"p12\" U \"p13\")) & G(F(\"p14\" & X(\"p15\" & X(\"p16\" & X(\"p17\")))))))";
    
    vector<BatchAtomicProposition> batchAPs;
    for (int i = 0; i < 18; i++) {
        uint16_t tsState = i % 6;
        vector<bool> caps(13, false);
        caps[5] = true;  // All have GPS
        if (i % 3 == 0) caps[0] = true;  // Some have movement
        if (i % 4 == 0) caps[3] = true;  // Some have camera
        
        batchAPs.push_back(BatchAtomicProposition(i, tsState, caps, 0));
    }

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_9.dot");
    return buchi;
}

/**
 * Test 6: Conjunctive-Disjunctive Mixed Operators
 * Combines multiple conjunctions with disjunction, nested until and next
 * Complexity: 8 APs, mixed conjunction/disjunction branches, deep nesting
 * G((F("p0" & X(!"p1" U "p2")))) & G(F("p1")) & (G(F("p3")) & G(F("p5")) | G(F("p4" & X("p0")) & G(F("p6" & X("p7")))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton8() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\")))))";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 9: Standardized High-Complexity Formula (Conjunctive Pattern)
 * 18 APs with until-based liveness properties, variant of Test 8 with AND instead of OR
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern throughout, conjunctive top-level
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton9() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    buchi->visualize("output/automaton_test_buchi_9.dot");
    return buchi;
}
/**
 * Test 10: Standardized High-Complexity Formula (Variant 1)
 * 18 APs with until-based liveness properties; same structure as Test 8
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern with X chaining
 * G(F(!"p0" U "p1")) & ... & G(F(!"p10" U "p11") & F(!"p12" U "p13")) & G(F("p14" & X(...X("p17"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton10() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    batchAPs.push_back(BatchAtomicProposition(11, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p11
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 11: Standardized High-Complexity Formula (Variant 2)
 * 18 APs with until-based liveness properties; same structure as Test 9
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 * G(F(!"p0" U "p1")) & ... & G(F(!"p10" U "p11") & F(!"p12" U "p13")) & G(F("p14" & X(...X("p17"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton11() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p12\") & X(\"p13\"))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    batchAPs.push_back(BatchAtomicProposition(11, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p11
    batchAPs.push_back(BatchAtomicProposition(12, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p12
    batchAPs.push_back(BatchAtomicProposition(13, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p13
    
    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

/**
 * Test 12: Standardized High-Complexity Formula (Variant 2)
 * 18 APs with until-based liveness properties; same structure as Test 9
 * Complexity: 18 APs, standardized G(F(!pX U pY)) pattern with conjunctive grouping
 * G(F(!"p0" U "p1")) & ... & G(F(!"p10" U "p11") & F(!"p12" U "p13")) & G(F("p14" & X(...X("p17"))))
 */
BuchiAutomaton* createTestInfiniteBuchiAutomaton12() {
    string ltl_str = "G((F(\"p0\" & X(!\"p1\" U \"p2\")))) & G(F(\"p1\")) & (G(F(\"p3\")) & G(F(\"p5\")) & G(F((\"p12\") & X(\"p13\"))) | G(F(\"p4\" & X(\"p0\")) & G(F(\"p6\" & X(\"p7\"))))) & F(\"p9\") & F(\"p10\") & F(\"p11\") & F(\"p14\") & (!\"p9\" U \"p10\")";
    
    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(0, 0, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p6
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p7
    batchAPs.push_back(BatchAtomicProposition(9, 3, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p9
    batchAPs.push_back(BatchAtomicProposition(10, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));  // p10
    batchAPs.push_back(BatchAtomicProposition(11, 2, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p11
    batchAPs.push_back(BatchAtomicProposition(12, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p12
    batchAPs.push_back(BatchAtomicProposition(13, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p13
    batchAPs.push_back(BatchAtomicProposition(14, 5, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));  // p14

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton13() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (!(\"p1\") U (\"p2\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

BuchiAutomaton* createTestInfiniteBuchiAutomaton14() { 
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// ADVANCED INFINITE Test 15: Complex nesting with conditional sequencing
BuchiAutomaton* createTestInfiniteBuchiAutomaton15() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (F(\"p8\")) & (!(\"p1\") U (\"p2\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(8, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}

// ============================================================================
// Create test environment with TS and GridWorld with 6 robots and 6 regions
BuchiAutomaton* createTestInfiniteBuchiAutomaton16() {
    string ltl_str = "((F(\"p1\")) & (F(\"p2\")) & (F(\"p3\")) & (F(\"p4\")) & (F(\"p5\")) & (F(\"p6\")) & (F(\"p7\")) & (F(\"p8\")))";

    vector<BatchAtomicProposition> batchAPs;
    batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(3, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(4, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(5, 5, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(6, 3, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(7, 4, {false, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    batchAPs.push_back(BatchAtomicProposition(8, 0, {true, false, false, true, false, true, false, false, false, false, false, false, false}, 0));
    

    LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
    BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
    return buchi;
}



// ============================================================================
//Create test environment with TS and GridWorld with 6 robots and 6 regions
void createTestEnvironment6(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(210, 210);
    cout << "✓ GridWorld created (210x210)" << endl;
    
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
    grid = new GridWorld(210, 210);
    cout << "✓ GridWorld created (210x210)" << endl;
    
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
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem
    mrs = new MultiRobotSystem();
    
    // Position all robots in room 0 (centered at Point(180, 140))
    Robot* r1 = new Robot(1, "Rover_1", Point(180, 140));
    r1->initializeCapabilities(13);
    r1->enableCapability(RobotCapability::SENSOR_GPS); //C
    mrs->addRobot(r1);
    
    Robot* r2 = new Robot(2, "Rover_2", Point(170, 140));
    r2->initializeCapabilities(13);
    r2->enableCapability(RobotCapability::MOVEMENT_GROUND); //A
    mrs->addRobot(r2);
    
    Robot* r3 = new Robot(3, "Rover_3", Point(190, 140));
    r3->initializeCapabilities(13);
    r3->enableCapability(RobotCapability::SENSOR_CAMERA); // B
    mrs->addRobot(r3);
    
    
    cout << "✓ MultiRobotSystem created with 3 robots" << endl;
}

// ============================================================================
//Create test environment with TS and GridWorld with 15 robots and 6 regions
void createTestEnvironment15(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(210, 210);
    cout << "✓ GridWorld created (210x210)" << endl;
    
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
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 15 robots
    mrs = new MultiRobotSystem();
    
    // Position 15 robots in a 3x5 grid, directly adjacent (1-unit spacing)
    // Grid starts at (140, 120) in room 0
    for (int i = 1; i <= 15; i++) {
        int col = (i - 1) % 3;  // 0-2 horizontal
        int row = (i - 1) / 3;  // 0-4 vertical
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
    
    cout << "✓ MultiRobotSystem created with 15 robots" << endl;
}


// ============================================================================
//Create test environment with TS and GridWorld with 15 robots and 6 regions
void createTestEnvironment45(TS*& ts, GridWorld*& grid, Environment*& env, MultiRobotSystem*& mrs) {
// Allocate GridWorld
    grid = new GridWorld(210, 210);
    cout << "✓ GridWorld created (210x210)" << endl;
    
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
    env->mapTSStateToGrid(0, Point(180, 140), 50, 140);    // State 0 centered at (180,140)
    env->mapTSStateToGrid(1, Point(180, 40), 50, 70);   // State 1 centered at (180,40)
    env->mapTSStateToGrid(2, Point(100, 100), 60, 200);   // State 2 centered at (100,100)
    env->mapTSStateToGrid(3, Point(50, 30), 50, 180);   // State 3 centered at (50,30)
    env->mapTSStateToGrid(4, Point(50, 100), 50, 110);   // State 4 centered at (50,100)
    env->mapTSStateToGrid(5, Point(50, 150), 50, 40);   // State 5 centered at (50,150)
    cout << "✓ Mapped 6 states to grid regions" << endl;
    
    // Create MultiRobotSystem with 45 robots
    mrs = new MultiRobotSystem();
    
    // Position 45 robots in a 9x5 grid, all in room 0
    // Grid starts at (135, 120), directly adjacent (1-unit spacing)
    for (int i = 1; i <= 45; i++) {
        int col = (i - 1) % 9;  // 0-8 horizontal
        int row = (i - 1) / 9;  // 0-4 vertical
        int x = 135 + col;
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
    
    cout << "✓ MultiRobotSystem created with 45 robots" << endl;
}

