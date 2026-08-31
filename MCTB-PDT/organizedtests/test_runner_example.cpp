#include "TestRunManager.h"
#include "TaskAllocationAlgorithms.h"
#include <iostream>

/**
 * @file test_runner_example.cpp
 * @brief Example usage of TestRunManager for organizing thesis tests
 * 
 * Usage pattern:
 * 1. Create TestRunManager instance
 * 2. Initialize directory structure
 * 3. Run test suite with different configurations
 * 4. Store results by category
 * 5. Export for analysis/plotting
 */

int main() {
    // ==================== SETUP ====================
    TestRunManager manager("./test_results");
    manager.initialize();
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "RUNNING COMPREHENSIVE TEST SUITE" << std::endl;
    std::cout << std::string(70, '=') << "\n" << std::endl;
    
    // ==================== TEST 1: AUTOMATON STATE SCALING ====================
    {
        std::cout << "\n[TEST 1] Automaton State Scaling (5-150 states)\n";
        
        std::vector<int> state_counts = {5, 10, 15, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120, 140, 150};
        
        for (int states : state_counts) {
            for (int trial = 1; trial <= 3; ++trial) {  // 3 trials per configuration
                try {
                    // Create automaton with 'states' states
                    // TODO: createTestBuchiAutomaton(states)
                    BuchiAutomaton* buchi = nullptr;  // = createTestBuchiAutomaton(states);
                    
                    // Run algorithm
                    // TODO: allocAlg->intensiveInterTaskRelationshipTreeSearch(...)
                    AlgorithmMetrics metrics;
                    // metrics = allocAlg->getMetrics();
                    
                    // Store result
                    std::map<std::string, std::string> params;
                    params["num_automaton_states"] = std::to_string(states);
                    params["num_robots"] = "6";           // Fixed
                    params["num_regions"] = "6";          // Fixed
                    params["avg_capabilities"] = "2";     // Fixed
                    
                    manager.storeRun(TestRunManager::TestCategory::AUTOMATON_STATES,
                                    metrics,
                                    params,
                                    trial);
                    
                    // delete buchi;
                    // delete allocAlg;
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error in automaton states test (states=" << states 
                              << ", trial=" << trial << "): " << e.what() << std::endl;
                }
            }
        }
    }
    
    // ==================== TEST 2: NUM_ROBOTS SCALING ====================
    {
        std::cout << "\n[TEST 2] Robot Count Scaling (3-20 robots)\n";
        
        std::vector<int> robot_counts = {3, 4, 5, 6, 7, 8, 10, 12, 15, 17, 20};
        
        for (int robots : robot_counts) {
            for (int trial = 1; trial <= 3; ++trial) {
                try {
                    // Create environment with 'robots' robots
                    // TODO: createTestEnvironment(robots)
                    AlgorithmMetrics metrics;
                    // metrics = runAlgorithm(robots);
                    
                    std::map<std::string, std::string> params;
                    params["num_robots"] = std::to_string(robots);
                    params["num_automaton_states"] = "50";  // Fixed
                    params["num_regions"] = "6";            // Fixed
                    
                    manager.storeRun(TestRunManager::TestCategory::NUM_ROBOTS,
                                    metrics,
                                    params,
                                    trial);
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error in robot count test (robots=" << robots 
                              << "): " << e.what() << std::endl;
                }
            }
        }
    }
    
    // ==================== TEST 3: TS_REGIONS SCALING ====================
    {
        std::cout << "\n[TEST 3] Transition System Regions (5-40 regions)\n";
        
        std::vector<int> region_counts = {5, 7, 10, 12, 15, 18, 20, 25, 30, 35, 40};
        
        for (int regions : region_counts) {
            for (int trial = 1; trial <= 2; ++trial) {
                try {
                    // Create TS with 'regions' regions
                    AlgorithmMetrics metrics;
                    
                    std::map<std::string, std::string> params;
                    params["num_regions"] = std::to_string(regions);
                    params["num_robots"] = "6";            // Fixed
                    params["num_automaton_states"] = "50";  // Fixed
                    
                    manager.storeRun(TestRunManager::TestCategory::TS_REGIONS,
                                    metrics,
                                    params,
                                    trial);
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error in TS regions test (regions=" << regions 
                              << "): " << e.what() << std::endl;
                }
            }
        }
    }
    
    // ==================== TEST 4: AVERAGE CAPABILITIES ====================
    {
        std::cout << "\n[TEST 4] Average Capabilities Variation (1-5)\n";
        
        std::vector<double> avg_caps = {1.0, 1.4, 1.8, 2.2, 2.6, 3.0, 3.4, 3.8, 4.4, 5.0};
        
        for (double avg_cap : avg_caps) {
            for (int trial = 1; trial <= 2; ++trial) {
                try {
                    // Create robot fleet with avg_cap capabilities per robot
                    AlgorithmMetrics metrics;
                    
                    std::map<std::string, std::string> params;
                    params["avg_capabilities"] = std::to_string(avg_cap).substr(0, 3);
                    params["num_robots"] = "10";           // Fixed
                    params["num_automaton_states"] = "50";  // Fixed
                    
                    manager.storeRun(TestRunManager::TestCategory::AVG_CAPABILITIES,
                                    metrics,
                                    params,
                                    trial);
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error in avg capabilities test (avg=" << avg_cap 
                              << "): " << e.what() << std::endl;
                }
            }
        }
    }
    
    // ==================== TEST 5: ROBOT HOMOGENEITY ====================
    {
        std::cout << "\n[TEST 5] Robot Homogeneity Variation (0.2-3)\n";
        
        std::vector<double> homogeneities = {0.2, 0.4, 0.6, 0.8, 1.0, 1.3, 1.6, 1.9, 2.3, 2.7, 3.0};
        
        for (double homogen : homogeneities) {
            for (int trial = 1; trial <= 2; ++trial) {
                try {
                    // Create robot fleet with 'homogen' homogeneity index
                    AlgorithmMetrics metrics;
                    
                    std::map<std::string, std::string> params;
                    params["homogeneity"] = std::to_string(homogen).substr(0, 3);
                    params["num_robots"] = "6";            // Fixed
                    params["num_automaton_states"] = "50";  // Fixed
                    
                    manager.storeRun(TestRunManager::TestCategory::ROBOT_HOMOGENEITY,
                                    metrics,
                                    params,
                                    trial);
                    
                } catch (const std::exception& e) {
                    std::cerr << "Error in homogeneity test (h=" << homogen 
                              << "): " << e.what() << std::endl;
                }
            }
        }
    }
    
    // ==================== ANALYSIS & EXPORT ====================
    
    std::cout << "\n" << std::string(70, '=') << std::endl;
    std::cout << "TEST EXECUTION COMPLETE" << std::endl;
    std::cout << std::string(70, '=') << "\n" << std::endl;
    
    // Print progress summary
    manager.printTestProgress();
    
    // Export all results
    std::cout << "\nExporting results for analysis...\n" << std::endl;
    manager.exportAllToCSV();
    
    // Export statistics for plotting
    manager.exportStatisticsToCSV(TestRunManager::TestCategory::AUTOMATON_STATES, 
                                 "./test_results/exports/automaton_states_stats.csv");
    manager.exportStatisticsToCSV(TestRunManager::TestCategory::NUM_ROBOTS, 
                                 "./test_results/exports/num_robots_stats.csv");
    manager.exportStatisticsToCSV(TestRunManager::TestCategory::TS_REGIONS, 
                                 "./test_results/exports/ts_regions_stats.csv");
    manager.exportStatisticsToCSV(TestRunManager::TestCategory::AVG_CAPABILITIES, 
                                 "./test_results/exports/avg_capabilities_stats.csv");
    manager.exportStatisticsToCSV(TestRunManager::TestCategory::ROBOT_HOMOGENEITY, 
                                 "./test_results/exports/robot_homogeneity_stats.csv");
    
    // Export summary report
    manager.exportSummaryReport("./test_results/summary_report.md");
    
    std::cout << "\n✓ All results exported to ./test_results/exports/\n" << std::endl;
    
    return 0;
}
