#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <sstream>

// Master Test Runner - Executes all independent variable tests
// This program orchestrates the execution of all test suites and aggregates results

int main(int argc, char* argv[]) {
    std::cout << "======================================" << std::endl;
    std::cout << "HMRTA Test Suite Master Runner" << std::endl;
    std::cout << "======================================\n" << std::endl;
    
    // Define all test programs
    std::vector<std::string> testPrograms = {
        "test_automaton_states",
        "test_number_robots",
        "test_transition_system_regions",
        "test_average_capabilities",
        "test_robot_homogeneity"
    };
    
    std::vector<std::string> resultFiles = {
        "automaton_states_results.csv",
        "number_robots_results.csv",
        "transition_system_regions_results.csv",
        "average_capabilities_results.csv",
        "robot_homogeneity_results.csv"
    };
    
    // Record overall start time
    auto overallStart = std::chrono::high_resolution_clock::now();
    
    // Execute each test suite
    for (size_t i = 0; i < testPrograms.size(); ++i) {
        std::cout << "\n[" << (i+1) << "/" << testPrograms.size() << "] Running: " 
                  << testPrograms[i] << std::endl;
        std::cout << "-------------------------------------------" << std::endl;
        
        // Construct command to run test
        std::string command = "./" + testPrograms[i];
        
        // Execute test program
        auto start = std::chrono::high_resolution_clock::now();
        int result = system(command.c_str());
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::seconds>(end - start);
        
        if (result == 0) {
            std::cout << "✓ Completed in " << duration.count() << " seconds" << std::endl;
        } else {
            std::cout << "✗ FAILED - Exit code: " << result << std::endl;
        }
    }
    
    // Record overall end time
    auto overallEnd = std::chrono::high_resolution_clock::now();
    auto totalDuration = std::chrono::duration_cast<std::chrono::seconds>(overallEnd - overallStart);
    
    // Aggregate results
    std::cout << "\n======================================" << std::endl;
    std::cout << "Test Execution Complete" << std::endl;
    std::cout << "======================================" << std::endl;
    std::cout << "Total execution time: " << totalDuration.count() << " seconds" << std::endl;
    std::cout << "\nGenerated Result Files:" << std::endl;
    for (const auto& file : resultFiles) {
        std::cout << "  - " << file << std::endl;
    }
    
    // Create master results summary
    std::ofstream summaryFile("test_run_summary.txt");
    summaryFile << "HMRTA Test Suite Execution Summary\n";
    summaryFile << "==================================\n";
    summaryFile << "Total execution time: " << totalDuration.count() << " seconds\n";
    summaryFile << "\nTest suites executed:\n";
    for (size_t i = 0; i < testPrograms.size(); ++i) {
        summaryFile << "  " << (i+1) << ". " << testPrograms[i] << " -> " << resultFiles[i] << "\n";
    }
    summaryFile << "\nTo plot results, use:\n";
    summaryFile << "  python plot_results.py\n";
    summaryFile.close();
    
    std::cout << "\nSummary saved to: test_run_summary.txt" << std::endl;
    std::cout << "\nNext steps:" << std::endl;
    std::cout << "1. Review the generated CSV files for raw data" << std::endl;
    std::cout << "2. Run: python plot_results.py" << std::endl;
    std::cout << "3. Check generated plots in ./plots/ directory" << std::endl;
    
    return 0;
}
