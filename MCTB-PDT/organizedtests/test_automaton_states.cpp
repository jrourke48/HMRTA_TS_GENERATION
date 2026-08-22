#include <iostream>
#include <vector>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <cstring>

// Test: Number of Automaton States (5-150 states)
// Fixed: 6 robots, 6 regions, 3 different Buchi automata

int main() {
    std::cout << "=== Test Suite: Automaton States ===" << std::endl;
    std::cout << "Variable: Number of automaton states" << std::endl;
    std::cout << "Range: 5-150 states" << std::endl;
    std::cout << "Fixed: 6 robots, 6 regions" << std::endl;
    std::cout << "\n";

    // Test configurations
    std::vector<int> automatonStates = {5, 15, 25, 35, 45, 55, 65, 75, 85, 95, 105, 115, 125, 135, 150};
    
    // Data collection
    std::ofstream resultsFile("automaton_states_results.csv");
    resultsFile << "Test#,States,Computation_Time_ms,Memory_MB,Quality_Score,Status\n";
    
    int testNum = 1;
    for (int states : automatonStates) {
        std::cout << "Test " << testNum << ": " << states << " automaton states... ";
        
        // TODO: Call TestingAutomaton with states parameter
        // auto start = std::chrono::high_resolution_clock::now();
        // int result = runTestWithAutomatonStates(states, 6, 6);
        // auto end = std::chrono::high_resolution_clock::now();
        // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "PENDING" << std::endl;
        // resultsFile << testNum << "," << states << ",0,0,0,PENDING\n";
        
        testNum++;
    }
    
    resultsFile.close();
    std::cout << "\nResults saved to automaton_states_results.csv" << std::endl;
    
    return 0;
}
