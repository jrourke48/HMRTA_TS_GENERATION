#include "AlgorithmMetrics.h"
#include <iostream>
#include <vector>

/**
 * @file AlgorithmMetrics_Example.cpp
 * @brief Example usage of AlgorithmMetrics class for thesis evaluation
 * 
 * This file demonstrates how to integrate AlgorithmMetrics into your
 * task allocation algorithm for comprehensive performance evaluation.
 */

// Placeholder for your actual algorithm
class TaskAllocationAlgorithm {
public:
    // Mock function - replace with your actual algorithm
    void solve(int num_robots, int num_states, int num_regions) {
        // Simulate work
        for (int i = 0; i < 1000000; i++) {
            volatile int x = i * i;
        }
    }
};

// ==================== EXAMPLE 1: BASIC SINGLE RUN ====================
void example_basic_single_run() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 1: Basic Single Algorithm Run" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    
    // Define problem instance
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = 10;
    vars.num_automaton_states = 16;
    vars.num_automaton_edges = 32;
    vars.num_atomic_propositions = 5;
    vars.total_robot_capabilities = 25;
    vars.num_ts_regions = 20;
    vars.num_inter_task_constraints = 3;
    
    metrics.setIndependentVariables(vars);
    
    // Run algorithm with timing
    TaskAllocationAlgorithm algorithm;
    metrics.startTimer();
    algorithm.solve(vars.num_robots, vars.num_automaton_states, vars.num_ts_regions);
    metrics.stopTimer();
    
    // Record correctness metrics
    AlgorithmMetrics::CorrectnessMetrics correctness;
    correctness.ltl_satisfaction_rate = 1.0;
    correctness.capability_satisfaction_rate = 1.0;
    correctness.inter_task_constraint_violations = 0;
    correctness.acceptance_condition_reached = true;
    correctness.feasibility_status = "FEASIBLE";
    metrics.recordCorrectness(correctness);
    
    // Record subtree efficiency metrics
    AlgorithmMetrics::SubtreeEfficiencyMetrics efficiency;
    efficiency.total_nodes_generated = 1542;
    efficiency.total_nodes_expanded = 856;
    efficiency.total_nodes_pruned = 686;
    efficiency.nodes_satisfying_ltl = 42;
    efficiency.tree_memory_bytes = 2048000;
    efficiency.product_automaton_memory_bytes = 8192000;
    efficiency.full_product_automaton_nodes = 5200;
    metrics.recordSubtreeEfficiency(efficiency);
    
    // Record solution quality metrics
    AlgorithmMetrics::SolutionQualityMetrics quality;
    quality.makespan_seconds = 45.32;
    quality.sum_of_travel_times_seconds = 287.44;
    quality.total_travel_distance = 156.8;
    quality.max_individual_travel_distance = 28.5;
    quality.robots_utilized = 9;
    quality.tasks_per_robot = {5, 5, 4, 4, 3, 3, 2, 2, 1, 0};
    quality.individual_travel_times = {45.3, 44.8, 38.9, 37.2, 28.1, 26.5, 15.3, 12.4, 8.9, 0.0};
    quality.individual_travel_distances = {28.5, 27.9, 23.4, 21.8, 15.6, 14.2, 8.1, 6.3, 4.0, 0.0};
    metrics.recordSolutionQuality(quality);
    
    // Compute derived metrics and display
    metrics.computeDerivedMetrics();
    metrics.printSummary();
}

// ==================== EXAMPLE 2: RUNTIME VS ROBOTS ====================
void example_scalability_robots() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 2: Runtime Scalability - Varying Number of Robots" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    std::vector<int> robot_counts = {5, 10, 20, 50, 100};
    
    for (int num_robots : robot_counts) {
        AlgorithmMetrics::IndependentVariables vars;
        vars.num_robots = num_robots;
        vars.num_automaton_states = 16;
        vars.num_automaton_edges = 32;
        vars.num_atomic_propositions = 5;
        vars.total_robot_capabilities = num_robots * 2.5;  // ~2.5 cap/robot
        vars.num_ts_regions = 20;
        vars.num_inter_task_constraints = 3;
        
        metrics.setIndependentVariables(vars);
        
        TaskAllocationAlgorithm algorithm;
        metrics.startTimer();
        algorithm.solve(num_robots, 16, 20);
        metrics.stopTimer();
        
        double runtime = metrics.getRuntime().total_computation_time_ms;
        metrics.addRuntimeVsRobots(num_robots, runtime);
        
        std::cout << "N=" << num_robots << " robots: " << runtime << " ms" << std::endl;
    }
    
    std::cout << "\n✓ Scalability data collected. Use for plotting runtime(N) curve." << std::endl;
}

// ==================== EXAMPLE 3: RUNTIME VS AUTOMATON STATES ====================
void example_scalability_automaton() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 3: Runtime Scalability - Varying Automaton States" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    std::vector<int> state_counts = {4, 8, 16, 32, 64, 128};
    
    for (int num_states : state_counts) {
        AlgorithmMetrics::IndependentVariables vars;
        vars.num_robots = 10;
        vars.num_automaton_states = num_states;
        vars.num_automaton_edges = num_states * 2;  // Approximate edges
        vars.num_atomic_propositions = 5;
        vars.total_robot_capabilities = 25;
        vars.num_ts_regions = 20;
        vars.num_inter_task_constraints = 3;
        
        metrics.setIndependentVariables(vars);
        
        TaskAllocationAlgorithm algorithm;
        metrics.startTimer();
        algorithm.solve(10, num_states, 20);
        metrics.stopTimer();
        
        double runtime = metrics.getRuntime().total_computation_time_ms;
        metrics.addRuntimeVsAutomatonStates(num_states, runtime);
        
        std::cout << "|S_B|=" << num_states << ": " << runtime << " ms" << std::endl;
    }
    
    std::cout << "\n✓ Automaton scalability data collected." << std::endl;
    std::cout << "  The original algorithm scales ~O(|S_B|^2) under certain assumptions." << std::endl;
}

// ==================== EXAMPLE 4: MULTI-PARAMETER STUDY ====================
void example_multi_parameter_study() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 4: Multi-Parameter Efficiency Study" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    
    // Test different capability densities
    std::vector<double> densities = {0.25, 0.50, 0.75, 1.00};
    
    for (double density : densities) {
        int total_capabilities = static_cast<int>(10 * 5 * density);  // 10 robots * 5 base * density
        
        AlgorithmMetrics::IndependentVariables vars;
        vars.num_robots = 10;
        vars.num_automaton_states = 16;
        vars.num_automaton_edges = 32;
        vars.num_atomic_propositions = 5;
        vars.total_robot_capabilities = total_capabilities;
        vars.num_ts_regions = 20;
        vars.num_inter_task_constraints = 3;
        
        metrics.setIndependentVariables(vars);
        
        TaskAllocationAlgorithm algorithm;
        metrics.startTimer();
        algorithm.solve(10, 16, 20);
        metrics.stopTimer();
        
        double runtime = metrics.getRuntime().total_computation_time_ms;
        metrics.addRuntimeVsCapabilityDensity(density, runtime);
        
        std::cout << "Capability Density=" << density << ": " << runtime << " ms" << std::endl;
    }
    
    std::cout << "\n✓ Capability density impact measured." << std::endl;
}

// ==================== EXAMPLE 5: FULL PRODUCT COMPARISON ====================
void example_full_product_comparison() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 5: Tree Search vs Full Product Automaton" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = 5;
    vars.num_automaton_states = 8;
    vars.num_automaton_edges = 16;
    vars.num_atomic_propositions = 4;
    vars.total_robot_capabilities = 12;
    vars.num_ts_regions = 10;
    vars.num_inter_task_constraints = 2;
    
    metrics.setIndependentVariables(vars);
    
    // Run tree-search algorithm
    TaskAllocationAlgorithm algorithm;
    metrics.startTimer();
    algorithm.solve(5, 8, 10);
    metrics.stopTimer();
    
    // Record efficiency metrics
    AlgorithmMetrics::SubtreeEfficiencyMetrics efficiency;
    efficiency.total_nodes_generated = 856;
    efficiency.total_nodes_expanded = 542;
    efficiency.total_nodes_pruned = 314;
    efficiency.tree_memory_bytes = 1024000;
    
    // Full product automaton (for small instances, we can compute this)
    efficiency.full_product_automaton_nodes = 5 * 8 * 10;  // approx: robots * states * regions
    efficiency.product_automaton_memory_bytes = 8192000;
    
    metrics.recordSubtreeEfficiency(efficiency);
    metrics.computeDerivedMetrics();
    
    const auto& eff = metrics.getSubtreeEfficiency();
    
    std::cout << "TREE-SEARCH RESULTS:" << std::endl;
    std::cout << "  Total Nodes Generated: " << eff.total_nodes_generated << std::endl;
    std::cout << "  Total Nodes Pruned: " << eff.total_nodes_pruned << std::endl;
    std::cout << "  Pruning Ratio: " << eff.pruning_ratio << " (" 
              << (eff.pruning_ratio * 100) << "%)" << std::endl;
    
    std::cout << "\nCOMPARISON METRICS:" << std::endl;
    std::cout << "  Full Product Nodes: " << eff.full_product_automaton_nodes << std::endl;
    std::cout << "  Tree Nodes: " << eff.total_nodes_generated << std::endl;
    std::cout << "  State-Space Reduction: " << eff.state_space_reduction << " nodes" << std::endl;
    std::cout << "  Explored-Product Ratio: " << eff.explored_product_ratio << std::endl;
    std::cout << "  Memory Reduction Ratio: " << eff.memory_reduction_ratio << " (lower is better)" << std::endl;
    std::cout << "  Runtime Speedup: " << eff.runtime_speedup_percent << "%" << std::endl;
}

// ==================== EXAMPLE 6: DETAILED REPORT WITH EXPORT ====================
void example_report_and_export() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "EXAMPLE 6: Generate Reports and Export Data" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    AlgorithmMetrics metrics;
    
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = 15;
    vars.num_automaton_states = 32;
    vars.num_automaton_edges = 64;
    vars.num_atomic_propositions = 7;
    vars.total_robot_capabilities = 45;
    vars.num_ts_regions = 30;
    vars.num_inter_task_constraints = 5;
    
    metrics.setIndependentVariables(vars);
    
    TaskAllocationAlgorithm algorithm;
    metrics.startTimer();
    algorithm.solve(15, 32, 30);
    metrics.stopTimer();
    
    AlgorithmMetrics::CorrectnessMetrics correctness;
    correctness.ltl_satisfaction_rate = 0.95;
    correctness.capability_satisfaction_rate = 1.0;
    correctness.inter_task_constraint_violations = 0;
    correctness.acceptance_condition_reached = true;
    correctness.feasibility_status = "FEASIBLE";
    metrics.recordCorrectness(correctness);
    
    AlgorithmMetrics::SubtreeEfficiencyMetrics efficiency;
    efficiency.total_nodes_generated = 3204;
    efficiency.total_nodes_pruned = 2105;
    efficiency.full_product_automaton_nodes = 14400;
    efficiency.tree_memory_bytes = 5242880;
    efficiency.product_automaton_memory_bytes = 67108864;
    metrics.recordSubtreeEfficiency(efficiency);
    
    AlgorithmMetrics::SolutionQualityMetrics quality;
    quality.makespan_seconds = 78.45;
    quality.sum_of_travel_times_seconds = 542.30;
    quality.total_travel_distance = 289.5;
    quality.max_individual_travel_distance = 35.2;
    quality.robots_utilized = 14;
    quality.tasks_per_robot = {7, 7, 6, 6, 5, 5, 5, 4, 4, 3, 3, 2, 1, 1, 0};
    metrics.recordSolutionQuality(quality);
    
    metrics.computeDerivedMetrics();
    
    // Print detailed report
    metrics.printDetailedReport();
    
    // Export to files
    metrics.exportToCSV("c:\\temp\\algorithm_metrics.csv");
    metrics.exportToJSON("c:\\temp\\algorithm_metrics.json");
    
    std::cout << "\n✓ Reports generated and exported to:\n"
              << "  CSV: c:\\temp\\algorithm_metrics.csv\n"
              << "  JSON: c:\\temp\\algorithm_metrics.json" << std::endl;
}

// ==================== MAIN ====================

int main() {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "ALGORITHM METRICS - COMPREHENSIVE USAGE EXAMPLES" << std::endl;
    std::cout << std::string(80, '=') << std::endl;
    
    // Run all examples
    example_basic_single_run();
    example_scalability_robots();
    example_scalability_automaton();
    example_multi_parameter_study();
    example_full_product_comparison();
    example_report_and_export();
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "ALL EXAMPLES COMPLETED" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    return 0;
}
