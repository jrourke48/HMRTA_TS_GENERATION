#include "AlgorithmMetrics.h"
#include <iostream>
#include <iomanip>
#include <fstream>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <string>
#include <vector>

// ==================== INITIALIZATION ====================

void AlgorithmMetrics::setIndependentVariables(const IndependentVariables& vars) {
    iv_ = vars;
    
    // Compute derived independent variables
    if (vars.num_robots > 0) {
        iv_.avg_capabilities_per_robot = 
            static_cast<double>(vars.total_robot_capabilities) / vars.num_robots;
        iv_.capability_homogeneity = 
            static_cast<double>(vars.total_robot_capabilities) / vars.num_robots;
    }
}

// ==================== TIMER MANAGEMENT ====================

void AlgorithmMetrics::startTimer() {
    overall_start_time_ = std::chrono::high_resolution_clock::now();
}

void AlgorithmMetrics::stopTimer() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - overall_start_time_);
    runtime_.total_computation_time_ms = duration.count();
}

void AlgorithmMetrics::recordHighLevelStart() {
    high_level_start_time_ = std::chrono::high_resolution_clock::now();
}

void AlgorithmMetrics::recordHighLevelStop() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - high_level_start_time_);
    runtime_.high_level_runtime_ms = duration.count();
}

void AlgorithmMetrics::recordLowLevelStart() {
    low_level_start_time_ = std::chrono::high_resolution_clock::now();
}

void AlgorithmMetrics::recordLowLevelStop() {
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        end_time - low_level_start_time_);
    runtime_.low_level_runtime_ms = duration.count();
}

// ==================== RECORD METRICS ====================

void AlgorithmMetrics::recordCorrectness(const CorrectnessMetrics& correctness) {
    correctness_ = correctness;
}

void AlgorithmMetrics::recordSubtreeEfficiency(const SubtreeEfficiencyMetrics& efficiency) {
    subtree_efficiency_ = efficiency;
}

void AlgorithmMetrics::recordSolutionQuality(const SolutionQualityMetrics& quality) {
    solution_quality_ = quality;
}

// ==================== DERIVED METRICS COMPUTATION ====================

double AlgorithmMetrics::computePruningRatio() const {
    if (subtree_efficiency_.total_nodes_generated == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_pruned) / 
           subtree_efficiency_.total_nodes_generated;
}

double AlgorithmMetrics::computeExploredProductRatio() const {
    if (subtree_efficiency_.full_product_automaton_nodes == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_generated) / 
           subtree_efficiency_.full_product_automaton_nodes;
}

double AlgorithmMetrics::computeTreeProductRatio() const {
    if (subtree_efficiency_.full_product_automaton_nodes == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.total_nodes_generated) / 
           subtree_efficiency_.full_product_automaton_nodes;
}

double AlgorithmMetrics::computeMemoryReductionRatio() const {
    if (subtree_efficiency_.product_automaton_memory_bytes == 0) return 0.0;
    return static_cast<double>(subtree_efficiency_.tree_memory_bytes) / 
           subtree_efficiency_.product_automaton_memory_bytes;
}

double AlgorithmMetrics::computeRobotUtilizationRatio() const {
    if (iv_.num_robots == 0) return 0.0;
    return static_cast<double>(solution_quality_.robots_utilized) / iv_.num_robots;
}

double AlgorithmMetrics::computeLoadBalanceVariance() const {
    if (solution_quality_.tasks_per_robot.empty()) return 0.0;
    
    double mean = std::accumulate(solution_quality_.tasks_per_robot.begin(),
                                   solution_quality_.tasks_per_robot.end(), 0.0) / 
                  solution_quality_.tasks_per_robot.size();
    
    double sq_sum = 0.0;
    for (int tasks : solution_quality_.tasks_per_robot) {
        sq_sum += (tasks - mean) * (tasks - mean);
    }
    return sq_sum / solution_quality_.tasks_per_robot.size();
}

void AlgorithmMetrics::computeDerivedMetrics() {
    // Subtree efficiency derived metrics
    subtree_efficiency_.pruning_ratio = computePruningRatio();
    subtree_efficiency_.explored_product_ratio = computeExploredProductRatio();
    subtree_efficiency_.tree_product_ratio = computeTreeProductRatio();
    subtree_efficiency_.memory_reduction_ratio = computeMemoryReductionRatio();
    subtree_efficiency_.state_space_reduction = 
        subtree_efficiency_.full_product_automaton_nodes - 
        subtree_efficiency_.total_nodes_generated;
    
    // Solution quality derived metrics
    solution_quality_.robot_utilization_ratio = computeRobotUtilizationRatio();
    solution_quality_.load_balance_variance = computeLoadBalanceVariance();
}

// ==================== BATCH RUNTIME PARAMETER UPDATES ====================

void AlgorithmMetrics::addRuntimeVsRobots(int num_robots, double time_ms) {
    runtime_.runtime_vs_num_robots.push_back({num_robots, time_ms});
}

void AlgorithmMetrics::addRuntimeVsAutomatonStates(int states, double time_ms) {
    runtime_.runtime_vs_automaton_states.push_back({states, time_ms});
}

void AlgorithmMetrics::addRuntimeVsAutomatonEdges(int edges, double time_ms) {
    runtime_.runtime_vs_automaton_edges.push_back({edges, time_ms});
}

void AlgorithmMetrics::addRuntimeVsCapabilityDensity(double density, double time_ms) {
    runtime_.runtime_vs_capability_density.push_back({density, time_ms});
}

void AlgorithmMetrics::addRuntimeVsCapabilityHomogeneity(double homogeneity, double time_ms) {
    runtime_.runtime_vs_capability_homogeneity.push_back({homogeneity, time_ms});
}

void AlgorithmMetrics::addRuntimeVsAtomicPropositions(int ap, double time_ms) {
    runtime_.runtime_vs_atomic_propositions.push_back({ap, time_ms});
}

void AlgorithmMetrics::addRuntimeVsEnvironmentSize(int env_size, double time_ms) {
    runtime_.runtime_vs_environment_size.push_back({env_size, time_ms});
}

void AlgorithmMetrics::addRuntimeVsRegions(int regions, double time_ms) {
    runtime_.runtime_vs_ts_regions.push_back({regions, time_ms});
}

// ==================== REPORTING ====================

void AlgorithmMetrics::printSummary() const {
    std::cout << "\n" << std::string(80, '=') << std::endl;
    std::cout << "ALGORITHM METRICS SUMMARY" << std::endl;
    std::cout << std::string(80, '=') << "\n" << std::endl;
    
    // Independent Variables
    std::cout << "INDEPENDENT VARIABLES:" << std::endl;
    std::cout << "  Automaton States: " << iv_.num_automaton_states << std::endl;
    std::cout << "  Automaton Edges: " << iv_.num_automaton_edges << std::endl;
    std::cout << "  Atomic Propositions: " << iv_.num_atomic_propositions << std::endl;
    std::cout << "  Number of Robots: " << iv_.num_robots << std::endl;
    std::cout << "  Total Robot Capabilities: " << iv_.total_robot_capabilities << std::endl;
    std::cout << "  Transition System Regions: " << iv_.num_ts_regions << std::endl;
    std::cout << "  Avg Capabilities/Robot: " << std::fixed << std::setprecision(2) 
              << iv_.avg_capabilities_per_robot << std::endl;
    std::cout << "  Capability Homogeneity: " << iv_.capability_homogeneity << std::endl;
    std::cout << "  Inter-task Constraints: " << iv_.num_inter_task_constraints << "\n" << std::endl;
    
    // Correctness Metrics
    std::cout << "CORRECTNESS METRICS:" << std::endl;
    std::cout << "  LTL Satisfaction Rate: " << std::fixed << std::setprecision(4) 
              << correctness_.ltl_satisfaction_rate << std::endl;
    std::cout << "  Capability Satisfaction Rate: " << correctness_.capability_satisfaction_rate << std::endl;
    std::cout << "  Constraint Violations: " << correctness_.inter_task_constraint_violations << std::endl;
    std::cout << "  Acceptance Reached: " << (correctness_.acceptance_condition_reached ? "YES" : "NO") << std::endl;
    std::cout << "  Feasibility: " << correctness_.feasibility_status << "\n" << std::endl;
    
    // Runtime Metrics
    std::cout << "RUNTIME METRICS:" << std::endl;
    std::cout << "  Total Computation Time: " << std::fixed << std::setprecision(2) 
              << runtime_.total_computation_time_ms << " ms" << std::endl;
    std::cout << "  High-Level Runtime: " << runtime_.high_level_runtime_ms << " ms" << std::endl;
    std::cout << "  Low-Level Runtime: " << runtime_.low_level_runtime_ms << " ms" << "\n" << std::endl;
    
    // Subtree Efficiency
    std::cout << "SUBTREE EFFICIENCY METRICS:" << std::endl;
    std::cout << "  Total Nodes Generated: " << subtree_efficiency_.total_nodes_generated << std::endl;
    std::cout << "  Total Nodes Expanded: " << subtree_efficiency_.total_nodes_expanded << std::endl;
    std::cout << "  Total Nodes Pruned: " << subtree_efficiency_.total_nodes_pruned << std::endl;
    std::cout << "  Pruning Ratio: " << std::fixed << std::setprecision(4) 
              << subtree_efficiency_.pruning_ratio << std::endl;
    std::cout << "  Memory Reduction Ratio: " << subtree_efficiency_.memory_reduction_ratio << std::endl;
    std::cout << "  State-Space Reduction: " << subtree_efficiency_.state_space_reduction << "\n" << std::endl;
    
    // Solution Quality
    std::cout << "SOLUTION QUALITY METRICS:" << std::endl;
    std::cout << "  Makespan: " << std::fixed << std::setprecision(2) 
              << solution_quality_.makespan_seconds << " sec" << std::endl;
    std::cout << "  Sum of Travel Times: " << solution_quality_.sum_of_travel_times_seconds << " sec" << std::endl;
    std::cout << "  Total Travel Distance: " << solution_quality_.total_travel_distance << std::endl;
    std::cout << "  Max Individual Distance: " << solution_quality_.max_individual_travel_distance << std::endl;
    std::cout << "  Robots Utilized: " << solution_quality_.robots_utilized << " / " 
              << iv_.num_robots << std::endl;
    std::cout << "  Robot Utilization Ratio: " << std::fixed << std::setprecision(4) 
              << solution_quality_.robot_utilization_ratio << std::endl;
    std::cout << "  Load Balance Variance: " << solution_quality_.load_balance_variance << std::endl;
    
    std::cout << "\n" << std::string(80, '=') << std::endl;
}

void AlgorithmMetrics::printDetailedReport() const {
    printSummary();
    
    std::cout << "\nDETAILED RUNTIME ANALYSIS:" << std::endl;
    std::string separator(80, '-');
    std::cout << separator << std::endl;
    
    // Runtime vs robots
    if (!runtime_.runtime_vs_num_robots.empty()) {
        std::cout << "Runtime vs Number of Robots:" << std::endl;
        for (const auto& entry : runtime_.runtime_vs_num_robots) {
            std::cout << "  N=" << entry.first << ": " << std::fixed << std::setprecision(2) 
                      << entry.second << " ms" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Runtime vs automaton states
    if (!runtime_.runtime_vs_automaton_states.empty()) {
        std::cout << "Runtime vs Automaton States:" << std::endl;
        for (const auto& entry : runtime_.runtime_vs_automaton_states) {
            std::cout << "  |S_B|=" << entry.first << ": " << std::fixed << std::setprecision(2) 
                      << entry.second << " ms" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Runtime vs automaton edges
    if (!runtime_.runtime_vs_automaton_edges.empty()) {
        std::cout << "Runtime vs Automaton Edges:" << std::endl;
        for (const auto& entry : runtime_.runtime_vs_automaton_edges) {
            std::cout << "  Edges=" << entry.first << ": " << std::fixed << std::setprecision(2) 
                      << entry.second << " ms" << std::endl;
        }
        std::cout << std::endl;
    }
    
    // Runtime vs capability density
    if (!runtime_.runtime_vs_capability_density.empty()) {
        std::cout << "Runtime vs Capability Density:" << std::endl;
        for (const auto& entry : runtime_.runtime_vs_capability_density) {
            std::cout << "  Density=" << std::fixed << std::setprecision(2) << entry.first 
                      << ": " << entry.second << " ms" << std::endl;
        }
        std::cout << std::endl;
    }
}

void AlgorithmMetrics::exportToCSV(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << std::endl;
        return;
    }
    
    // Header
    outfile << "Metric,Value,Unit\n";
    
    // Independent Variables
    outfile << "Automaton States," << iv_.num_automaton_states << ",count\n";
    outfile << "Automaton Edges," << iv_.num_automaton_edges << ",count\n";
    outfile << "Atomic Propositions," << iv_.num_atomic_propositions << ",count\n";
    outfile << "Number of Robots," << iv_.num_robots << ",count\n";
    outfile << "Total Robot Capabilities," << iv_.total_robot_capabilities << ",count\n";
    outfile << "Transition System Regions," << iv_.num_ts_regions << ",count\n";
    outfile << "Avg Capabilities/Robot," << std::fixed << std::setprecision(4) 
            << iv_.avg_capabilities_per_robot << ",ratio\n";
    outfile << "Capability Homogeneity," << iv_.capability_homogeneity << ",ratio\n";
    
    // Runtime Metrics
    outfile << "Total Computation Time," << std::fixed << std::setprecision(2) 
            << runtime_.total_computation_time_ms << ",ms\n";
    outfile << "High-Level Runtime," << runtime_.high_level_runtime_ms << ",ms\n";
    outfile << "Low-Level Runtime," << runtime_.low_level_runtime_ms << ",ms\n";
    
    // Correctness Metrics
    outfile << "LTL Satisfaction Rate," << std::fixed << std::setprecision(4) 
            << correctness_.ltl_satisfaction_rate << ",ratio\n";
    outfile << "Capability Satisfaction Rate," << correctness_.capability_satisfaction_rate << ",ratio\n";
    outfile << "Constraint Violations," << correctness_.inter_task_constraint_violations << ",count\n";
    
    // Subtree Efficiency
    outfile << "Total Nodes Generated," << subtree_efficiency_.total_nodes_generated << ",count\n";
    outfile << "Total Nodes Pruned," << subtree_efficiency_.total_nodes_pruned << ",count\n";
    outfile << "Pruning Ratio," << std::fixed << std::setprecision(4) 
            << subtree_efficiency_.pruning_ratio << ",ratio\n";
    outfile << "Memory Reduction Ratio," << subtree_efficiency_.memory_reduction_ratio << ",ratio\n";
    outfile << "State-Space Reduction," << subtree_efficiency_.state_space_reduction << ",count\n";
    
    // Solution Quality
    outfile << "Makespan," << std::fixed << std::setprecision(2) 
            << solution_quality_.makespan_seconds << ",seconds\n";
    outfile << "Sum of Travel Times," << solution_quality_.sum_of_travel_times_seconds << ",seconds\n";
    outfile << "Total Travel Distance," << solution_quality_.total_travel_distance << ",units\n";
    outfile << "Max Individual Distance," << solution_quality_.max_individual_travel_distance << ",units\n";
    outfile << "Robots Utilized," << solution_quality_.robots_utilized << ",count\n";
    outfile << "Robot Utilization Ratio," << std::fixed << std::setprecision(4) 
            << solution_quality_.robot_utilization_ratio << ",ratio\n";
    
    outfile.close();
    std::cout << "Metrics exported to: " << filename << std::endl;
}

void AlgorithmMetrics::exportToJSON(const std::string& filename) const {
    std::ofstream outfile(filename);
    if (!outfile.is_open()) {
        std::cerr << "ERROR: Could not open file: " << filename << std::endl;
        return;
    }
    
    outfile << "{\n";
    outfile << "  \"independent_variables\": {\n";
    outfile << "    \"automaton_states\": " << iv_.num_automaton_states << ",\n";
    outfile << "    \"automaton_edges\": " << iv_.num_automaton_edges << ",\n";
    outfile << "    \"atomic_propositions\": " << iv_.num_atomic_propositions << ",\n";
    outfile << "    \"num_robots\": " << iv_.num_robots << ",\n";
    outfile << "    \"total_robot_capabilities\": " << iv_.total_robot_capabilities << ",\n";
    outfile << "    \"ts_regions\": " << iv_.num_ts_regions << ",\n";
    outfile << "    \"avg_capabilities_per_robot\": " << std::fixed << std::setprecision(4) 
            << iv_.avg_capabilities_per_robot << ",\n";
    outfile << "    \"inter_task_constraints\": " << iv_.num_inter_task_constraints << "\n";
    outfile << "  },\n";
    
    outfile << "  \"runtime_metrics\": {\n";
    outfile << "    \"total_computation_time_ms\": " << std::fixed << std::setprecision(2) 
            << runtime_.total_computation_time_ms << ",\n";
    outfile << "    \"high_level_runtime_ms\": " << runtime_.high_level_runtime_ms << ",\n";
    outfile << "    \"low_level_runtime_ms\": " << runtime_.low_level_runtime_ms << "\n";
    outfile << "  },\n";
    
    outfile << "  \"correctness_metrics\": {\n";
    outfile << "    \"ltl_satisfaction_rate\": " << std::fixed << std::setprecision(4) 
            << correctness_.ltl_satisfaction_rate << ",\n";
    outfile << "    \"capability_satisfaction_rate\": " 
            << correctness_.capability_satisfaction_rate << ",\n";
    outfile << "    \"constraint_violations\": " 
            << correctness_.inter_task_constraint_violations << ",\n";
    outfile << "    \"acceptance_condition_reached\": " 
            << (correctness_.acceptance_condition_reached ? "true" : "false") << ",\n";
    outfile << "    \"feasibility_status\": \"" << correctness_.feasibility_status << "\"\n";
    outfile << "  },\n";
    
    outfile << "  \"subtree_efficiency_metrics\": {\n";
    outfile << "    \"total_nodes_generated\": " << subtree_efficiency_.total_nodes_generated << ",\n";
    outfile << "    \"total_nodes_expanded\": " << subtree_efficiency_.total_nodes_expanded << ",\n";
    outfile << "    \"total_nodes_pruned\": " << subtree_efficiency_.total_nodes_pruned << ",\n";
    outfile << "    \"pruning_ratio\": " << std::fixed << std::setprecision(4) 
            << subtree_efficiency_.pruning_ratio << ",\n";
    outfile << "    \"memory_reduction_ratio\": " 
            << subtree_efficiency_.memory_reduction_ratio << ",\n";
    outfile << "    \"state_space_reduction\": " 
            << subtree_efficiency_.state_space_reduction << "\n";
    outfile << "  },\n";
    
    outfile << "  \"solution_quality_metrics\": {\n";
    outfile << "    \"makespan_seconds\": " << std::fixed << std::setprecision(2) 
            << solution_quality_.makespan_seconds << ",\n";
    outfile << "    \"sum_of_travel_times\": " 
            << solution_quality_.sum_of_travel_times_seconds << ",\n";
    outfile << "    \"total_travel_distance\": " 
            << solution_quality_.total_travel_distance << ",\n";
    outfile << "    \"max_individual_travel_distance\": " 
            << solution_quality_.max_individual_travel_distance << ",\n";
    outfile << "    \"robots_utilized\": " << solution_quality_.robots_utilized << ",\n";
    outfile << "    \"robot_utilization_ratio\": " << std::fixed << std::setprecision(4) 
            << solution_quality_.robot_utilization_ratio << "\n";
    outfile << "  }\n";
    outfile << "}\n";
    
    outfile.close();
    std::cout << "Metrics exported to: " << filename << std::endl;
}
