#ifndef ALGORITHM_METRICS_H
#define ALGORITHM_METRICS_H

#include <chrono>
#include <vector>
#include <map>
#include <string>

/**
 * @class AlgorithmMetrics
 * @brief Comprehensive metrics tracking for thesis evaluation
 * 
 * Tracks independent variables, correctness metrics, runtime metrics,
 * subtree efficiency metrics, and solution quality metrics across all
 * experimental evaluations.
 */
class AlgorithmMetrics {
public:
    // ==================== INDEPENDENT VARIABLES ====================
    
    struct IndependentVariables {
        // Automaton characteristics
        int num_automaton_states = 0;
        int num_automaton_edges = 0;
        int num_atomic_propositions = 0;
        
        // Robot fleet characteristics
        int num_robots = 0;
        int total_robot_capabilities = 0;
        int num_ts_regions = 0;
        
        // Derived values
        double avg_capabilities_per_robot = 0.0;  // total_capabilities / num_robots
        double capability_homogeneity = 0.0;       // independent_capabilities / num_robots
        
        // Task characteristics
        int num_inter_task_constraints = 0;
    };
    
    // ==================== RUNTIME AND SCALABILITY METRICS ====================
    
    struct RuntimeMetrics {
        // Primary timing measurements
        double total_computation_time_ms = 0.0;          // milliseconds  
        
        // Parameterized runtime analysis (collected across runs)
        std::vector<std::pair<int, double>> runtime_vs_num_robots;           // (N, time_ms)
        std::vector<std::pair<int, double>> runtime_vs_automaton_states;     // (|S_B|, time_ms)
        std::vector<std::pair<int, double>> runtime_vs_automaton_edges;      // (edges, time_ms)
        std::vector<std::pair<double, double>> runtime_vs_capability_density; // (density, time_ms)
        std::vector<std::pair<double, double>> runtime_vs_capability_homogeneity; // (homogeneity, time_ms)
        std::vector<std::pair<int, double>> runtime_vs_atomic_propositions;   // (AP, time_ms)
        std::vector<std::pair<int, double>> runtime_vs_environment_size;      // (env_size, time_ms)
        std::vector<std::pair<int, double>> runtime_vs_ts_regions;            // (regions, time_ms)
    };
    
    // ==================== SUBTREE EFFICIENCY METRICS ====================
    
    struct SubtreeEfficiencyMetrics {
        // Node counts
        //nodes in the search tree
        long long total_nodes_generated = 0;
        //total nodes planning
        long long total_nodes_planning = 0;
        // nodes pruned difference between the two above
        long long total_nodes_traversed = 0;
        long long total_nodes_pruned = 0;
        // nodes satisfying the LTL formula gives you the number of feasible paths
        long long nodes_satisfying_ltl = 0;              // OTH or TRA nodes
        
        // Memory usage: currently do not know how to get this. 
        long long task_allocation_algorithm_memory_bytes = 0;
    
        // Product automaton comparison (for small instances)
        long long full_product_automaton_nodes = 0;
        long long full_product_automaton_edges = 0;
        long long full_product_automaton_memory_bytes = 0;
        
        // Derived efficiency metrics
        double pruning_ratio = 0.0;                      // pruned / total_generated
        double explored_product_ratio = 0.0;             // generated / full_product_nodes
        double tree_product_ratio = 0.0;                 // tree_nodes / full_product_nodes
        double memory_reduction_ratio = 0.0;             // tree_memory / product_memory
        double optimality_gap_percent = 0.0;             // (J_tree - J_full) / J_full * 100
        double runtime_speedup_percent = 0.0;            // (time_prod - time_tree) / time_prod * 100
        double percent_nodes_in_tree = 0.0;              // (generated / traversed) * 100
        long long state_space_reduction = 0;             // product_nodes - tree_nodes
    };
    
    // ==================== SOLUTION QUALITY METRICS ====================
    
    struct SolutionQualityMetrics {
        // Time-based metrics
        double tree_makespan_seconds = 0.0;                   // Total mission completion time
        double product_makespan_seconds = 0.0;           // Total mission completion time for the full product automaton
        double sum_of_travel_times_seconds = 0.0;        // Total team effort
        double total_travel_distance = 0.0;
        double max_individual_travel_distance = 0.0;
        
        // Load balance and utilization
        double load_balance_variance = 0.0;              // Measure of task distribution evenness
        int robots_utilized = 0;                         // Count of robots assigned at least one task
        double robot_utilization_ratio = 0.0;            // robots_utilized / total_robots
        
        // Per-robot metrics
        std::vector<double> individual_travel_times;
        std::vector<double> individual_travel_distances;
        std::vector<int> tasks_per_robot;
    };
    
    // ==================== CLASS INTERFACE ====================
    
public:
    AlgorithmMetrics() = default;
    virtual ~AlgorithmMetrics() = default;
    
    // Initialization
    void clearMetrics();  // Safely reset all metrics without calling default constructor
    void setIndependentVariables(const IndependentVariables& vars);
    void setTaskMemoryUsage(long long bytes);
    void setFullProductAutomatonMetrics(long long nodes, long long edges, long long memory_bytes);
    void startTimer();
    void stopTimer();
    
    // Compute derived metrics
    void computeDerivedMetrics();
    
    // Data access
    const IndependentVariables& getIndependentVariables() const { return iv_; }
    const RuntimeMetrics& getRuntime() const { return runtime_; }
    const SubtreeEfficiencyMetrics& getSubtreeEfficiency() const { return subtree_efficiency_; }
    const SolutionQualityMetrics& getSolutionQuality() const { return solution_quality_; }
    
    // Reporting
    void printSummary() const;
    void printDetailedReport() const;
    void exportToCSV(const std::string& filename) const;
    void exportToJSON(const std::string& filename) const;
    
    // Solution quality metrics updates
    void setSolutionMakespan(double tree_makespan_sec, double product_makespan_sec = 0.0);
    void setSumOfTravelTimes(double sum_seconds);
    void setTravelDistance(double total_distance, double max_individual_distance = 0.0);
    void setRobotsUtilized(int count);
    void setIndividualTravelTime(int robot_id, double time_seconds);
    void setIndividualTravelDistance(int robot_id, double distance);
    void setTasksPerRobot(int robot_id, int task_count);
    
    // Batch runtime parameter updates
    void addRuntimeVsRobots(int num_robots, double time_ms);
    void addRuntimeVsAutomatonStates(int states, double time_ms);
    void addRuntimeVsAutomatonEdges(int edges, double time_ms);
    void addRuntimeVsCapabilityDensity(double density, double time_ms);
    void addRuntimeVsCapabilityHomogeneity(double homogeneity, double time_ms);
    void addRuntimeVsAtomicPropositions(int ap, double time_ms);
    void addRuntimeVsEnvironmentSize(int env_size, double time_ms);
    void addRuntimeVsRegions(int regions, double time_ms);
    
public:
    // Direct access to metric structures for algorithm tracking
    IndependentVariables iv_;
    RuntimeMetrics runtime_;
    SubtreeEfficiencyMetrics subtree_efficiency_;
    SolutionQualityMetrics solution_quality_;
    
private:
    // Timer management
    std::chrono::high_resolution_clock::time_point overall_start_time_;
    
    // Helper methods
    double computePruningRatio() const;
    double computePercentNodesInTree() const;
    double computeExploredProductRatio() const;
    double computeTreeProductRatio() const;
    double computeMemoryReductionRatio() const;
    double computeRobotUtilizationRatio() const;
    double computeLoadBalanceVariance() const;
};

#endif // ALGORITHM_METRICS_H
