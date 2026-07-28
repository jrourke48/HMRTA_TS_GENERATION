# AlgorithmMetrics Class Documentation

## Overview

The `AlgorithmMetrics` class provides comprehensive tracking and reporting of all thesis evaluation metrics across three evaluation dimensions:

1. **Correctness & Feasibility** - LTL satisfaction, capability satisfaction, constraint violations
2. **Computational Efficiency** - Runtime analysis, scalability, node generation & pruning
3. **Solution Quality** - Mission completion time, task distribution, resource utilization

## Independent Variables

Variables that characterize the problem instance and environment:

| Variable | Type | Meaning |
|----------|------|---------|
| `num_automaton_states` | int | Number of states in Büchi automaton $\|S_{\mathcal{B}}\|$ |
| `num_automaton_edges` | int | Number of transitions in automaton |
| `num_atomic_propositions` | int | Number of atomic propositions in LTL formula |
| `num_robots` | int | Team size $N$ |
| `total_robot_capabilities` | int | Sum of all capabilities across fleet |
| `num_ts_regions` | int | Number of regions in transition system |
| `avg_capabilities_per_robot` | double | Average capabilities per robot |
| `capability_homogeneity` | double | Measure of fleet homogeneity |
| `num_inter_task_constraints` | int | Number of task inter-dependencies |

## Metrics Hierarchy

### 1. Correctness Metrics

```cpp
struct CorrectnessMetrics {
    double ltl_satisfaction_rate;              // [0, 1]
    double capability_satisfaction_rate;       // [0, 1]
    int inter_task_constraint_violations;
    bool acceptance_condition_reached;
    std::string feasibility_status;            // "FEASIBLE", "INFEASIBLE", "UNKNOWN"
};
```

**When to record:**
- After algorithm completes or terminates
- If LTL property is satisfied by solution
- If all required capabilities are available in team
- If all task constraints are satisfied

### 2. Runtime Metrics

Tracks computation time and scalability analysis:

```cpp
struct RuntimeMetrics {
    double total_computation_time_ms;
    double high_level_runtime_ms;
    double low_level_runtime_ms;
    
    // Parametric analysis (collected across runs)
    std::vector<std::pair<int, double>> runtime_vs_num_robots;
    std::vector<std::pair<int, double>> runtime_vs_automaton_states;
    std::vector<std::pair<int, double>> runtime_vs_automaton_edges;
    std::vector<std::pair<double, double>> runtime_vs_capability_density;
    // ... more parametric measurements
};
```

**Key measurements:**
- `runtime_vs_num_robots`: Scale test with N=5, 10, 20, 50, 100 robots
- `runtime_vs_automaton_states`: Scale test with |S_B|=4, 8, 16, 32, 64, 128
- `runtime_vs_automaton_edges`: Transition-density scaling
- `runtime_vs_capability_density`: Capability coverage in fleet
- `runtime_vs_capability_homogeneity`: Fleet homogeneity impact

### 3. Subtree Efficiency Metrics

Compares tree-based search against full product automaton:

```cpp
struct SubtreeEfficiencyMetrics {
    // Node tracking
    long long total_nodes_generated;
    long long total_nodes_expanded;
    long long total_nodes_pruned;
    long long nodes_satisfying_ltl;            // OTH or TRA nodes
    
    // Memory comparison
    long long tree_memory_bytes;
    long long product_automaton_memory_bytes;
    long long full_product_automaton_nodes;
    
    // Derived efficiency ratios
    double pruning_ratio;                      // pruned / total
    double explored_product_ratio;             // generated / full_product
    double tree_product_ratio;                 // tree_nodes / full_product_nodes
    double memory_reduction_ratio;             // tree_memory / product_memory
    double optimality_gap_percent;
    double runtime_speedup_percent;
    long long state_space_reduction;
};
```

**Key insights:**
- **Pruning Ratio**: Higher ratio = more effective pruning strategy
- **State-Space Reduction**: Demonstrates scalability beyond full product
- **Memory Reduction Ratio**: Shows practical memory savings (< 1.0 is good)
- **Runtime Speedup %**: Quantifies tree-search advantage

### 4. Solution Quality Metrics

Evaluates the quality of the computed solution:

```cpp
struct SolutionQualityMetrics {
    // Time metrics
    double makespan_seconds;                   // Mission completion time
    double sum_of_travel_times_seconds;        // Total team effort
    double total_travel_distance;
    double max_individual_travel_distance;
    
    // Load balancing
    double load_balance_variance;
    int robots_utilized;
    double robot_utilization_ratio;
    
    // Per-robot details
    std::vector<double> individual_travel_times;
    std::vector<double> individual_travel_distances;
    std::vector<int> tasks_per_robot;
};
```

**Interpretation:**
- **Makespan**: Lower is better (parallelizable completion)
- **Sum of travel times**: Lower is better (team efficiency)
- **Load balance**: Lower variance indicates even distribution
- **Robot utilization**: % of robots actually used in solution

## Usage Examples

### Basic Usage

```cpp
#include "AlgorithmMetrics.h"

int main() {
    AlgorithmMetrics metrics;
    
    // Set independent variables
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = 10;
    vars.num_automaton_states = 16;
    vars.num_automaton_edges = 32;
    vars.num_atomic_propositions = 5;
    vars.total_robot_capabilities = 25;
    vars.num_ts_regions = 20;
    vars.num_inter_task_constraints = 3;
    
    metrics.setIndependentVariables(vars);
    
    // Record measurements
    metrics.startTimer();
    
    // ... run algorithm ...
    
    metrics.stopTimer();
    
    // Record correctness
    AlgorithmMetrics::CorrectnessMetrics correctness;
    correctness.ltl_satisfaction_rate = 1.0;
    correctness.capability_satisfaction_rate = 1.0;
    correctness.acceptance_condition_reached = true;
    correctness.feasibility_status = "FEASIBLE";
    metrics.recordCorrectness(correctness);
    
    // Compute and display results
    metrics.computeDerivedMetrics();
    metrics.printSummary();
    metrics.exportToCSV("metrics.csv");
    metrics.exportToJSON("metrics.json");
    
    return 0;
}
```

### Scalability Testing (Runtime vs Robots)

```cpp
AlgorithmMetrics metrics;

for (int num_robots : {5, 10, 20, 50, 100}) {
    // Configure problem with num_robots
    AlgorithmMetrics::IndependentVariables vars;
    vars.num_robots = num_robots;
    // ... set other variables ...
    metrics.setIndependentVariables(vars);
    
    // Run algorithm
    metrics.startTimer();
    runAlgorithm(/* ... */);
    metrics.stopTimer();
    
    // Record result
    metrics.addRuntimeVsRobots(num_robots, metrics.getRuntime().total_computation_time_ms);
}

metrics.printDetailedReport();
```

### Scalability Testing (Runtime vs Automaton States)

```cpp
for (int states : {4, 8, 16, 32, 64, 128}) {
    // Create Büchi automaton with 'states' states
    
    metrics.startTimer();
    runAlgorithm(/* ... */);
    metrics.stopTimer();
    
    metrics.addRuntimeVsAutomatonStates(states, 
                                       metrics.getRuntime().total_computation_time_ms);
}
```

### Subtree Efficiency Analysis

```cpp
AlgorithmMetrics::SubtreeEfficiencyMetrics efficiency;

// After tree-search execution
efficiency.total_nodes_generated = tree_nodes_count;
efficiency.total_nodes_pruned = pruned_nodes_count;
efficiency.tree_memory_bytes = calculateTreeMemory();

// Compute full product automaton (for small instances)
efficiency.full_product_automaton_nodes = buildFullProduct().nodeCount();
efficiency.product_automaton_memory_bytes = calculateProductMemory();

metrics.recordSubtreeEfficiency(efficiency);
metrics.computeDerivedMetrics();

// Now efficiency.pruning_ratio, .explored_product_ratio, etc. are computed
```

### Solution Quality Tracking

```cpp
AlgorithmMetrics::SolutionQualityMetrics quality;

quality.makespan_seconds = computeMakespan(solution);
quality.sum_of_travel_times_seconds = computeTotalTravelTime(solution);
quality.total_travel_distance = computeTotalDistance(solution);
quality.max_individual_travel_distance = computeMaxIndividualDistance(solution);

// Track per-robot metrics
for (const auto& robot : solution.robots) {
    quality.individual_travel_times.push_back(robot.total_time);
    quality.individual_travel_distances.push_back(robot.total_distance);
    quality.tasks_per_robot.push_back(robot.task_count);
    
    if (robot.task_count > 0) {
        quality.robots_utilized++;
    }
}

metrics.recordSolutionQuality(quality);
metrics.computeDerivedMetrics();
```

## Export Formats

### CSV Export
```csv
Metric,Value,Unit
Automaton States,16,count
Number of Robots,10,count
Total Computation Time,125.45,ms
LTL Satisfaction Rate,1.0000,ratio
Makespan,45.32,seconds
...
```

### JSON Export
```json
{
  "independent_variables": {
    "automaton_states": 16,
    "num_robots": 10,
    ...
  },
  "runtime_metrics": {
    "total_computation_time_ms": 125.45,
    ...
  },
  ...
}
```

## Integration with Algorithm

1. **Initialization**: Create metrics instance and set independent variables
2. **During execution**: Use startTimer/stopTimer, recordHighLevel/LowLevel for sub-components
3. **Node tracking**: Increment counters for generated/pruned/expanded nodes
4. **After execution**: Record all metric structures and call computeDerivedMetrics()
5. **Analysis**: Use printSummary(), printDetailedReport(), or export functions

## Recommended Experiment Design

### Phase 1: Correctness Verification
- Small instances (N≤5, |S_B|≤16)
- Verify against full product automaton
- Record all correctness metrics

### Phase 2: Scalability Analysis
- Fix all but one independent variable
- Vary: N, |S_B|, capability density, homogeneity
- Collect runtime and node count data
- Generate scaling plots

### Phase 3: Efficiency Comparison
- For instances where full product succeeds: compare directly
- Track pruning effectiveness
- Measure memory savings

### Phase 4: Solution Quality
- Analyze makespan, load balance, utilization
- Compare greedy vs optimal allocations (when available)

## Performance Targets

- **Pruning Ratio**: > 0.7 (70% of generated nodes pruned)
- **State-Space Reduction**: Product nodes - Tree nodes should be large
- **Runtime Speedup**: > 50% faster than full product (when comparable)
- **Robot Utilization**: Should approach 100% for well-designed instances
- **Makespan**: Within 10-20% of theoretical lower bound
