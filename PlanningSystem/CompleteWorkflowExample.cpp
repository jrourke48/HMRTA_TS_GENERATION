#include "IntegratedPlanningSystem.h"
#include "VoronoiPruningPipeline.h"
#include <iostream>
#include <cmath>

/**
 * COMPLETE WORKFLOW EXAMPLE
 * 
 * This demonstrates the full capability-aware multi-robot planning system:
 * - Fleet of heterogeneous robots with different capabilities
 * - Workspace partitioned into discrete regions
 * - LTL task specifications over regions
 * - Capability-aware Voronoi pruning
 * - Product automaton synthesis
 * - Optimal plan generation
 */

int main() {
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║   Capability-Aware Multi-Robot Task Planning System        ║\n";
    std::cout << "║        Using Weighted Voronoi Partitions & LTL             ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n";
    
    // ========== SCENARIO: Warehouse Multi-Robot System ==========
    
    std::cout << "\n[SCENARIO] Warehouse Inspection Task\n";
    std::cout << "=========================================\n";
    std::cout << "- Warehouse divided into 6 zones (regions)\n";
    std::cout << "- 3 heterogeneous robots with different capabilities\n";
    std::cout << "- Task: Inspect all priority zones in sequence\n\n";
    
    // ========== Define Robots with Realistic Capabilities ==========
    
    std::vector<RobotCapabilities> robots;
    
    // Heavy-load robot: Slow but high capacity
    robots.push_back({
        .id = 0,
        .position = Eigen::Vector2f(0.0f, 0.0f),
        .velocity = 0.5f,      // Slow
        .senseRange = 3.0f,
        .capacity = 50.0f,     // High capacity
        .localizationError = 0.15f
    });
    std::cout << "Robot 0 (Heavy-Load): position(0,0), v=0.5 m/s, capacity=50kg\n";
    
    // Balanced robot: Medium everything
    robots.push_back({
        .id = 1,
        .position = Eigen::Vector2f(5.0f, 0.0f),
        .velocity = 1.5f,      // Medium speed
        .senseRange = 4.0f,
        .capacity = 20.0f,     // Medium capacity
        .localizationError = 0.10f
    });
    std::cout << "Robot 1 (Balanced): position(5,0), v=1.5 m/s, capacity=20kg\n";
    
    // Scout robot: Fast with high sensing
    robots.push_back({
        .id = 2,
        .position = Eigen::Vector2f(10.0f, 0.0f),
        .velocity = 3.0f,      // Fast
        .senseRange = 6.0f,    // High sensing range
        .capacity = 5.0f,      // Low capacity
        .localizationError = 0.05f
    });
    std::cout << "Robot 2 (Scout): position(10,0), v=3.0 m/s, sense=6m\n";
    
    // ========== Define Warehouse Regions ==========
    
    std::vector<DiscreteRegion> regions;
    regions.push_back({.id = 0, .label = "MainStorage", .centroid = Eigen::Vector2f(0.0f, 0.0f)});
    regions.push_back({.id = 1, .label = "LoadingDock", .centroid = Eigen::Vector2f(3.0f, 0.0f)});
    regions.push_back({.id = 2, .label = "GridSection1", .centroid = Eigen::Vector2f(6.0f, 0.0f)});
    regions.push_back({.id = 3, .label = "GridSection2", .centroid = Eigen::Vector2f(9.0f, 0.0f)});
    regions.push_back({.id = 4, .label = "RoverZone", .centroid = Eigen::Vector2f(12.0f, 0.0f)});
    regions.push_back({.id = 5, .label = "ControlCenter", .centroid = Eigen::Vector2f(15.0f, 0.0f)});
    
    std::cout << "\nWarehouse Zones:\n";
    for (const auto& r : regions) {
        std::cout << "  " << r.label << " @ (" << r.centroid.x() << ", " << r.centroid.y() << ")\n";
    }
    
    // ========== Define Capability Weights ==========
    
    CapabilityWeights weights;
    weights.lambda_v = 2.0f;       // Speed is important for inspection
    weights.lambda_r = 3.0f;       // Sensing range is critical
    weights.lambda_p = 1.0f;       // Capacity less relevant
    weights.lambda_sigma = 2.0f;   // Need good localization
    
    // ========== Define LTL Task Specification ==========
    
    std::string ltlFormula = "F (p0 & F (p1 & F (p2 & F p3)))";
    std::cout << "\nTask Specification (LTL):\n";
    std::cout << "  Formula: " << ltlFormula << "\n";
    std::cout << "  Meaning: Eventually visit MainStorage, then LoadingDock,\n";
    std::cout << "           then GridSection1, then GridSection2\n";
    
    // ========== Instantiate Planning System ==========
    
    IntegratedPlanningSystem planner;
    IntegratedPlanningSystem::PlanningConfig config;
    config.ltlFormula = ltlFormula;
    config.capabilityWeights = weights;
    config.enablePruning = true;
    config.enableVisualization = true;
    
    // ========== Execute Planning ==========
    
    std::cout << "\n========== EXECUTING PLANNING PIPELINE ==========\n";
    
    auto result = planner.executePlanning(robots, regions, config);
    
    if (!result.success) {
        std::cerr << "\n[FAILED] Planning failed: " << result.message << "\n";
        return 1;
    }
    
    // ========== Display Results ==========
    
    std::cout << "\n========== PLANNING RESULTS ==========\n";
    std::cout << "Status: SUCCESS\n";
    std::cout << "Plan Length: " << result.plan.size() << " states\n\n";
    
    std::cout << "Execution Sequence:\n";
    for (uint32_t i = 0; i < result.plan.size(); ++i) {
        uint32_t regionId = result.plan[i];
        if (regionId < regions.size()) {
            std::cout << "  Step " << i << ": " << regions[regionId].label << "\n";
        }
    }
    
    // ========== Feasibility Analysis ==========
    
    std::cout << "\n========== FEASIBILITY ANALYSIS ==========\n";
    std::cout << "Robot-Region Assignments (based on weighted Voronoi):\n\n";
    
    for (const auto& robot : robots) {
        std::cout << "Robot " << robot.id << ":\n";
        float weight = 0.0f;
        for (const auto& comp : {robot.velocity, robot.senseRange, robot.capacity, robot.localizationError}) {
            weight += comp * comp;  // Simplified calculation
        }
        std::cout << "  Weight: " << weight << "\n";
        
        std::cout << "  Can Service: ";
        bool found = false;
        for (const auto& region : regions) {
            // Check feasibility from result
            auto it = result.feasibilityMask.find(robot.id);
            if (it != result.feasibilityMask.end()) {
                auto region_it = it->second.find(region.id);
                if (region_it != it->second.end() && region_it->second) {
                    std::cout << region.label << " ";
                    found = true;
                }
            }
        }
        if (!found) {
            std::cout << "(none determined by centroid sampling)";
        }
        std::cout << "\n";
    }
    
    // ========== Capability Analysis ==========
    
    std::cout << "\n========== CAPABILITY RANKING ==========\n";
    
    struct RobotScore {
        uint32_t id;
        float score;
        std::string specialty;
    };
    
    std::vector<RobotScore> scores;
    for (const auto& r : robots) {
        float score = weights.lambda_v * r.velocity 
                    + weights.lambda_r * r.senseRange 
                    + weights.lambda_p * r.capacity 
                    - weights.lambda_sigma * r.localizationError;
        
        std::string specialty;
        if (r.velocity > 2.5f) specialty = "Speed";
        else if (r.senseRange > 5.0f) specialty = "Sensing";
        else if (r.capacity > 25.0f) specialty = "Capacity";
        else specialty = "Balanced";
        
        scores.push_back({r.id, score, specialty});
    }
    
    std::cout << "Capability Scores (weighted):\n";
    for (const auto& s : scores) {
        std::cout << "  Robot " << s.id << ": " << s.score << " [" << s.specialty << "]\n";
    }
    
    // ========== Export Results ==========
    
    std::cout << "\n========== EXPORTING RESULTS ==========\n";
    
    planner.exportFeasibilityMatrix(
        result.feasibilityMask,
        regions,
        "feasibility_matrix.txt"
    );
    
    // ========== Plan Execution Simulation ==========
    
    std::cout << "\n========== SIMULATING PLAN EXECUTION ==========\n";
    
    PlanExecutor executor;
    PlanExecutor::ExecutionConfig execConfig;
    execConfig.synchronous = true;
    execConfig.maxExecutionTime = 300.0f;
    
    auto executionTrace = executor.executePlan(result.plan, robots, execConfig);
    
    std::cout << "\nExecution Summary:\n";
    std::cout << "  Completed: " << (executionTrace.completed ? "YES" : "NO") << "\n";
    std::cout << "  Total Time: " << executionTrace.executionTime << " seconds\n";
    std::cout << "  Status: " << executionTrace.finalStatus << "\n";
    
    std::cout << "\nRobot Instructions:\n";
    auto instructions = executor.generateRobotInstructions(result.plan, nullptr, regions);
    for (const auto& instr : instructions) {
        std::cout << "  " << instr << "\n";
    }
    
    // ========== Completion Summary ==========
    
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════╗\n";
    std::cout << "║                   WORKFLOW COMPLETED                       ║\n";
    std::cout << "║                                                            ║\n";
    std::cout << "║  ✓ Voronoi partitioning computed                           ║\n";
    std::cout << "║  ✓ Feasibility constraints applied                         ║\n";
    std::cout << "║  ✓ Product automaton synthesized                           ║\n";
    std::cout << "║  ✓ Optimal plan found                                      ║\n";
    std::cout << "║  ✓ Execution simulation completed                          ║\n";
    std::cout << "║                                                            ║\n";
    std::cout << "║  Next Steps: Deploy on real robots with feedback control   ║\n";
    std::cout << "╚════════════════════════════════════════════════════════════╝\n\n";
    
    return 0;
}
