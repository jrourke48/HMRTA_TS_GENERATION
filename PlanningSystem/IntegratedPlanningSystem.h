#ifndef INTEGRATED_PLANNING_SYSTEM_H
#define INTEGRATED_PLANNING_SYSTEM_H

#include "VoronoiPruningPipeline.h"
#include "ProductAutomaton.h"
#include "BuchiAutomaton.h"
#include "TS.h"
#include <vector>
#include <string>
#include <memory>

/**
 * IntegratedPlanningSystem
 * 
 * High-level abstraction that combines:
 * - Robot capabilities from fleet specification
 * - Workspace discretization (grid or regions)
 * - LTL task specifications
 * - Capability-aware pruning
 * - Multi-robot plan synthesis
 */
class IntegratedPlanningSystem {
public:
    struct PlanningConfig {
        std::string ltlFormula;
        CapabilityWeights capabilityWeights;
        bool enablePruning = true;
        bool enableVisualization = false;
        float gridResolution = 1.0f;
    };
    
    struct PlanningResult {
        bool success = false;
        std::vector<uint32_t> plan;
        FeasibilityMask feasibilityMask;
        std::string message;
    };
    
public:
    IntegratedPlanningSystem() = default;
    ~IntegratedPlanningSystem();
    
    /**
     * Execute complete planning workflow:
     * 1. Parse robot capabilities
     * 2. Create workspace and discretization
     * 3. Translate LTL to Büchi automaton
     * 4. Apply weighted Voronoi pruning
     * 5. Build product automaton
     * 6. Search for acceptable plan
     */
    PlanningResult executePlanning(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        const PlanningConfig& config
    );
    
    /**
     * Analyze impact of pruning on search space
     */
    struct AnalysisResult {
        uint32_t originalTSStates = 0;
        uint32_t prunedTSStates = 0;
        uint32_t originalProductStates = 0;
        uint32_t prunedProductStates = 0;
        float compressionRatio = 1.0f;
        std::string report;
    };
    
    AnalysisResult analyzeCompressionRatio(
        const ProductAutomaton* originalProduct,
        const ProductAutomaton* prunedProduct
    ) const;
    
    /**
     * Visualize results to dot files
     */
    void exportProductAutomaton(
        const ProductAutomaton* product,
        const std::string& filename
    ) const;
    
    void exportFeasibilityMatrix(
        const FeasibilityMask& mask,
        const std::vector<DiscreteRegion>& regions,
        const std::string& filename
    ) const;

private:
    std::unique_ptr<VoronoiPruningPipeline> pipeline;
    std::unique_ptr<TS> transitionSystem;
    std::unique_ptr<ProductAutomaton> productAutomaton;
};

/**
 * Plan Executor - executes the synthesized plan on real robots
 */
class PlanExecutor {
public:
    struct ExecutionConfig {
        bool synchronous = true;       // Wait for all robots or execute concurrently
        float maxExecutionTime = 300.0; // Seconds
        bool enableReplanning = false;  // Replan if disturbance detected
    };
    
    struct ExecutionTrace {
        bool completed = false;
        std::vector<uint32_t> executedStates;
        std::vector<std::string> robotActions;
        float executionTime = 0.0f;
        std::string finalStatus;
    };
    
public:
    /**
     * Execute plan with feedback
     */
    ExecutionTrace executePlan(
        const std::vector<uint32_t>& plan,
        const std::vector<RobotCapabilities>& robots,
        const ExecutionConfig& config
    );
    
    /**
     * Generate high-level robot instructions from product automaton states
     */
    std::vector<std::string> generateRobotInstructions(
        const std::vector<uint32_t>& plan,
        const ProductAutomaton* product,
        const std::vector<DiscreteRegion>& regions
    ) const;
};

#endif // INTEGRATED_PLANNING_SYSTEM_H
