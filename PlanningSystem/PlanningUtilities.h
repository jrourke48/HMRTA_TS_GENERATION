#ifndef PLANNING_UTILITIES_H
#define PLANNING_UTILITIES_H

#include "VoronoiPruningPipeline.h"
#include "ProductAutomaton.h"
#include <vector>
#include <string>
#include <Eigen/Dense>

/**
 * Utility functions for the planning system
 */
namespace PlanningUtils {

    // ========== Test Scenario Builders ==========
    
    /**
     * Create a simple test scenario
     */
    struct TestScenario {
        std::string name;
        std::vector<RobotCapabilities> robots;
        std::vector<DiscreteRegion> regions;
        std::string ltlFormula;
        bool expectedSuccess;
    };
    
    TestScenario buildRobotCircleScenario();
    TestScenario buildGridScenario();
    TestScenario buildAsymmetricScenario();
    
    // ========== Visualization Utilities ==========
    
    /**
     * Export power diagram as SVG
     */
    void exportPowerDiagramSVG(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        float mapWidth, float mapHeight,
        const std::string& filename
    );
    
    /**
     * Export feasibility matrix as CSV
     */
    void exportFeasibilityCSV(
        const FeasibilityMask& mask,
        const std::vector<DiscreteRegion>& regions,
        const std::string& filename
    );
    
    /**
     * Export product automaton as DOT file
     */
    void exportProductDOT(
        const ProductAutomaton* product,
        const std::vector<DiscreteRegion>& regions,
        const std::string& filename
    );
    
    // ========== Analysis Utilities ==========
    
    /**
     * Analyze system metrics
     */
    struct SystemMetrics {
        uint32_t numRobots;
        uint32_t numRegions;
        float avgRobotCapability;
        float capabilityVariance;
        uint32_t avgFeasibleRegionsPerRobot;
        float coverageRatio;  // % of regions covered by at least one robot
    };
    
    SystemMetrics analyzeSystem(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        const FeasibilityMask& mask
    );
    
    /**
     * Print detailed system analysis
     */
    void printSystemAnalysis(const SystemMetrics& metrics);
    
    /**
     * Validate feasibility mask
     */
    bool validateFeasibilityMask(
        const FeasibilityMask& mask,
        uint32_t numRobots,
        uint32_t numRegions
    );
    
    // ========== Performance Profiling ==========
    
    struct PerformanceProfile {
        double powerDiagramTime;
        double feasibilityTime;
        double productConstructionTime;
        double planSearchTime;
        double totalTime;
        uint32_t productStates;
        uint32_t productEdges;
    };
    
    /**
     * Run complete pipeline with timing
     */
    PerformanceProfile profilePipeline(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        TS* transitionSystem,
        spot::twa_graph_ptr buchi,
        const CapabilityWeights& weights
    );
    
    /**
     * Print performance report
     */
    void printPerformanceReport(const PerformanceProfile& profile);
    
    // ========== Validation Utilities ==========
    
    /**
     * Verify plan satisfies LTL formula
     */
    bool verifyPlanSatisfiesFormula(
        const std::vector<uint32_t>& plan,
        const std::string& ltlFormula,
        const std::vector<DiscreteRegion>& regions
    );
    
    /**
     * Check for redundant states in plan
     */
    std::vector<uint32_t> optimizePlan(
        const std::vector<uint32_t>& plan
    );
    
    // ========== Debugging Utilities ==========
    
    /**
     * Print feasibility mask in human-readable format
     */
    void printFeasibilityMatrix(
        const FeasibilityMask& mask,
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions
    );
    
    /**
     * Trace power distance calculations
     */
    void tracePowerDistances(
        const Eigen::Vector2f& point,
        const std::vector<RobotCapabilities>& robots,
        const CapabilityWeights& weights
    );
    
    /**
     * Generate detailed execution log
     */
    std::string generateExecutionLog(
        const std::vector<uint32_t>& plan,
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions
    );

} // namespace PlanningUtils

#endif // PLANNING_UTILITIES_H
