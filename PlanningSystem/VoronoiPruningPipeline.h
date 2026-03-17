#ifndef VORONOI_PRUNING_PIPELINE_H
#define VORONOI_PRUNING_PIPELINE_H

#include "Automaton.h"
#include "ProductAutomaton.h"
#include "BuchiAutomaton.h"
#include "TS.h"
#include <vector>
#include <map>
#include <set>
#include <cstdint>
#include <Eigen/Dense>
#include <string>
#include <spot/twa/twagraph.hh>

// Robot capability structure
struct RobotCapabilities {
    uint32_t id;
    Eigen::Vector2f position;      // 2D start position (site s_i)
    float velocity;                // v_i - max velocity
    float senseRange;              // r_i^sense - sensing range
    float capacity;                // p_i^max - payload capacity
    float localizationError;        // sigma_i^loc - localization uncertainty
};

// Capability weights for the power diagram
struct CapabilityWeights {
    float lambda_v = 1.0f;         // Weight on velocity
    float lambda_r = 1.0f;         // Weight on sensing range
    float lambda_p = 1.0f;         // Weight on payload capacity
    float lambda_sigma = 1.0f;     // Weight on localization error (negative contribution)
};

// Represents a discrete region in the workspace
struct DiscreteRegion {
    uint32_t id;
    std::string label;
    Eigen::Vector2f centroid;
    std::vector<Eigen::Vector2f> vertices;  // Polygon vertices if needed
};

// Feasibility mask type: feasibility[robot_id][region_id] = {0,1}
typedef std::map<uint32_t, std::map<uint32_t, uint8_t>> FeasibilityMask;

class VoronoiPruningPipeline {
public:
    VoronoiPruningPipeline();
    ~VoronoiPruningPipeline();

    // Main pipeline execution
    ProductAutomaton* executeFullPipeline(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        TS* transitionSystem,
        spot::twa_graph_ptr buchiAutomaton,
        const CapabilityWeights& weights = CapabilityWeights(),
        const std::vector<uint32_t>& goalRegions = {}
    );

    // Step 1: Compute weighted Voronoi / Power diagram
    void computePowerDiagram(
        const std::vector<RobotCapabilities>& robots,
        const CapabilityWeights& weights
    );

    // Step 2: Compute feasibility mask from power diagram
    void computeFeasibilityMask(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        FeasibilityMask& outMask
    );

    // Step 3: Check feasibility satisfaction
    bool checkGoalFeasibility(
        const FeasibilityMask& mask,
        const std::vector<uint32_t>& goalRegions
    ) const;

    // Step 4: Prune TS labels based on feasibility
    void pruneLabeling(
        TS* transitionSystem,
        const FeasibilityMask& mask
    );

    // Step 5: Build product automaton
    ProductAutomaton* buildProductAutomaton(
        TS* transitionSystem,
        spot::twa_graph_ptr buchiAutomaton
    );

    // Step 6: Graph search for accepting plan
    bool findAcceptingPlan(
        ProductAutomaton* product,
        std::vector<uint32_t>& outPlan
    );

    // Utility: Compute robot weight based on capabilities
    float computeRobotWeight(
        const RobotCapabilities& robot,
        const CapabilityWeights& weights
    ) const;

    // Utility: Compute power distance
    float computePowerDistance(
        const Eigen::Vector2f& point,
        const RobotCapabilities& robot,
        float robotWeight
    ) const;

    // Utility: Check if region intersects Voronoi cell
    bool regionIntersectsVoronoiCell(
        const DiscreteRegion& region,
        const Eigen::Vector2f& robotPos,
        std::vector<Eigen::Vector2f>& allRobotPositions,
        std::vector<float>& allRobotWeights
    ) const;

    // Getters
    const FeasibilityMask& getFeasibilityMask() const { return currentFeasibilityMask; }
    const std::map<uint32_t, Eigen::Vector2f>& getVoronoiCenters() const { return voronoiCenters; }

private:
    FeasibilityMask currentFeasibilityMask;
    std::map<uint32_t, Eigen::Vector2f> voronoiCenters;  // Robot ID -> Voronoi cell center
    std::map<uint32_t, std::set<uint32_t>> voronoiCells;  // Robot ID -> set of region IDs in cell
};

#endif // VORONOI_PRUNING_PIPELINE_H
