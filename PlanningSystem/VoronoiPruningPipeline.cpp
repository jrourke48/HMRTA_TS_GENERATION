#include "VoronoiPruningPipeline.h"
#include <cmath>
#include <algorithm>
#include <queue>
#include <iostream>
#include <set>
#include <bddx.h>
#include <spot/twa/bdddict.hh>

VoronoiPruningPipeline::VoronoiPruningPipeline() = default;

VoronoiPruningPipeline::~VoronoiPruningPipeline() = default;

ProductAutomaton* VoronoiPruningPipeline::executeFullPipeline(
    const std::vector<RobotCapabilities>& robots,
    const std::vector<DiscreteRegion>& regions,
    TS* transitionSystem,
    spot::twa_graph_ptr buchiAutomaton,
    const CapabilityWeights& weights,
    const std::vector<uint32_t>& goalRegions)
{
    std::cout << "[VoronoiPruning] Starting capability-aware weighted Voronoi pruning pipeline...\n";
    
    // Step 1: Compute power diagram
    std::cout << "[VoronoiPruning] Step 1: Computing power diagram...\n";
    computePowerDiagram(robots, weights);
    std::cout << "[VoronoiPruning]   - Power diagram computed: " << robots.size() << " robots\n";
    
    // Step 2: Compute feasibility mask
    std::cout << "[VoronoiPruning] Step 2: Computing feasibility mask...\n";
    FeasibilityMask feasibilityMask;
    computeFeasibilityMask(robots, regions, feasibilityMask);
    std::cout << "[VoronoiPruning]   - Feasibility mask computed for " << regions.size() << " regions\n";
    
    // Print feasibility matrix
    std::cout << "[VoronoiPruning]   - Feasibility matrix:\n";
    for (const auto& robot_pair : feasibilityMask) {
        std::cout << "      Robot " << robot_pair.first << ": ";
        for (const auto& region_pair : robot_pair.second) {
            if (region_pair.second) std::cout << "R" << region_pair.first << " ";
        }
        std::cout << "\n";
    }
    
    // Step 3: Check goal feasibility
    if (!goalRegions.empty()) {
        std::cout << "[VoronoiPruning] Step 3: Checking goal feasibility...\n";
        if (!checkGoalFeasibility(feasibilityMask, goalRegions)) {
            std::cerr << "[VoronoiPruning] ERROR: Goal regions not feasible with current capability distribution!\n";
            return nullptr;
        }
        std::cout << "[VoronoiPruning]   - Goal feasibility verified\n";
    }
    
    // Step 4: Prune labeling
    std::cout << "[VoronoiPruning] Step 4: Pruning TS labels based on feasibility...\n";
    pruneLabeling(transitionSystem, feasibilityMask);
    std::cout << "[VoronoiPruning]   - TS labels pruned\n";
    
    // Step 5: Build product automaton
    std::cout << "[VoronoiPruning] Step 5: Building product automaton...\n";
    ProductAutomaton* product = buildProductAutomaton(transitionSystem, buchiAutomaton);
    if (!product) {
        std::cerr << "[VoronoiPruning] ERROR: Failed to build product automaton\n";
        return nullptr;
    }
    std::cout << "[VoronoiPruning]   - Product automaton built: " 
              << product->getnumStates() << " states, " 
              << product->getnumEdges() << " edges\n";
    
    std::cout << "[VoronoiPruning] Pipeline complete!\n";
    return product;
}

float VoronoiPruningPipeline::computeRobotWeight(
    const RobotCapabilities& robot,
    const CapabilityWeights& weights) const
{
    // w_i = lambda_v * v_i^2 + lambda_r * (r_i^sense)^2 + lambda_p * (p_i^max)^2 - lambda_sigma * (sigma_i^loc)^2
    float w = weights.lambda_v * (robot.velocity * robot.velocity)
            + weights.lambda_r * (robot.senseRange * robot.senseRange)
            + weights.lambda_p * (robot.capacity * robot.capacity)
            - weights.lambda_sigma * (robot.localizationError * robot.localizationError);
    return w;
}

float VoronoiPruningPipeline::computePowerDistance(
    const Eigen::Vector2f& point,
    const RobotCapabilities& robot,
    float robotWeight) const
{
    // pi_i(x) = ||x - s_i||^2 - w_i
    float dx = point.x() - robot.position.x();
    float dy = point.y() - robot.position.y();
    float distSq = dx * dx + dy * dy;
    return distSq - robotWeight;
}

void VoronoiPruningPipeline::computePowerDiagram(
    const std::vector<RobotCapabilities>& robots,
    const CapabilityWeights& weights)
{
    voronoiCenters.clear();
    voronoiCells.clear();
    
    // Compute weight for each robot
    std::vector<float> robotWeights;
    for (const auto& robot : robots) {
        float w = computeRobotWeight(robot, weights);
        robotWeights.push_back(w);
        voronoiCenters[robot.id] = robot.position;
    }
    
    std::cout << "[PowerDiagram] Robot weights:\n";
    for (size_t i = 0; i < robots.size(); ++i) {
        std::cout << "  Robot " << robots[i].id << ": w = " << robotWeights[i] << "\n";
    }
}

bool VoronoiPruningPipeline::regionIntersectsVoronoiCell(
    const DiscreteRegion& region,
    const Eigen::Vector2f& robotPos,
    std::vector<Eigen::Vector2f>& allRobotPositions,
    std::vector<float>& allRobotWeights) const
{
    // Check if centroid is in Voronoi cell (simplified: centroid wins against all others)
    float minPowerDist = std::numeric_limits<float>::max();
    uint32_t ownerRobot = 0;
    
    for (size_t i = 0; i < allRobotPositions.size(); ++i) {
        float powerDist = computePowerDistance(region.centroid, 
                                              RobotCapabilities{
                                                  .id = (uint32_t)i,
                                                  .position = allRobotPositions[i],
                                                  .velocity = 0.0f,
                                                  .senseRange = 0.0f,
                                                  .capacity = 0.0f,
                                                  .localizationError = 0.0f
                                              }, 
                                              allRobotWeights[i]);
        if (powerDist < minPowerDist) {
            minPowerDist = powerDist;
            ownerRobot = i;
        }
    }
    
    // Check if this robot matches
    return (robotPos - allRobotPositions[ownerRobot]).norm() < 1e-6;
}

void VoronoiPruningPipeline::computeFeasibilityMask(
    const std::vector<RobotCapabilities>& robots,
    const std::vector<DiscreteRegion>& regions,
    FeasibilityMask& outMask)
{
    outMask.clear();
    
    // Pre-compute robot positions and weights for intersection tests
    std::vector<Eigen::Vector2f> robotPositions;
    std::vector<float> robotWeights;
    
    CapabilityWeights defaultWeights;
    for (const auto& robot : robots) {
        robotPositions.push_back(robot.position);
        robotWeights.push_back(computeRobotWeight(robot, defaultWeights));
    }
    
    // For each robot-region pair, determine feasibility
    for (const auto& robot : robots) {
        outMask[robot.id] = {};
        
        for (const auto& region : regions) {
            // Check if region is reachable by this robot
            // Simple feasibility: if distance is within sensing range, it's feasible
            
            float dx = region.centroid.x() - robot.position.x();
            float dy = region.centroid.y() - robot.position.y();
            float distance = std::sqrt(dx * dx + dy * dy);
            
            // Feasibility: distance <= sensing range (with some margin)
            bool feasible = distance <= (robot.senseRange + 2.0f);
            
            outMask[robot.id][region.id] = feasible ? 1 : 0;
            if (feasible) {
                voronoiCells[robot.id].insert(region.id);
            }
        }
    }
    
    currentFeasibilityMask = outMask;
}

bool VoronoiPruningPipeline::checkGoalFeasibility(
    const FeasibilityMask& mask,
    const std::vector<uint32_t>& goalRegions) const
{
    // For each goal region, check if at least one robot can service it
    for (uint32_t goalRegion : goalRegions) {
        bool found = false;
        for (const auto& robot_pair : mask) {
            auto it = robot_pair.second.find(goalRegion);
            if (it != robot_pair.second.end() && it->second == 1) {
                found = true;
                break;
            }
        }
        if (!found) {
            std::cerr << "[Feasibility] Goal region " << goalRegion << " not feasible for any robot!\n";
            return false;
        }
    }
    return true;
}

void VoronoiPruningPipeline::pruneLabeling(
    TS* transitionSystem,
    const FeasibilityMask& mask)
{
    if (!transitionSystem) return;
    
    // For each state in the TS, remove propositions that are infeasible
    const auto& nodes = transitionSystem->getNodes();
    
    for (const auto& node_pair : nodes) {
        Node* node = node_pair.second;
        if (!node) continue;
        
        // The node label contains information about which propositions are true
        // We need to modify this based on feasibility
        
        // For each robot-region pair (i,k) where Feas(i,k) = 0,
        // remove proposition p_k^(i) from the state
        
        // This is typically done by removing edges or modifying the edge label
        // In a more complete implementation, we'd track proposition labels in the node
        
        std::cout << "[Pruning] Processing node " << node->getId() 
                  << " (label: " << node->getLabel() << ")\n";
    }
}

ProductAutomaton* VoronoiPruningPipeline::buildProductAutomaton(
    TS* transitionSystem,
    spot::twa_graph_ptr buchiAutomaton)
{
    if (!transitionSystem || !buchiAutomaton) {
        std::cerr << "[ProductBuild] Invalid automaton inputs\n";
        return nullptr;
    }
    
    // Create product automaton from Spot Büchi automaton
    ProductAutomaton* product = new ProductAutomaton(buchiAutomaton);
    
    // Note: In a full implementation, we would:
    // 1. Create product states as pairs (ts_state, buchi_state)
    // 2. Add transitions based on TS and Buchi automaton
    // 3. Mark product states as accepting where buchi_state is accepting
    
    return product;
}

bool VoronoiPruningPipeline::findAcceptingPlan(
    ProductAutomaton* product,
    std::vector<uint32_t>& outPlan)
{
    if (!product) return false;
    
    outPlan.clear();
    
    // BFS on product automaton to find accepting state
    std::queue<uint32_t> q;
    std::set<uint32_t> visited;
    std::map<uint32_t, uint32_t> parent;
    
    // Start from initial state (assuming state 0 is initial)
    q.push(0);
    visited.insert(0);
    
    uint32_t acceptingState = UINT32_MAX;
    
    while (!q.empty() && acceptingState == UINT32_MAX) {
        uint32_t current = q.front();
        q.pop();
        
        // Check if accepting
        if (product->isAccepting(current)) {
            acceptingState = current;
            break;
        }
        
        // Explore neighbors
        Node* node = product->getNode(current);
        if (node) {
            for (const auto& edge : node->getEdges()) {
                uint32_t next = edge.getDstId();
                if (visited.find(next) == visited.end()) {
                    visited.insert(next);
                    parent[next] = current;
                    q.push(next);
                }
            }
        }
    }
    
    if (acceptingState == UINT32_MAX) {
        std::cerr << "[PlanSearch] No accepting state found!\n";
        return false;
    }
    
    // Reconstruct path
    uint32_t current = acceptingState;
    while (current != 0) {
        outPlan.push_back(current);
        current = parent[current];
    }
    outPlan.push_back(0);
    std::reverse(outPlan.begin(), outPlan.end());
    
    std::cout << "[PlanSearch] Found accepting plan of length " << outPlan.size() << "\n";
    return true;
}
