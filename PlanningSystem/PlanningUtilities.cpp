#include "PlanningUtilities.h"
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <iostream>
#include <set>

namespace PlanningUtils {

    // ========== Test Scenario Builders ==========
    
    TestScenario buildRobotCircleScenario() {
        TestScenario scenario;
        scenario.name = "RobotCircle";
        scenario.expectedSuccess = true;
        
        // Three robots arranged in circle
        float angle_step = 2.0f * M_PI / 3.0f;
        float radius = 5.0f;
        
        for (int i = 0; i < 3; ++i) {
            float angle = i * angle_step;
            RobotCapabilities robot;
            robot.id = i;
            robot.position = Eigen::Vector2f(radius * cos(angle), radius * sin(angle));
            robot.velocity = 1.0f + 0.5f * i;
            robot.senseRange = 3.0f + i;
            robot.capacity = 10.0f;
            robot.localizationError = 0.1f;
            scenario.robots.push_back(robot);
        }
        
        // Three regions at center and corners
        scenario.regions.push_back({0, "Center", Eigen::Vector2f(0, 0)});
        scenario.regions.push_back({1, "North", Eigen::Vector2f(0, 3)});
        scenario.regions.push_back({2, "South", Eigen::Vector2f(0, -3)});
        
        scenario.ltlFormula = "F (p0 & F (p1 & F p2))";
        
        return scenario;
    }
    
    TestScenario buildGridScenario() {
        TestScenario scenario;
        scenario.name = "Grid";
        scenario.expectedSuccess = true;
        
        // Grid of robots
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                RobotCapabilities robot;
                robot.id = i * 2 + j;
                robot.position = Eigen::Vector2f(i * 5.0f, j * 5.0f);
                robot.velocity = 1.5f;
                robot.senseRange = 4.0f;
                robot.capacity = 15.0f;
                robot.localizationError = 0.1f;
                scenario.robots.push_back(robot);
            }
        }
        
        // Grid of regions
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                DiscreteRegion region;
                region.id = i * 3 + j;
                region.label = "Region_" + std::to_string(region.id);
                region.centroid = Eigen::Vector2f(i * 3.0f, j * 3.0f);
                scenario.regions.push_back(region);
            }
        }
        
        scenario.ltlFormula = "F (p0 & F (p4 & F p8))";
        
        return scenario;
    }
    
    TestScenario buildAsymmetricScenario() {
        TestScenario scenario;
        scenario.name = "Asymmetric";
        scenario.expectedSuccess = false;
        
        // One high-capability robot at origin
        RobotCapabilities robot1;
        robot1.id = 0;
        robot1.position = Eigen::Vector2f(0, 0);
        robot1.velocity = 3.0f;
        robot1.senseRange = 10.0f;
        robot1.capacity = 50.0f;
        robot1.localizationError = 0.05f;
        scenario.robots.push_back(robot1);
        
        // One low-capability robot far away
        RobotCapabilities robot2;
        robot2.id = 1;
        robot2.position = Eigen::Vector2f(50, 50);
        robot2.velocity = 0.5f;
        robot2.senseRange = 1.0f;
        robot2.capacity = 1.0f;
        robot2.localizationError = 0.5f;
        scenario.robots.push_back(robot2);
        
        // Regions clustered near robot1
        scenario.regions.push_back({0, "RegionA", Eigen::Vector2f(1, 1)});
        scenario.regions.push_back({1, "RegionB", Eigen::Vector2f(2, 2)});
        
        // Region far from both (should be infeasible)
        scenario.regions.push_back({2, "RegionC", Eigen::Vector2f(100, 100)});
        
        scenario.ltlFormula = "F (p0 & F (p1 & F p2))";  // Will fail because p2 unreachable
        
        return scenario;
    }
    
    // ========== Visualization Utilities ==========
    
    void exportPowerDiagramSVG(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        float mapWidth, float mapHeight,
        const std::string& filename)
    {
        std::ofstream out(filename);
        
        out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
        out << "<svg width=\"" << mapWidth << "\" height=\"" << mapHeight 
            << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";
        
        // Draw background
        out << "<rect width=\"" << mapWidth << "\" height=\"" << mapHeight 
            << "\" fill=\"white\" stroke=\"black\"/>\n";
        
        // Draw regions
        for (const auto& region : regions) {
            float x = region.centroid.x() * 10.0f;
            float y = region.centroid.y() * 10.0f;
            out << "<circle cx=\"" << x << "\" cy=\"" << y 
                << "\" r=\"5\" fill=\"lightblue\" stroke=\"blue\"/>\n";
            out << "<text x=\"" << x << "\" y=\"" << y << "\" text-anchor=\"middle\">"
                << region.label << "</text>\n";
        }
        
        // Draw robots
        for (const auto& robot : robots) {
            float x = robot.position.x() * 10.0f;
            float y = robot.position.y() * 10.0f;
            out << "<circle cx=\"" << x << "\" cy=\"" << y 
                << "\" r=\"8\" fill=\"red\" stroke=\"darkred\"/>\n";
            out << "<text x=\"" << x << "\" y=\"" << y << "\" text-anchor=\"middle\" fill=\"white\">"
                << "R" << robot.id << "</text>\n";
        }
        
        out << "</svg>\n";
        out.close();
        
        std::cout << "[Export] Power diagram SVG: " << filename << "\n";
    }
    
    void exportFeasibilityCSV(
        const FeasibilityMask& mask,
        const std::vector<DiscreteRegion>& regions,
        const std::string& filename)
    {
        std::ofstream out(filename);
        
        // Header
        out << "Robot";
        for (const auto& region : regions) {
            out << "," << region.label;
        }
        out << "\n";
        
        // Data
        for (const auto& robot_pair : mask) {
            out << "Robot" << robot_pair.first;
            for (const auto& region : regions) {
                auto it = robot_pair.second.find(region.id);
                out << "," << (it != robot_pair.second.end() && it->second ? "1" : "0");
            }
            out << "\n";
        }
        
        out.close();
        std::cout << "[Export] Feasibility CSV: " << filename << "\n";
    }
    
    void exportProductDOT(
        const ProductAutomaton* product,
        const std::vector<DiscreteRegion>& regions,
        const std::string& filename)
    {
        if (!product) return;
        
        std::ofstream out(filename);
        out << "digraph ProductAutomaton {\n";
        out << "  rankdir=LR;\n";
        out << "  node [shape=circle];\n";
        
        const auto& nodes = product->getNodes();
        for (const auto& node_pair : nodes) {
            uint32_t nodeId = node_pair.first;
            bool isAccepting = product->isAccepting(nodeId);
            
            out << "  s" << nodeId << " [";
            if (isAccepting) out << "shape=doublecircle, ";
            out << "label=\"" << nodeId << "\"];\n";
        }
        
        out << "\n";
        
        for (const auto& node_pair : nodes) {
            uint32_t srcId = node_pair.first;
            Node* node = node_pair.second;
            
            if (node) {
                for (const auto& edge : node->getEdges()) {
                    uint32_t dstId = edge.getDstId();
                    out << "  s" << srcId << " -> s" << dstId << ";\n";
                }
            }
        }
        
        out << "}\n";
        out.close();
        std::cout << "[Export] Product automaton DOT: " << filename << "\n";
    }
    
    // ========== Analysis Utilities ==========
    
    SystemMetrics analyzeSystem(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        const FeasibilityMask& mask)
    {
        SystemMetrics metrics;
        metrics.numRobots = robots.size();
        metrics.numRegions = regions.size();
        
        // Average robot capability
        float totalCapability = 0.0f;
        for (const auto& r : robots) {
            totalCapability += r.velocity + r.senseRange + r.capacity;
        }
        metrics.avgRobotCapability = totalCapability / (3.0f * robots.size());
        
        // Capability variance
        float variance = 0.0f;
        for (const auto& r : robots) {
            float capability = (r.velocity + r.senseRange + r.capacity) / 3.0f;
            variance += (capability - metrics.avgRobotCapability) * 
                       (capability - metrics.avgRobotCapability);
        }
        metrics.capabilityVariance = variance / robots.size();
        
        // Feasible regions per robot
        uint32_t totalFeasible = 0;
        for (const auto& robot_pair : mask) {
            uint32_t count = 0;
            for (const auto& region_pair : robot_pair.second) {
                if (region_pair.second) count++;
            }
            totalFeasible += count;
        }
        metrics.avgFeasibleRegionsPerRobot = totalFeasible / robots.size();
        
        // Coverage ratio
        std::set<uint32_t> coveredRegions;
        for (const auto& robot_pair : mask) {
            for (const auto& region_pair : robot_pair.second) {
                if (region_pair.second) {
                    coveredRegions.insert(region_pair.first);
                }
            }
        }
        metrics.coverageRatio = static_cast<float>(coveredRegions.size()) / regions.size();
        
        return metrics;
    }
    
    void printSystemAnalysis(const SystemMetrics& metrics) {
        std::cout << "\n========== SYSTEM ANALYSIS ==========\n";
        std::cout << "Robots: " << metrics.numRobots << "\n";
        std::cout << "Regions: " << metrics.numRegions << "\n";
        std::cout << "Avg Robot Capability: " << metrics.avgRobotCapability << "\n";
        std::cout << "Capability Variance: " << metrics.capabilityVariance << "\n";
        std::cout << "Avg Feasible Regions/Robot: " << metrics.avgFeasibleRegionsPerRobot << "\n";
        std::cout << "Coverage Ratio: " << (metrics.coverageRatio * 100.0f) << "%\n";
        std::cout << "====================================\n\n";
    }
    
    bool validateFeasibilityMask(
        const FeasibilityMask& mask,
        uint32_t numRobots,
        uint32_t numRegions)
    {
        if (mask.size() != numRobots) {
            std::cerr << "[Validation] Expected " << numRobots << " robots but found " 
                      << mask.size() << "\n";
            return false;
        }
        
        for (const auto& robot_pair : mask) {
            if (robot_pair.second.size() != numRegions) {
                std::cerr << "[Validation] Robot " << robot_pair.first 
                          << " has " << robot_pair.second.size() << " regions instead of " 
                          << numRegions << "\n";
                return false;
            }
        }
        
        return true;
    }
    
    // ========== Performance Profiling ==========
    
    struct PipelineTimer {
        std::chrono::high_resolution_clock::time_point start;
        std::chrono::high_resolution_clock::time_point end;
        
        void begin() { start = std::chrono::high_resolution_clock::now(); }
        double elapsed_ms() {
            end = std::chrono::high_resolution_clock::now();
            return std::chrono::duration<double, std::milli>(end - start).count();
        }
    };
    
    PerformanceProfile profilePipeline(
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions,
        TS* transitionSystem,
        spot::twa_graph_ptr buchi,
        const CapabilityWeights& weights)
    {
        PerformanceProfile profile;
        PipelineTimer timer;
        auto overallStart = std::chrono::high_resolution_clock::now();
        
        VoronoiPruningPipeline pipeline;
        
        // Time power diagram
        timer.begin();
        pipeline.computePowerDiagram(robots, weights);
        profile.powerDiagramTime = timer.elapsed_ms();
        
        // Time feasibility
        timer.begin();
        FeasibilityMask mask;
        pipeline.computeFeasibilityMask(robots, regions, mask);
        profile.feasibilityTime = timer.elapsed_ms();
        
        // Time product construction
        timer.begin();
        ProductAutomaton* product = pipeline.buildProductAutomaton(transitionSystem, buchi);
        profile.productConstructionTime = timer.elapsed_ms();
        
        if (product) {
            profile.productStates = product->getnumStates();
            profile.productEdges = product->getnumEdges();
            
            // Time plan search
            timer.begin();
            std::vector<uint32_t> plan;
            pipeline.findAcceptingPlan(product, plan);
            profile.planSearchTime = timer.elapsed_ms();
            
            delete product;
        }
        
        auto overallEnd = std::chrono::high_resolution_clock::now();
        profile.totalTime = std::chrono::duration<double, std::milli>(overallEnd - overallStart).count();
        
        return profile;
    }
    
    void printPerformanceReport(const PerformanceProfile& profile) {
        std::cout << "\n========== PERFORMANCE PROFILE ==========\n";
        std::cout << "Power Diagram:           " << profile.powerDiagramTime << " ms\n";
        std::cout << "Feasibility Computation: " << profile.feasibilityTime << " ms\n";
        std::cout << "Product Construction:    " << profile.productConstructionTime << " ms\n";
        std::cout << "Plan Search:             " << profile.planSearchTime << " ms\n";
        std::cout << "Total:                   " << profile.totalTime << " ms\n";
        std::cout << "Product Automaton Size:  " << profile.productStates << " states, " 
                  << profile.productEdges << " edges\n";
        std::cout << "========================================\n\n";
    }
    
    // ========== Debugging Utilities ==========
    
    void printFeasibilityMatrix(
        const FeasibilityMask& mask,
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions)
    {
        std::cout << "\n========== FEASIBILITY MATRIX ==========\n";
        std::cout << "Robot/Region";
        for (const auto& r : regions) {
            std::cout << "\t" << r.label;
        }
        std::cout << "\n";
        
        for (const auto& robot_pair : mask) {
            std::cout << "Robot" << robot_pair.first;
            for (const auto& region : regions) {
                auto it = robot_pair.second.find(region.id);
                std::cout << "\t" << (it != robot_pair.second.end() && it->second ? "✓" : "✗");
            }
            std::cout << "\n";
        }
        std::cout << "=======================================\n\n";
    }
    
    void tracePowerDistances(
        const Eigen::Vector2f& point,
        const std::vector<RobotCapabilities>& robots,
        const CapabilityWeights& weights)
    {
        std::cout << "\n[Trace] Power distances for point (" << point.x() << ", " << point.y() << "):\n";
        
        for (const auto& robot : robots) {
            float dx = point.x() - robot.position.x();
            float dy = point.y() - robot.position.y();
            float distSq = dx * dx + dy * dy;
            
            float w = weights.lambda_v * robot.velocity * robot.velocity
                    + weights.lambda_r * robot.senseRange * robot.senseRange
                    + weights.lambda_p * robot.capacity * robot.capacity
                    - weights.lambda_sigma * robot.localizationError * robot.localizationError;
            
            float powerDist = distSq - w;
            
            std::cout << "  Robot " << robot.id << ": distance²=" << distSq 
                      << " weight=" << w << " power_dist=" << powerDist << "\n";
        }
    }
    
    std::string generateExecutionLog(
        const std::vector<uint32_t>& plan,
        const std::vector<RobotCapabilities>& robots,
        const std::vector<DiscreteRegion>& regions)
    {
        std::stringstream ss;
        ss << "=== EXECUTION LOG ===\n";
        ss << "Plan length: " << plan.size() << " steps\n\n";
        
        for (uint32_t i = 0; i < plan.size(); ++i) {
            ss << "Step " << i << ": ";
            if (plan[i] < regions.size()) {
                ss << regions[plan[i]].label;
            }
            ss << " (state " << plan[i] << ")\n";
        }
        
        return ss.str();
    }

} // namespace PlanningUtils
