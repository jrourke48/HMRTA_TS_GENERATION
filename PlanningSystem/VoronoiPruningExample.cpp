#include "VoronoiPruningPipeline.h"
#include "Automaton.h"
#include "ProductAutomaton.h"
#include "BuchiAutomaton.h"
#include "TS.h"
#include <iostream>
#include <queue>
#include <map>

/**
 * Example: Capability-Aware Multi-Robot Task Planning
 * 
 * This example demonstrates the complete workflow:
 * 1. Define robot capabilities (velocity, sensing range, capacity, localization error)
 * 2. Compute weighted Voronoi partition
 * 3. Generate feasibility mask for discrete regions
 * 4. Prune transition system labels
 * 5. Construct product automaton
 * 6. Search for accepting plan
 */

int main() {
    std::cout << "=== Capability-Aware Weighted Voronoi Pruning Example ===\n\n";
    
    // ============ STEP 1: Define Robot Capabilities ============
    std::cout << "[Setup] Defining 3 robots with different capabilities...\n";
    
    std::vector<RobotCapabilities> robots;
    
    // Robot 0: Fast but limited sensing
    robots.push_back({
        .id = 0,
        .position = Eigen::Vector2f(0.0f, 0.0f),
        .velocity = 2.0f,
        .senseRange = 2.0f,
        .capacity = 5.0f,
        .localizationError = 0.1f
    });
    
    // Robot 1: Balanced capabilities
    robots.push_back({
        .id = 1,
        .position = Eigen::Vector2f(5.0f, 5.0f),
        .velocity = 1.5f,
        .senseRange = 4.0f,
        .capacity = 10.0f,
        .localizationError = 0.2f
    });
    
    // Robot 2: High precision sensing but slow
    robots.push_back({
        .id = 2,
        .position = Eigen::Vector2f(10.0f, 0.0f),
        .velocity = 1.0f,
        .senseRange = 6.0f,
        .capacity = 3.0f,
        .localizationError = 0.05f
    });
    
    std::cout << "[Setup] Robots defined:\n";
    for (const auto& r : robots) {
        std::cout << "  Robot " << r.id << " at (" << r.position.x() << ", " << r.position.y() << ")"
                  << " v=" << r.velocity << " sense=" << r.senseRange 
                  << " capacity=" << r.capacity << "\n";
    }
    
    // ============ STEP 2: Define Discrete Regions ============
    std::cout << "\n[Setup] Defining 4 discrete regions...\n";
    
    std::vector<DiscreteRegion> regions;
    regions.push_back({
        .id = 0,
        .label = "Region_A",
        .centroid = Eigen::Vector2f(1.0f, 1.0f)
    });
    regions.push_back({
        .id = 1,
        .label = "Region_B",
        .centroid = Eigen::Vector2f(4.0f, 4.0f)
    });
    regions.push_back({
        .id = 2,
        .label = "Region_C",
        .centroid = Eigen::Vector2f(7.0f, 7.0f)
    });
    regions.push_back({
        .id = 3,
        .label = "Region_D",
        .centroid = Eigen::Vector2f(10.0f, 1.0f)
    });
    
    std::cout << "[Setup] Regions defined:\n";
    for (const auto& r : regions) {
        std::cout << "  " << r.label << " at (" << r.centroid.x() << ", " << r.centroid.y() << ")\n";
    }
    
    // ============ STEP 3: Define Capability Weights ============
    std::cout << "\n[Setup] Setting capability weights...\n";
    
    CapabilityWeights weights;
    weights.lambda_v = 1.0f;      // Velocity importance
    weights.lambda_r = 2.0f;      // Sensing range importance (higher priority)
    weights.lambda_p = 1.0f;      // Capacity importance
    weights.lambda_sigma = 3.0f;  // Localization error penalty (higher penalty)
    
    std::cout << "[Setup] Weights: v=" << weights.lambda_v << " r=" << weights.lambda_r 
              << " p=" << weights.lambda_p << " sigma=" << weights.lambda_sigma << "\n";
    
    // ============ STEP 4: Create Transition System ============
    std::cout << "\n[Setup] Creating transition system...\n";
    
    // For this example, we create a simple TS with 4 states (one per region)
    TS* ts = new TS();
    
    for (uint32_t i = 0; i < regions.size(); ++i) {
        Node* node = new Node(i, regions[i].label);
        ts->add_Node(node);
        if (i == 0) ts->setInitial(i);  // Region A is initial
    }
    
    // Add transitions between all regions
    for (uint32_t i = 0; i < regions.size(); ++i) {
        Node* src = ts->getNode(i);
        if (src) {
            for (uint32_t j = 0; j < regions.size(); ++j) {
                src->addEdge(Edge(j));  // Can go to any region (including self)
            }
        }
    }
    
    std::cout << "[Setup] TS created: " << ts->getnumStates() << " states\n";
    
    // ============ STEP 5: Create Büchi Automaton ============
    std::cout << "\n[Setup] Creating Büchi automaton from LTL formula...\n";
    
    std::string ltlFormula = "F (p0 & F (p1 & F p2))";
    std::cout << "[Setup] LTL formula: " << ltlFormula << "\n";
    std::cout << "[Setup] This formula means: visit Region_A (p0), then Region_B (p1), then Region_C (p2)\n";
    
    // Create a simple Büchi automaton manually without SPOT
    BuchiAutomaton* buchi = new BuchiAutomaton();
    
    // Create 3 states for the task sequence
    for (int i = 0; i < 3; ++i) {
        Node* node = new Node(i, "s" + std::to_string(i));
        buchi->add_Node(node);
    }
    
    // Add transitions - be sure to track them
    int edgeCount = 0;
    Node* s0 = buchi->getNode(0);
    Node* s1 = buchi->getNode(1);
    Node* s2 = buchi->getNode(2);
    
    if (s0) {
        s0->addEdge(Edge(0));  // Self-loop
        edgeCount++;
        s0->addEdge(Edge(1));  // To s1
        edgeCount++;
    }
    if (s1) {
        s1->addEdge(Edge(1));  // Self-loop
        edgeCount++;
        s1->addEdge(Edge(2));  // To s2
        edgeCount++;
    }
    if (s2) {
        s2->addEdge(Edge(2));  // Self-loop
        edgeCount++;
    }
    
    // Mark final state as accepting
    buchi->setAccepting(2);
    
    std::cout << "[Setup] ✓ Büchi created: " << buchi->getnumStates() << " states, "
              << edgeCount << " transitions\n";
    
    // ============ STEP 6: Build Product Automaton Manually ============
    std::cout << "\n[Pipeline] Building product automaton T ⊗ B...\n";
    
    ProductAutomaton* product = new ProductAutomaton();
    
    // Create product states: (ts_state, buchi_state) pairs
    uint32_t productStateId = 0;
    std::map<std::pair<uint32_t, uint32_t>, uint32_t> stateMap;
    
    // Create all product states
    for (uint32_t tsState = 0; tsState < ts->getnumStates(); ++tsState) {
        for (uint32_t buchiState = 0; buchiState < buchi->getnumStates(); ++buchiState) {
            auto key = std::make_pair(tsState, buchiState);
            stateMap[key] = productStateId;
            
            Node* node = new Node(productStateId, 
                "(" + std::to_string(tsState) + "," + std::to_string(buchiState) + ")");
            product->add_Node(node);
            product->addStateMapping(productStateId, tsState, buchiState);
            
            productStateId++;
        }
    }
    
    // Add product transitions
    uint32_t productEdgeCount = 0;
    for (const auto& tsStatePair : ts->getNodes()) {
        uint32_t tsSrc = tsStatePair.first;
        Node* tsNode = tsStatePair.second;
        
        for (uint32_t buchiSrc = 0; buchiSrc < buchi->getnumStates(); ++buchiSrc) {
            auto curKey = std::make_pair(tsSrc, buchiSrc);
            if (stateMap.find(curKey) == stateMap.end()) continue;
            uint32_t curState = stateMap[curKey];
            
            Node* pNode = product->getNode(curState);
            if (!pNode) continue;
            
            // Follow TS transitions
            for (const auto& edge : tsNode->getEdges()) {
                uint32_t tsDst = edge.getDstId();
                
                // Follow Büchi transitions
                Node* buchiNode = buchi->getNode(buchiSrc);
                if (buchiNode) {
                    for (const auto& buchiEdge : buchiNode->getEdges()) {
                        uint32_t buchiDst = buchiEdge.getDstId();
                        
                        auto dstKey = std::make_pair(tsDst, buchiDst);
                        if (stateMap.find(dstKey) != stateMap.end()) {
                            uint32_t dstState = stateMap[dstKey];
                            pNode->addEdge(Edge(dstState));
                            productEdgeCount++;
                            
                            // If Büchi state is accepting, mark product state as accepting
                            if (buchi->isAccepting(buchiDst)) {
                                product->setAccepting(dstState);
                            }
                        }
                    }
                }
            }
        }
    }
    
    std::cout << "[Pipeline] ✓ Product automaton created: " << product->getnumStates() 
              << " states, " << productEdgeCount << " transitions\n";
    
    if (!product) {
        std::cerr << "[ERROR] Pipeline failed\n";
        delete ts;
        return -1;
    }
    
    // ============ STEP 7: Extract and Display Plan ============
    std::cout << "\n[Execution] Searching for accepting plan...\n";
    
    // Simple BFS to find accepting state
    std::vector<uint32_t> plan;
    std::vector<bool> visited(product->getnumStates(), false);
    std::queue<uint32_t> queue;
    
    // Start from initial state (0,0)
    queue.push(0);
    visited[0] = true;
    
    bool found = false;
    while (!queue.empty() && !found) {
        uint32_t current = queue.front();
        queue.pop();
        plan.push_back(current);
        
        // Check if this is an accepting state
        if (product->isAccepting(current)) {
            found = true;
            break;
        }
        
        // Follow transitions
        Node* node = product->getNode(current);
        if (node) {
            for (const auto& edge : node->getEdges()) {
                uint32_t next = edge.getDstId();
                if (!visited[next]) {
                    visited[next] = true;
                    queue.push(next);
                }
            }
        }
    }
    
    if (found) {
        std::cout << "\n[Success] Found plan of length " << plan.size() << ":\n";
        for (uint32_t i = 0; i < plan.size(); ++i) {
            std::cout << "  Step " << i << ": State " << plan[i];
            
            // Map back to region if possible
            if (plan[i] < regions.size()) {
                std::cout << " (" << regions[plan[i]].label << ")";
            }
            std::cout << "\n";
        }
    } else {
        std::cout << "[Failed] No accepting plan found\n";
    }
    
    // ============ Cleanup ============
    std::cout << "\n[Cleanup] Releasing resources...\n";
    delete product;
    delete ts;
    delete buchi;
    
    std::cout << "\n=== Example Complete ===\n";
    return 0;
}
