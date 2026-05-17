#include "BuchiAutomaton.h"
#include <algorithm>

BuchiAutomaton::~BuchiAutomaton() {
    // Clean up dynamically allocated nodes
    // Note: Node pointers should be managed by a smart pointer in a production system
}

void BuchiAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool BuchiAutomaton::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}
std::vector<std::string> BuchiAutomaton::getEdgeLabels(uint16_t srcId, uint16_t dstId) const {
    std::vector<std::string> labels;
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return labels;
    
    Node* srcNode = it->second;
    std::vector<Edge> edges = srcNode->getEdges();
    
    for (const auto& edge : edges) {
        if (edge.getDstId() == dstId) {
            labels.push_back(edge.getLabel());
        }
    }
    return labels;
}
