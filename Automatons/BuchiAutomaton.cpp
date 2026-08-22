#include "BuchiAutomaton.h"
#include <algorithm>
#include <stack>
#include <set>

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

std::vector<std::vector<uint16_t>> BuchiAutomaton::getTrueAPs(uint16_t srcId, uint16_t dstId) const {
    std::vector<std::vector<uint16_t>> trueAPsList;
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return trueAPsList;
    
    Node* srcNode = it->second;
    std::vector<Edge> edges = srcNode->getEdges();
    
    for (const auto& edge : edges) {
        if (edge.getDstId() == dstId) {
            for (const auto& apVec : edge.getTrueAPs()) {
                //check if the vector is over length 1 if so skip the and clauses for now
                if (apVec.size() <= 1) {
                    trueAPsList.push_back(apVec);
                }
            }
        }
    }
    return trueAPsList;
}