#include "ProductAutomaton.h"
#include <sstream>
#include <map>
#include <set>
#include <stack>
#include <algorithm>
#include <spot/twaalgos/dot.hh>

ProductAutomaton::ProductAutomaton() {
}

ProductAutomaton::ProductAutomaton(spot::twa_graph_ptr spotAutomaton) {
    if (!spotAutomaton) return;
    this->spotAutomaton = spotAutomaton;
    
    // Generate DOT representation from Spot automaton
    std::ostringstream dotStream;
    spot::print_dot(dotStream, spotAutomaton);
    std::string dotContent = dotStream.str();
    initStateMapping(dotContent);  // Initialize state mapping from DOT content
    // Parse the DOT content to build the automaton edges and accepting states
    parseProductFromDot(dotContent);
}

// Parse product automaton from DOT representation
void ProductAutomaton::parseProductFromDot(const std::string& dotContent) {
    std::istringstream stream(dotContent);
    std::string line;
    std::map<unsigned, std::vector<std::pair<unsigned, std::string>>> edges;
    std::set<unsigned> acceptingNodeIds;

    // Create nodes based on all entries in stateMapping (already populated)
    for (const auto& mappingPair : stateMapping) {
        uint16_t nodeId = mappingPair.first;
        const std::string& label = mappingPair.second;
        Node* node = new Node(nodeId, label, true);
        std::pair<uint16_t, std::vector<uint16_t>> productStates = node->getProductStates();  // Initialize product states based on label
        // for (uint16_t robotState : productStates.second) {
        //     // You can perform additional processing with robotState if needed
        //     std::cout << "Node " << nodeId << " has robot state: " << robotState << std::endl;
        // }
        add_Node(node);
    }
    
    while (std::getline(stream, line)) {
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for accepting state marker: peripheries=2
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX && line.find("peripheries=2") != std::string::npos) {
                    acceptingNodeIds.insert(nodeId);
                }
            }
            continue;
        }
        
        // Check for edge definition: <src> -> <dst> [label="..."]
        size_t arrow_pos = line.find("->");
        if (arrow_pos != std::string::npos) {
            // Extract source
            unsigned src;
            std::istringstream src_stream(line.substr(0, arrow_pos));
            if (!(src_stream >> src)) continue;
            
            // Extract destination
            size_t after_arrow = arrow_pos + 2;
            size_t bracket_start = line.find('[', after_arrow);
            if (bracket_start == std::string::npos) continue;
            
            std::string dst_str = line.substr(after_arrow, bracket_start - after_arrow);
            dst_str.erase(0, dst_str.find_first_not_of(" \t"));
            dst_str.erase(dst_str.find_last_not_of(" \t") + 1);
            
            unsigned dst;
            if (!(std::istringstream(dst_str) >> dst)) continue;
            
            // Skip initial edge (I -> state)
            if (src == UINT_MAX) continue;
            
            edges[src].push_back(std::make_pair(dst, ""));
        }
    }
    
    // Add edges
    for (const auto& srcEntry : edges) {
        Node* srcNode = getNode(static_cast<uint16_t>(srcEntry.first));
        if (srcNode != nullptr) {
            for (const auto& dstLabelPair : srcEntry.second) {
                unsigned dst = dstLabelPair.first;
                
                if (dst <= UINT16_MAX) {
                    Edge e(static_cast<uint16_t>(dst));
                    srcNode->addEdge(e);
                    numEdges++;
                }
            }
        }
    }
    
    // Mark accepting states (peripheries=2 in DOT)
    for (unsigned nodeId : acceptingNodeIds) {
        if (nodeId <= UINT16_MAX) {
            setAccepting(static_cast<uint16_t>(nodeId));
        }
    }
}

// Extract label from DOT bracket content
std::string ProductAutomaton::extractLabelFromDotBrackets(const std::string& content) const {
    size_t label_pos = content.find("label=");
    if (label_pos == std::string::npos) return "true";
    
    label_pos += 6;  // strlen("label=")
    
    // Skip whitespace and opening quote
    while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '"')) {
        label_pos++;
    }
    
    std::string label;
    while (label_pos < content.length()) {
        char c = content[label_pos];
        
        if (c == '\\' && label_pos + 1 < content.length()) {
            char next = content[label_pos + 1];
            if (next == '"') {
                label += '"';
                label_pos += 2;
            } else if (next == 'n') {
                label += ':';
                label_pos += 2;
                
                // Skip whitespace after newline
                while (label_pos < content.length() && (content[label_pos] == ' ' || content[label_pos] == '\t')) {
                    label_pos++;
                }
                
                // Grab acceptance marks
                while (label_pos < content.length()) {
                    char c = content[label_pos];
                    if (c == '"') break;
                    label += c;
                    label_pos++;
                }
                break;
            } else {
                label += c;
                label_pos++;
            }
        } else if (c == '"') {
            break;
        } else {
            label += c;
            label_pos++;
        }
    }
    
    // Clean up label
    label.erase(label.find_last_not_of(" \t") + 1);
    return label.empty() ? "true" : label;
}
ProductAutomaton::ProductAutomaton(const TS& ts, const MultiRobotSystem& mrs, const BuchiAutomaton& buchiAutomaton) {
    // Initialize product automaton based on the individual components
    // This is a placeholder implementation and should be replaced with actual logic
    if (buchiAutomaton.getNumStates() > 100 || ts.getNumStates() > 20 || mrs.getNumRobots() > 10) return;
    
    // Get the number of robots in the multi-robot system 
    uint8_t numRobots = mrs.getNumRobots();
    
    // Convert the Buchi automaton to a Spot automaton
    spot::twa_graph_ptr buchiSpot = buchiAutomaton.getSpotAutomaton();
    
    // Convert the transition system to a Spot automaton
    spot::twa_graph_ptr tsSpot = ts.toSpotAutomaton(buchiSpot->get_dict());

    // Initialize the product automaton as the transition system Spot automaton
    spot::twa_graph_ptr productSpot = spot::product(tsSpot, tsSpot);
    
    // Initialize state mapping with TS nodes
    std::ostringstream initialDotStream;
    spot::print_dot(initialDotStream, productSpot);
    std::string initialDotContent = initialDotStream.str();
    initStateMapping(initialDotContent);
    
    // Create synchronized product of each TS for each robot
    for (uint8_t i = 2; i < numRobots; ++i) {
        productSpot = spot::product(tsSpot, productSpot);
        // Generate DOT representation from Spot automaton
        std::ostringstream dotStream;
        spot::print_dot(dotStream, productSpot);
        std::string dotContent = dotStream.str();
        // Update state mapping for the current product automaton and every time we create a new product we need to update the state mapping for the new product automaton
        updateStateMapping(dotContent);

    }
    // do the final synchronization of the product automaton
    productSpot = spot::product(buchiSpot, productSpot);
    // Export and update state mapping for final product
    std::ostringstream finalDotStream;
    spot::print_dot(finalDotStream, productSpot);
    std::string finalDotContent = finalDotStream.str();
    updateStateMapping(finalDotContent);
    
    // Store the Spot automaton and parse it using the already-populated stateMapping
    this->spotAutomaton = productSpot;
    
    // Now parse to build the internal representation (nodes, edges, accepting states)
    // This will use the stateMapping we just populated above
    std::ostringstream dotStream;
    spot::print_dot(dotStream, productSpot);
    parseProductFromDot(dotStream.str());
}

std::vector<uint16_t> ProductAutomaton::OptimalAcceptingPath(uint16_t startState) {
    std::vector<uint16_t> result;
    // TODO: Implement optimal path finding algorithm
    return result;
}

ProductAutomaton::~ProductAutomaton() {
    // Clean up dynamically allocated nodes
    for (auto& pair : nodeMap) {
        delete pair.second;
    }
    nodeMap.clear();
}

void ProductAutomaton::setAccepting(uint16_t stateId) {
    // Add to accepting states if not already present
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    if (it == acceptingStates.end()) {
        acceptingStates.push_back(stateId);
    }
}

bool ProductAutomaton::isAccepting(uint16_t stateId) const {
    auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
    return it != acceptingStates.end();
}

const std::vector<uint16_t>& ProductAutomaton::getAcceptingStates() const {
    return acceptingStates;
}

void ProductAutomaton::add_Node(Node* node) {
    if (node == nullptr) return;
    
    uint16_t nodeId = node->getId();
    
    // Add to nodeMap for quick access
    nodeMap[nodeId] = node;
    
    // Increment node count
    numNodes++;
}

bool ProductAutomaton::isAdjacent(uint16_t srcId, uint16_t dstId) const {
    // Find source node
    auto it = nodeMap.find(srcId);
    if (it == nodeMap.end()) return false;
    
    Node* srcNode = it->second;
    
    // Check if there's an edge from srcNode to dstId
    return srcNode->isAdjacent(dstId);
}
void ProductAutomaton::addStateMapping(uint16_t productState, const std::string& label) {
    // Store the label string at this product state key
    stateMapping[productState] = label;
}

void ProductAutomaton::initStateMapping(const std::string& dotContent) {
    std::istringstream stream(dotContent);
    std::string line;
    
    while (std::getline(stream, line)) {
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for node definition: <id> [label="..." peripheries=2 ...]
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX) {
                    // Extract label from node definition
                    size_t bracket_end = line.find(']', bracket_start);
                    if (bracket_end == std::string::npos) bracket_end = line.length();
                    
                    std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    std::string nodeLabel = extractLabelFromDotBrackets(bracket_content);
                    
                    stateMapping[static_cast<uint16_t>(nodeId)] = nodeLabel;
                }
            }
            continue;
        }
    }
}
void ProductAutomaton::updateStateMapping(const std::string& dotContent) {
    // Save a copy of the current stateMapping to reference old labels
    std::map<uint16_t, std::string> oldStateMapping = stateMapping;
    
    std::istringstream stream(dotContent);
    std::string line;
    std::set<unsigned> seenNodes;
    
     while (std::getline(stream, line)) {
        // Trim line
        line.erase(0, line.find_first_not_of(" \t"));
        line.erase(line.find_last_not_of(" \t") + 1);
        
        if (line.empty() || line[0] == '}' || line[0] == '#') continue;
        
        // Check for node definition: <id> [label="..." peripheries=2 ...]
        size_t bracket_start = line.find('[');
        if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
            unsigned nodeId;
            std::istringstream iss(line);
            if (iss >> nodeId) {
                if (nodeId != UINT_MAX) {
                    seenNodes.insert(nodeId);
                    
                    // Extract label from node definition
                    size_t bracket_end = line.find(']', bracket_start);
                    if (bracket_end == std::string::npos) bracket_end = line.length();
                    
                    std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                    std::string nodeLabel = extractLabelFromDotBrackets(bracket_content);
                    
                    // The node label needs to be updated in the mapping
                    // Look up the old ID from the saved oldStateMapping
                    uint16_t oldId = getIdFromLabel(nodeLabel);
                    if (oldStateMapping.find(oldId) != oldStateMapping.end()) {
                        std::string oldLabel = oldStateMapping[oldId];
                        std::string expanded = replaceLabel(nodeLabel, oldLabel);
                        stateMapping[nodeId] = expanded;
                    } else {
                        stateMapping[nodeId] = nodeLabel;
                    }
                    
                }
            }
            continue;
        }
        
    }
}

uint16_t ProductAutomaton::getIdFromLabel(const std::string& label) const {
    // The ID is the number after the comma
    size_t commaPos = label.find(',');
    if (commaPos != std::string::npos) {
        // Extract everything after the comma
        std::string afterComma = label.substr(commaPos + 1);
        // Trim whitespace
        afterComma.erase(0, afterComma.find_first_not_of(" \t"));
        afterComma.erase(afterComma.find_last_not_of(" \t") + 1);
        return static_cast<uint16_t>(std::stoi(afterComma));
    } else {
        // No comma found, return the whole label as the ID
        return static_cast<uint16_t>(std::stoi(label));
    }
}

std::string ProductAutomaton::replaceLabel(const std::string& oldLabel, const std::string& additionalLabel) {
    // Find the first comma
    size_t commaPos = oldLabel.find(',');
    if (commaPos != std::string::npos) {
        // Keep everything up to and including the comma, replace everything after
        return oldLabel.substr(0, commaPos + 1) + additionalLabel;
    } else {
        // No comma found, append with comma
        std::string newLabel = oldLabel;
        if (!newLabel.empty()) {
            newLabel += ",";
        }
        newLabel += additionalLabel;
        return newLabel;
    }
}


std::string ProductAutomaton::getStateMapping(uint16_t productState) const {
    auto it = stateMapping.find(productState);
    if (it != stateMapping.end()) {
        return it->second;
    }
    
    // Return empty string if state not found
    return "";
}


