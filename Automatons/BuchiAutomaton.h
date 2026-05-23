#ifndef BUCHI_AUTOMATON_H
#define BUCHI_AUTOMATON_H

#include "Automaton.h"
#include <cstdint>
#include <vector>
#include <string>
#include <sstream>
#include <memory>
#include <algorithm>
#include <fstream>
#include <cstdlib>
#include <map>
#include <set>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/tl/parse.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/tl/formula.hh>
#include "../Task Batch Planning Decision Tree/LTLFormula/LTLFormula.h"
#include "../Task Batch Planning Decision Tree/LTLFormula/BatchAtomicProposition.h"

class BuchiAutomaton : public Automaton {
private:
    std::vector<uint16_t> acceptingStates;  // Set of accepting states (Büchi accepting states)
    const LTLFormula* ltlFormula;  // LTL formula associated with the Büchi automaton (externally owned)
    spot::twa_graph_ptr spotAutomaton;  // Store the Spot automaton for visualization
    uint16_t initialState;  // Initial state of the Büchi automaton

public:
    // Constructor that takes a pointer to an LTL formula object (externally owned)
    BuchiAutomaton(const LTLFormula* formula) : ltlFormula(formula) {
        try {
            // Extract spot formula from LTLFormula and build automaton
            spot::translator trans;
            spot::twa_graph_ptr buchiAut = trans.run(ltlFormula->getSpotFormula());
            // Convert spot buchi to our buchi automaton
            fromSpotAutomaton(buchiAut);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to create Buchi automaton from formula: " + std::string(e.what()));
        }
    }
    
    ~BuchiAutomaton() override;
    // Override pure virtual methods from Automaton
    void add_Node(Node* node) override;
    bool isAdjacent(uint16_t srcId, uint16_t dstId) const override;

    // Büchi-specific methods
    // Getters for initial and accepting states
    void setInitial(uint16_t stateId) {
        initialState = stateId;
    };
    uint16_t getInitialState() const {
        return initialState;
    };
    // Get the list of accepting states
    void setAccepting(uint16_t stateId) {
        // Add to accepting states if not already present
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        if (it == acceptingStates.end()) {
            acceptingStates.push_back(stateId);
        }
    };
    // Check if a state is an accepting state
    bool isAcceptingState(uint16_t stateId) const {
        return std::find(acceptingStates.begin(), acceptingStates.end(), stateId) != acceptingStates.end();
    };
    //get the edge label for a given source and destination
    std::vector<std::string> getEdgeLabels(uint16_t srcId, uint16_t dstId) const;

    void fromSpotAutomaton(spot::twa_graph_ptr spotAutomaton) {
        if (!spotAutomaton) return;
        
        // Store the spot automaton for visualization
        this->spotAutomaton = spotAutomaton;
        
        // Clear existing data
        nodeMap.clear();
        acceptingStates.clear();
        numNodes = 0;
        numEdges = 0;
        
        // Generate DOT and parse it completely
        std::ostringstream dotStream;
        spot::print_dot(dotStream, spotAutomaton);
        std::string dotContent = dotStream.str();
        
        // Parse DOT to extract nodes, edges, initial state, and accepting states
        parseBuchiFromDot(dotContent);
    };

    // Comprehensive DOT parser that builds the Büchi automaton
    void parseBuchiFromDot(const std::string& dotContent) {
        std::istringstream stream(dotContent);
        std::string line;
        initialState = 0;  // Default to state 0
        std::map<unsigned, std::vector<std::pair<unsigned, std::string>>> edges;  // src -> [(dst, label)]
        std::set<unsigned> seenNodes;
        
        while (std::getline(stream, line)) {
            // Trim line
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);
            
            if (line.empty() || line[0] == '}' || line[0] == '#') continue;
            
            // Check for initial state indicator: I -> <state>
            if (line.find("I ->") == 0) {
                size_t state_start = line.find_last_of(" \t");
                if (state_start != std::string::npos) {
                    std::string stateStr = line.substr(state_start + 1);
                    std::istringstream iss(stateStr);
                    iss >> initialState;  // Store the initial state
                }
                continue;
            }
            
            // Check for node definition: <id> [label="..."]
            size_t bracket_start = line.find('[');
            if (bracket_start != std::string::npos && line.find("->") == std::string::npos) {
                unsigned nodeId;
                std::istringstream iss(line);
                if (iss >> nodeId) {
                    // Check if this is the invisible initial node
                    if (nodeId != UINT_MAX) {  // Skip if parsing failed
                        seenNodes.insert(nodeId);
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
                
                // Extract destination and bracket content
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
                
                seenNodes.insert(src);
                seenNodes.insert(dst);
                
                // Extract label and acceptance info from brackets
                size_t bracket_end = line.find(']', bracket_start);
                if (bracket_end == std::string::npos) bracket_end = line.length();
                
                std::string bracket_content = line.substr(bracket_start + 1, bracket_end - bracket_start - 1);
                std::string edgeLabel = extractLabelFromDotBrackets(bracket_content);
                
                // Check for acceptance mark {0} in the bracket content
                bool hasAcceptanceMark = (bracket_content.find("{0}") != std::string::npos);
                if (hasAcceptanceMark) {
                    edgeLabel += " {0}";  // Append to label for later detection
                }
                
                edges[src].push_back(std::make_pair(dst, edgeLabel));
            }
        }
        
        // Create nodes based on discovered state IDs
        for (unsigned nodeId : seenNodes) {
            Node* node = new Node(nodeId);
            add_Node(node);
        }
        
        // Add edges
        for (const auto& srcEntry : edges) {
            Node* srcNode = getNode(srcEntry.first);
            if (srcNode != nullptr) {
                for (const auto& dstLabelPair : srcEntry.second) {
                    unsigned dst = dstLabelPair.first;
                    std::string label = dstLabelPair.second;
                    
                    // Check if label contains acceptance mark {0}
                    bool isAccepting = label.find("{0}") != std::string::npos;
                    if (isAccepting) {
                        setAccepting(dst);
                    }
                    
                    Edge e(dst, label, 1);
                    srcNode->addEdge(e);
                    numEdges++;
                }
            }
        }
    }

    // Extract label from DOT bracket content, handling escaped quotes and newlines
    std::string extractLabelFromDotBrackets(const std::string& content) const {
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
                    // Stop at newline - don't include acceptance marks
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
        
        // Clean up label - remove trailing whitespace and {0} acceptance marks
        label.erase(label.find_last_not_of(" \t") + 1);
        return label.empty() ? "true" : label;
    }

    bool isAccepting(uint16_t stateId) const {
        auto it = std::find(acceptingStates.begin(), acceptingStates.end(), stateId);
        return it != acceptingStates.end();
    };
    const std::vector<uint16_t>& getAcceptingStates() const {
        return acceptingStates;
    };

    // Method to get the Spot formula object
    spot::formula get_ltl_formula() const {
        if (!ltlFormula) return spot::formula();
        return ltlFormula->getSpotFormula();
    };

    // Method to get the LTLFormula object
    const LTLFormula* getLTLFormula() const {
        return ltlFormula;
    };

    // Visualization methods
    void saveToDot(const std::string& filename) const {
        if (!spotAutomaton) {
            throw std::runtime_error("Spot automaton not available for visualization");
        }
        std::ofstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open file: " + filename);
        }
        spot::print_dot(file, spotAutomaton);
        file.close();
    }

    void convertDotToPng(const std::string& dotFile, const std::string& pngFile) const {
        std::string command = "dot -Tpng \"" + dotFile + "\" -o \"" + pngFile + "\"";
        int result = system(command.c_str());
        if (result != 0) {
            throw std::runtime_error("Failed to convert DOT to PNG. Ensure graphviz is installed.");
        }
    }

    void visualize(const std::string& outputPrefix) const {
        std::string dotFile = outputPrefix + ".dot";
        std::string pngFile = outputPrefix + ".png";
        saveToDot(dotFile);
        convertDotToPng(dotFile, pngFile);
    }

};

#endif
