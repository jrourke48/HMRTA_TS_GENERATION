#ifndef EDGE_NODE_H
#define EDGE_NODE_H

#include <cstdint>
#include <vector>
#include <string>

// Edge class representing a transition between nodes in the automaton
// For a simple unweighted automaton, we only need the destination node ID.
// For a weighted automaton, we can also include a weight property.
class Edge {
private:
    // destination node ID for this edge
    uint32_t dstId;
    std::string label; //label for the edge
    // For weighted edges, we can add a weight property
    uint32_t weight;
public:
    // Constructors
    Edge() = default;
    Edge(uint32_t dstId) : dstId(dstId), weight(1) {}
    Edge(uint32_t dstId, uint32_t weight) : dstId(dstId), weight(weight) {}
    Edge(uint32_t dstId, const std::string& label) : dstId(dstId), label(label), weight(1) {}
    Edge(uint32_t dstId, const std::string& label, uint32_t weight = 1) : dstId(dstId), label(label), weight(weight) {}
    //constructor for subclasses
    virtual ~Edge() = default;
    
    //getters and setters
    uint32_t getDstId() const { return dstId; };
    uint32_t getWeight() const { return weight; };
    std::string getLabel() const { return label; };
    void setDstId(uint32_t id) { dstId = id; };
    void setWeight(uint32_t w) { weight = w; };
    void setLabel(const std::string& label) { this->label = label; };
};

// Node class representing a state in the automaton
class Node{
private:
    // For a simple unweighted automaton, we can just store the node ID.
    // For a more complex automaton, we can also include additional properties (e.g., labels, accepting state flag).
    uint32_t id;
    std::string label; //label for the node
    uint32_t numEdges; //total number of outgoing edges for the node
    std::vector<Edge> edges; // outgoing edges from this node
public:
    // Constructors
    Node() = default;
    Node(uint32_t id) : id(id), label(""), numEdges(0) {}
    Node(uint32_t id, const std::string& label) : id(id), label(label), numEdges(0) {}
    //constructor for subclasses
    virtual ~Node() = default;
    //getters and setters
    uint32_t getId() const { return id; };
    void setId(uint32_t id) { this->id = id; };
    std::string getLabel() const { return label; };
    void setLabel(const std::string& label) { this->label = label; };
    std::vector<Edge> getEdges() const { return edges; };
    //add an edge to this node
    void addEdge(const Edge& edge) { edges.push_back(edge);
        numEdges++;
    };
    //check if there is a direct edge to a given destination node
    bool isAdjacent(uint32_t dstId) const {
        for (const Edge& edge : edges) {
            if (edge.getDstId() == dstId) {
                return true;
            }
        }
        return false;
    };
    //node label can be used for more complex automata where states have labels
    virtual std::string to_string() { return std::string("Node") + std::to_string(this->id); };

};

#endif // EDGE_NODE_H