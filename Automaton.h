#include <cstdint>
#include <vector>
#include <string>
#include <map>
#include <Edge_Node.h>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/product.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twa/twagraph.hh>
#include <spot/twa/twaproduct.hh>
#include <spot/twaalgos/emptiness.hh>
#include <bddx.h>


class Automaton{
    protected:
        uint32_t numNodes = 0;
        uint32_t numEdges = 0;
        std::map<uint32_t, Node*> nodeMap;  //map from node ID to node pointer for quick access
    
    public:
        Automaton() = default;
        virtual ~Automaton() = default;
        
        //getters and setters
        uint32_t getnumStates() const { return numNodes; }
        uint32_t getnumEdges() const { return numEdges; }
        
        //add a node to the automaton
        virtual void add_Node(Node* node) = 0;
        
        //check if there is a direct edge between two nodes
        virtual bool isAdjacent(uint32_t srcId, uint32_t dstId) const = 0;
        
        //get a node by ID
        Node* getNode(uint32_t nodeId) const {
            auto it = nodeMap.find(nodeId);
            if (it != nodeMap.end()) {
                return it->second;
            }
            return nullptr;
        }
        
        //get all nodes
        const std::map<uint32_t, Node*>& getNodes() const {
            return nodeMap;
        }
};
