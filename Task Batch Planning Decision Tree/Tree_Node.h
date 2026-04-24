
#include "Automatons/Node.h"
#include <vector>

class Tree_Node {
    private:
        uint32_t id; // Unique identifier for the node
        Tree_Node* ParentNode; // Pointer to the parent node
        Node* automaton_state; // Associated NBA state
        Node* ts_state; // Associated transition system state
        std::vector<bool> robo_task_allocation; // Vector indicating which robot is allocated to which task
        std::vector<uint16_t> times; // Vector of approximate times for each robot to complete its allocated tasks
        int8_t batch; // Batch number for the node
        TASK_PROGRESS prog; // Type of the node (pre, tra, suf, oth)
    public:
        enum class TASK_PROGRESS {
            PRE,
            TRA,
            SUF,
            OTH
        };
        
        // Constructor
        Tree_Node(uint32_t id, Tree_Node* parent, Node* automatonState, Node* tsState, 
                  std::vector<bool> taskAllocation, std::vector<uint16_t> times, int8_t batch, TASK_PROGRESS prog);
        
        // Destructor
        ~Tree_Node();
        
        // Tree structure methods
        Tree_Node* getParent() const;
        
        // Getter methods
        uint32_t getId() const;
        Node* getAutomatonState() const;
        Node* getTSState() const;
        const std::vector<bool>& getRoboTaskAllocation() const;
        const bool isRobotAllocated(uint16_t robotIndex) const;
        const std::vector<uint16_t>& getTimes() const;
        const uint16_t getTimeForRobot(uint16_t robotIndex) const;
        int8_t getBatch() const;
        TASK_PROGRESS getProgress() const;
        
        // Setter methods
        void setAutomatonState(Node* state);
        void setTSState(Node* state);
        void setRoboTaskAllocation(const std::vector<bool>& allocation);
        void setTimes(const std::vector<uint16_t>& newTimes);
        void setBatch(int8_t newBatch);
        void setProgress(TASK_PROGRESS newProg);
};