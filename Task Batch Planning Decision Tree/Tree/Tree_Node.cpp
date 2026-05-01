#include "Tree_Node.h"

/**
 * Tree_Node - Constructor
 * Initializes a tree node with automaton and transition system states
 */
Tree_Node::Tree_Node(uint32_t id, Tree_Node* parent, Node* automatonState, Node* tsState,
                     std::vector<bool> taskAllocation, std::vector<uint16_t> times, 
                     int8_t batch, TASK_PROGRESS prog)
    : id(id), ParentNode(parent), automaton_state(automatonState), ts_state(tsState),
      robo_task_allocation(taskAllocation), times(times), batch(batch), prog(prog) {
}

/**
 * Tree_Node - Destructor
 */
Tree_Node::~Tree_Node() {
    // Note: We don't delete automaton_state and ts_state as they are managed externally
}

/**
 * getParent - Get the parent node
 */
Tree_Node* Tree_Node::getParent() const {
    return ParentNode;
}

/**
 * getId - Get the unique identifier
 */
uint32_t Tree_Node::getId() const {
    return id;
}

/**
 * getAutomatonState - Get the associated NBA state
 */
Node* Tree_Node::getAutomatonState() const {
    return automaton_state;
}

/**
 * getTSState - Get the associated transition system state
 */
Node* Tree_Node::getTSState() const {
    return ts_state;
}

/**
 * getRoboTaskAllocation - Get the task allocation vector
 */
const std::vector<bool>& Tree_Node::getRoboTaskAllocation() const {
    return robo_task_allocation;
}

/**
 * isRobotAllocated - Check if a specific robot is allocated a task
 */
bool Tree_Node::isRobotAllocated(uint16_t robotIndex) const {
    if (robotIndex >= robo_task_allocation.size()) {
        return false;
    }
    return robo_task_allocation[robotIndex];
}

/**
 * getTimes - Get the times vector
 */
const std::vector<uint16_t>& Tree_Node::getTimes() const {
    return times;
}

/**
 * getTimeForRobot - Get the time estimate for a specific robot
 */
uint16_t Tree_Node::getTimeForRobot(uint16_t robotIndex) const {
    if (robotIndex >= times.size()) {
        return 0;
    }
    return times[robotIndex];
}

/**
 * getBatch - Get the batch number
 */
int8_t Tree_Node::getBatch() const {
    return batch;
}

/**
 * getProgress - Get the progress type
 */
Tree_Node::TASK_PROGRESS Tree_Node::getProgress() const {
    return prog;
}

/**
 * setAutomatonState - Set the automaton state
 */
void Tree_Node::setAutomatonState(Node* state) {
    automaton_state = state;
}

/**
 * setTSState - Set the transition system state
 */
void Tree_Node::setTSState(Node* state) {
    ts_state = state;
}

/**
 * setRoboTaskAllocation - Set the task allocation vector
 */
void Tree_Node::setRoboTaskAllocation(const std::vector<bool>& allocation) {
    robo_task_allocation = allocation;
}

/**
 * setTimes - Set the times vector
 */
void Tree_Node::setTimes(const std::vector<uint16_t>& newTimes) {
    times = newTimes;
}

/**
 * setBatch - Set the batch number
 */
void Tree_Node::setBatch(int8_t newBatch) {
    batch = newBatch;
}

/**
 * setProgress - Set the progress type
 */
void Tree_Node::setProgress(TASK_PROGRESS newProg) {
    prog = newProg;
}
