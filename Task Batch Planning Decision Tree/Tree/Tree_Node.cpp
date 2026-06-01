#include "Tree_Node.h"
#include <algorithm>

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
 * Tree_Node - Constructor (simplified)
 * Initializes a tree node with only a parent, automaton, transition system states, and batch
 * Other fields are initialized with default values
 */
Tree_Node::Tree_Node(uint32_t id, Tree_Node* parent, Node* automatonState, Node* tsState, int8_t batch)
    : id(id), ParentNode(parent), automaton_state(automatonState), ts_state(tsState),
      robo_task_allocation(), times(), batch(batch), prog(TASK_PROGRESS::PRE) {
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
 * setParent - Set the parent node
 */
void Tree_Node::setParent(Tree_Node* newParent) {
    ParentNode = newParent;
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
void Tree_Node::setProgress(TASK_PROGRESS curProg) {
    if (curProg == TASK_PROGRESS::PRE) {
        prog = TASK_PROGRESS::TRA;
    }
    else if (curProg == TASK_PROGRESS::TRA) {
        prog = TASK_PROGRESS::SUF;
    }
    else {
        prog = TASK_PROGRESS::OTH;
    }
}

/**
 * setId - Set the unique identifier
 */
void Tree_Node::setId(uint32_t newId) {
    id = newId;
}

/**
 * getSortedTimes - Returns times sorted by value with robot indices preserved
 * Returns vector of (robotIndex, time) pairs sorted by time in ascending order
 * Does NOT modify the original times vector
 */
std::vector<std::pair<uint16_t, uint16_t>> Tree_Node::getSortedTimes() const {
    std::vector<std::pair<uint16_t, uint16_t>> result;
    
    if (times.empty()) {
        return result;
    }
    
    // Create vector of indices [0, 1, 2, ..., n-1]
    std::vector<uint16_t> indices(times.size());
    for (size_t i = 0; i < times.size(); ++i) {
        indices[i] = static_cast<uint16_t>(i);
    }
    
    // Sort indices based on their corresponding times
    if (times.size() > 1) {
        quickSortIndices(indices, 0, static_cast<int>(indices.size()) - 1);
    }
    
    // Build result as (robotIndex, time) pairs in sorted order
    for (uint16_t idx : indices) {
        result.push_back({idx, times[idx]});
    }
    
    return result;
}

/**
 * quickSortIndices - Helper method for quicksort algorithm
 * Sorts indices array based on their corresponding times values
 * Preserves mapping between sorted times and robot indices
 */
void Tree_Node::quickSortIndices(std::vector<uint16_t>& indices, int low, int high) const {
    if (low < high) {
        // Partition based on times at these indices
        uint16_t pivotIdx = indices[high];
        uint16_t pivotTime = times[pivotIdx];
        int i = low - 1;
        
        // Partition: move indices with smaller times to the left
        for (int j = low; j < high; ++j) {
            if (times[indices[j]] < pivotTime) {
                ++i;
                std::swap(indices[i], indices[j]);
            }
        }
        
        // Place pivot index in correct position
        std::swap(indices[i + 1], indices[high]);
        int partitionIndex = i + 1;
        
        // Recursively sort left and right partitions
        quickSortIndices(indices, low, partitionIndex - 1);
        quickSortIndices(indices, partitionIndex + 1, high);
    }
}
