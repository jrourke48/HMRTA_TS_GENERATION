#include <iostream>
#include <cassert>
#include <vector>
#include "../Tree/PlanningDecisionTree.h"
#include "../Tree/Tree_Node.h"
#include "../Environment/Point.h"
#include "../Environment/GridWorld.h"
#include "../Environment/Environment.h"

using namespace std;

void testPointClass() {
    cout << "\n=== Testing Point Class ===" << endl;
    
    Point p1(5, 10);
    cout << "Created Point p1(5, 10): (" << p1.getX() << ", " << p1.getY() << ")" << endl;
    
    Point p2(3, 4);
    cout << "Created Point p2(3, 4): (" << p2.getX() << ", " << p2.getY() << ")" << endl;
    
    double dist = p1.distance(p2);
    cout << "Distance between p1 and p2: " << dist << endl;
    
    Point p3 = p1 + p2;
    cout << "p1 + p2 = (" << p3.getX() << ", " << p3.getY() << ")" << endl;
    assert(p3.getX() == 8 && p3.getY() == 14);
    
    Point p4 = p1 - p2;
    cout << "p1 - p2 = (" << p4.getX() << ", " << p4.getY() << ")" << endl;
    assert(p4.getX() == 2 && p4.getY() == 6);
    
    assert(p1 != p2);
    assert(p1 == Point(5, 10));
    
    cout << "✓ Point class tests passed!" << endl;
}

void testGridWorld() {
    cout << "\n=== Testing GridWorld Class ===" << endl;
    
    GridWorld grid(20, 20);
    cout << "Created GridWorld: " << grid.getWidth() << " x " << grid.getHeight() << endl;
    
    Point p1(5, 5);
    Point p2(10, 10);
    
    assert(grid.isFree(p1));
    cout << "Cell (5, 5) is free: " << (grid.isFree(p1) ? "true" : "false") << endl;
    
    grid.setObstacle(p1);
    cout << "Set (5, 5) as obstacle" << endl;
    assert(grid.isObstacle(p1));
    
    grid.clearCell(p1);
    cout << "Cleared (5, 5)" << endl;
    assert(grid.isFree(p1));
    
    grid.setCost(p2, 50);
    uint8_t cost = grid.getCost(p2);
    cout << "Set cost at (10, 10) to 50, retrieved: " << (int)cost << endl;
    assert(cost == 50);
    
    cout << "✓ GridWorld class tests passed!" << endl;
}

void testTreeNode() {
    cout << "\n=== Testing Tree_Node Class ===" << endl;
    
    Node mockNode1(1);
    Node mockNode2(2);
    
    vector<bool> taskAlloc = {true, false, true};
    vector<uint16_t> times = {10, 20, 30};
    
    Tree_Node node(1, nullptr, &mockNode1, &mockNode2, taskAlloc, times, 1, Tree_Node::TASK_PROGRESS::PRE);
    cout << "Created Tree_Node with ID: " << node.getId() << endl;
    
    assert(node.getId() == 1);
    assert(node.getParent() == nullptr);
    assert(node.getBatch() == 1);
    assert(node.getProgress() == Tree_Node::TASK_PROGRESS::PRE);
    
    const auto& allocVec = node.getRoboTaskAllocation();
    cout << "Task allocation: [" << allocVec[0] << ", " << allocVec[1] << ", " << allocVec[2] << "]" << endl;
    assert(allocVec[0] == true && allocVec[1] == false && allocVec[2] == true);
    
    assert(node.isRobotAllocated(0) == true);
    assert(node.isRobotAllocated(1) == false);
    
    const auto& timesVec = node.getTimes();
    cout << "Times: [" << timesVec[0] << ", " << timesVec[1] << ", " << timesVec[2] << "]" << endl;
    assert(timesVec[0] == 10 && timesVec[1] == 20 && timesVec[2] == 30);
    
    uint16_t robotTime = node.getTimeForRobot(0);
    cout << "Time for robot 0: " << robotTime << endl;
    assert(robotTime == 10);
    
    node.setBatch(2);
    assert(node.getBatch() == 2);
    cout << "Updated batch to: " << node.getBatch() << endl;
    
    node.setProgress(Tree_Node::TASK_PROGRESS::TRA);
    assert(node.getProgress() == Tree_Node::TASK_PROGRESS::TRA);
    cout << "Updated progress to: TRA" << endl;
    
    cout << "✓ Tree_Node class tests passed!" << endl;
}

void testPlanningTree() {
    cout << "\n=== Testing PlanningDecisionTree Class ===" << endl;
    
    Node mockNode1(1);
    Node mockNode2(2);
    
    vector<bool> taskAlloc = {true, false, true};
    vector<uint16_t> times = {10, 20, 30};
    
    // Create root node
    PlanningDecisionTree tree(1, &mockNode1, &mockNode2, taskAlloc, times, 1, Tree_Node::TASK_PROGRESS::PRE);
    cout << "Created PlanningDecisionTree with root ID: 1" << endl;
    
    assert(tree.getRoot() != nullptr);
    cout << "Root node ID: " << tree.getRoot()->getId() << endl;
    assert(tree.getRoot()->getId() == 1);
    
    assert(tree.getNodeCount() == 1);
    cout << "Node count: " << tree.getNodeCount() << endl;
    
    // Insert child node
    Node mockNode3(3);
    vector<bool> taskAlloc2 = {false, true, false};
    vector<uint16_t> times2 = {15, 25, 35};
    
    Tree_Node* childNode = tree.insertNode(tree.getRoot(), 2, &mockNode3, &mockNode1, taskAlloc2, times2, 1, Tree_Node::TASK_PROGRESS::TRA);
    cout << "Inserted child node with ID: 2" << endl;
    
    assert(childNode != nullptr);
    assert(childNode->getId() == 2);
    assert(childNode->getParent() == tree.getRoot());
    assert(tree.getNodeCount() == 2);
    cout << "Node count after insertion: " << tree.getNodeCount() << endl;
    
    assert(!tree.isEmpty());
    cout << "Tree is not empty: " << (!tree.isEmpty() ? "true" : "false") << endl;
    
    cout << "✓ PlanningDecisionTree class tests passed!" << endl;
}

void testEnvironment() {
    cout << "\n=== Testing Environment Class ===" << endl;
    
    GridWorld* grid = new GridWorld(30, 30);
    Environment env(nullptr, grid);
    
    cout << "Created Environment with GridWorld" << endl;
    
    Point p1(10, 10);
    assert(env.isFree(p1));
    cout << "Point (10, 10) is free" << endl;
    
    env.getGridWorld()->setObstacle(p1);
    assert(env.isObstacle(p1));
    cout << "Set (10, 10) as obstacle" << endl;
    
    cout << "✓ Environment class tests passed!" << endl;
    
    delete grid;
}

int main() {
    cout << "Starting Planning Decision Tree Tests..." << endl;
    
    try {
        testPointClass();
        testGridWorld();
        testTreeNode();
        testPlanningTree();
        testEnvironment();
        
        cout << "\n" << string(50, '=') << endl;
        cout << "✓ ALL TESTS PASSED!" << endl;
        cout << string(50, '=') << endl;
        
        return 0;
    } catch (const exception& e) {
        cerr << "\n✗ TEST FAILED with exception: " << e.what() << endl;
        return 1;
    }
}
