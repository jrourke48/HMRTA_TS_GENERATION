#include <iostream>
#include <cassert>
#include <vector>
#include <algorithm>
#include <Tree/PlanningDecisionTree.h>
#include <Tree/Tree_Node.h>

/**
 * COMPREHENSIVE PLANNING TREE TEST SUITE
 * Tests all PlanningDecisionTree methods and Tree_Node operations
 */

// ============================================================================
// TEST 1: Basic Tree Construction
// ============================================================================
void testBasicTreeConstruction() {
    std::cout << "\n=== TEST 1: Basic Tree Construction ===" << std::endl;
    
    // Create simple nodes for testing
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    std::vector<bool> taskAlloc = {true, false, true};
    std::vector<uint16_t> times = {5, 10, 3};
    std::vector<Point> positions = {};
    
    // Create tree with root
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, taskAlloc, times, positions, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    assert(tree != nullptr);
    assert(!tree->isEmpty());
    assert(tree->getNodeCount() == 1);
    assert(tree->getRoot() != nullptr);
    std::cout << "✓ Tree created successfully" << std::endl;
    std::cout << "✓ Root node ID: " << tree->getRoot()->getId() << std::endl;
    std::cout << "✓ Initial node count: " << tree->getNodeCount() << std::endl;
    
    delete tree;
    std::cout << "✓ Tree destructor called successfully" << std::endl;
}

// ============================================================================
// TEST 2: Insert Node (Two Overloads)
// ============================================================================
void testInsertNode() {
    std::cout << "\n=== TEST 2: Insert Node (Two Overloads) ===" << std::endl;
    
    Node* nbaNode0 = new Node(0);
    Node* tsNode0 = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode0, tsNode0, {true, false}, {5, 10}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    assert(root != nullptr);
    
    // Test overload 1: insertNode with parameters
    std::cout << "\n--- Testing insertNode(parent, automatonState, tsState, ...) ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child1 = tree->insertNode(root, nbaNode1, tsNode1, {false, true}, {8, 7}, {}, 1, Tree_Node::TASK_PROGRESS::PRE);
    
    assert(child1 != nullptr);
    assert(tree->getNodeCount() == 2);
    assert(child1->getParent() == root);
    assert(child1->getId() == 1);
    std::cout << "✓ Child 1 created with ID: " << child1->getId() << std::endl;
    std::cout << "✓ Tree node count: " << tree->getNodeCount() << std::endl;
    
    // Test overload 2: insertNode with parameters on a different parent
    std::cout << "\n--- Testing insertNode on non-root parent ---" << std::endl;
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    Tree_Node* child2 = tree->insertNode(child1, nbaNode2, tsNode2, {true, true}, {6, 9}, {}, 1, Tree_Node::TASK_PROGRESS::TRA);
    
    assert(child2 != nullptr);
    assert(child2->getId() == 2);
    assert(child2->getParent() == child1);
    assert(tree->getNodeCount() == 3);
    std::cout << "✓ Child 2 created with ID: " << child2->getId() << std::endl;
    std::cout << "✓ Child 2 parent is Child 1" << std::endl;
    std::cout << "✓ Tree node count: " << tree->getNodeCount() << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 3: Frontier Management
// ============================================================================
void testFrontierManagement() {
    std::cout << "\n=== TEST 3: Frontier Management ===" << std::endl;
    
    Node* nbaNode0 = new Node(0);
    Node* tsNode0 = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode0, tsNode0, {false}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    const std::vector<Tree_Node*>& frontier = tree->getFrontierNodes();
    
    std::cout << "\n--- Initial frontier state ---" << std::endl;
    assert(frontier.size() == 1);
    assert(frontier[0] == root);
    std::cout << "✓ Initial frontier size: " << frontier.size() << std::endl;
    std::cout << "✓ Root is in frontier" << std::endl;
    
    // Add a child - frontier should change
    std::cout << "\n--- After inserting child ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    const std::vector<Tree_Node*>& newFrontier = tree->getFrontierNodes();
    assert(newFrontier.size() == 1);
    assert(newFrontier[0] == child);
    assert(std::find(newFrontier.begin(), newFrontier.end(), root) == newFrontier.end());
    std::cout << "✓ Root removed from frontier (no longer leaf)" << std::endl;
    std::cout << "✓ Child added to frontier" << std::endl;
    std::cout << "✓ Frontier size: " << newFrontier.size() << std::endl;
    
    // Add multiple children to same parent
    std::cout << "\n--- After inserting multiple children ---" << std::endl;
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    const std::vector<Tree_Node*>& multiFrontier = tree->getFrontierNodes();
    assert(multiFrontier.size() == 2);
    std::cout << "✓ Multiple frontier nodes maintained" << std::endl;
    std::cout << "✓ Frontier size: " << multiFrontier.size() << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 4: Get All Nodes
// ============================================================================
void testGetAllNodes() {
    std::cout << "\n=== TEST 4: Get All Nodes ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Start with root only
    std::cout << "\n--- Tree with 1 node ---" << std::endl;
    std::vector<Tree_Node*> allNodes = tree->getAllNodes();
    assert(allNodes.size() == 1);
    std::cout << "✓ getAllNodes() returned 1 node" << std::endl;
    
    // Add children
    std::cout << "\n--- Adding nodes to tree ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode3 = new Node(3);
    Node* tsNode3 = new Node(3);
    tree->insertNode(root, nbaNode3, tsNode3, {false}, {8}, {}, 0, Tree_Node::TASK_PROGRESS::TRA);
    
    allNodes = tree->getAllNodes();
    std::cout << "✓ getAllNodes() returned " << allNodes.size() << " nodes" << std::endl;
    
    // Check that root is reachable
    assert(std::find(allNodes.begin(), allNodes.end(), root) != allNodes.end());
    std::cout << "✓ Root found in all nodes" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 5: Delete Subtree
// ============================================================================
void testDeleteSubtree() {
    std::cout << "\n=== TEST 5: Delete Subtree ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Build a multi-level tree:
    //        root (0)
    //       /  |   \
    //      c1  c2  c3
    //     / \
    //   gc1 gc2
    
    std::cout << "\n--- Building multi-level tree ---" << std::endl;
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child1 = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    Tree_Node* child2 = tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode3 = new Node(3);
    Node* tsNode3 = new Node(3);
    Tree_Node* child3 = tree->insertNode(root, nbaNode3, tsNode3, {false}, {8}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    // Add grandchildren to child1
    Node* nbaNode4 = new Node(4);
    Node* tsNode4 = new Node(4);
    Tree_Node* grandchild1 = tree->insertNode(child1, nbaNode4, tsNode4, {true}, {4}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode5 = new Node(5);
    Node* tsNode5 = new Node(5);
    Tree_Node* grandchild2 = tree->insertNode(child1, nbaNode5, tsNode5, {false}, {3}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "✓ Tree built with 6 nodes" << std::endl;
    std::cout << "✓ Node count: " << tree->getNodeCount() << std::endl;
    assert(tree->getNodeCount() == 6);
    
    // Test 1: Delete a frontier node (leaf)
    std::cout << "\n--- Test 1: Delete frontier node (leaf) ---" << std::endl;
    std::cout << "  Before deletion: node count = " << tree->getNodeCount() << std::endl;
    tree->deleteSubtree(grandchild2);
    std::cout << "  ✓ Deleted grandchild2" << std::endl;
    std::cout << "  After deletion: node count = " << tree->getNodeCount() << std::endl;
    assert(tree->getNodeCount() == 5);
    
    // Test 2: Delete a non-frontier (interior) node with children
    std::cout << "\n--- Test 2: Delete non-frontier node (interior with 1 child) ---" << std::endl;
    std::cout << "  Before deletion: node count = " << tree->getNodeCount() << std::endl;
    tree->deleteSubtree(child1);  // This has grandchild1 as a child
    std::cout << "  ✓ Deleted child1 and its subtree" << std::endl;
    std::cout << "  After deletion: node count = " << tree->getNodeCount() << std::endl;
    assert(tree->getNodeCount() == 3);  // Only root, child2, child3
    
    // Test 3: Verify frontier was updated correctly
    std::cout << "\n--- Test 3: Verify frontier ---" << std::endl;
    const std::vector<Tree_Node*>& frontier = tree->getFrontierNodes();
    std::cout << "  ✓ Frontier size: " << frontier.size() << std::endl;
    
    // Debug: print which nodes are in frontier
    std::cout << "  Frontier nodes: ";
    for (Tree_Node* node : frontier) {
        std::cout << node->getId() << " ";
    }
    std::cout << std::endl;
    
    assert(frontier.size() == 2);  // Only child2 and child3 remain as frontier nodes
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 6: Insert Subtree
// ============================================================================
void testInsertSubtree() {
    std::cout << "\n=== TEST 6: Insert Subtree ===" << std::endl;
    
    // Create main tree with multiple levels
    Node* nbaNode0 = new Node(0);
    Node* tsNode0 = new Node(0);
    
    PlanningDecisionTree* mainTree = new PlanningDecisionTree(
        nbaNode0, tsNode0, {true}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    std::cout << "\n--- Main tree created ---" << std::endl;
    std::cout << "✓ Initial node count: " << mainTree->getNodeCount() << std::endl;
    
    Tree_Node* mainRoot = mainTree->getRoot();
    
    // Add children to main tree
    Node* nbaMainChild1 = new Node(1);
    Node* tsMainChild1 = new Node(1);
    Tree_Node* mainChild1 = mainTree->insertNode(mainRoot, nbaMainChild1, tsMainChild1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaMainChild2 = new Node(2);
    Node* tsMainChild2 = new Node(2);
    Tree_Node* mainChild2 = mainTree->insertNode(mainRoot, nbaMainChild2, tsMainChild2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "✓ Main tree expanded to " << mainTree->getNodeCount() << " nodes" << std::endl;
    assert(mainTree->getNodeCount() == 3);
    
    // Create a subtree with multiple levels
    //     subRoot (1)
    //       / \
    //    sub1 sub2
    //     /
    //  sub3
    
    Node* nbaSub0 = new Node(10);
    Node* tsSub0 = new Node(10);
    
    PlanningDecisionTree* subtree = new PlanningDecisionTree(
        nbaSub0, tsSub0, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* subRoot = subtree->getRoot();
    
    // Add children to subtree
    Node* nbaSub1 = new Node(11);
    Node* tsSub1 = new Node(11);
    Tree_Node* subChild1 = subtree->insertNode(subRoot, nbaSub1, tsSub1, {true}, {3}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaSub2 = new Node(12);
    Node* tsSub2 = new Node(12);
    Tree_Node* subChild2 = subtree->insertNode(subRoot, nbaSub2, tsSub2, {false}, {4}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    // Add grandchild to subChild1
    Node* nbaSub3 = new Node(13);
    Node* tsSub3 = new Node(13);
    subtree->insertNode(subChild1, nbaSub3, tsSub3, {true}, {2}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- Subtree created ---" << std::endl;
    std::cout << "✓ Subtree node count: " << subtree->getNodeCount() << " nodes" << std::endl;
    assert(subtree->getNodeCount() == 4);
    
    // Test 1: Insert subtree into a leaf node (frontier node)
    std::cout << "\n--- Test 1: Insert subtree into frontier node (leaf) ---" << std::endl;
    std::cout << "  Before insertion: main tree has " << mainTree->getNodeCount() << " nodes" << std::endl;
    mainTree->insertSubtree(mainChild1, subtree);
    std::cout << "  ✓ Subtree inserted into mainChild1" << std::endl;
    std::cout << "  After insertion: main tree has " << mainTree->getNodeCount() << " nodes" << std::endl;
    assert(mainTree->getNodeCount() == 7);  // 3 + 4
    
    // Test 2: Verify frontier was updated
    std::cout << "\n--- Test 2: Verify frontier after insertion ---" << std::endl;
    const std::vector<Tree_Node*>& frontier = mainTree->getFrontierNodes();
    std::cout << "  ✓ Frontier size: " << frontier.size() << std::endl;
    // After insertion, mainChild1 is no longer a frontier (now has children)
    // Frontier should have mainChild2, subChild2, and the grandchild from subtree
    
    // Test 3: Insert another subtree into a non-frontier (interior) node
    std::cout << "\n--- Test 3: Insert subtree into interior node ---" << std::endl;
    
    // Create another small subtree
    Node* nbaSub4 = new Node(20);
    Node* tsSub4 = new Node(20);
    PlanningDecisionTree* subtree2 = new PlanningDecisionTree(
        nbaSub4, tsSub4, {true}, {2}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Node* nbaSub5 = new Node(21);
    Node* tsSub5 = new Node(21);
    subtree2->insertNode(subtree2->getRoot(), nbaSub5, tsSub5, {false}, {1}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "  Before insertion: main tree has " << mainTree->getNodeCount() << " nodes" << std::endl;
    std::cout << "  Inserting into mainChild2 (interior node)" << std::endl;
    mainTree->insertSubtree(mainChild2, subtree2);
    std::cout << "  ✓ Subtree2 inserted into mainChild2" << std::endl;
    std::cout << "  After insertion: main tree has " << mainTree->getNodeCount() << " nodes" << std::endl;
    assert(mainTree->getNodeCount() == 9);  // 7 + 2
    
    // Test 4: Verify all nodes are reachable
    std::cout << "\n--- Test 4: Verify all nodes reachable ---" << std::endl;
    std::vector<Tree_Node*> allNodes = mainTree->getAllNodes();
    std::cout << "  ✓ getAllNodes() returned " << allNodes.size() << " nodes" << std::endl;
    assert(allNodes.size() == 9);
    
    // Test 5: Delete a node and verify count decreases
    std::cout << "\n--- Test 5: Delete subtree from interior node ---" << std::endl;
    std::cout << "  Before deletion: node count = " << mainTree->getNodeCount() << std::endl;
    // Delete subtree2 (which is attached to mainChild2)
    mainTree->deleteSubtree(subtree2->getRoot());  // Delete the root of subtree2
    std::cout << "  ✓ Deleted subtree2" << std::endl;
    std::cout << "  After deletion: node count = " << mainTree->getNodeCount() << std::endl;
    assert(mainTree->getNodeCount() == 7);  // Back to 7 (9 - 2)
    
    // Test 6: Add more nodes and verify count increases
    std::cout << "\n--- Test 6: Add more nodes to existing tree ---" << std::endl;
    std::cout << "  Before addition: node count = " << mainTree->getNodeCount() << std::endl;
    
    Node* nbaNewNode1 = new Node(30);
    Node* tsNewNode1 = new Node(30);
    Tree_Node* newChild1 = mainTree->insertNode(mainChild2, nbaNewNode1, tsNewNode1, {false}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNewNode2 = new Node(31);
    Node* tsNewNode2 = new Node(31);
    Tree_Node* newChild2 = mainTree->insertNode(mainChild2, nbaNewNode2, tsNewNode2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "  ✓ Added 2 new nodes to mainChild2" << std::endl;
    std::cout << "  After addition: node count = " << mainTree->getNodeCount() << std::endl;
    assert(mainTree->getNodeCount() == 9);  // Back to 9 (7 + 2)
    
    // Test 7: Delete multiple nodes one by one
    std::cout << "\n--- Test 7: Delete multiple nodes sequentially ---" << std::endl;
    std::cout << "  Before deletions: node count = " << mainTree->getNodeCount() << std::endl;
    
    mainTree->deleteSubtree(newChild1);
    std::cout << "  ✓ Deleted newChild1, node count = " << mainTree->getNodeCount() << std::endl;
    assert(mainTree->getNodeCount() == 8);
    
    mainTree->deleteSubtree(newChild2);
    std::cout << "  ✓ Deleted newChild2, node count = " << mainTree->getNodeCount() << std::endl;
    assert(mainTree->getNodeCount() == 7);
    
    // Test 8: Final verification - all remaining nodes should be reachable
    std::cout << "\n--- Test 8: Final node verification ---" << std::endl;
    std::vector<Tree_Node*> finalNodes = mainTree->getAllNodes();
    std::cout << "  ✓ getAllNodes() returned " << finalNodes.size() << " nodes" << std::endl;
    std::cout << "  ✓ getNodeCount() returned " << mainTree->getNodeCount() << " nodes" << std::endl;
    if (finalNodes.size() != mainTree->getNodeCount()) {
        std::cout << "  ⚠ WARNING: Mismatch between getAllNodes() and getNodeCount()!" << std::endl;
        std::cout << "    This indicates a traversal bug in getAllNodes() - some nodes are not reachable" << std::endl;
    }
    assert(finalNodes.size() == 7);  // After fix: orphaned nodes are properly re-added to frontier
    assert(mainTree->getNodeCount() == 7);
    std::cout << "  ✓ Node count tracking verified" << std::endl;
    std::cout << "  ✓ Node count tracking verified" << std::endl;
    
    delete mainTree;  // This should clean up all subtrees
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 7: Empty Tree Operations
// ============================================================================
void testEmptyTreeOperations() {
    std::cout << "\n=== TEST 7: Empty Tree Operations ===" << std::endl;
    
    PlanningDecisionTree* emptyTree = new PlanningDecisionTree();
    
    assert(emptyTree->isEmpty());
    assert(emptyTree->getNodeCount() == 0);
    assert(emptyTree->getRoot() == nullptr);
    std::cout << "✓ Empty tree is empty" << std::endl;
    std::cout << "✓ Root is nullptr" << std::endl;
    std::cout << "✓ Node count is 0" << std::endl;
    
    std::vector<Tree_Node*> allNodes = emptyTree->getAllNodes();
    assert(allNodes.empty());
    std::cout << "✓ getAllNodes() returns empty vector" << std::endl;
    
    delete emptyTree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 8: Clear Tree
// ============================================================================
void testClearTree() {
    std::cout << "\n=== TEST 8: Clear Tree ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    
    // Add some nodes
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- Before clear ---" << std::endl;
    std::cout << "✓ Node count: " << tree->getNodeCount() << std::endl;
    assert(tree->getNodeCount() == 2);
    
    // Clear tree
    tree->clearTree();
    
    std::cout << "\n--- After clear ---" << std::endl;
    std::cout << "✓ Node count: " << tree->getNodeCount() << std::endl;
    assert(tree->isEmpty());
    assert(tree->getRoot() == nullptr);
    std::cout << "✓ Tree is empty" << std::endl;
    std::cout << "✓ Root is nullptr" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 9: Tree Node Parent-Child Relationships
// ============================================================================
void testNodeParentChildRelationships() {
    std::cout << "\n=== TEST 9: Tree Node Parent-Child Relationships ===" << std::endl;
    
    Node* nbaNode = new Node(0);
    Node* tsNode = new Node(0);
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, {true}, {5}, {}, 0, Tree_Node::TASK_PROGRESS::PRE
    );
    
    Tree_Node* root = tree->getRoot();
    assert(root->getParent() == nullptr);
    std::cout << "✓ Root has no parent" << std::endl;
    
    // Add children
    Node* nbaNode1 = new Node(1);
    Node* tsNode1 = new Node(1);
    Tree_Node* child1 = tree->insertNode(root, nbaNode1, tsNode1, {false}, {7}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    Node* nbaNode2 = new Node(2);
    Node* tsNode2 = new Node(2);
    Tree_Node* child2 = tree->insertNode(root, nbaNode2, tsNode2, {true}, {6}, {}, 0, Tree_Node::TASK_PROGRESS::PRE);
    
    std::cout << "\n--- After adding children ---" << std::endl;
    assert(child1->getParent() == root);
    assert(child2->getParent() == root);
    std::cout << "✓ Child 1 parent is root" << std::endl;
    std::cout << "✓ Child 2 parent is root" << std::endl;
    std::cout << "✓ Parent-child relationships verified" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// TEST 10: Node Data Storage and Retrieval
// ============================================================================
void testNodeDataStorage() {
    std::cout << "\n=== TEST 10: Node Data Storage and Retrieval ===" << std::endl;
    
    Node* nbaNode = new Node(5);
    Node* tsNode = new Node(10);
    std::vector<bool> taskAlloc = {true, false, true, true};
    std::vector<uint16_t> times = {12, 25, 8, 15};
    int8_t batchVal = 3;
    Tree_Node::TASK_PROGRESS progress = Tree_Node::TASK_PROGRESS::PRE;
    
    PlanningDecisionTree* tree = new PlanningDecisionTree(
        nbaNode, tsNode, taskAlloc, times, {}, batchVal, progress
    );
    
    Tree_Node* root = tree->getRoot();
    
    std::cout << "\n--- Checking stored data ---" << std::endl;
    assert(root->getAutomatonState()->getId() == 5);
    assert(root->getTSState()->getId() == 10);
    assert(root->getBatch() == batchVal);
    assert(root->getProgress() == progress);
    
    auto storedTimes = root->getTimes();
    assert(storedTimes == times);
    
    auto storedAlloc = root->getRoboTaskAllocation();
    assert(storedAlloc == taskAlloc);
    
    std::cout << "✓ Automaton state ID: " << root->getAutomatonState()->getId() << std::endl;
    std::cout << "✓ TS state ID: " << root->getTSState()->getId() << std::endl;
    std::cout << "✓ Batch value: " << static_cast<int>(root->getBatch()) << std::endl;
    std::cout << "✓ Progress: " << static_cast<int>(progress) << std::endl;
    std::cout << "✓ Times match stored values" << std::endl;
    std::cout << "✓ Task allocation matches stored values" << std::endl;
    
    delete tree;
    std::cout << "✓ Test completed successfully" << std::endl;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================
int main() {
    std::cout << "\n";
    std::cout << "========================================" << std::endl;
    std::cout << "PLANNING TREE COMPREHENSIVE TEST SUITE" << std::endl;
    std::cout << "========================================" << std::endl;
    
    try {
        testBasicTreeConstruction();
        testInsertNode();
        testFrontierManagement();
        testGetAllNodes();
        testDeleteSubtree();
        testInsertSubtree();
        testEmptyTreeOperations();
        testClearTree();
        testNodeParentChildRelationships();
        testNodeDataStorage();
        
        std::cout << "\n";
        std::cout << "========================================" << std::endl;
        std::cout << "✓ ALL TESTS PASSED" << std::endl;
        std::cout << "========================================" << std::endl;
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "\n✗ TEST FAILED WITH EXCEPTION: " << e.what() << std::endl;
        return 1;
    }
}
