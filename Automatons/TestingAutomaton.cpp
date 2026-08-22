#include <string>
#include <iostream>
#include <fstream>
#include <sstream>
#include <set>
#include <cstdlib>
#include <ctime>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/product.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twa/twagraph.hh>
#include <spot/twa/twaproduct.hh>
#include <spot/twaalgos/emptiness.hh>
#include <bddx.h>
#include "../Transition_Systems/GridWorldTransitionSystem.h"
#include "TS.h"
#include "BuchiAutomaton.h"
#include "ProductAutomaton.h"
#include "Environment/Environment.h"
#include "Environment/Point.h"
#include "MultiRobotSystem/MultiRobotSystem.h"
#include "MultiRobotSystem/Robot.h"
#include "MultiRobotSystem/RobotCapabilities.h"
#include "LTLFormula/LTLFormula.h"
#include "LTLFormula/BatchAtomicProposition.h"

int main()
{
    try {
        std::cout << "=== Multi-Robot Task Plan Visualization ===" << std::endl;
        
    // Allocate GridWorld
        GridWorld* grid = new GridWorld(21, 21);
        std::cout << "✓ GridWorld created (20x20)" << std::endl;
        
        // Allocate Transition System
        TS* ts = new TS();
        
        // Add 6 states 
        Node* node0 = new Node(0, "S0");
        Node* node1 = new Node(1, "S1");
        Node* node2 = new Node(2, "S2");
        Node* node3 = new Node(3, "S3");
        Node* node4 = new Node(4, "S4");
        Node* node5 = new Node(5, "S5");

        //with edges: 0-2 1-2 2-3 2-4 2-5
        node0->addEdge(Edge(2));
        node2->addEdge(Edge(0));
        node1->addEdge(Edge(2));
        node2->addEdge(Edge(1));
        node3->addEdge(Edge(2));
        node2->addEdge(Edge(3));
        node4->addEdge(Edge(2));
        node2->addEdge(Edge(4));
        node5->addEdge(Edge(2));
        node2->addEdge(Edge(5));
        
        // Add nodes to TS
        ts->add_Node(node0);
        ts->add_Node(node1);
        ts->add_Node(node2);
        ts->add_Node(node3);
        ts->add_Node(node4);
        ts->add_Node(node5);
        ts->setInitial(0);
        
        std::cout << "✓ Transition System created" << std::endl;
        std::cout << "  - States: " << ts->getNumStates() << std::endl;
        std::cout << "  - Initial state: 0" << std::endl;
        
        // Allocate Environment
        Environment* env = new Environment(ts, grid);
        std::cout << "✓ Environment created" << std::endl;
        
        // Map states to grid regions
        env->mapTSStateToGrid(0, Point(17, 15), 8, 12);    // State 0 centered at (17,15), 4x6 region
        env->mapTSStateToGrid(1, Point(17, 4), 8, 8);   // State 1 centered at (17,4)
        env->mapTSStateToGrid(2, Point(10, 10), 6, 22);   // State 2 centered at (10,11)
        env->mapTSStateToGrid(3, Point(3, 3), 8, 6);   // State 3 centered at (5,5)
        env->mapTSStateToGrid(4, Point(3, 10), 8, 8);   // State 4 centered at (5,10)
        env->mapTSStateToGrid(5, Point(3, 17), 8, 8);   // State 5 centered at (5,15)
        std::cout << "✓ Mapped 6 states to grid regions" << std::endl;
        
        // Create MultiRobotSystem
        MultiRobotSystem* mrs = new MultiRobotSystem();
        if (!mrs) {
            std::cerr << "Failed to create MultiRobotSystem" << std::endl;
            return 1;
        }
        
        // Position all robots in room 0 (centered at Point(18, 14))
        Robot* r0 = new Robot(0, "R0", Point(18, 14));
        r0->initializeCapabilities(13);
        r0->enableCapability(RobotCapability::SENSOR_GPS); //C
        mrs->addRobot(r0);
        
        Robot* r1 = new Robot(1, "R1", Point(17, 14));
        r1->initializeCapabilities(13);
        r1->enableCapability(RobotCapability::MOVEMENT_GROUND); //A
        mrs->addRobot(r1);
        
        // Robot* r2 = new Robot(2, "R2", Point(19, 14));
        // r2->initializeCapabilities(13);
        // r2->enableCapability(RobotCapability::SENSOR_CAMERA); // B
        // mrs->addRobot(r2);
        // Robot* r3 = new Robot(3, "R3", Point(18, 13));
        // r3->initializeCapabilities(13);
        // r3->enableCapability(RobotCapability::SENSOR_GPS); // C
        // mrs->addRobot(r3);
        
        // Robot* r4 = new Robot(4, "R4", Point(18, 15));
        // r4->initializeCapabilities(13);
        // r4->enableCapability(RobotCapability::MOVEMENT_GROUND);
        // mrs->addRobot(r4);
        
        // Robot* r5 = new Robot(5, "R5", Point(17, 15));
        // r5->initializeCapabilities(13);
        // r5->enableCapability(RobotCapability::SENSOR_CAMERA);
        // mrs->addRobot(r5);
    
    std::cout << "✓ MultiRobotSystem created with 3 robots" << std::endl;
        
        // Add random obstacles for interesting pathfinding
        std::cout << "Adding random obstacles..." << std::endl;
        srand(time(0));
        int numObstacles = 15;
        for (int i = 0; i < numObstacles; i++) {
            int x = rand() % 20;
            int y = rand() % 20;
            env->addObstacle(Point(x, y));
        }
        std::cout << "✓ Added " << numObstacles << " random obstacles" << std::endl;
        std::string ltl_str = "G(F\"p1\") && G(F\"p2\")";
    
        std::vector<BatchAtomicProposition> batchAPs;
        batchAPs.push_back(BatchAtomicProposition(1, 1, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(2, 2, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        batchAPs.push_back(BatchAtomicProposition(3, 3, {true, false, false, false, false, false, false, false, false, false, false, false, false}, 0));
        
        LTLFormula* ltlFormula = new LTLFormula(ltl_str, batchAPs);
        BuchiAutomaton* buchi = new BuchiAutomaton(ltlFormula);
        buchi->visualize("output/buchi_automaton_tree_test");
        std::cout << "✓ BuchiAutomaton created" << std::endl;
    
    std::cout << "\n=== Büchi Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << buchi->getNumStates() << std::endl;
    std::cout << "Edges: " << buchi->getNumEdges() << std::endl;
    std::cout << "Accepting States: " << buchi->getAcceptingStates().size() << std::endl;

    //=========================================================================
    // 5. Test ProductAutomaton constructor
    //=========================================================================
    ProductAutomaton productAutomaton(*ts, *mrs, *buchi);

    std::cout << "\n=== Product Automaton (Constructor) ===" << std::endl;
    std::cout << "States: " << productAutomaton.getNumStates() << std::endl;
    std::cout << "Edges: " << productAutomaton.getNumEdges() << std::endl;
    std::cout << "Accepting States (internal): " << productAutomaton.getAcceptingStates().size() << std::endl;
    for (auto state : productAutomaton.getAcceptingStates()) {
        std::cout << "  - State " << state << std::endl;
    }

    // Export ProductAutomaton's Spot automaton to DOT file
    std::string dotFile = "output/product_automaton.dot";
    std::string pngFile = "output/product_automaton.png";
    spot::twa_graph_ptr spotAut = productAutomaton.getSpotAutomaton();
    
    // DEBUG: Show ProductAutomaton's internal acceptance semantics
    std::cout << "\n=== ProductAutomaton Internal Acceptance Analysis ===" << std::endl;
    std::cout << "Accepting states (from internal vector):" << std::endl;
    const auto& acceptingStates = productAutomaton.getAcceptingStates();
    if (acceptingStates.empty()) {
        std::cout << "  (No accepting states)" << std::endl;
    } else {
        for (auto state : acceptingStates) {
            Node* node = productAutomaton.getNode(state);
            std::string label = (node != nullptr) ? node->getLabel() : "UNKNOWN";
            std::cout << "  State " << state << " [" << label << "]" << std::endl;
        }
    }
    std::cout << "Total accepting states: " << acceptingStates.size() << " / " << productAutomaton.getNumStates() << std::endl;

    // //testing how the product ts states look
    // spot::twa_graph_ptr buchiSpot = buchi->getSpotAutomaton();
    // // Convert the transition system to a Spot automaton
    // spot::twa_graph_ptr tsSpot = ts->toSpotAutomaton(buchiSpot->get_dict());

    // // Initialize the product automaton as the transition system Spot automaton
    // spot::twa_graph_ptr productSpot = tsSpot;
    // productSpot = spot::product(tsSpot, productSpot);
    // productSpot = spot::product(tsSpot, productSpot);
    
    // // Export intermediate TS×TS product to DOT
    // std::string tsTsDotFile = "output/ts_product_ts.dot";
    // std::string tsTsPngFile = "output/ts_product_ts.png";
    // std::ofstream tsTsOut(tsTsDotFile);
    // spot::print_dot(tsTsOut, productSpot);
    // tsTsOut.close();
    // std::cout << "\n✓ Exported TS×TS product to " << tsTsDotFile << std::endl;
    
    // std::string command = "dot -Tpng " + tsTsDotFile + " -o " + tsTsPngFile;
    // int ret = system(command.c_str());
    // if (ret == 0) {
    //     std::cout << "✓ Generated TS×TS product PNG: " << tsTsPngFile << std::endl;
    // }

    if (spotAut) {
        std::ofstream out(dotFile);
        spot::print_dot(out, spotAut);
        out.close();
        std::cout << "✓ Exported ProductAutomaton to " << dotFile << std::endl;
        
        // Generate PNG from DOT file using graphviz
        std::string command = "dot -Tpng " + dotFile + " -o " + pngFile;
        int ret = system(command.c_str());
        if (ret == 0) {
            std::cout << "✓ Generated PNG visualization: " << pngFile << std::endl;
        } else {
            std::cout << "⚠ Could not generate PNG (graphviz may not be installed)" << std::endl;
        }
    } else {
        std::cout << "⚠ ProductAutomaton's Spot automaton is null" << std::endl;
    }
    
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception occurred: " << e.what() << std::endl;
        return 1;
    }
}