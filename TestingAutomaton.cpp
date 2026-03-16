#include <string>
#include <iostream>
#include <fstream>
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
#include "Transition_Systems/GridWorldTransitionSystem.h"
#include "TS.h"
#include "BuchiAutomaton.h"
#include "ProductAutomaton.h"

int main()
{
    //=========================================================================
    // 1. Create a Grid World Transition System
    //=========================================================================
    std::cout << "========================================\n";
    std::cout << "Testing New Automaton Classes\n";
    std::cout << "========================================\n\n";

    // Create a 2x2 grid
    TransitionSystem gridTS(2, 2);

    std::cout << "Grid World Transition System: " << gridTS.grid_width << "x" << gridTS.grid_height << " grid\n";
    std::cout << "States: " << gridTS.numStates() << "\n";
    std::cout << "Atomic Props: " << gridTS.numAPs() << "\n\n";

    //=========================================================================
    // 2. Convert Grid World TS to new TS Automaton class
    //=========================================================================
    TS tsAutomaton(gridTS);
    
    std::cout << "=== TS Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << tsAutomaton.getnumStates() << std::endl;
    std::cout << "Edges: " << tsAutomaton.getnumEdges() << std::endl;

    //=========================================================================
    // 3. Parse LTL Formula
    //=========================================================================
    std::string input = "GF (\"2\" & X (!\"2\" U \"1\"))";
    
    spot::parsed_formula pf = spot::parse_infix_psl(input);
    if (pf.format_errors(std::cerr))
        return 1;
    spot::formula f = pf.f;
    
    std::cout << "\n=== Input Formula ===" << std::endl;
    std::cout << "Infix: " << spot::str_psl(f) << std::endl;
    std::cout << "LaTeX: ";
    print_latex_psl(std::cout, f) << std::endl;

    //=========================================================================
    // 4. Translate LTL to Büchi Automaton using new BuchiAutomaton class
    //=========================================================================
    spot::translator trans;
    spot::twa_graph_ptr buchiSpot = trans.run(f);
    
    BuchiAutomaton buchiAutomaton;
    buchiAutomaton.fromSpotAutomaton(buchiSpot);
    
    std::cout << "\n=== Büchi Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << buchiAutomaton.getnumStates() << std::endl;
    std::cout << "Edges: " << buchiAutomaton.getnumEdges() << std::endl;
    std::cout << "Accepting States: " << buchiAutomaton.getAcceptingStates().size() << std::endl;

    //=========================================================================
    // 5. Compute Product Automaton using new ProductAutomaton class
    //=========================================================================
    // Create product using Spot (for comparison and accepting run detection)
    spot::twa_graph_ptr tsSpot = tsAutomaton.toSpotAutomaton();
    spot::twa_graph_ptr productSpot = spot::product(tsSpot, buchiSpot);
    
    // Create product automaton from Spot result
    ProductAutomaton productAutomaton(productSpot);

    std::cout << "\n=== Product Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << productAutomaton.getnumStates() << std::endl;
    std::cout << "Edges: " << productAutomaton.getnumEdges() << std::endl;

    //=========================================================================
    // 6. Check for Accepting Run (Emptiness Check)
    //=========================================================================
    std::cout << "\n=== Emptiness Check ===" << std::endl;
    
    auto run = productSpot->accepting_run();
    if (run) {
        std::cout << "SATISFIABLE: An accepting run exists!" << std::endl;
        
        std::cout << "\n--- Accepting Run ---" << std::endl;
        std::cout << "Prefix (path to cycle):" << std::endl;
        for (const auto& step : run->prefix) {
            std::cout << "  State " << step.s << std::endl;
        }
        std::cout << "Cycle (accepting loop):" << std::endl;
        for (const auto& step : run->cycle) {
            std::cout << "  State " << step.s << std::endl;
        }
        
        // Highlight the accepting run in the product automaton
        run->highlight(5);
    } else {
        std::cout << "UNSATISFIABLE: No accepting run exists." << std::endl;
        std::cout << "The LTL formula cannot be satisfied on this transition system." << std::endl;
    }

    //=========================================================================
    // 7. Export to DOT files for visualization
    //=========================================================================
    std::ofstream buchiDot("output/buchi_automaton.dot");
    spot::print_dot(buchiDot, buchiSpot);
    std::cout << "\nExported: output/buchi_automaton.dot\n";

    std::ofstream tsDot("output/ts_automaton.dot");
    spot::print_dot(tsDot, tsSpot);
    std::cout << "Exported: output/ts_automaton.dot\n";

    std::ofstream productDot("output/product_automaton.dot");
    spot::print_dot(productDot, productSpot);
    std::cout << "Exported: output/product_automaton.dot\n";

    return 0;
}
