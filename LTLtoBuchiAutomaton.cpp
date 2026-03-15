#include <string>
#include <iostream>
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

int main()
{
    //=========================================================================
    // 1. Create a Grid World Transition System
    //=========================================================================
    std::cout << "========================================\n";
    std::cout << "LTL to Büchi with GridWorld TS\n";
    std::cout << "========================================\n\n";

    // Create a 2x2 grid
    TransitionSystem ts(2, 2);

    std::cout << "Transition System: " << ts.grid_width << "x" << ts.grid_height << " grid\n";
    std::cout << "States: " << ts.numStates() << "\n";
    std::cout << "Atomic Props: " << ts.numAPs() << "\n\n";

    // //=========================================================================
    // // 2. Create shared BDD dictionary with all TS APs
    // //=========================================================================
    // auto sharedDict = ts.createBDDDictionary();

    //=========================================================================
    // 3. Parse LTL Formula
    //=========================================================================
    std::string input = "GF (\"2\" & X (!\"2\" U \"1\"))";
    
    spot::parsed_formula pf = spot::parse_infix_psl(input);
    if (pf.format_errors(std::cerr))
        return 1;
    spot::formula f = pf.f;
    
    std::cout << "=== Input Formula ===" << std::endl;
    std::cout << "Infix: " << spot::str_psl(f) << std::endl;
    std::cout << "LaTeX: ";
    print_latex_psl(std::cout, f) << std::endl;

    //=========================================================================
    // 4. Translate LTL to Büchi Automaton (using shared dictionary)
    //=========================================================================
    spot::translator trans;  // Pass dict to constructor
    spot::twa_graph_ptr buchiAut = trans.run(f);
    
    std::cout << "\n=== Büchi Automaton ===" << std::endl;
    std::cout << "States: " << buchiAut->num_states() << std::endl;
    std::cout << "Edges: " << buchiAut->num_edges() << std::endl;
    std::cout << "Initial state: " << buchiAut->get_init_state_number() << std::endl;
    std::cout << "Atomic propositions: ";
    for (auto ap : buchiAut->ap())
        std::cout << ap << " ";
    std::cout << std::endl;
    std::cout << "Acceptance sets: " << buchiAut->num_sets() << std::endl;

    //=========================================================================
    // 5. Convert TS to Spot Automaton
    //========================================================================  
    // Use the Büchi automaton's dictionary so APs match
    spot::twa_graph_ptr tsAut = ts.toSpotAutomaton(buchiAut->get_dict());
    
    std::cout << "\n=== TS Automaton ===" << std::endl;
    std::cout << "States: " << tsAut->num_states() << std::endl;
    std::cout << "Edges: " << tsAut->num_edges() << std::endl;
    
    // spot::twa_graph_ptr tsAut  = buchiAut;

    //=========================================================================
    // 5. Compute Product Automaton
    //=========================================================================
    spot::twa_graph_ptr product = spot::product(tsAut, buchiAut);

    std::cout << "\n=== Product Automaton ===" << std::endl;
    std::cout << "States: " << product->num_states() << std::endl;
    std::cout << "Edges: " << product->num_edges() << std::endl;
    std::cout << "Acceptance sets: " << product->num_sets() << std::endl;

    //=========================================================================
    // 6. Check for Accepting Run (Emptiness Check)
    //=========================================================================
    std::cout << "\n=== Emptiness Check ===" << std::endl;
    
    auto run = product->accepting_run();
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
    spot::print_dot(buchiDot, buchiAut);
    std::cout << "\nExported: output/buchi_automaton.dot\n";

    std::ofstream tsDot("output/ts_automaton.dot");
    spot::print_dot(tsDot, tsAut);
    std::cout << "Exported: output/ts_automaton.dot\n";

    std::ofstream productDot("output/product_automaton.dot");
    spot::print_dot(productDot, product);
    std::cout << "Exported: output/product_automaton.dot\n";

    return 0;
}
