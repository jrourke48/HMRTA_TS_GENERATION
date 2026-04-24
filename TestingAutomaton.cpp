#include <string>
#include <iostream>
#include <fstream>
#include <cstdlib>
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
#include "Automatons/TS.h"
#include "Automatons/BuchiAutomaton.h"
#include "Automatons/ProductAutomaton.h"

int main()
{
    //=========================================================================
    // 1. Create a Grid World Transition System
    //=========================================================================
    std::cout << "========================================\n";
    std::cout << "Testing New Automaton Classes\n";
    std::cout << "========================================\n\n";

    // Create a 2columx3row grid
    TransitionSystem gridTS(2, 3);

    std::cout << "Grid World Transition System: " << gridTS.grid_width << "x" << gridTS.grid_height << " grid\n";
    std::cout << "States: " << gridTS.numStates() << "\n";
    std::cout << "Atomic Props: " << gridTS.numAPs() << "\n\n";

    //=========================================================================
    // 2. Convert Grid World TS to new TS Automaton class
    //=========================================================================
    TS tsAutomaton(&gridTS);
    
    std::cout << "=== TS Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << tsAutomaton.getnumStates() << std::endl;
    std::cout << "Edges: " << tsAutomaton.getnumEdges() << std::endl;

    //=========================================================================
    // 3. Parse LTL Formula
    //=========================================================================
    std::string input = "GF (\"4\" & X (!\"4\" U \"0\"))";
    
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
    // Important: both automata must share the same BDD dictionary for product()
    spot::twa_graph_ptr tsSpot = tsAutomaton.toSpotAutomaton(buchiSpot->get_dict());
    spot::twa_graph_ptr productSpot = spot::product(tsSpot, buchiSpot);
    
    // Create product automaton from Spot result
    ProductAutomaton productAutomaton(productSpot);

    std::cout << "\n=== Product Automaton (New Class) ===" << std::endl;
    std::cout << "States: " << productAutomaton.getnumStates() << std::endl;
    std::cout << "Edges: " << productAutomaton.getnumEdges() << std::endl;

    //=========================================================================
    // 5b. Validate Product Automaton Conversion
    //=========================================================================
    std::cout << "\n=== Validating Product Automaton Conversion ===" << std::endl;
    
    bool conversion_valid = true;
    
    // Check 1: State count match
    if (productAutomaton.getnumStates() != productSpot->num_states()) {
        std::cout << "ERROR: State count mismatch!\n";
        std::cout << "  Custom: " << productAutomaton.getnumStates() << "\n";
        std::cout << "  Spot:   " << productSpot->num_states() << "\n";
        conversion_valid = false;
    } else {
        std::cout << "✓ State count matches: " << productAutomaton.getnumStates() << "\n";
    }
    
    // Check 2: Edge count match
    unsigned long spotEdgeCount = 0;
    for (auto edge : productSpot->edges()) {
        spotEdgeCount++;
    }
    if (productAutomaton.getnumEdges() != spotEdgeCount) {
        std::cout << "ERROR: Edge count mismatch!\n";
        std::cout << "  Custom: " << productAutomaton.getnumEdges() << "\n";
        std::cout << "  Spot:   " << spotEdgeCount << "\n";
        conversion_valid = false;
    } else {
        std::cout << "✓ Edge count matches: " << productAutomaton.getnumEdges() << "\n";
    }
    
    // Check 3: Verify accepting states
    unsigned long spotAcceptingEdges = 0;
    for (auto edge : productSpot->edges()) {
        if (edge.acc != spot::acc_cond::mark_t()) {
            spotAcceptingEdges++;
        }
    }
    auto customAcceptingStates = productAutomaton.getAcceptingStates();
    if (customAcceptingStates.size() == 0 && spotAcceptingEdges > 0) {
        std::cout << "WARNING: No accepting states in custom automaton but Spot has accepting edges!\n";
        std::cout << "  Spot has " << spotAcceptingEdges << " accepting edges\n";
        conversion_valid = false;
    } else {
        std::cout << "✓ Accepting states identified: " << customAcceptingStates.size() << "\n";
    }
    
    // Check 4: Spot-check a few edges for correctness
    bool edges_match = true;
    unsigned checked_edges = 0;
    unsigned max_check = std::min(5u, (unsigned)spotEdgeCount);
    for (auto spotEdge : productSpot->edges()) {
        if (checked_edges >= max_check) break;
        
        // Check if this edge exists in custom automaton
        if (!productAutomaton.isAdjacent(spotEdge.src, spotEdge.dst)) {
            std::cout << "ERROR: Edge (" << spotEdge.src << " -> " << spotEdge.dst << ") missing in custom automaton!\n";
            edges_match = false;
        }
        checked_edges++;
    }
    
    if (edges_match && checked_edges > 0) {
        std::cout << "✓ Spot-checked " << checked_edges << " edges - all present and correct\n";
    }
    
    if (conversion_valid) {
        std::cout << "✓ Product automaton conversion VALID\n";
    } else {
        std::cout << "✗ Product automaton conversion has issues\n";
    }

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
    // 6b. Validate Product Automaton Structure
    //=========================================================================
    std::cout << "\n=== Product Automaton Validation ===" << std::endl;
    std::cout << "Product has " << productSpot->num_states() << " states\n";
    std::cout << "TS has " << tsSpot->num_states() << " states, Büchi has " << buchiSpot->num_states() << " states\n";
    std::cout << "Max expected product states: " << (tsSpot->num_states() * buchiSpot->num_states()) << "\n";
    
    // Count product edges
    unsigned long productEdges = 0;
    for (auto edge : productSpot->edges()) {
        productEdges++;
    }
    std::cout << "Product has " << productEdges << " edges\n";
    
    // Check initial states
    std::cout << "Product initial state: " << productSpot->get_init_state_number() << "\n";
    
    // Count accepting states in product
    unsigned long acceptingCount = 0;
    for (unsigned s = 0; s < productSpot->num_states(); ++s) {
        for (auto edge : productSpot->out(s)) {
            if (edge.acc != spot::acc_cond::mark_t()) {
                acceptingCount++;
                break;
            }
        }
    }
    std::cout << "Product has " << acceptingCount << " states with accepting transitions\n";
    std::cout << "Emptiness check result: " << (run ? "NOT EMPTY (formula satisfiable)" : "EMPTY (formula unsatisfiable)") << "\n";

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

    //=========================================================================
    // 8. Convert DOT files to SVG and PNG
    //=========================================================================
    std::cout << "\n=== Generating Visualizations ===" << std::endl;
    
    // Convert to SVG
    int ret_svg_buchi = system("dot -Tsvg output/buchi_automaton.dot -o output/buchi_automaton.svg");
    int ret_svg_ts = system("dot -Tsvg output/ts_automaton.dot -o output/ts_automaton.svg");
    int ret_svg_product = system("dot -Tsvg output/product_automaton.dot -o output/product_automaton.svg");
    
    // Convert to PNG
    int ret_png_buchi = system("dot -Tpng output/buchi_automaton.dot -o output/buchi_automaton.png");
    int ret_png_ts = system("dot -Tpng output/ts_automaton.dot -o output/ts_automaton.png");
    int ret_png_product = system("dot -Tpng output/product_automaton.dot -o output/product_automaton.png");
    
    if (ret_svg_buchi == 0 && ret_svg_ts == 0 && ret_svg_product == 0) {
        std::cout << "Successfully generated SVG files.\n";
    } else {
        std::cout << "Warning: Some SVG files could not be generated. Make sure graphviz is installed.\n";
    }
    
    if (ret_png_buchi == 0 && ret_png_ts == 0 && ret_png_product == 0) {
        std::cout << "Successfully generated PNG files.\n";
    } else {
        std::cout << "Warning: Some PNG files could not be generated. Make sure graphviz is installed.\n";
    }

    return 0;
}
