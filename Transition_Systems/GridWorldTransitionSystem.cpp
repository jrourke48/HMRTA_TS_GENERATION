#include "GridWorldTransitionSystem.h"

// Constructor without initial states (assumes initial state 0,0)
TransitionSystem::TransitionSystem(int width, int height) 
    : grid_width(width), grid_height(height) {
    // Generate atomic propositions - one per cell
    initial_states.insert(createState(0, 0));
    current_state = createState(0, 0);
    // Generate all states
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int id = cellId(x, y);
            atomic_props.insert(AtomicProposition(id, std::to_string(id)));
            states.insert(createState(x, y));
        }
    }
}

// Constructor with specified initial states
TransitionSystem::TransitionSystem(int width, int height,
                     const std::unordered_set<State, StateHash>& init_states)
    : grid_width(width), grid_height(height) {
    // Generate atomic propositions - one per cell
    initial_states = init_states;
    current_state = *initial_states.begin();
    // Generate all states
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            int id = cellId(x, y);
            atomic_props.insert(AtomicProposition(id, std::to_string(id)));
            states.insert(createState(x, y));
        }
    }
}

// Helper function to extract x coordinate from state props
int TransitionSystem::getX(const State& s) const {
    try {
        AtomicProposition ap = *(s.props.begin()); // Assuming only one AP is true for a valid state
        int id = ap.getId();
        return cellX(id);
    } catch (const std::exception& e) {
        std::cerr << "Error extracting coordinates from state: " << e.what() << "\n";
        return -1; // Invalid state
    }
}

// Helper function to extract y coordinate from state props
int TransitionSystem::getY(const State& s) const {
    try {
        AtomicProposition ap = *(s.props.begin()); // Assuming only one AP is true for a valid state
        int id = ap.getId();
        return cellY(id);
    } catch (const std::exception& e) {
        std::cerr << "Error extracting coordinates from state: " << e.what() << "\n";
        return -1; // Invalid state
    }
}

// Helper function to create a state from x,y coordinates
State TransitionSystem::createState(int x, int y) const {
    State s;
    int id = cellId(x, y);
    std::string apName = std::to_string(id);
    for (const auto& ap : atomic_props) {
        if (ap.getName() == apName) {
            AtomicProposition ap_copy = ap;
            s.props.insert(ap_copy);
            break;
        }
    }
    return s;
}

// Check if a state is valid (within bounds and not an obstacle)
bool TransitionSystem::isValid(const State& s) const {
    int x = getX(s);
    int y = getY(s);
    if (x < 0 || x >= grid_width || y < 0 || y >= grid_height)
        return false;
    return true;
}

// Perform a state transition based on the given action
void TransitionSystem::StateTransition(const Action& a) {
    int curr_x = getX(current_state);
    int curr_y = getY(current_state);
    int new_x = curr_x;
    int new_y = curr_y;

    switch (a) {
        case Action::UP:        new_y += 1; break;
        case Action::DOWN:      new_y -= 1; break;
        case Action::LEFT:      new_x -= 1; break;
        case Action::RIGHT:     new_x += 1; break;
    }

    State next_state = createState(new_x, new_y);
    if (isValid(next_state)) {
        current_state = next_state;
    } else {
        std::cerr << "Invalid transition attempted from (" << curr_x << ", " << curr_y 
                  << ") to (" << new_x << ", " << new_y << ")\n";
    }
}

// Generate a vector of valid successor states from given state
std::vector<Transition> TransitionSystem::successors(const State& s) const {
    std::vector<Transition> result;

    struct Move {
        Action action;
        int dx;
        int dy;
        double cost;
    };

    std::vector<Move> moves = {
        {Action::UP,        0,  1, 1.0},
        {Action::DOWN,      0, -1, 1.0},
        {Action::LEFT,     -1,  0, 1.0},
        {Action::RIGHT,     1,  0, 1.0},
    };

    int curr_x = getX(s);
    int curr_y = getY(s);

    for (const Move& move : moves) {
        State next = createState(curr_x + move.dx, curr_y + move.dy);
        if (isValid(next)) {
            result.push_back(Transition{next, move.action, move.cost});
        }
    }

    return result;
}

// Display successors for a given state
void TransitionSystem::display_successors(const std::vector<Transition>& succs) {
    for (const auto& t : succs) {
        std::cout << t.toString() << "\n";
    }
}

// Get AP names (sorted for consistency)
std::vector<std::string> TransitionSystem::getAPNames() const {
    std::vector<std::string> names;
    for (const auto& ap : atomic_props) {
        names.push_back(ap.getName());
    }
    std::sort(names.begin(), names.end());
    return names;
}

// Get labels (true APs) for a state as a set of strings
std::unordered_set<std::string> TransitionSystem::getLabels(const State& s) const {
    std::unordered_set<std::string> labels;
    for (const auto& ap : s.props) {
        labels.insert(ap.getName());
    }
    return labels;
}

// Convert state to integer ID (for Spot automaton)
int TransitionSystem::stateToId(const State& s) const {
    int x = getX(s);
    int y = getY(s);
    return y * grid_width + x;
}

// Convert integer ID back to State
State TransitionSystem::idToState(int id) const {
    int x = id % grid_width;
    int y = id / grid_width;
    return createState(x, y);
}

// Label function: return label as a string where the position is encoded
std::string TransitionSystem::label(const State& s) {
    std::string label = "";
    for (const auto& prop : s.props) {
        label += prop.getName() + ", ";
    }
    if (!label.empty()) {
        label.pop_back(); // Remove trailing space
        label.pop_back(); // Remove trailing comma
    }
    return label;
}

// Convert action to string for display
std::string TransitionSystem::actionToString(Action a) {
    switch (a) {
        case Action::UP: return "UP";
        case Action::DOWN: return "DOWN";
        case Action::LEFT: return "LEFT";
        case Action::RIGHT: return "RIGHT";
    }
    return "UNKNOWN";
}

// Create a BDD dictionary for TS automata with all APs pre-registered
spot::bdd_dict_ptr TransitionSystem::createBDDDictionary() const {
    auto dict = spot::make_bdd_dict();
    
    // Pre-register all APs in this TS
    for (const auto& ap : atomic_props) {
        auto formula_ap = spot::formula::ap(ap.getName());
        dict->register_proposition(formula_ap, nullptr);
    }
    
    return dict;
}

// Convert to Spot-compatible automaton
spot::twa_graph_ptr TransitionSystem::toSpotAutomaton(const spot::bdd_dict_ptr& dict) const {
    auto useDict = dict ? dict : spot::make_bdd_dict();
    auto aut = spot::make_twa_graph(useDict);

    // Set up acceptance condition (universal, no acceptance sets needed)
    aut->set_acceptance(0, spot::acc_cond::acc_code::t());

    // Create states for the automaton
    unsigned nStates = static_cast<unsigned>(numStates());
    aut->new_states(nStates);

    // Set initial state
    if (!initial_states.empty()) {
        const State& initState = *initial_states.begin();
        unsigned initId = static_cast<unsigned>(stateToId(initState));
        aut->set_init_state(initId);
    }

    // Build mapping from AP names to BDD variable indices
    auto apNames = getAPNames();
    std::map<std::string, int> apToVar;
    
    // Just use the AP names directly as variable indices (0, 1, 2, 3...)
    for (size_t i = 0; i < apNames.size(); ++i) {
        apToVar[apNames[i]] = static_cast<int>(i);
    }

    // Build all edges with their BDD labels
    for (const auto& s : states) {
        if (!isValid(s)) continue;
        
        unsigned srcId = static_cast<unsigned>(stateToId(s));
        
        for (const auto& transition : successors(s)) {
            unsigned dstId = static_cast<unsigned>(stateToId(transition.next));
            
            // Build BDD label from the destination state's properties
            bdd edgeLabel = bddtrue;
            
            // For each registered AP, check if it holds in the destination state
            for (const auto& [apName, varIdx] : apToVar) {
                bool apHolds = false;
                for (const auto& prop : transition.next.props) {
                    if (prop.getName() == apName) {
                        apHolds = true;
                        break;
                    }
                }
                
                // AND the appropriate BDD term
                if (apHolds) {
                    edgeLabel &= bdd_ithvar(varIdx);
                } else {
                    edgeLabel &= bdd_nithvar(varIdx);
                }
            }
            
            // Add edge with the constructed BDD label
            aut->new_edge(srcId, dstId, edgeLabel);
        }
    }

    return aut;
}

// Export automaton to DOT format (for GraphViz visualization)
void TransitionSystem::exportDot(const std::string& filename) const {
    auto aut = toSpotAutomaton();
    std::ofstream dotfile(filename);
    if (!dotfile) {
        std::cerr << "Failed to open DOT output file: " << filename << "\n";
        return;
    }
    spot::print_dot(dotfile, aut);
    std::cout << "Exported DOT file: " << filename << "\n";
    std::cout << "Render with: dot -Tpng " << filename << " -o " 
              << filename.substr(0, filename.rfind('.')) << ".png\n";
}

// Export any Spot automaton to DOT format
void TransitionSystem::exportAutomatonDot(const spot::twa_graph_ptr& aut, const std::string& filename) {
    std::ofstream dotfile(filename);
    if (!dotfile) {
        std::cerr << "Failed to open DOT output file: " << filename << "\n";
        return;
    }
    spot::print_dot(dotfile, aut);
    std::cout << "Exported DOT file: " << filename << "\n";
}

// Export nodes and edges for gnuplot visualization
void TransitionSystem::exportGnuplot(const std::string& nodesPath,
                       const std::string& edgesPath,
                       const std::string& scriptPath) const {
    const std::string nodesLabelPath = nodesPath + ".labels";
    const std::string edgesLabelPath = edgesPath + ".labels";

    std::ofstream nodes(nodesPath);
    std::ofstream edges(edgesPath);
    std::ofstream nodesLabels(nodesLabelPath);
    std::ofstream edgesLabels(edgesLabelPath);
    std::ofstream script(scriptPath);

    if (!nodes || !edges || !nodesLabels || !edgesLabels || !script) {
        std::cerr << "Failed to open gnuplot output files.\n";
        return;
    }

    auto stateLabel = [](const State& s) {
        std::vector<std::string> names;
        names.reserve(s.props.size());
        for (const auto& ap : s.props) {
            names.push_back(ap.getName());
        }
        std::sort(names.begin(), names.end());
        std::ostringstream oss;
        for (size_t i = 0; i < names.size(); ++i) {
            if (i > 0) oss << ",";
            oss << names[i];
        }
        return oss.str();
    };

    std::vector<State> stateList(states.begin(), states.end());
    std::sort(stateList.begin(), stateList.end(), [&](const State& a, const State& b) {
        int ax = getX(a), ay = getY(a);
        int bx = getX(b), by = getY(b);
        if (ax != bx) return ax < bx;
        return ay < by;
    });

    const double scale = 12.0;
    for (const auto& s : stateList) {
        int x = getX(s);
        int y = getY(s);
        double sx = x * scale;
        double sy = y * scale;
        nodes << sx << " " << sy << "\n";
        nodesLabels << sx << " " << sy << " \"" << stateLabel(s) << "\"\n";
    }

    auto stateToCoords = [&](const State& s) -> std::pair<double, double> {
        return {getX(s) * scale, getY(s) * scale};
    };

    for (const auto& s : stateList) {
        auto [x1, y1] = stateToCoords(s);
        auto succ = successors(s);
        for (size_t edge_idx = 0; edge_idx < succ.size(); ++edge_idx) {
            const auto& t = succ[edge_idx];
            auto [x2, y2] = stateToCoords(t.next);

            double dx = x2 - x1;
            double dy = y2 - y1;
            double dist = std::sqrt(dx * dx + dy * dy);
            double offset = 0.6 * scale;

            double px, py;
            if (dist > 0.001) {
                px = -dy / dist;
                py = dx / dist;
            } else {
                px = 1.0;
                py = 0.0;
            }

            double cx = (x1 + x2) / 2.0 + px * offset;
            double cy = (y1 + y2) / 2.0 + py * offset;

            edges << x1 << " " << y1 << "\n";
            edges << cx << " " << cy << "\n";
            edges << x2 << " " << y2 << "\n\n";

            edgesLabels << cx << " " << cy << " \"" << actionToString(t.action) << "\"\n";
        }
    }

    double scaled_width = grid_width * scale;
    double scaled_height = grid_height * scale;
    script << "set terminal pngcairo size 1200,1000\n";
    script << "set output 'atomic_ts.png'\n";
    script << "set title 'Atomic Transition System'\n";
    script << "set size ratio -1\n";
    script << "set grid\n";
    script << "set key off\n";
    script << "set xrange [-" << scale << ":" << (scaled_width + scale) << "]\n";
    script << "set yrange [-" << scale << ":" << (scaled_height + scale) << "]\n";
    script << "plot \\\n";
    script << "    '" << edgesPath << "' with lines smooth bezier lc rgb '#999999' lw 2, \\\n";
    script << "    '" << nodesPath << "' with points pt 7 ps 10.0 lc rgb '#000000', \\\n";
    script << "    '" << nodesLabelPath << "' with labels tc rgb '#FFFFFF' font ',7' offset 0,0, \\\n";
    script << "    '" << edgesLabelPath << "' with labels tc rgb '#7A0019' font ',5' offset 0,0\n";
}

// Transition::toString() implementation
std::string Transition::toString() const {
    std::ostringstream oss;
    oss << TransitionSystem::actionToString(action) << " -> cell ";
    for (const auto& ap : next.props) {
        oss << ap.getName();
        break;
    }
    oss << ", Cost: " << cost;
    return oss.str();
}
