#include <vector>
#include <string>
#include <unordered_set>
#include <cstdint>
#include <cassert>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <unordered_map>
#include <spot/twa/twagraph.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/dot.hh>
#include <bddx.h>


//grid-based transition system implementation for one robot moving in 2D space
enum class Action : uint8_t {UP, DOWN, LEFT, RIGHT};

// Atomic Proposition structure
class AtomicProposition {
private:
    long id {0};
    std::string name;
public:
    AtomicProposition() = default;
    AtomicProposition(long i, std::string n) : id(i), name(std::move(n)) {}
    std::string getName() const { return name; }
    long getId() const { return id; }
    void setId(long new_id) {id = new_id;}
    bool operator==(const AtomicProposition& other) const {
        return id == other.id;
    }
};
// Hash for AtomicProposition so we can use it in unordered containers
struct AtomicProposition_Hash {
    std::size_t operator()(const AtomicProposition& ap) const {
        return std::hash<std::string>{}(ap.getName());
    }
};

// State represents a position on a 2D grid
class State {
public:
    // Set of atomic propositions true in this state
    std::unordered_set<AtomicProposition, AtomicProposition_Hash> props;
    bool operator==(const State& other) const {
        return props == other.props;
    }
};

// Hash for State so we can use it in unordered containers
struct StateHash {
    std::size_t operator()(const State& s) const {
        // hash function for the unordered_set of AtomicProposition
        std::size_t seed = 0;
        for (const auto& ap : s.props) {
            seed ^= AtomicProposition_Hash{}(ap) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
        }
        return seed;
    }
};

class Transition {
public:
    State next;
    Action action;
    double cost{1.0};
    bool operator==(const Transition& other) const {
        return next == other.next && action == other.action && cost == other.cost;
    }
    std::string toString() const;
};

class TransitionSystem {
public:
    int grid_width;
    int grid_height;
    std::unordered_set<State, StateHash> states;
    std::unordered_set<AtomicProposition, AtomicProposition_Hash> atomic_props;
    std::unordered_set<State, StateHash> initial_states;
    std::unordered_set<State, StateHash> obstacles;
    State current_state;

    // Convert (x,y) to cell ID
    int cellId(int x, int y) const {
        return y * grid_width + x;
    }

    // Extract x from cell ID
    int cellX(int id) const {
        return id % grid_width;
    }

    // Extract y from cell ID  
    int cellY(int id) const {
        return id / grid_width;
    }

    // Helper function to extract x coordinate from state props
    int getX(const State& s) const {
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
    int getY(const State& s) const {
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
    State createState(int x, int y) const {
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

    // Constructor without initial states assume initial state 0,0
    TransitionSystem(int width, int height) : grid_width(width), grid_height(height) {
        // Generate atomic propositions - one per cell
        // Set initial state to (0,0)
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

    // Alternative constructor with specified initial states
    TransitionSystem(int width, int height,
                     const std::unordered_set<State, StateHash>& init_states)
        : grid_width(width), grid_height(height) {
        // Generate atomic propositions - one per cell
        // Set initial states
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

    // Check if a state is valid (within bounds and not an obstacle)
    bool isValid(const State& s) const {
        int x = getX(s);
        int y = getY(s);
        if (x < 0 || x >= grid_width || y < 0 || y >= grid_height)
            return false;
        return true;
    }
    
    // Getter for current state
    State getCurrentState() const {
        return current_state;
    }

    // Perform a state transition based on the given action
    void StateTransition(const Action& a) {
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

    // Generate a vector of valid successor states from current state
    std::vector<Transition> successors(const State& s) const {
        std::vector<Transition> result;

        // Define all possible moves
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
    static void display_successors(const std::vector<Transition>& succs) {
        for (const auto& t : succs) {
            std::cout << t.toString() << "\n";
        }
    }

    //getter for number of atomic propositions
    int32_t numAPs() const {
        return static_cast<int32_t>(atomic_props.size());
    }

    //getter for number of states
    int32_t numStates() const {
        return static_cast<int32_t>(states.size());
    }

    // Get AP names (sorted for consistency)
    std::vector<std::string> getAPNames() const {
        std::vector<std::string> names;
        for (const auto& ap : atomic_props) {
            names.push_back(ap.getName());
        }
        std::sort(names.begin(), names.end());
        return names;
    }

    // Get labels (true APs) for a state as a set of strings
    std::unordered_set<std::string> getLabels(const State& s) const {
        std::unordered_set<std::string> labels;
        for (const auto& ap : s.props) {
            if (ap.getValue()) {
                labels.insert(ap.getName());
            }
        }
        return labels;
    }

    // Convert state to integer ID (for Spot automaton)
    int stateToId(const State& s) const {
        int x = getX(s);
        int y = getY(s);
        return y * grid_width + x;
    }

    // Convert integer ID back to State
    State idToState(int id) const {
        int x = id % grid_width;
        int y = id / grid_width;
        return createState(x, y);
    }

    // Label function: return true AP as a string where the position is encoded
    std::string label(const State& s) {
        int x = getX(s);
        int y = getY(s);
        return "(" + std::to_string(x) + "," + std::to_string(y) + ")";
    }

    // Convert action to string for display
    static std::string actionToString(Action a) {
        switch (a) {
            case Action::UP: return "UP";
            case Action::DOWN: return "DOWN";
            case Action::LEFT: return "LEFT";
            case Action::RIGHT: return "RIGHT";
        }
        return "UNKNOWN";
    }

    // Convert to Spot-compatible automaton
    spot::twa_graph_ptr toSpotAutomaton(const spot::bdd_dict_ptr& dict = nullptr) const {
        auto useDict = dict ? dict : spot::make_bdd_dict();
        auto aut = spot::make_twa_graph(useDict);

        // No acceptance condition for TS (all states trivially accept)
        aut->set_acceptance(0, spot::acc_cond::acc_code::t());

        // Create states (width * height states)
        int nStates = grid_width * grid_height;
        aut->new_states(static_cast<unsigned>(nStates));

        // Set initial state
        if (!initial_states.empty()) {
            const State& initState = *initial_states.begin();
            aut->set_init_state(static_cast<unsigned>(stateToId(initState)));
        }

        // Register atomic propositions and build var map
        auto apNames = getAPNames();
        std::unordered_map<std::string, int> apVars;
        for (const auto& name : apNames) {
            apVars[name] = aut->register_ap(name);
        }

        // Helper: convert state labels to BDD
        auto labelToBdd = [&](const State& s) {
            auto labels = getLabels(s);
            bdd cond = bddtrue;
            for (const auto& name : apNames) {
                int var = apVars.at(name);
                cond &= (labels.count(name) > 0) ? bdd_ithvar(var) : bdd_nithvar(var);
            }
            return cond;
        };

        // Add edges for all states
        for (const auto& s : states) {
            if (!isValid(s)) continue;
            unsigned srcId = static_cast<unsigned>(stateToId(s));

            for (const auto& t : successors(s)) {
                unsigned dstId = static_cast<unsigned>(stateToId(t.next));
                // Label edges with the labels of the destination state
                bdd cond = labelToBdd(t.next);
                aut->new_edge(srcId, dstId, cond);
            }
        }

        return aut;
    }

    // Export automaton to DOT format (for GraphViz visualization)
    void exportDot(const std::string& filename) const {
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
    static void exportAutomatonDot(const spot::twa_graph_ptr& aut, const std::string& filename) {
        std::ofstream dotfile(filename);
        if (!dotfile) {
            std::cerr << "Failed to open DOT output file: " << filename << "\n";
            return;
        }
        spot::print_dot(dotfile, aut);
        std::cout << "Exported DOT file: " << filename << "\n";
    }

    // Export nodes and edges for gnuplot visualization
    void exportGnuplot(const std::string& nodesPath,
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
                if (ap.getValue()) {
                    names.push_back(ap.getName());
                }
            }
            std::sort(names.begin(), names.end());
            std::ostringstream oss;
            for (size_t i = 0; i < names.size(); ++i) {
                if (i > 0) oss << ",";
                oss << names[i];
            }
            return oss.str();
        };

        // Build a stable list of states for layout
        std::vector<State> stateList(states.begin(), states.end());
        std::sort(stateList.begin(), stateList.end(), [&](const State& a, const State& b) {
            int ax = getX(a), ay = getY(a);
            int bx = getX(b), by = getY(b);
            if (ax != bx) return ax < bx;
            return ay < by;
        });

        // Nodes: use grid coordinates directly (x,y positions), scaled up
        const double scale = 12.0;  // Scale to spread nodes further apart
        for (const auto& s : stateList) {
            int x = getX(s);
            int y = getY(s);
            double sx = x * scale;
            double sy = y * scale;
            nodes << sx << " " << sy << "\n";
            nodesLabels << sx << " " << sy << " \"" << stateLabel(s) << "\"\n";
        }

        // Build map from state to scaled grid coordinates for edge drawing
        auto stateToCoords = [&](const State& s) -> std::pair<double, double> {
            return {getX(s) * scale, getY(s) * scale};
        };

        // Edges as curved line segments to avoid overlapping labels
        // For each edge, create a bezier curve by adding an offset control point
        for (const auto& s : stateList) {
            auto [x1, y1] = stateToCoords(s);
            auto succ = successors(s);
            for (size_t edge_idx = 0; edge_idx < succ.size(); ++edge_idx) {
                const auto& t = succ[edge_idx];
                auto [x2, y2] = stateToCoords(t.next);

                // Calculate perpendicular offset to curve the edge
                double dx = x2 - x1;
                double dy = y2 - y1;
                double dist = std::sqrt(dx * dx + dy * dy);
                double offset = 0.6 * scale;  // Control curve amount

                double px, py;  // perpendicular direction
                if (dist > 0.001) {
                    px = -dy / dist;
                    py = dx / dist;
                } else {
                    px = 1.0;
                    py = 0.0;
                }

                // Offset control point
                double cx = (x1 + x2) / 2.0 + px * offset;
                double cy = (y1 + y2) / 2.0 + py * offset;

                // Write 3 points: start, control, end (using center coordinates)
                edges << x1 << " " << y1 << "\n";
                edges << cx << " " << cy << "\n";
                edges << x2 << " " << y2 << "\n\n";

                // Label at control point
                edgesLabels << cx << " " << cy << " \"" << actionToString(t.action) << "\"\n";
            }
        }

        // Gnuplot script with smooth bezier curves
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
};

// Transition::toString() implementation (after TransitionSystem is defined)
std::string Transition::toString() const {
    std::ostringstream oss;
    oss << TransitionSystem::actionToString(action) << " -> cell ";
    // Get the cell ID from the state's AP
    for (const auto& ap : next.props) {
        if (ap.getValue()) {
            oss << ap.getName();
            break;
        }
    }
    oss << ", Cost: " << cost;
    return oss.str();
}

// Only compile main when this file is the main translation unit
#ifndef GRIDWORLD_TS_NO_MAIN
// Example usage
int main() {
    // Create a 4x4 grid
    TransitionSystem ts(4, 4);

    // Start at position (0, 0)
    State s0 = ts.createState(0, 0);

    std::cout << "Transition System: " << ts.grid_width << "x" << ts.grid_height << " grid\n";
    std::cout << "Obstacles: " << ts.obstacles.size() << "\n\n";

    // Generate successors from initial state
    auto successors = ts.successors(s0);
    std::cout << "Successors from state" << ts.label(s0) << ": " 
              << successors.size() << "\n";
    // Display successors
    TransitionSystem::display_successors(successors);

    // Show label for initial state
    std::cout << "\nLabel at (" << ts.getX(s0) << ", " << ts.getY(s0) << "): "
              << ts.label(s0) << "\n";

    // Export gnuplot files (nodes, edges, script)
    ts.exportGnuplot("output/atomic_ts_nodes.dat",
                     "output/atomic_ts_edges.dat",
                     "output/atomic_ts_plot.gp");

    // Export DOT file for GraphViz visualization
    ts.exportDot("output/transition_system.dot");

    // Convert to Spot-compatible automaton and print in HOA format
    auto spot_ts = ts.toSpotAutomaton();
    std::cout << "\n=== Spot-Compatible TS (HOA) ===\n";
    spot::print_hoa(std::cout, spot_ts);

    return 0;
}
#endif // GRIDWORLD_TS_NO_MAIN