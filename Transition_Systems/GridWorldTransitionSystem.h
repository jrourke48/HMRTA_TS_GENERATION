#ifndef GRIDWORLD_TRANSITION_SYSTEM_H
#define GRIDWORLD_TRANSITION_SYSTEM_H

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

// Grid-based transition system implementation for one robot moving in 2D space
enum class Action : uint8_t { UP, DOWN, LEFT, RIGHT };

// Atomic Proposition structure
class AtomicProposition {
private:
    long id{ 0 };
    std::string name;

public:
    AtomicProposition() = default;
    AtomicProposition(long i, std::string n) : id(i), name(std::move(n)) {}
    
    std::string getName() const { return name; }
    long getId() const { return id; }
    void setId(long new_id) { id = new_id; }
    
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

// Transition represents a transition with an action and cost
class Transition {
public:
    State next;
    Action action;
    double cost{ 1.0 };
    
    bool operator==(const Transition& other) const {
        return next == other.next && action == other.action && cost == other.cost;
    }
    
    std::string toString() const;
};

// TransitionSystem - represents a grid-world transition system
class TransitionSystem {
public:
    int grid_width;
    int grid_height;
    std::unordered_set<State, StateHash> states;
    std::unordered_set<AtomicProposition, AtomicProposition_Hash> atomic_props;
    std::unordered_set<State, StateHash> initial_states;
    std::unordered_set<State, StateHash> obstacles;
    State current_state;

    // Constructors
    TransitionSystem(int width, int height);
    TransitionSystem(int width, int height, const std::unordered_set<State, StateHash>& init_states);

    // Grid coordinate helpers
    int cellId(int x, int y) const { return y * grid_width + x; }
    int cellX(int id) const { return id % grid_width; }
    int cellY(int id) const { return id / grid_width; }

    // State coordinate extraction
    int getX(const State& s) const;
    int getY(const State& s) const;

    // State creation
    State createState(int x, int y) const;

    // State validation
    bool isValid(const State& s) const;

    // Getter for current state
    State getCurrentState() const { return current_state; }

    // Perform a state transition based on the given action
    void StateTransition(const Action& a);

    // Generate a vector of valid successor states from given state
    std::vector<Transition> successors(const State& s) const;

    // Display successors for a given state
    static void display_successors(const std::vector<Transition>& succs);

    // Getters for counts
    int32_t numAPs() const { return static_cast<int32_t>(atomic_props.size()); }
    int32_t numStates() const { return static_cast<int32_t>(states.size()); }

    // Get AP names (sorted for consistency)
    std::vector<std::string> getAPNames() const;

    // Get labels (true APs) for a state as a set of strings
    std::unordered_set<std::string> getLabels(const State& s) const;

    // Convert state to/from integer ID (for Spot automaton)
    int stateToId(const State& s) const;
    State idToState(int id) const;

    // Label function: return true AP as a string where the position is encoded
    std::string label(const State& s);

    // Convert action to string for display
    static std::string actionToString(Action a);

    // Export any Spot automaton to DOT format
    static void exportAutomatonDot(const spot::twa_graph_ptr& aut, const std::string& filename);

    // Export nodes and edges for gnuplot visualization
    void exportGnuplot(const std::string& nodesPath,
                       const std::string& edgesPath,
                       const std::string& scriptPath) const;
};

#endif // GRIDWORLD_TRANSITION_SYSTEM_H
