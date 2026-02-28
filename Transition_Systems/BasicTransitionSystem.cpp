#include <vector>
#include <string>
#include <unordered_set>
#include <cstdint>
#include <iostream>
#include <algorithm>
#include <cmath>

//grid-based transition system implementation for one robot moving in 2D space
enum class Action : uint8_t { UP, DOWN, LEFT, RIGHT, UPRIGHT, UPLEFT, DOWNRIGHT, DOWNLEFT, WAIT };

// State represents a position on a 2D grid
struct State {
    int x{0};
    int y{0};

    bool operator==(const State& other) const {
        return x == other.x && y == other.y;
    }
};

// Hash for State so we can use it in unordered containers
struct StateHash {
    std::size_t operator()(const State& s) const {
        return std::hash<int>{}(s.x) ^ (std::hash<int>{}(s.y) << 1);
    }
};

struct Transition {
    State next;
    Action action;
    double cost{1.0};
};

class TransitionSystem {
public:
    int grid_width;
    int grid_height;
    std::unordered_set<State, StateHash> obstacles;

    explicit TransitionSystem(int width, int height)
        : grid_width(width), grid_height(height)  {}

    // Add an obstacle at position (x, y)
    void addObstacle(int x, int y) {
        obstacles.insert(State{x, y});
    }

    // Check if a state is valid (within bounds and not an obstacle)
    bool isValid(const State& s) const {
        if (s.x < 0 || s.x >= grid_width || s.y < 0 || s.y >= grid_height)
            return false;
        return obstacles.find(s) == obstacles.end();
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
            {Action::UPRIGHT,   1,  1, std::sqrt(2.0)},
            {Action::UPLEFT,   -1,  1, std::sqrt(2.0)},
            {Action::DOWNRIGHT, 1, -1, std::sqrt(2.0)},
            {Action::DOWNLEFT, -1, -1, std::sqrt(2.0)},
            {Action::WAIT,      0,  0, 0.5}
        };

        for (const Move& move : moves) {
            State next{s.x + move.dx, s.y + move.dy};
            if (isValid(next)) {
                result.push_back(Transition{next, move.action, move.cost});
            }
        }

        return result;
    }

    // Label function: return location as string only true AP is the position
    std::string label(const State& s) const {
        return "(" + std::to_string(s.x) + "," + std::to_string(s.y) + ")";
    }

    // Convert action to string for display
    static std::string actionToString(Action a) {
        switch (a) {
            case Action::UP: return "UP";
            case Action::DOWN: return "DOWN";
            case Action::LEFT: return "LEFT";
            case Action::RIGHT: return "RIGHT";
            case Action::UPRIGHT: return "UPRIGHT";
            case Action::UPLEFT: return "UPLEFT";
            case Action::DOWNRIGHT: return "DOWNRIGHT";
            case Action::DOWNLEFT: return "DOWNLEFT";
            case Action::WAIT: return "WAIT";
        }
        return "UNKNOWN";
    }
};

// Example usage
int main() {
    //10x10 grid
    TransitionSystem ts(10, 10);

    // Add obstacles
    ts.addObstacle(5, 5);
    ts.addObstacle(5, 6);
    ts.addObstacle(6, 5);

    // Start at position (0, 0)
    State s0{0, 0};

    std::cout << "Transition System: " << ts.grid_width << "x" << ts.grid_height << " grid\n";
    std::cout << "Obstacles: " << ts.obstacles.size() << "\n\n";

    //successors from initial state
    auto successors = ts.successors(s0);
    std::cout << "Successors from state (" << s0.x << ", " << s0.y << "): " 
              << successors.size() << "\n";
    
    for (size_t i = 0; i < std::min<size_t>(successors.size(), 10); ++i) {
        const auto& t = successors[i];
        std::cout << "  " << TransitionSystem::actionToString(t.action) 
                  << " -> (" << t.next.x << ", " << t.next.y << ")"
                  << " cost=" << t.cost << "\n";
    }

    //label for initial state
    std::cout << "\nLabel at (" << s0.x << ", " << s0.y << "): "
              << ts.label(s0) << "\n";

    // Test a state in the center
    State s_center{5, 4};
    std::cout << "\nSuccessors from state (" << s_center.x << ", " << s_center.y << "): "
              << ts.successors(s_center).size() << "\n";
    auto center_successors = ts.successors(s_center);
    for (const auto& t : center_successors) {
        std::cout << "  " << TransitionSystem::actionToString(t.action)
                  << " -> (" << t.next.x << ", " << t.next.y << ")"
                  << " cost=" << t.cost << "\n";
    }

    return 0;
}