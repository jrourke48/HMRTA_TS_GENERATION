#include "Environment/DStarGridWorld.h"
#include "Environment/GridWorld.h"
#include <algorithm>
#include <iostream>
#include <cmath>

// ==================== CONSTRUCTOR & DESTRUCTOR ====================

DStarGridWorldPlanner::DStarGridWorldPlanner(GridWorld* gridworld)
    : gridworld_(gridworld), km_(0.0), has_valid_path_(false),
      expanded_count_(0), update_count_(0) {
}

DStarGridWorldPlanner::~DStarGridWorldPlanner() {
}

// ==================== PUBLIC INTERFACE ====================

std::vector<Point> DStarGridWorldPlanner::plan(const Point& start, const Point& goal) {
    reset();
    
    sstart_ = start;
    sgoal_ = goal;
    scurrent_ = start;
    km_ = 0.0;
    
    // Initialize rhs and g
    // rhs(goal) = 0, all others = infinity
    rhs_[goal] = 0.0;
    g_[goal] = std::numeric_limits<double>::infinity();
    
    for (uint32_t x = 0; x < gridworld_->getWidth(); ++x) {
        for (uint32_t y = 0; y < gridworld_->getHeight(); ++y) {
            Point p(x, y);
            if (!(p.getX() == goal.getX() && p.getY() == goal.getY())) {
                rhs_[p] = std::numeric_limits<double>::infinity();
                g_[p] = std::numeric_limits<double>::infinity();
            }
        }
    }
    
    // Add goal to open list
    open_.push({calculateKey(goal), goal});
    
    // Compute shortest path
    computeShortestPath();
    
    // Check if start is reachable
    has_valid_path_ = (g_[start] != std::numeric_limits<double>::infinity());
    
    if (!has_valid_path_) {
        return std::vector<Point>();  // Empty path
    }
    
    return extractPath(start);
}

std::vector<Point> DStarGridWorldPlanner::replan(const Point& current, const std::vector<Point>& changed_cells) {
    scurrent_ = current;
    km_ += heuristic(sstart_, scurrent_);
    sstart_ = scurrent_;
    
    // Update costs for changed cells
    for (const auto& changed : changed_cells) {
        update_count_++;
        updateVertex(changed);
        
        // Also update neighbors since edge costs may have changed
        auto neighbors = getSuccessors(changed);
        for (const auto& neighbor : neighbors) {
            updateVertex(neighbor);
        }
    }
    
    // Recompute path
    computeShortestPath();
    
    // Check if still reachable
    has_valid_path_ = (g_[scurrent_] != std::numeric_limits<double>::infinity());
    
    if (!has_valid_path_) {
        return std::vector<Point>();
    }
    
    return extractPath(scurrent_);
}

double DStarGridWorldPlanner::getCost(const Point& pos) const {
    auto it = g_.find(pos);
    if (it == g_.end()) {
        return std::numeric_limits<double>::infinity();
    }
    return it->second;
}

// ==================== CORE D* LITE FUNCTIONS ====================

DStarGridWorldPlanner::DStarKey DStarGridWorldPlanner::calculateKey(const Point& s) const {
    double g_val = getG(s);
    double rhs_val = getRhs(s);
    double min_val = std::min(g_val, rhs_val);
    
    return {min_val + heuristic(s, sstart_) + km_, min_val};
}

double DStarGridWorldPlanner::getG(const Point& s) const {
    auto it = g_.find(s);
    return (it == g_.end()) ? std::numeric_limits<double>::infinity() : it->second;
}

double DStarGridWorldPlanner::getRhs(const Point& s) const {
    // Goal state has rhs = 0
    if (s.getX() == sgoal_.getX() && s.getY() == sgoal_.getY()) {
        return 0.0;
    }
    
    auto it = rhs_.find(s);
    return (it == rhs_.end()) ? std::numeric_limits<double>::infinity() : it->second;
}

double DStarGridWorldPlanner::heuristic(const Point& s) const {
    return heuristic(s, sgoal_);
}

double DStarGridWorldPlanner::heuristic(const Point& from, const Point& to) const {
    // Euclidean distance
    int dx = from.getX() - to.getX();
    int dy = from.getY() - to.getY();
    return std::sqrt(dx * dx + dy * dy);
}

double DStarGridWorldPlanner::movementCost(const Point& from, const Point& to) const {
    // If target is obstacle, infinite cost
    if (gridworld_->isObstacle(to)) {
        return std::numeric_limits<double>::infinity();
    }
    
    // Base cost: 1.0 for cardinal, sqrt(2) for diagonal
    int dx = std::abs((int)from.getX() - (int)to.getX());
    int dy = std::abs((int)from.getY() - (int)to.getY());
    
    if (dx + dy == 1) {
        // Cardinal direction
        return 1.0;
    } else if (dx == 1 && dy == 1) {
        // Diagonal direction
        return std::sqrt(2.0);
    } else {
        // Not adjacent
        return std::numeric_limits<double>::infinity();
    }
}

std::vector<Point> DStarGridWorldPlanner::getSuccessors(const Point& s) const {
    std::vector<Point> successors;
    int x = s.getX();
    int y = s.getY();
    
    // 8-directional neighbors
    int dx[] = {-1, -1, -1,  0,  0,  1,  1,  1};
    int dy[] = {-1,  0,  1, -1,  1, -1,  0,  1};
    
    for (int i = 0; i < 8; ++i) {
        int nx = x + dx[i];
        int ny = y + dy[i];
        
        if (gridworld_->isInBounds(Point(nx, ny))) {
            successors.push_back(Point(nx, ny));
        }
    }
    
    return successors;
}

std::vector<Point> DStarGridWorldPlanner::getPredecessors(const Point& s) const {
    // Predecessors are same as successors in undirected grid
    return getSuccessors(s);
}

void DStarGridWorldPlanner::updateVertex(const Point& s) {
    // If not goal, recompute rhs from successors
    if (!(s.getX() == sgoal_.getX() && s.getY() == sgoal_.getY())) {
        double min_cost = std::numeric_limits<double>::infinity();
        
        auto successors = getSuccessors(s);
        for (const auto& succ : successors) {
            double cost = movementCost(s, succ) + getG(succ);
            min_cost = std::min(min_cost, cost);
        }
        
        rhs_[s] = min_cost;
    }
    
    // Remove from open list if already there (lazy deletion)
    // Note: priority_queue doesn't support remove, so we mark inconsistent states
    // and filter on expansion
    
    // If g != rhs, add/update in open list
    if (getG(s) != getRhs(s)) {
        open_.push({calculateKey(s), s});
    }
}

void DStarGridWorldPlanner::computeShortestPath() {
    while (true) {
        // Check if open list is empty
        if (open_.empty()) {
            break;
        }
        
        // Get state with minimum key
        auto [key, s] = open_.top();
        open_.pop();
        
        // Check if this is stale (g != rhs) - lazy deletion
        if (key > calculateKey(s)) {
            // Key has changed, re-insert with new key
            open_.push({calculateKey(s), s});
            continue;
        }
        
        // If g > rhs, state is overconsistent
        if (getG(s) > getRhs(s)) {
            g_[s] = getRhs(s);
            expanded_count_++;
            
            // Update predecessors
            auto preds = getPredecessors(s);
            for (const auto& pred : preds) {
                updateVertex(pred);
            }
        }
        // If g < rhs, state is underconsistent
        else if (getG(s) < getRhs(s)) {
            g_[s] = std::numeric_limits<double>::infinity();
            updateVertex(s);
            
            // Update predecessors
            auto preds = getPredecessors(s);
            for (const auto& pred : preds) {
                updateVertex(pred);
            }
        }
        // If g == rhs, state is consistent - done
        else {
            break;
        }
        
        // Safety check: don't expand too many states
        if (static_cast<uint32_t>(expanded_count_) > gridworld_->getWidth() * gridworld_->getHeight()) {
            break;
        }
    }
}

std::vector<Point> DStarGridWorldPlanner::extractPath(const Point& current) const {
    std::vector<Point> path;
    Point current_pos = current;
    
    // Follow decreasing g values toward goal
    int max_steps = gridworld_->getWidth() * gridworld_->getHeight();
    int steps = 0;
    
    while (!(current_pos.getX() == sgoal_.getX() && current_pos.getY() == sgoal_.getY()) && steps < max_steps) {
        path.push_back(current_pos);
        
        // Find neighbor with minimum g value
        double min_g = std::numeric_limits<double>::infinity();
        Point next = current_pos;
        
        auto successors = getSuccessors(current_pos);
        for (const auto& succ : successors) {
            double cost = movementCost(current_pos, succ) + getG(succ);
            if (cost < min_g) {
                min_g = cost;
                next = succ;
            }
        }
        
        // If no improvement possible, path is blocked
        if (next.getX() == current_pos.getX() && next.getY() == current_pos.getY()) {
            break;
        }
        
        current_pos = next;
        steps++;
    }
    
    // Add goal if reached
    if (current_pos.getX() == sgoal_.getX() && current_pos.getY() == sgoal_.getY()) {
        path.push_back(sgoal_);
    }
    
    return path;
}

void DStarGridWorldPlanner::reset() {
    g_.clear();
    rhs_.clear();
    
    // Clear priority queue by creating new one
    open_ = std::priority_queue<DStarEntry,
                               std::vector<DStarEntry>,
                               std::greater<DStarEntry>>();
    
    expanded_count_ = 0;
    update_count_ = 0;
    has_valid_path_ = false;
}
