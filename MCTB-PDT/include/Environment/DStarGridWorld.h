#ifndef DSTAR_GRIDWORLD_H
#define DSTAR_GRIDWORLD_H

#include <queue>
#include <unordered_map>
#include <vector>
#include <cmath>
#include <limits>
#include "Environment/Point.h"

class GridWorld;

/**
 * D* Lite Planner for GridWorld
 * Finds optimal paths in 2D grids with dynamic obstacle updates
 * 
 * Key features:
 * - Backward A* search from goal to start
 * - Incremental replanning when obstacles change
 * - Efficient for unknown/changing environments
 */
class DStarGridWorldPlanner {
public:
    // ==================== Public Interface ====================
    
    /**
     * Constructor
     * @param gridworld Reference to the GridWorld to plan in
     */
    DStarGridWorldPlanner(GridWorld* gridworld);
    ~DStarGridWorldPlanner();
    
    /**
     * Compute initial path from start to goal
     * @param start Starting position
     * @param goal Goal position
     * @return Vector of Points representing the path (empty if no path exists)
     */
    std::vector<Point> plan(const Point& start, const Point& goal);
    
    /**
     * Replan when obstacles change
     * @param current Robot's current position
     * @param changed_cells Cells that changed since last plan
     * @return Updated path from current position to goal (empty if no path exists)
     */
    std::vector<Point> replan(const Point& current, const std::vector<Point>& changed_cells);
    
    /**
     * Get cost to reach goal from any position
     * @param pos Position to query
     * @return Cost (distance), or infinity if no path exists
     */
    double getCost(const Point& pos) const;
    
    // ==================== Diagnostics ====================
    
    int getNumExpanded() const { return expanded_count_; }
    int getNumUpdated() const { return update_count_; }
    bool hasValidPath() const { return has_valid_path_; }
    
private:
    // ==================== Internal Types ====================
    
    struct DStarKey {
        double k1, k2;  // Ordered key pair for priority queue
        
        bool operator<(const DStarKey& other) const {
            if (k1 != other.k1) return k1 < other.k1;
            return k2 < other.k2;
        }
        
        bool operator>(const DStarKey& other) const {
            return other < *this;
        }
        
        bool operator==(const DStarKey& other) const {
            return k1 == other.k1 && k2 == other.k2;
        }
    };
    
    // Hash for unordered_map with Point keys
    struct PointHash {
        size_t operator()(const Point& p) const {
            return std::hash<uint32_t>()(((uint32_t)p.getX() << 16) | (uint32_t)p.getY());
        }
    };
    
    struct PointEqual {
        bool operator()(const Point& a, const Point& b) const {
            return a.getX() == b.getX() && a.getY() == b.getY();
        }
    };
    
    // ==================== State Maps ====================
    
    // g(s) = best estimate of cost-to-goal
    std::unordered_map<Point, double, PointHash, PointEqual> g_;
    
    // rhs(s) = one-step lookahead from successors
    std::unordered_map<Point, double, PointHash, PointEqual> rhs_;
    
    // Open list: priority queue of (key, point) pairs
    using DStarEntry = std::pair<DStarKey, Point>;
    std::priority_queue<DStarEntry,
                       std::vector<DStarEntry>,
                       std::greater<DStarEntry>> open_;
    
    // ==================== D* Lite State ====================
    
    GridWorld* gridworld_;
    Point sstart_;          // Start position
    Point sgoal_;           // Goal position
    Point scurrent_;        // Current robot position (for replan)
    double km_;             // Key modifier for incremental search
    bool has_valid_path_;
    
    // Diagnostics
    int expanded_count_;
    int update_count_;
    
    // ==================== Core D* Lite Functions ====================
    
    /**
     * Calculate D* key for state s
     */
    DStarKey calculateKey(const Point& s) const;
    
    /**
     * Get g value (cost estimate) for state
     */
    double getG(const Point& s) const;
    
    /**
     * Get rhs value (one-step lookahead) for state
     */
    double getRhs(const Point& s) const;
    
    /**
     * Heuristic: straight-line distance to goal
     */
    double heuristic(const Point& s) const;
    
    /**
     * Movement cost between adjacent cells
     * Respects obstacles and terrain costs in GridWorld
     */
    double movementCost(const Point& from, const Point& to) const;
    
    /**
     * Get valid successor states (8-directional neighbors)
     */
    std::vector<Point> getSuccessors(const Point& s) const;
    
    /**
     * Get valid predecessor states
     */
    std::vector<Point> getPredecessors(const Point& s) const;
    
    /**
     * Update state s and add/remove from open list as needed
     */
    void updateVertex(const Point& s);
    
    /**
     * Main D* Lite search loop
     */
    void computeShortestPath();
    
    /**
     * Extract final path by following decreasing g values to start
     */
    std::vector<Point> extractPath(const Point& current) const;
    
    /**
     * Clear all planning data
     */
    void reset();
};

#endif // DSTAR_GRIDWORLD_H
