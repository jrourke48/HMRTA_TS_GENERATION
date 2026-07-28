#include "../include/Environment/DStarGridWorld.h"
#include "../include/Environment/GridWorld.h"
#include <iostream>
#include <cassert>
#include <iomanip>

/**
 * Test Suite for D* Lite GridWorld Planner
 * Tests pathfinding, obstacle avoidance, replanning, and edge cases
 */

class DStarTestSuite {
private:
    int tests_passed_ = 0;
    int tests_failed_ = 0;
    
    void printTestHeader(const std::string& test_name) {
        std::cout << "\n" << std::string(70, '=') << std::endl;
        std::cout << "TEST: " << test_name << std::endl;
        std::cout << std::string(70, '=') << std::endl;
    }
    
    void passTest(const std::string& msg) {
        std::cout << "  ✓ PASS: " << msg << std::endl;
        tests_passed_++;
    }
    
    void failTest(const std::string& msg) {
        std::cout << "  ✗ FAIL: " << msg << std::endl;
        tests_failed_++;
    }
    
    void printPath(const std::vector<Point>& path) {
        std::cout << "  Path: ";
        for (size_t i = 0; i < path.size(); ++i) {
            std::cout << "(" << path[i].getX() << "," << path[i].getY() << ")";
            if (i < path.size() - 1) std::cout << " -> ";
        }
        std::cout << std::endl;
        std::cout << "  Length: " << path.size() << " waypoints" << std::endl;
    }
    
public:
    
    // ==================== TEST 1: Simple Open Path ====================
    void testSimpleOpenPath() {
        printTestHeader("Simple Open Path in Empty Grid");
        
        GridWorld grid(20, 20);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(0, 0);
        Point goal(10, 10);
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should not be empty");
            return;
        }
        
        if (path.front().getX() != start.getX() || path.front().getY() != start.getY()) {
            failTest("Path should start at start position");
            return;
        }
        
        if (path.back().getX() != goal.getX() || path.back().getY() != goal.getY()) {
            failTest("Path should end at goal position");
            return;
        }
        
        // Check path length is roughly diagonal distance (approximately 14-15 waypoints)
        double expected_distance = std::sqrt(10*10 + 10*10);  // ~14.14
        if (path.size() < expected_distance - 2) {
            failTest("Path seems too short");
            return;
        }
        
        printPath(path);
        std::cout << "  Expected distance: ~" << expected_distance << std::endl;
        std::cout << "  Planner expanded " << planner.getNumExpanded() << " states" << std::endl;
        passTest("Path found from (0,0) to (10,10)");
    }
    
    // ==================== TEST 2: Straight Line Path ====================
    void testStraightLinePath() {
        printTestHeader("Straight Horizontal Path");
        
        GridWorld grid(30, 10);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(0, 5);
        Point goal(29, 5);
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should exist on clear horizontal line");
            return;
        }
        
        // All waypoints should have same Y coordinate
        for (const auto& p : path) {
            if (p.getY() != 5) {
                failTest("Path should stay on Y=5");
                return;
            }
        }
        
        printPath(path);
        passTest("Horizontal path maintained constant Y coordinate");
    }
    
    // ==================== TEST 3: Obstacle Avoidance ====================
    void testObstacleAvoidance() {
        printTestHeader("Path Around Wall Obstacle");
        
        GridWorld grid(20, 20);
        DStarGridWorldPlanner planner(&grid);
        
        // Create vertical wall from (10, 3) to (10, 16)
        for (int y = 3; y <= 16; ++y) {
            grid.setObstacle(Point(10, y));
        }
        
        Point start(5, 10);
        Point goal(15, 10);
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should exist around obstacle");
            return;
        }
        
        // Check no waypoint is on obstacle
        for (const auto& p : path) {
            if (grid.isObstacle(p)) {
                failTest("Path goes through obstacle!");
                return;
            }
        }
        
        printPath(path);
        std::cout << "  Obstacle: vertical wall at X=10, Y=[3..16]" << std::endl;
        passTest("Path successfully avoided obstacle");
    }
    
    // ==================== TEST 4: No Path Scenario ====================
    void testNoPathBlocked() {
        printTestHeader("No Path - Goal Blocked by Wall");
        
        GridWorld grid(20, 20);
        DStarGridWorldPlanner planner(&grid);
        
        // Create box around goal
        Point goal(15, 10);
        for (int x = 14; x <= 16; ++x) {
            for (int y = 9; y <= 11; ++y) {
                if (!(x == 15 && y == 10)) {  // Don't block goal itself
                    grid.setObstacle(Point(x, y));
                }
            }
        }
        
        Point start(5, 10);
        auto path = planner.plan(start, goal);
        
        if (!path.empty()) {
            failTest("Path should be empty when goal is blocked");
            return;
        }
        
        if (planner.hasValidPath()) {
            failTest("hasValidPath() should return false");
            return;
        }
        
        std::cout << "  Goal surrounded by obstacles" << std::endl;
        passTest("Correctly detected unreachable goal");
    }
    
    // ==================== TEST 5: Replanning ====================
    void testReplanning() {
        printTestHeader("Replanning After Obstacle Changes");
        
        GridWorld grid(30, 20);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(2, 10);
        Point goal(28, 10);
        
        // Initial plan - should be direct
        auto path1 = planner.plan(start, goal);
        
        if (path1.empty()) {
            failTest("Initial path should exist");
            return;
        }
        
        int initial_length = path1.size();
        printPath(path1);
        std::cout << "  Initial plan length: " << initial_length << std::endl;
        
        // Now block the path with a wall
        Point robot_pos(10, 10);  // Robot has moved to this position
        std::vector<Point> changed_cells;
        
        for (int y = 8; y <= 12; ++y) {
            grid.setObstacle(Point(10, y));
            changed_cells.push_back(Point(10, y));
        }
        
        std::cout << "  Added wall at X=10, Y=[8..12]" << std::endl;
        
        // Replan from current position
        auto path2 = planner.replan(robot_pos, changed_cells);
        
        if (path2.empty()) {
            failTest("Replanned path should exist (can go around)");
            return;
        }
        
        // Check new path avoids obstacle
        for (const auto& p : path2) {
            if (grid.isObstacle(p)) {
                failTest("Replanned path goes through obstacle!");
                return;
            }
        }
        
        printPath(path2);
        std::cout << "  Replanned length: " << path2.size() << std::endl;
        std::cout << "  Planner updated " << planner.getNumUpdated() << " states" << std::endl;
        passTest("Replanning successfully navigated around new obstacle");
    }
    
    // ==================== TEST 6: Cost Calculation ====================
    void testCostCalculation() {
        printTestHeader("Cost-to-Goal Calculation");
        
        GridWorld grid(15, 15);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(0, 0);
        Point goal(10, 10);
        
        planner.plan(start, goal);
        
        // Cost at start should be ~sqrt(10^2 + 10^2) = 14.14
        double cost_at_start = planner.getCost(start);
        double expected = std::sqrt(10*10 + 10*10);
        
        std::cout << "  Cost at start (0,0): " << std::fixed << std::setprecision(2) 
                  << cost_at_start << std::endl;
        std::cout << "  Expected: ~" << expected << std::endl;
        
        if (cost_at_start < expected - 1 || cost_at_start > expected + 1) {
            failTest("Cost seems incorrect");
            return;
        }
        
        // Cost at goal should be 0
        double cost_at_goal = planner.getCost(goal);
        std::cout << "  Cost at goal (10,10): " << cost_at_goal << std::endl;
        
        if (cost_at_goal != 0.0) {
            failTest("Cost at goal should be 0");
            return;
        }
        
        passTest("Costs calculated correctly");
    }
    
    // ==================== TEST 7: Diagonal Movement ====================
    void testDiagonalMovement() {
        printTestHeader("Diagonal Movement Handling");
        
        GridWorld grid(10, 10);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(0, 0);
        Point goal(5, 5);
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should exist");
            return;
        }
        
        // With diagonal movement, path should be roughly 5*sqrt(2) ≈ 7 waypoints
        int expected_waypoints = 6;  // Rough estimate
        
        std::cout << "  Diagonal path waypoints: " << path.size() << std::endl;
        std::cout << "  Expected: ~" << expected_waypoints << std::endl;
        
        // Path should use diagonals efficiently
        if (path.size() > 10) {
            failTest("Path seems inefficient for diagonal movement");
            return;
        }
        
        printPath(path);
        passTest("Diagonal movement used efficiently");
    }
    
    // ==================== TEST 8: Large Grid Performance ====================
    void testLargeGridPerformance() {
        printTestHeader("Performance on Large Grid (100x100)");
        
        GridWorld grid(100, 100);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(5, 5);
        Point goal(95, 95);
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should exist on large grid");
            return;
        }
        
        std::cout << "  Grid size: 100x100" << std::endl;
        std::cout << "  Start: (5,5), Goal: (95,95)" << std::endl;
        std::cout << "  Path found with " << path.size() << " waypoints" << std::endl;
        std::cout << "  States expanded: " << planner.getNumExpanded() << std::endl;
        
        double expected_distance = std::sqrt(90*90 + 90*90);
        std::cout << "  Expected distance: ~" << expected_distance << std::endl;
        
        if (path.size() < expected_distance - 5) {
            failTest("Path seems too short");
            return;
        }
        
        passTest("Large grid planning completed successfully");
    }
    
    // ==================== TEST 9: Adjacent Goal ====================
    void testAdjacentGoal() {
        printTestHeader("Goal Adjacent to Start");
        
        GridWorld grid(10, 10);
        DStarGridWorldPlanner planner(&grid);
        
        Point start(5, 5);
        Point goal(6, 5);  // Adjacent horizontally
        
        auto path = planner.plan(start, goal);
        
        if (path.empty()) {
            failTest("Path should exist to adjacent cell");
            return;
        }
        
        // Should have exactly 2 waypoints: start and goal
        if (path.size() != 2) {
            failTest("Path to adjacent cell should have 2 waypoints");
            return;
        }
        
        printPath(path);
        passTest("Adjacent goal handling correct");
    }
    
    // ==================== TEST 10: Same Start and Goal ====================
    void testSameStartGoal() {
        printTestHeader("Start and Goal at Same Position");
        
        GridWorld grid(10, 10);
        DStarGridWorldPlanner planner(&grid);
        
        Point pos(5, 5);
        
        auto path = planner.plan(pos, pos);
        
        // Should have at least the position itself
        if (path.empty()) {
            failTest("Path should include start position");
            return;
        }
        
        // Should be trivial path
        if (path.size() > 2) {
            failTest("Path is too long for start==goal");
            return;
        }
        
        std::cout << "  Start and goal: (5,5)" << std::endl;
        std::cout << "  Path length: " << path.size() << std::endl;
        passTest("Same start/goal handled correctly");
    }
    
    // ==================== Public Test Runner ====================
    void runAllTests() {
        std::cout << "\n\n";
        std::cout << "+" << std::string(68, '=') << "+" << std::endl;
        std::cout << "|" << std::string(15, ' ') << "D* LITE GRIDWORLD TEST SUITE" << std::string(25, ' ') << "|" << std::endl;
        std::cout << "+" << std::string(68, '=') << "+" << std::endl;
        
        testSimpleOpenPath();
        testStraightLinePath();
        testObstacleAvoidance();
        testNoPathBlocked();
        testReplanning();
        testCostCalculation();
        testDiagonalMovement();
        testLargeGridPerformance();
        testAdjacentGoal();
        testSameStartGoal();
        
        printSummary();
    }
    
    void printSummary() {
        std::cout << "\n\n";
        std::cout << "+" << std::string(68, '=') << "+" << std::endl;
        std::cout << "|" << std::string(68, ' ') << "|" << std::endl;
        std::cout << "|  TESTS PASSED: " << std::setw(3) << tests_passed_ 
                  << "  |  TESTS FAILED: " << std::setw(3) << tests_failed_ 
                  << std::string(16, ' ') << "|" << std::endl;
        std::cout << "|" << std::string(68, ' ') << "|" << std::endl;
        
        if (tests_failed_ == 0) {
            std::cout << "|" << std::string(15, ' ') << "✓ ALL TESTS PASSED!" 
                      << std::string(35, ' ') << "|" << std::endl;
        } else {
            std::cout << "|" << std::string(12, ' ') << "✗ SOME TESTS FAILED - Review above" 
                      << std::string(22, ' ') << "|" << std::endl;
        }
        
        std::cout << "|" << std::string(68, ' ') << "|" << std::endl;
        std::cout << "+" << std::string(68, '=') << "+" << std::endl;
    }
};

// ==================== MAIN ====================

int main() {
    try {
        DStarTestSuite test_suite;
        test_suite.runAllTests();
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
