#include "Environment/gridvis.h"
#include "Environment/DStarGridWorld.h"
#include <raylib.h>
#include <iostream>
#include <vector>
#include <cmath>

// Helper: Convert grid point to screen pixel coordinates
static void gridToScreen(
    Point gridPos,
    int cellSize,
    int margin,
    int& screenX,
    int& screenY
) {
    screenX = margin + gridPos.getX() * cellSize;
    screenY = margin + gridPos.getY() * cellSize;
}

// Helper: Compute a single D* path from start to goal
static std::vector<Point> compute_dstar_path_single(
    const Environment& env,
    const Point& startPos,
    const Point& goalPos
) {
    std::vector<Point> path;
    
    // Get the GridWorld from environment
    const GridWorld* gridWorld = env.getGridWorld();
    if (!gridWorld) {
        std::cerr << "compute_dstar_path_single: GridWorld is null" << std::endl;
        return path;
    }
    
    // Create D* Lite planner for this grid
    DStarGridWorldPlanner planner(const_cast<GridWorld*>(gridWorld));
    
    // Compute path from start to goal
    path = planner.plan(startPos, goalPos);
    
    return path;
}

/**
 * compute_dstar_paths - Compute D* paths for each robot through the task tree
 * 
 * For each Tree_Node in the path:
 *   - Extract goal position from TS state
 *   - For each allocated robot:
 *     - Run D* from robot's current position to goal
 *     - Append to robot's path
 */
RobotPathMap compute_dstar_paths(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath
) {
    RobotPathMap robotPaths;
    
    // Initialize empty paths for all robots
    for (uint32_t i = 0; i < mrs.getNumRobots(); ++i) {
        robotPaths[i] = std::vector<Point>();
    }
    
    // For each node in the optimal path
    for (const Tree_Node* node : optimalPath) {
        if (!node) continue;
        
        // Get which robots are allocated to this node
        const std::vector<bool>& allocation = node->getRoboTaskAllocation();
        const std::vector<Point>& robotPositions = node->getRobotPositions();
        
        // For each allocated robot
        for (uint32_t robotId = 0; robotId < allocation.size(); ++robotId) {
            if (!allocation[robotId]) continue;  // Robot not allocated to this node
            
            // Get robot's current position (or use provided position from node)
            Point currentPos;
            if (robotId < robotPositions.size()) {
                currentPos = robotPositions[robotId];
            } else {
                const std::vector<Point>& positions = mrs.getRobotPositions();
                if (robotId < positions.size()) {
                    currentPos = positions[robotId];
                } else {
                    std::cerr << "compute_dstar_paths: Robot " << robotId << " position not found" << std::endl;
                    continue;
                }
            }
            
            //goal position is the ts center
            Point goalPos = env.TSStateIdToGridCenter(node->getTSState()->getId());
            
            // Compute D* path from current position to goal
            std::vector<Point> segmentPath = compute_dstar_path_single(env, currentPos, goalPos);
            
            // Append path, avoiding duplicate endpoints
            for (size_t i = 0; i < segmentPath.size(); ++i) {
                if (i == 0 && !robotPaths[robotId].empty()) {
                    continue;  // Skip first point if it's a duplicate of last point
                }
                robotPaths[robotId].push_back(segmentPath[i]);
            }
        }
    }
    
    return robotPaths;
}

/**
 * visualize_environment - Renders grid, TS regions, obstacles, robots, and D* paths
 */
void visualize_environment(
    const Environment& env,
    const MultiRobotSystem& mrs,
    const std::vector<Tree_Node*>& optimalPath,
    const RobotPathMap& robotPaths,
    const char* windowTitle
) {
    const GridWorld* gridWorld = env.getGridWorld();
    if (!gridWorld) {
        std::cerr << "visualize_environment: GridWorld is null" << std::endl;
        return;
    }

    const uint32_t gridW = gridWorld->getWidth();
    const uint32_t gridH = gridWorld->getHeight();

    const int cellSize = 60;
    const int margin = 40;
    const int screenW = margin * 2 + gridW * cellSize;
    const int screenH = margin * 2 + gridH * cellSize;

    InitWindow(screenW, screenH, windowTitle);
    SetTargetFPS(60);

    // Compute paths if not provided
    RobotPathMap paths = robotPaths;
    if (paths.empty() && !optimalPath.empty()) {
        std::cout << "Computing D* paths for robots..." << std::endl;
        paths = compute_dstar_paths(env, mrs, optimalPath);
    }

    // Color palette for TS states
    Color stateColors[] = {
        Color{100, 200, 100, 180},  // green
        Color{100, 150, 255, 180},  // blue
        Color{255, 200, 100, 180},  // orange
        Color{200, 100, 255, 180},  // purple
        Color{255, 100, 150, 180},  // pink
        Color{100, 255, 200, 180},  // cyan
        Color{200, 200, 100, 180},  // olive
        Color{150, 100, 200, 180},  // violet
    };
    const int numStateColors = sizeof(stateColors) / sizeof(stateColors[0]);

    // Color palette for robots (distinct and visible)
    Color robotColors[] = {
        Color{255, 0, 0, 255},      // red
        Color{0, 0, 255, 255},      // blue
        Color{0, 255, 0, 255},      // green
        Color{255, 255, 0, 255},    // yellow
        Color{255, 0, 255, 255},    // magenta
        Color{0, 255, 255, 255},    // cyan
        Color{255, 127, 0, 255},    // orange
        Color{127, 0, 255, 255},    // purple
    };
    const int numRobotColors = sizeof(robotColors) / sizeof(robotColors[0]);

    while (!WindowShouldClose()) {
        BeginDrawing();
        ClearBackground(RAYWHITE);

        // Draw grid cells with TS state coloring
        for (uint32_t y = 0; y < gridH; ++y) {
            for (uint32_t x = 0; x < gridW; ++x) {
                Point cellPos(x, y);
                int screenX, screenY;
                gridToScreen(cellPos, cellSize, margin, screenX, screenY);

                // Fill cell based on obstacle status and TS state
                if (env.isObstacle(cellPos)) {
                    DrawRectangle(screenX, screenY, cellSize, cellSize, DARKGRAY);
                } else {
                    // Color by TS state
                    uint32_t stateId = env.gridToTSStateId(cellPos);
                    Color stateCol = stateColors[stateId % numStateColors];
                    DrawRectangle(screenX, screenY, cellSize, cellSize, stateCol);
                }

                // Draw grid lines
                DrawRectangleLines(screenX, screenY, cellSize, cellSize, LIGHTGRAY);
            }
        }

        // Draw each robot's D* path
        for (const auto& [robotId, path] : paths) {
            Color pathColor = robotColors[robotId % numRobotColors];
            // Reduce opacity for path visualization
            pathColor.a = 100;
            
            for (const Point& p : path) {
                int screenX, screenY;
                gridToScreen(p, cellSize, margin, screenX, screenY);
                DrawRectangle(screenX, screenY, cellSize, cellSize, pathColor);
            }
        }

        // Draw robots at their current positions
        std::vector<Point> robotPositions = mrs.getRobotPositions();
        for (uint32_t i = 0; i < mrs.getNumRobots() && i < robotPositions.size(); ++i) {
            const Point& robotPos = robotPositions[i];
            int screenX, screenY;
            gridToScreen(robotPos, cellSize, margin, screenX, screenY);

            Color robotColor = robotColors[i % numRobotColors];
            int centerX = screenX + cellSize / 2;
            int centerY = screenY + cellSize / 2;
            int radius = cellSize / 3;

            DrawCircle(centerX, centerY, radius, robotColor);
            DrawCircleLines(centerX, centerY, radius + 2, BLACK);

            // Draw robot ID
            std::string robotLabel = "R" + std::to_string(i);
            DrawText(robotLabel.c_str(), centerX - 8, centerY - 8, 14, WHITE);
        }

        // Draw legend
        int legendY = 10;
        DrawText("Robot Paths (D*):", 10, legendY, 12, BLACK);
        legendY += 20;

        // Robot legend
        for (uint32_t i = 0; i < mrs.getNumRobots() && i < 6; ++i) {
            Color robotColor = robotColors[i % numRobotColors];
            DrawCircle(17, legendY + 8, 5, robotColor);
            std::string label = "Robot " + std::to_string(i);
            DrawText(label.c_str(), 30, legendY, 12, BLACK);
            legendY += 20;
        }

        EndDrawing();
    }

    CloseWindow();
}



