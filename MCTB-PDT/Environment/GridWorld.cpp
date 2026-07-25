#include "GridWorld.h"

/**
 * GridWorld - Constructor
 */
GridWorld::GridWorld(uint16_t w, uint16_t h): width(w), height(h) {
    // Initialize costmap with all free cells (0)
    costmap.resize(height, std::vector<uint8_t>(width, 0));
}

/**
 * GridWorld - Destructor
 */
GridWorld::~GridWorld() {
}

/**
 * getCost - Get the cost at a specific grid cell
 */
uint8_t GridWorld::getCost(const Point& point) const {
    uint32_t x = point.getX();
    uint32_t y = point.getY();
    if (!isInBounds(point)) {
        return 255;  // Out of bounds = obstacle
    }
    return costmap[y][x];
}

/**
 * setCost - Set the cost at a specific grid cell
 */
void GridWorld::setCost(const Point& point, uint8_t cost) {
    uint32_t x = point.getX();
    uint32_t y = point.getY();
    if (!isInBounds(point)) {
        return;
    }
    costmap[y][x] = cost;
}

/**
 * isObstacle - Check if a cell is an obstacle (cost = 255)
 */
bool GridWorld::isObstacle(const Point& point) const {
    return getCost(point) == 255;
}

/**
 * isFree - Check if a cell is free (cost = 0)
 */
bool GridWorld::isFree(const Point& point) const {
    return getCost(point) == 0;
}

/**
 * isInBounds - Check if coordinates are within grid bounds
 */
bool GridWorld::isInBounds(const Point& point) const {
    uint32_t x = point.getX();
    uint32_t y = point.getY();
    return x < width && y < height;
}

/**
 * clearAll - Clear all cells (set to 0)
 */
void GridWorld::clearAll() {
    for (auto& row : costmap) {
        std::fill(row.begin(), row.end(), 0);
    }
}

/**
 * fillObstacles - Fill all cells with obstacles (set to 255)
 */
void GridWorld::fillObstacles() {
    for (auto& row : costmap) {
        std::fill(row.begin(), row.end(), 255);
    }
}

