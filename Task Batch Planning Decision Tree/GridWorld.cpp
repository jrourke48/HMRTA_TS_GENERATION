#include "GridWorld.h"

/**
 * GridWorld - Constructor
 */
GridWorld::GridWorld(uint32_t w, uint32_t h, const std::string& name)
    : width(w), height(h), gridName(name) {
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
uint8_t GridWorld::getCost(uint32_t x, uint32_t y) const {
    if (!isInBounds(x, y)) {
        return 255;  // Out of bounds = obstacle
    }
    return costmap[y][x];
}

/**
 * setCost - Set the cost at a specific grid cell
 */
void GridWorld::setCost(uint32_t x, uint32_t y, uint8_t cost) {
    if (!isInBounds(x, y)) {
        return;
    }
    costmap[y][x] = cost;
}

/**
 * isObstacle - Check if a cell is an obstacle (cost = 255)
 */
bool GridWorld::isObstacle(uint32_t x, uint32_t y) const {
    return getCost(x, y) == 255;
}

/**
 * isFree - Check if a cell is free (cost = 0)
 */
bool GridWorld::isFree(uint32_t x, uint32_t y) const {
    return getCost(x, y) == 0;
}

/**
 * isInBounds - Check if coordinates are within grid bounds
 */
bool GridWorld::isInBounds(uint32_t x, uint32_t y) const {
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
