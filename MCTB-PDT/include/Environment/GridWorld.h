#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include "Point.h"
#include <cstdint>
#include <vector>
#include <string>

/**
 * GridWorld - Represents a 2D grid with costmap
 * Used as the physical environment for the robots
 */
class GridWorld {
private:
    uint16_t width;
    uint16_t height;
    std::vector<std::vector<uint8_t>> costmap;  // 2D costmap: 0 = free, 1-254 = cost, 255 = obstacle
    
public:
    // Constructor
    GridWorld(uint16_t w, uint16_t h);
    
    // Destructor
    ~GridWorld();
    
    // Getters
    uint32_t getWidth() const { return width; }
    uint32_t getHeight() const { return height; }
    uint8_t getCost(const Point& point) const;
    const std::vector<std::vector<uint8_t>>& getCostmap() const { return costmap; }
    
    // Setters
    void setCost(const Point& point, uint8_t cost);

    // Grid queries
    bool isObstacle(const Point& point) const;
    bool isFree(const Point& point) const;
    bool isInBounds(const Point& point) const;
    
    // Grid operations
    void setObstacle(const Point& point) { setCost(point, 255); }
    void clearCell(const Point& point) { setCost(point, 0); }
    void clearAll();
    void fillObstacles();
    void 
};

#endif // GRIDWORLD_H
