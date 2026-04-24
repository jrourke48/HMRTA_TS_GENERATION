#ifndef GRIDWORLD_H
#define GRIDWORLD_H

#include <cstdint>
#include <vector>
#include <string>

/**
 * GridWorld - Represents a 2D grid with costmap
 * Used as the physical environment for the robots
 */
class GridWorld {
private:
    uint32_t width;
    uint32_t height;
    std::vector<std::vector<uint8_t>> costmap;  // 2D costmap: 0 = free, 1-254 = cost, 255 = obstacle
    std::string gridName;
    
public:
    // Constructor
    GridWorld(uint32_t w, uint32_t h, const std::string& name = "GridWorld");
    
    // Destructor
    ~GridWorld();
    
    // Getters
    uint32_t getWidth() const { return width; }
    uint32_t getHeight() const { return height; }
    std::string getGridName() const { return gridName; }
    uint8_t getCost(uint32_t x, uint32_t y) const;
    const std::vector<std::vector<uint8_t>>& getCostmap() const { return costmap; }
    
    // Setters
    void setCost(uint32_t x, uint32_t y, uint8_t cost);
    void setGridName(const std::string& name) { gridName = name; }
    
    // Grid queries
    bool isObstacle(uint32_t x, uint32_t y) const;
    bool isFree(uint32_t x, uint32_t y) const;
    bool isInBounds(uint32_t x, uint32_t y) const;
    
    // Grid operations
    void setObstacle(uint32_t x, uint32_t y) { setCost(x, y, 255); }
    void clearCell(uint32_t x, uint32_t y) { setCost(x, y, 0); }
    void clearAll();
    void fillObstacles();
};

#endif // GRIDWORLD_H
