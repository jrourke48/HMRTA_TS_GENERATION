#ifndef POINT_H
#define POINT_H

#include <cstdint>
#include <cmath>

/**
 * Point - Represents a 2D coordinate point
 */
class Point {
private:
    uint32_t x;
    uint32_t y;
    
public:
    // Constructors
    Point();
    Point(uint32_t x, uint32_t y);
    
    // Destructor
    ~Point();
    
    // Getters
    uint32_t getX() const { return x; }
    uint32_t getY() const { return y; }
    
    // Setters
    void setX(uint32_t xVal) { x = xVal; }
    void setY(uint32_t yVal) { y = yVal; }
    void setPoint(uint32_t xVal, uint32_t yVal) { x = xVal; y = yVal; }
    
    // Operators
    bool operator==(const Point& other) const;
    bool operator!=(const Point& other) const;
    Point operator+(const Point& other) const;
    Point operator-(const Point& other) const;
    
    // Utility methods
    double distance(const Point& other) const;
    double distance() const;  // Distance from origin (0,0)
    bool isOrigin() const { return x == 0 && y == 0; }
};

#endif // POINT_H
