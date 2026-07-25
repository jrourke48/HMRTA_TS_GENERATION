#ifndef POINT_H
#define POINT_H

#include <cstdint>
#include <cmath>

/**
 * Point - Represents a 2D coordinate point
 */
class Point {
private:
    uint16_t x;
    uint16_t y;
    
public:
    // Constructors
    Point();
    Point(uint16_t x, uint16_t y);
    
    // Destructor
    ~Point();
    
    // Getters
    uint16_t getX() const { return x; }
    uint16_t getY() const { return y; }
    
    // Setters
    void setX(uint16_t xVal) { x = xVal; }
    void setY(uint16_t yVal) { y = yVal; }
    void setPoint(uint16_t xVal, uint16_t yVal) { x = xVal; y = yVal; }
    
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
