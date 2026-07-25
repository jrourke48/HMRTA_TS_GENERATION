#include "Point.h"

/**
 * Point - Default Constructor
 */
Point::Point() : x(0), y(0) {
}

/**
 * Point - Constructor with coordinates
 */
Point::Point(uint16_t xVal, uint16_t yVal) : x(xVal), y(yVal) {
}

/**
 * Point - Destructor
 */
Point::~Point() {
}

/**
 * operator== - Check if two points are equal
 */
bool Point::operator==(const Point& other) const {
    return x == other.x && y == other.y;
}

/**
 * operator!= - Check if two points are not equal
 */
bool Point::operator!=(const Point& other) const {
    return !(*this == other);
}

/**
 * operator+ - Add two points
 */
Point Point::operator+(const Point& other) const {
    return Point(x + other.x, y + other.y);
}

/**
 * operator- - Subtract two points
 */
Point Point::operator-(const Point& other) const {
    // Handle unsigned underflow by clamping to 0
    uint32_t newX = (x >= other.x) ? (x - other.x) : 0;
    uint32_t newY = (y >= other.y) ? (y - other.y) : 0;
    return Point(newX, newY);
}

/**
 * distance - Calculate Euclidean distance to another point
 */
double Point::distance(const Point& other) const {
    double dx = static_cast<double>(x) - static_cast<double>(other.x);
    double dy = static_cast<double>(y) - static_cast<double>(other.y);
    return std::sqrt(dx * dx + dy * dy);
}

/**
 * distance - Calculate Euclidean distance from origin (0,0)
 */
double Point::distance() const {
    double dx = static_cast<double>(x);
    double dy = static_cast<double>(y);
    return std::sqrt(dx * dx + dy * dy);
}
