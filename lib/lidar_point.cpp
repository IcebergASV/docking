#include <iostream>
#include <sstream>
#include <vector>
#include "lidar_point.h"

// Default constructor
lidarPoint::lidarPoint() : distance_(0.0), angle_(0.0) {}

// Constructor with input parameters
lidarPoint::lidarPoint(double distance, double angle) : distance_(distance), angle_(angle) {}

// Getter for distance
double lidarPoint::getDistance() const {
    return distance_;
}

// Getter for angle
double lidarPoint::getAngle() const {
    return angle_;
}

// Setter for distance
void lidarPoint::setDistance(double distance) {
    distance_ = distance;
}

// Setter for angle
void lidarPoint::setAngle(double angle) {
    angle_ = angle;
}

// Overload << operator for lidarPoint objects
std::ostream& operator<<(std::ostream& os, const lidarPoint& point) {
    os << "(" << point.getDistance() << ", " << point.getAngle() << ")";
    return os;
}