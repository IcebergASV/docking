#ifndef LIDARPOINT_H
#define LIDARPOINT_H

/**
* @brief Represents a single point from a lidar scan
* 
* Contains a distance and an angle
*/
class lidarPoint {
public:
    // Constructors
    lidarPoint();
    lidarPoint(double distance, double angle);    

    // Getters
    double getDistance() const;
    double getAngle() const;

    // Setters
    void setDistance(double distance);
    void setAngle(double angle);

private:
    double distance_;
    double angle_;
};

//Operator Overloads
std::ostream& operator<<(std::ostream& os, const lidarPoint& point);

#endif // LIDARPOINT_H