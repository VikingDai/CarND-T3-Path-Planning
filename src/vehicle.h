#ifndef VEHICLE_H
#define VEHICLE_H

#include <cmath>

class Vehicle {
  public:

    // Vehicle Object attributes
    // This could also include stuff like this physical limits of the vehicle
    double length;
    double width;

    // Frenet Coordinates of the Vehicle
    double s;
    double d;

    // Cartesian Coordinates of the Vehicle
    double x;
    double y;

    // Turn Angle
    double yaw;

    // Velocity of the vehicle
    double vx;
    double vy;

    // Overall Speed of the vehicle - Simulator reports MPH!
    double speed;

    // Default Constructor
    Vehicle();

    // Non-Default Constuctors
    Vehicle(const double &len, const double &width,
            const double &x, const double &y,
            const double &s, const double &d,
            const double &speed, const double &yaw);

    // Default Destructor
    ~Vehicle();
};

#endif // __VEHICLE_H__