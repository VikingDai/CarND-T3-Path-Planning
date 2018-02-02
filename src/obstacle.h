#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <cmath>
#include <map>
#include <list>

class Obstacle {
  public:

    // Obstacle ID
    int id;

    // Frenet Coordinates of the Obstacle
    double s;
    double d;

    // Frenet Velocity of the Obstacle
    double s_dot;
    double d_dot;

    // Frenet Acceleration of the Obstacle
    double s_dot_dot;
    double d_dot_dot;

    // Overall speed
    double speed;

    // The timestamp atfor which this obstacle reading was gathered
    long t;

    // Default Constructor
    Obstacle();

    // Non-Default Constuctors
    Obstacle(const int &id, const long &ts, const double &s, const double &d, const double &speed);

    // Default Destructor
    ~Obstacle();
};

#endif // OBSTACLE_H