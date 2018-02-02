#include "vehicle.h"

// Default Constructor
Vehicle::Vehicle(){x = y = s = d = vx = vy = speed = yaw = 0;}

// Non-Default Constuctor
Vehicle::Vehicle(const double &len, const double &width,
            const double &x, const double &y,
            const double &s, const double &d,
            const double &speed, const double &yaw)
{
  this->length = len;
  this->width = width;

  this->x = x;
  this->y = y;
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->yaw = yaw;
  this->vx = speed*cos(yaw);
  this->vy = speed*sin(yaw);
}

// Default Destructor
Vehicle::~Vehicle(){}