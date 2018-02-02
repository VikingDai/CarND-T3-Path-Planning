#include "obstacle.h"

// Default Constructor
Obstacle::Obstacle(){}

// Non-Default Constuctors
Obstacle::Obstacle(const int &id, const long &ts, const double &s, const double &d, const double &speed)
{
  this->id = id;
  this->s = s;
  this->d = d;
  this->speed = speed;
  this->t = ts;
  this->s_dot = 0.0;
  this->d_dot = 0.0;
  this->s_dot_dot = 0.0;
  this->d_dot_dot = 0.0;
}

// Default Destructor
Obstacle::~Obstacle(){}