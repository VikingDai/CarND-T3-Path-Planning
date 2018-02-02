#include "road.h"

#include <iostream>

// Default Constructor
Road::Road()
{
  this->lanes = std::vector<Lane>();
  this->lane_count = 0;
  this->width = 0;
}

// Non-Default Constuctor
Road::Road(const std::vector<Lane> lanes, const double speed_limit)
{
  this->speed_limit = speed_limit;
  set_lanes(lanes);
}

// Default Destructor
Road::~Road(){}

// Set Lane Count
void Road::set_lanes(const std::vector<Lane> lanes)
{
  this->lanes = lanes;
  this->lane_count = lanes.size();
  this->width = 0;
  for(int i = 0; i < lane_count; ++i)
    this->width += lanes[i].width;
}

// Get the d value of the middle of a lane
double Road::get_lane_mid_frenet(const int lane_num) const
{
  if(lane_num < 0 || lane_num >= lane_count) return -1; // TO-DO: Throw error?

  double lane_d = lanes[lane_num].width / 2.0;
  for(int i = lane_num - 1; i >= 0; --i) lane_d += lanes[i].width;
  return lane_d;
}

// Get lane of vehicle
int Road::get_vehicle_lane(const Vehicle &v) const
{
  double j = 0;
  for(int i = 0; i < lane_count; ++i)
  {
    j += lanes[i].width;
    if(v.d <= j) return i;
  }
  return -1; // Error case -> TO-DO: Throw an error?
}

// Get lane of vehicle
int Road::get_vehicle_lane(const Obstacle &v) const
{
  double j = 0;
  for(int i = 0; i < lane_count; ++i)
  {
    j += lanes[i].width;
    if(v.d <= j) return i;
  }
  return -1; // Error case -> TO-DO: Throw an error?
}