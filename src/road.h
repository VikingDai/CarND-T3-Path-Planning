#ifndef ROAD_H
#define ROAD_H

#include <vector>
#include "vehicle.h"
#include "obstacle.h"
#include "lane.h"

class Road {
  public:

    // Default Constructor
    Road();

    // Non-Default Constuctor
    Road(const std::vector<Lane> lanes, const double speed_limit);

    // Default Destructor
    ~Road();

    // Set Lanes
    void set_lanes(const std::vector<Lane> lanes);

    // Get Lane's d value
    double get_lane_mid_frenet(const int lane_num) const;

    // Get lane of vehicle
    int get_vehicle_lane(const Vehicle &v) const;
    int get_vehicle_lane(const Obstacle &v) const;

    // Speed Limit
    double speed_limit;

    // Width of road in total
    double width;

    // Lane Count
    int lane_count;

    // Lane Width
    std::vector<Lane> lanes;
};

#endif // __ROAD_H__