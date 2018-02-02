#ifndef WAYMAP_H
#define WAYMAP_H

#include <vector>
#include <fstream>
#include <sstream>

class Map {
  public:

    // Map way points
    std::vector<double> waypoints_x;
    std::vector<double> waypoints_y;
    std::vector<double> waypoints_s;
    std::vector<double> waypoints_dx;
    std::vector<double> waypoints_dy;

    // Largest S value
    double max_s;

    void load_from_points(const std::vector<double> &waypoints_x,
         const std::vector<double> &waypoints_y,const std::vector<double> &waypoints_s,
         const std::vector<double> &waypoints_dx,const std::vector<double> &waypoints_dy);

    Map();
    Map(const std::string map_file_, double max_s);
    ~Map();
};

#endif // __WAYMAP_H__