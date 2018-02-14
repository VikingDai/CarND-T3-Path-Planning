#include "map.h"
#include "spline.h"

#include <iostream>

using namespace std;

// Interpolate spare map waypoints so we can have a better getXY function output (:
// Uses a window of 4 waypoints and fits them to a spline to interpolate in
// between:
//                    s_0            s_1            s_2
// P(-1)           *   .              .              .
//                     .              .              .
// P( 0)         * ----+ Interpolates .              .
//                     | this window  .              .
// P( 2)     * --------+--------------+ Interpolates .
//                                    | this window  .
// P( 3)  * --------------------------+--------------+
//
// P( 4) *
static void interpolate_map_waypoints(std::vector<double> &maps_s, std::vector<double> &maps_x, std::vector<double> &maps_y, const double &max_s)
{
  // Final map objects and vars
  double map_interp_spacing = 0.05;
  vector<double> interp_map_x;
  vector<double> interp_map_y;
  vector<double> interp_map_s;

  // look at a set of waypoints at a time and use a spline to interpolate between them
  // Go around the whole map and add points to the maps_x, y, s
  for(int i = 0; i < maps_s.size(); ++i)
  {
    int b = i - 1, a = (i + 1) % maps_s.size(), aa = (i + 2) % maps_s.size();
    if(b < 0) b = maps_s.size() - 1;

    double x_b = maps_x[b];
    double x   = maps_x[i];
    double x_a = maps_x[a];
    double x_aa = maps_x[aa];

    double y_b = maps_y[b];
    double y   = maps_y[i];
    double y_a = maps_y[a];
    double y_aa = maps_y[aa];

    double s_b = maps_s[b] - (b > i ? max_s : 0);
    double s   = maps_s[i];
    double s_a = maps_s[a] + (a < i ? max_s : 0);
    double s_aa = maps_s[aa] + (aa < i ? max_s : 0);

    tk::spline x_given_s;
    tk::spline y_given_s;
    x_given_s.set_points({s_b, s, s_a, s_aa}, {x_b, x, x_a, x_aa});
    y_given_s.set_points({s_b, s, s_a, s_aa}, {y_b, y, y_a, y_aa});

    for(double _s = s; _s < s_a; _s += map_interp_spacing)
    {
      double _x = x_given_s(_s);
      double _y = y_given_s(_s);

      interp_map_x.push_back(_x);
      interp_map_y.push_back(_y);
      interp_map_s.push_back(_s);
    }
  }
  maps_x = interp_map_x;
  maps_y = interp_map_y;
  maps_s = interp_map_s;

  std::cout << "Completed Map Interpolation" << std::endl;
}

Map::Map()
{
  waypoints_x = std::vector<double>();
  waypoints_y = std::vector<double>();
  waypoints_s = std::vector<double>();
  waypoints_dx = std::vector<double>();
  waypoints_dy = std::vector<double>();
}

Map::Map(const std::string map_file_, double max_s = 6945.554) // "../data/highway_map.csv", 6945.554
{
  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line))
  {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  interpolate_map_waypoints(map_waypoints_s, map_waypoints_x, map_waypoints_y, max_s);
  load_from_points(map_waypoints_x, map_waypoints_y, map_waypoints_s, map_waypoints_dx, map_waypoints_dy);
  this->max_s = max_s;
}

void Map::load_from_points(const std::vector<double> &waypoints_x,
         const std::vector<double> &waypoints_y,const std::vector<double> &waypoints_s,
         const std::vector<double> &waypoints_dx,const std::vector<double> &waypoints_dy)
{
  this->waypoints_x = waypoints_x;
  this->waypoints_y = waypoints_y;
  this->waypoints_s = waypoints_s;
  this->waypoints_dx = waypoints_dx;
  this->waypoints_dy = waypoints_dy;
}

Map::~Map(){/* Doesn't do anything */}