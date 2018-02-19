#include "map.h"
#include "spline.h"

#include <iostream>

using namespace std;

// Interpolate spare map waypoints so we can have a better getXY function output
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
// NOTE: I also use two points from the last interpolated window into the next
// to help smooth out transitions
static void interpolate_map_waypoints(std::vector<double> &maps_s, std::vector<double> &maps_x, std::vector<double> &maps_y, const double &max_s)
{
  // Final map objects and vars
  vector<double> interp_map_x;
  vector<double> interp_map_y;
  vector<double> interp_map_s;

  // Describe how to interpolate
  double interp_points = 650;
  double map_interp_spacing = 0.0;

  // To save points from the last interpolation to help with smoothing
  double save_s_1 = -1, save_x_1, save_y_1;
  double save_s_2 = -1, save_x_2, save_y_2;

  // look at a set of waypoints at a time and use a spline to interpolate between them
  // Go around the whole map and add points to the maps_x, y, s
  for(int i = 0; i < maps_s.size(); ++i)
  {
    // Get waypoint indices
    int b = i - 1, a = (i + 1) % maps_s.size(), aa = (i + 2) % maps_s.size();
    if(b < 0) b = maps_s.size() - 1;

    // Take one waypoint before and two after
    double x_b  = maps_x[b];
    double x    = maps_x[i];
    double x_a  = maps_x[a];
    double x_aa = maps_x[aa];

    double y_b  = maps_y[b];
    double y    = maps_y[i];
    double y_a  = maps_y[a];
    double y_aa = maps_y[aa];

    double s_b  = maps_s[b] - (b > i ? max_s : 0);
    double s    = maps_s[i];
    double s_a  = maps_s[a] + (a < i ? max_s : 0);
    double s_aa = maps_s[aa] + (aa < i ? max_s : 0);

    // Build a spline with the way points above
    tk::spline x_given_s;
    tk::spline y_given_s;
    if(save_s_2 != -1)
    {
      x_given_s.set_points({s_b, save_s_2, save_s_1, s, s_a, s_aa}, {x_b, save_x_2, save_x_1, x, x_a, x_aa});
      y_given_s.set_points({s_b, save_s_2, save_s_1, s, s_a, s_aa}, {y_b, save_y_2, save_y_1, y, y_a, y_aa});
    }
    else
    {
      x_given_s.set_points({s_b, s, s_a, s_aa}, {x_b, x, x_a, x_aa});
      y_given_s.set_points({s_b, s, s_a, s_aa}, {y_b, y, y_a, y_aa});
    }

    // Calculate the interp spacing value based on distance between waypoints
    map_interp_spacing = abs(s_a - s) / interp_points;

    // Interpolate in between s and s_a
    for(double _s = s; _s < s_a; _s += map_interp_spacing)
    {
      double _x = x_given_s(_s);
      double _y = y_given_s(_s);

      interp_map_x.push_back(_x);
      interp_map_y.push_back(_y);
      interp_map_s.push_back(_s);
    }

    // Grab a couple of points for reuse during next interpolate
    int c = interp_map_s.size();
    save_s_1 = interp_map_s[c - interp_points / 30];
    save_x_1 = interp_map_x[c - interp_points / 30];
    save_y_1 = interp_map_y[c - interp_points / 30];
    save_s_2 = interp_map_s[c - interp_points / 4];
    save_x_2 = interp_map_x[c - interp_points / 4];
    save_y_2 = interp_map_y[c - interp_points / 4];
  }

  maps_x = interp_map_x;
  maps_y = interp_map_y;
  maps_s = interp_map_s;
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