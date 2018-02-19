/******************************************************************************
| lane_keep.h                                                                 |
| ...                                                                         |
******************************************************************************/

#ifndef LANE_KEEP_H
#define LANE_KEEP_H

#include "behavior.h"
#include "trajectory.h"
#include <cmath>

class LaneKeep: public Behavior {
  public:
    LaneKeep();
    ~LaneKeep();

    int add_trajectories(TrajectorySet &t_set,
                          double si, double si_dot, double si_dot_dot,
                          double di, double di_dot, double di_dot_dot,
                          const int &current_lane, const int &reference_lane,
                          const Road &r, ObstacleTracker &o) const;

    double cost(const Trajectory &traj, const double &target_speed, const double &speed_limit, const double &target_d, const double &follow_ds) const;

    std::string name() const;

  private:

    // Cost calculation variables
    double dt;  // time step size for integral costs
    double k_j; // Coeff for jerk cost
    double k_t; // Coeff for time cost
    double k_s; // Coeff for lat movement cost
    double k_d; // Coeff for lon movement cost

    double k_lat; // weight of lat costs
    double k_lon; // weight of lon costs
};

#endif // __LANE_KEEP_H__