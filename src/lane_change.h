/******************************************************************************
| lane_keep.h                                                                 |
| ...                                                                         |
******************************************************************************/

#ifndef LANE_CHANGE_H
#define LANE_CHANGE_H

#include "behavior.h"
#include "trajectory.h"
#include <cmath>

class LaneChange: public Behavior {
  public:
    LaneChange();
    ~LaneChange();

    // Override the Behavior Type Default
    BehaviorSet get_next_behaviors(const double s, const double d,
                                   const Road &r, const int reference_lane) const;

    int add_trajectories(TrajectorySet &t_set,
                         double si, double si_dot, double si_dot_dot,
                         double di, double di_dot, double di_dot_dot,
                         const int &reference_lane, const Road &r,
                         ObstacleTracker &o) const;

    std::string name() const;

  private:

    // Cost function for a Lane Change
    double cost(const Trajectory &traj, const double &target_s_dot,
                const double &target_d, const double &follow_sf) const;

    // Distance away from target reference line at which point we can call
    // the lane change done
    double lc_diff_d;

    // Constants for time gap and distance buffer for a car
    // we would merge behind
    double distance_buffer; // D_0
    double time_gap;        // Tau

    // Cost calculation variables
    double dt;  // time step size for integral costs
    double k_j; // Coeff for jerk cost
    double k_a; // Coeff for jerk cost
    double k_t; // Coeff for time cost
    double k_s; // Coeff for lat movement cost
    double k_d; // Coeff for lon movement cost

    double k_safety; // Coeff for safety dist

    double k_lat; // weight of lateral costs (d)
    double k_lon; // weight of longitudinal costs (s)
};

#endif // __LANE_CHANGE_H__