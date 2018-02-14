/******************************************************************************
| Behavior.h                                                                  |
| ...                                                                         |
******************************************************************************/

#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include "trajectory.h"
#include "road.h"
#include "obstacle_tracker.h"

#include <vector>
#include <cmath>
#include <string>

class Behavior
{
  public:
    virtual ~Behavior() {}

    // Output the "name" of thie behavior for debug purposes
    virtual std::string name() const {return "Driving Behavior";}

    // Given a start state, road object, and surrounding vehicle status,
    // add all the possible trajectories for this behavior type AND their
    // costs to the incoming TrajectorySet reference.
    // NOTE: See below for how you might do a cost function.
    virtual void add_trajectories(TrajectorySet &t_set,
                                  double si, double si_dot, double si_dot_dot,
                                  double di, double di_dot, double di_dot_dot,
                                  const int &current_lane, const Road &r,
                                  ObstacleTracker &o) const = 0;

    // Cost Function for the Behavior Type:
    // ------------------------------------
    // Behavior costs are typically a polynomial in the form of:
    //
    // C = k_j * J_t + k_t * g(x) + k_s * h(x)
    //
    // Where,
    //   k_* are all constants
    //   g(x) is a function to describe time cost
    //   h(x) is a funcction to describe state change cost
    //
    // But, I'll leave the cost implementation up to the freedom of the developer
    // and just hand over some necessary values
    //
    // You MUST leave each trajectory you add to the t_set with a cost
    // An example might be something that looks like:
    //
    // --> double cost(const Trajectory &traj, const double &target_speed, const double &target_d){return 0.0};
};

#endif // __BEHAVIOR_H__
