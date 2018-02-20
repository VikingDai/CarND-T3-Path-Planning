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

// Typedef for a set of integers representing possible behavior id numbers
typedef std::vector<int> BehaviorSet;

class Behavior
{
  public:
    virtual ~Behavior() {}

    // ID/State number, for FSM models
    int id = -1;

    // Is this the current state?
    bool current = false;

    // Set of possible behaviors that follow this
    BehaviorSet next_behaviors = BehaviorSet();

    // Next Behavior interface so you can add logic to the next_behaviors
    // set based on the current state of the vehicle
    // NOTE: This interface may need to get more complicated in the future
    virtual BehaviorSet get_next_behaviors(const double s, const double d,
                                           const Road &r, const int reference_lane) const
    {
      return next_behaviors; // Defaults to whatever it was set to
    }

    // Output the "name" of thie behavior for debug purposes
    virtual std::string name() const {return "Driving Behavior";}

    // Given:
    // ------
    // a start state, road object, and surrounding vehicle status,
    // add all the possible trajectories for this behavior type AND their
    // costs to the incoming TrajectorySet reference.
    //
    // Returns:
    // --------
    // the number of trajectories added
    //
    // NOTE: See below for how you might do a cost function.
    virtual int add_trajectories(TrajectorySet &t_set,
                                 double si, double si_dot, double si_dot_dot,
                                 double di, double di_dot, double di_dot_dot,
                                 const int &reference_lane, const Road &r,
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
