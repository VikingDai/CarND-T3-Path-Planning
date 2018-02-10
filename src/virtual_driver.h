/******************************************************************************
| Virtual Driver.h                                                            |
| An abstraction of a Path Planner for driving a car in highway scenarios.    |
| This implementation is inspired by the "Optimal Trajectory Generation for   |
| Dynamic Street Scenarios in a Frenet Frame" (Werling, Ziegler, Kammel, and  |
| Thrun) paper. which tightly couples the traditional behavior planning and   |
| trajectory planning layers of a path planner to allow for a more reactive   |
| path plan in dynamic scenarios.                                             |
******************************************************************************/

#ifndef VIRTUAL_DRIVER_H
#define VIRTUAL_DRIVER_H

#include <cmath>
#include <vector>

#include "spline.h"

#include "vehicle.h"
#include "map.h"
#include "road.h"

#include "obstacle.h"
#include "obstacle_tracker.h"

#include "behavior.h"
#include "lane_keep.h"
#include "lane_follow.h"
#include "lane_change.h"

#include "jmt.h"
#include "trajectory.h"
#include "path.h"

/******************************************************************************
| VirtualDriver Class                                                         |
| ...                                                                         |
******************************************************************************/
class VirtualDriver {
  public:

    // Default Constructor
    VirtualDriver();

    // Destructor
    ~VirtualDriver();

    //-----------------------------------------------------------------------//
    // Virtual Driver Non-Default Constructors                               //
    //-----------------------------------------------------------------------//

    // Init with a vehicle, map, road, and desired planning horizon
    VirtualDriver(const Vehicle initial_status, const Road &r, const Map &m,
                  const int planning_horizon);

    //-----------------------------------------------------------------------//
    // Virtual Driver Public State Updates                                   //
    //-----------------------------------------------------------------------//

    // Receive Sensor Fusion Updates
    void sensor_fusion_updates(const long &ts, std::vector<Obstacle> &obs);

    // Update Vehicle status
    void vehicle_update(Vehicle v);

    // Get the path the vehicle is currently following
    void path_history_update(const Path &prev);

    // Update the map we're using
    void map_update(const Map &m);

    // Update road's characteristics
    void road_update(const Road &r);

    // Given the internal state, plan the path
    Path plan_route();

  private:

    //-----------------------------------------------------------------------//
    // Virtual Driver's Vehicle (Ego Vehicle)                                //
    //-----------------------------------------------------------------------//
    Vehicle mVeh; // Contains the status of our vehicle

    //-----------------------------------------------------------------------//
    // Localization Layer Outputs                                            //
    //-----------------------------------------------------------------------//
    Map mMap;     // The map of our known area
    Road mRoad;   // Road Status Variables --> Lanes, Coeff of Friction, etc.

    //-----------------------------------------------------------------------//
    // Prediction and Tracking Layer (Sensor Fusion Status)                  //
    //-----------------------------------------------------------------------//

    // Given Sensor Fusion data will aggregrate it over time, track objects
    // and provide information about them and the environment
    ObstacleTracker m_tracker;

    //-----------------------------------------------------------------------//
    // Trajectory Pooling Layer                                              //
    //-----------------------------------------------------------------------//

    // Define how much we plan ahead in "steps"
    int m_planning_horizon;

    // Define how far we went since the last planning round, used to determine
    // an accurate start point
    int m_prev_points_left;

    // Keep track of the last path we handed over so we can compare and grab
    // points based on how many we're told we've consumed
    Path m_last_path;

    // Save a window of N points we've followed for use in comfort calculations
    // and smoothing
    int m_last_followed_window_size;
    std::list<Point> m_last_followed_points;

    // Define what we were following after we last planned, used to define
    // subsequent start states (using the derivative functions)
    JMT m_cur_s_coeffs;
    JMT m_cur_d_coeffs;

    // Keep a set of possible vehicle states, which each define the rules for
    // their own trajectories and costs
    std::vector<Behavior *> m_vehicle_behaviors;

    // Generate a set of possible trajectories given our current state
    TrajectorySet generate_trajectories();

    //-----------------------------------------------------------------------//
    // Behavior Planner Layer                                                //
    //-----------------------------------------------------------------------//

    // Pick the optimal trajectory from our pool of possible trajectories
    bool comfortable(Trajectory &traj);
    Trajectory optimal_trajectory(TrajectorySet &possible_trajectories);

    // Once we've picked an optimal trajectory, turn it into a path
    Path generate_path(const Trajectory &traj);

    // Experimental smoothing of path for simulator
    Path generate_smoothed_path(const Trajectory &traj);
};

#endif // VIRTUAL_DRIVER_H