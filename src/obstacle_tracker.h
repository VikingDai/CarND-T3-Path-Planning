/******************************************************************************
| Obstacle_Tracker.h                                                          |
| An abstraction of the prediction and tracker layer. Can receive and ingest  |
| Obstacle statuses from the sensor fusion layer and both track them over     |
| time AND provide a prediction of what they're doing and where they'll be in |
| the future.                                                                 |
******************************************************************************/

#ifndef OBSTACLE_TRACKER_H
#define OBSTACLE_TRACKER_H

#include <map>
#include <list>
#include <vector>
#include <sstream>
#include <iomanip>

#include "road.h"
#include "obstacle.h"
#include "trajectory.h"

/******************************************************************************
| TrackedObstacle                                                             |
| This class represents an obstacle that is being watched by the tracker. It  |
| contains some rough averages of what the vehicle has been doing in the      |
| window we've been watching it, as well as an interface to get a current,    |
| instantaneous status AND predict the obstacle's movement into the future.   |
******************************************************************************/

class TrackedObstacle {
  public:

    // Defaults
    TrackedObstacle();
    ~TrackedObstacle();

    // non-Defaults
    TrackedObstacle(Obstacle &obs, const int history_buffer_size);

    // Get current status
    Obstacle current() const;

    // Getters for current/instantaneous status
    double s() const;
    double d() const;
    double s_dot() const;
    double d_dot() const;
    double s_dot_dot() const;
    double d_dot_dot() const;

    // Getters for averaged status values
    double avg_s() const;
    double avg_d() const;
    double avg_s_dot() const;
    double avg_d_dot() const;
    double avg_s_dot_dot() const;
    double avg_d_dot_dot() const;

    // Update Interface
    void update(Obstacle &obs);

    // Status and prediction interface
    double s_at(double t) const;
    double d_at(double t) const;
    double s_dot_at(double t) const;
    double d_dot_at(double t) const;

    // Debug printing
    std::string print() const;

    // Id of the obstacle we're tracking
    int id;

  private:

    // Average position of the obstacle
    double s_avg;
    double d_avg;

    // Velocity in Frenet
    double s_dot_avg;
    double d_dot_avg;

    // Acceleration in Frenet
    double s_dot_dot_avg;
    double d_dot_dot_avg;

    // Tracked Obstacle History
    // I'm tracking these obstacles by ID over time. The list allows O(1)
    // insert to the front and back. Front will represent the "freshest" data
    // [t = 0] --> [t = -1] --> ... --> [t = -N]
    double history_buffer_size;
    std::list<Obstacle> history;
};

/******************************************************************************
| ObstacleTracker                                                             |
| This class maintains the set of tracked obstacles as well as the state of   |
| the lanes.                                                                  |
******************************************************************************/

class ObstacleTracker {
  public:

    // Constructor/Destructor
    ObstacleTracker();
    ~ObstacleTracker();

    // Non-Defaults
    ObstacleTracker(const double view_distance, const int tracking_buffer_size);

    // Ingest and update state based on new Sensor Fusion data
    void update(const long &ts, std::vector<Obstacle> &obs, const Vehicle &veh, const Road &road);

    // Helpers to get other important values
    int vehicle_to_follow();
    Obstacle get_vehicle(const int &id);
    int vehicle_to_follow(const int &lane_num);
    double lane_speed(const int &lane_num);

    // Given a Trajectory, determine if that trajectory is safe and
    // collision free. Conservatively predicts the behavior of
    // all the tracked vehicles based on what they've done so far.
    bool trajectory_is_safe(const Trajectory &traj);

    // Debug output
    std::string get_debug_lanes();

    // Attributes of our tracker
    // Note: some of this is due to the nature of the simulator, some is needed
    double m_view_distance;
    int m_tracking_buffer_max;

    // Total set of obstacles
    std::map<int, TrackedObstacle> m_obstacles;

    // Set of obstacles mapped into lanes by integer ID
    std::vector<std::list<int>> m_lanes;

    // The most up to date car status
    Vehicle m_veh;

    // The most up to date Road status
    Road m_road;
};


#endif // __OBSTACLE_TRACKER_H__