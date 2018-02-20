/******************************************************************************
| lane_follow.cpp                                                             |
| ...                                                                         |
******************************************************************************/

#include "lane_follow.h"

#include <iostream>
// #define DEBUG
// #define DEBUG_COST

using namespace std;

LaneFollow::LaneFollow()
{
  distance_buffer = 3.0; // meters, fixed distance of safety
  time_gap = 1.0;       // seconds, time based distance of safety

  dt = 0.1;  // time delta for summing integral costs

  k_j = 2.0; // Coeff for jerk cost
  k_a = 2.0; // Coeff for acceleration cost
  k_t = 1.0; // Coeff for time cost
  k_s = 25.0; // Coeff for lat movement cost
  k_d = 1.0; // Coeff for lon movement cost

  k_safety = 140.0; // Coeff for safety dist

  k_lon = 1.0; // weight of lon costs
  k_lat = 0.5; // weight of lat costs
}

LaneFollow::~LaneFollow(){/* Nothing to do here*/}

std::string LaneFollow::name() const {return "Lane Following";}

int LaneFollow::add_trajectories(TrajectorySet &t_set,
                                 double si, double si_dot, double si_dot_dot,
                                 double di, double di_dot, double di_dot_dot,
                                 const int &reference_lane, const Road &r,
                                 ObstacleTracker &o) const
{

  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  #endif

  // Keep track of how many trajectories are added
  int added = 0;

  // Assuming we're following our lane, get the vehicle you're supposed to be
  // following from the tracker. If theres no one to follow, we're done
  int follow_id = o.vehicle_to_follow();
  if(follow_id == -1) return added;
  Obstacle following = o.get_vehicle(follow_id);

  // Grab the following objects position, speed, acceleration
  // Note: we could probably bake this into the prediction/tracker layer
  // as an API because we are, after all, watching these cars over frames.
  // It would probably help us get better trajectories
  // Note: We really dont want to try to match a sim car's accel because they
  // dont have the same rules we do. Might as well be safe
  double follow_s = following.s;
  double follow_s_dot = (following.s_dot == 0 ? following.speed : following.s_dot);
  double follow_a = 0.0;

  #ifdef DEBUG
  cout << " [-] Should be following 'Obstacle ID " << follow_id << "' at s = "
       << follow_s << " (Ego S = " << si << ")"
       << endl;
  #endif

  // Our target values
  double target_s_dot = r.speed_limit;
  double speed_limit = r.speed_limit;
  int current_lane = r.get_lane(di);
  double target_d = r.get_lane_mid_frenet(current_lane);

  // Iterate on possible T values for this behavior
  double target_T = max(abs(si_dot - follow_s_dot) / 2.0, 2.0); // seconds
  double dT = 0.5;
  double min_T = target_T - 0.0 * dT;
  double max_T = target_T + 4.0 * dT;

  #ifdef DEBUG
  cout << " [*] Trying " << ((max_T - min_T) / dT) << " combinations" << endl;
  cout << " [-] Varying given:" << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - Current Lane: " << current_lane << endl
       << "   - Follow ID: " << following.id << endl
       << "   - Follow (s,d): (" << follow_s << ", " << following.d << ")" << endl
       << "   - Follow s_dot: " << follow_s_dot << endl
       << "   - Follow s_dot_dot: " << follow_a << endl
       << "   - target_s_dot: " << target_s_dot << endl
       << "   - target_d: " << target_d << endl;
  #endif

  #ifdef DEBUG_COST
  static double max_c = 0.0;
  static double min_c = 100000000.0;
  static double avg_c = 0.0;
  static int N = 0;
  #endif

  // generate our trajectories
  for(double T = min_T; T <= max_T; T += dT)
  {
    // Predict where the following car will be in T seconds
    double follow_s_final = follow_s + follow_s_dot * T + 0.5 * follow_a * T * T;
    double follow_s_dot_final = follow_s_dot + follow_a * T;

    // check speed input - limit to speed limit
    double target_s_dot_dot = 0.0;
    if(follow_s_dot_final > speed_limit) follow_s_dot_final = speed_limit;

    // Determine our car's target s position based onthe car we're following
    double target_s = follow_s_final - (distance_buffer + time_gap * follow_s_dot_final);

    #ifdef DEBUG
    cout << " [-] Trying T = " << T << ":" << endl
         << "   - Follow s final: " << follow_s_final << endl
         << "   - Follow s_dot final: " << follow_s_dot_final << endl
         << "   - target_s: " << target_s << endl
         << "   - target_d: " << target_d << endl
         << "   - target_s_dot: " << target_s_dot << endl
         << "   - target_s_dot_dot: " << target_s_dot_dot << endl;
    #endif

    // S trajectory will be created given our target start state,
    // final target s position, and a target time horizon. The other
    // pieces were calculated with kinematics, or assumed.
    JMT s_path = JMT({si, si_dot, si_dot_dot}, {target_s, follow_s_dot_final, target_s_dot_dot}, T);

    // Vary just T to generate a matching d path.
    JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

    // Turn our JMTs into a full blown trajectory
    Trajectory traj = Trajectory(id, s_path, d_path, T);

    // Get the cost of the trajectory, set it
    double c = cost(traj, target_s_dot, target_d, follow_s_final);
    traj.cost = c;

    #ifdef DEBUG
    cout << "   - COST: " << c << endl;
    #endif

    // Add traj to our set of possible trajectories
    insert_traj_sorted(t_set, traj);
    ++added;

    #ifdef DEBUG_COST
    if(c < min_c) min_c = c;
    if(c > max_c) max_c = c;
    N++;
    avg_c = ((avg_c * (N - 1)) + c) / N;
    #endif
  }

  #ifdef DEBUG_COST
  cout << " [*] Behavior '" << name() << "' cost stats:" << endl
       << "   - Average: " << avg_c << endl
       << "   - max: " << max_c << endl
       << "   - min: " << min_c << endl;
  #endif

  return added;
}

// Calculate a cost for this behavior
double LaneFollow::cost(const Trajectory &traj, const double &target_s_dot,
                        const double &target_d, const double &follow_sf) const
{

  // Difference between target position and final position
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double s_delta_2 = (sf_dot - target_s_dot) * (sf_dot - target_s_dot);

  // Difference between target lane position and final position
  double df = traj.d.get_position_at(traj.T);
  double d_delta_2 = (df - target_d) * (df - target_d);

  // Jerk cost is summed up over the path for both trajectories
  double A_t_lat = 0.0;
  double A_t_lon = 0.0;
  double J_t_lat = 0.0;
  double J_t_lon = 0.0;
  for(double t = 0.0; t <= traj.T; t += dt)
  {
    // Penalize Acceleration over the trajectories
    A_t_lon += traj.s.get_acceleration_at(t) * traj.s.get_acceleration_at(t);
    A_t_lat += traj.d.get_acceleration_at(t) * traj.d.get_acceleration_at(t);

    // Penalize Jerk over the trajectories
    J_t_lon += traj.s.get_jerk_at(t) * traj.s.get_jerk_at(t);
    J_t_lat += traj.d.get_jerk_at(t) * traj.d.get_jerk_at(t);
  }

  // Try using AVERAGE jerk to get rid of the issues that come with using
  // longer time frames
  J_t_lon /= (traj.T / dt);
  J_t_lat /= (traj.T / dt);

  // Try using AVERAGE accel to get rid of the issues that come with using
  // longer time frames
  A_t_lon /= (traj.T / dt);
  A_t_lat /= (traj.T / dt);

  // Calculate a safety distance cost based on our final position and the
  // vehicle we're following's final position. Take away the follow dist
  // we've imposed
  double sf = traj.s.get_position_at(traj.T);
  double inv_follow_dist = 1.0 / (follow_sf - sf);
  if(follow_sf == -1) inv_follow_dist = 0.0;
  else if(sf >= follow_sf) inv_follow_dist = 0.2;
  double C_safety_dist = exp(k_safety * abs(inv_follow_dist));

  // Get the total Lateral Trajectory cost
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_s * s_delta_2) + (k_a * A_t_lon);

  // Get the total Longitudinal Trajectory cost
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_d * d_delta_2) + (k_a * A_t_lat);

  #ifdef DEBUG_COST
  cout << " [*] Cost Breakdown:" << endl
       << "   - C_lon: " << k_lon * C_lon << endl
       << "     - J_lon: " << (k_j * J_t_lon) << endl
       << "   - C_lat: " << k_lat * C_lat << endl
       << "     - J_lat: " << (k_j * J_t_lat) << endl
       << "   - C_safety_dist: " << C_safety_dist << endl
       << "   - TOTAL: " << k_lat * C_lat + k_lon * C_lon << endl;
  #endif

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_safety_dist;
}
