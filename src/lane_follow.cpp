/******************************************************************************
| lane_follow.cpp                                                             |
| ...                                                                         |
******************************************************************************/

#include "lane_follow.h"

#include <iostream>
#define DEBUG

using namespace std;

LaneFollow::LaneFollow(){

  distance_buffer = 10.0; // meters, fixed distance of safety past a time gap
  time_gap = 0.0;        // seconds,

  dt = 0.05;  // time delta for summing integral costs

  k_j = 25.0; // Coeff for jerk cost
  k_t = 2.0; // Coeff for time cost
  k_s = 50.0; // Coeff for lat movement cost
  k_d = 5.0; // Coeff for lon movement cost

  k_lon = 1.0; // weight of lon costs
  k_lat = 0.3; // weight of lat costs
}

LaneFollow::~LaneFollow(){/* Nothing to do here*/}

std::string LaneFollow::name() const {return "Lane Following";}

void LaneFollow::add_trajectories(TrajectorySet &t_set,
                                  double si, double si_dot, double si_dot_dot,
                                  double di, double di_dot, double di_dot_dot,
                                  const int &current_lane, const Road &r,
                                  ObstacleTracker &o) const {

  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  #endif

  // Assuming we're following our lane, get the vehicle you're supposed to be
  // following from the tracker. If theres no one to follow, we're done
  int follow_id = o.vehicle_to_follow();
  if(follow_id == -1) return;
  Obstacle following = o.get_vehicle(follow_id);

  // Grab the following objects position, speed, acceleration
  // Note: we could probably bake this into the prediction/tracker layer
  // as an API because we are, after all, watching these cars over frames.
  // It would probably help us get better trajectories
  double follow_s = following.s;
  double follow_s_dot = (following.s_dot == 0 ? following.speed : following.s_dot);
  double follow_a = following.s_dot_dot;

  // Keep lane if we're not thaaat close
  if((follow_s - si) < 20) return;

  // Our target values
  double speed_limit = r.speed_limit;
  double target_d = r.get_lane_mid_frenet(current_lane);

  // Iterate on possible T values for this behavior
  double min_T = 0.5;
  double max_T = 3.0;
  double dT = 0.5;

  #ifdef DEBUG
  cout << " [-] Varying given:" << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - Current Lane: " << current_lane << endl
       << "   - Follow ID: " << following.id << endl
       << "   - Follow (s,d): (" << follow_s << ", " << following.d << ")" << endl
       << "   - Follow s_dot: " << follow_s_dot << endl
       << "   - Follow s_dot_dot: " << follow_a << endl
       << "   - max_speed: " << speed_limit << endl
       << "   - target_d: " << target_d << endl;
  #endif

  // generate our trajectories
  for(double T = min_T; T < max_T; T += dT)
  {
    // Predict where the following car will be in T seconds
    double follow_s_final = follow_s + follow_s_dot * T + 0.5 * follow_a * T * T;
    double follow_v_final = follow_s_dot + follow_a * T;

    // check speed input - limit to speed limit
    double target_s_dot = follow_v_final;
    double target_s_dot_dot = follow_a;
    if(target_s_dot > speed_limit) target_s_dot = speed_limit;

    // Determine our car's target s position based onthe car we're following
    double target_s = follow_s_final - (distance_buffer + time_gap * follow_v_final);

    #ifdef DEBUG
    cout << " [-] Trying T = " << T << ":" << endl
         << "   - Follow s final: " << follow_s_final << endl
         << "   - Follow s_dot final: " << follow_v_final << endl
         << "   - target_s: " << target_s << endl
         << "   - target_d: " << target_d << endl
         << "   - target_s_dot: " << target_s_dot << endl
         << "   - target_s_dot_dot: " << target_s_dot_dot << endl;
    #endif

    // S trajectory will be created given our target start state,
    // final target s position, and a target time horizon
    JMT s_path = JMT({si, si_dot, si_dot_dot}, {target_s, target_s_dot, target_s_dot_dot}, T);

    // Vary just T to generate a matching d path.
    JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

    // Turn our JMTs into a full blown trajectory
    Trajectory traj = Trajectory(name(), s_path, d_path, T);

    // Get the cost of the trajectory, set it
    double c = cost(traj, target_s, target_d, target_s_dot, speed_limit);
    traj.cost = c;

    #ifdef DEBUG
    cout << "   - COST: " << c << endl;
    #endif

    // Add traj to our set of possible trajectories
    insert_traj_sorted(t_set, traj);
  }
}

// Calculate a cost for this behavior
double LaneFollow::cost(const Trajectory &traj, const double &target_s, const double &target_d, const double &target_s_dot, const double &speed_limit) const {

  // Difference between target position and final position
  double sf = traj.s.get_position_at(traj.T);
  double s_delta_2 = (sf - target_s) * (sf - target_s);

  // Difference between target lane position and final position
  double df = traj.d.get_position_at(traj.T);
  double d_delta_2 = (df - target_d) * (df - target_d);

  // Jerk cost is summed up over the path for both trajectories
  double V_t_lat = 0.0;
  double V_t_lon = 0.0;
  double A_t_lat = 0.0;
  double A_t_lon = 0.0;
  double J_t_lat = 0.0;
  double J_t_lon = 0.0;
  for(double t = 0.0; t <= traj.T; t += dt)
  {
    V_t_lat += traj.s.get_velocity_at(t) * traj.s.get_velocity_at(t);
    V_t_lon += traj.d.get_velocity_at(t) * traj.d.get_velocity_at(t);

    A_t_lat += traj.s.get_acceleration_at(t) * traj.s.get_acceleration_at(t);
    A_t_lon += traj.d.get_acceleration_at(t) * traj.d.get_acceleration_at(t);

    J_t_lat += traj.s.get_jerk_at(t) * traj.s.get_jerk_at(t);
    J_t_lon += traj.d.get_jerk_at(t) * traj.d.get_jerk_at(t);
  }

  // Penalty for being far from the target speed
  double s_dot_f = traj.s.get_velocity_at(traj.T);
  double speed_pen = (s_dot_f - target_s_dot) * (s_dot_f - target_s_dot);

  // Penalize driving slower than the speed limit to try to encourage
  // our ego to change lanes
  double speed_slow = (s_dot_f - speed_limit) * (s_dot_f - speed_limit);

  // Get the total Lateral Trajectory cost
  // s cost is penalizing the magnitude of the distance from target speed
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_s * s_delta_2) + V_t_lat + A_t_lat;

  // Get the total Longitudinal Trajectory cost
  // d cost is chosen as (df - target_d)^2 because we ideally converge on
  // d = target_d and keep that d and we want to punish trajectories that
  // dont converge. Its NOT integral because most trajectories should just
  // be working on converging the whole time. We dont want to punish slow
  // convergence because it actually might be ideal and most comfortable!
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_d * d_delta_2) + V_t_lon + A_t_lon;

  // Extra costs outside of the algorithm
  double C_extra = V_t_lat + V_t_lon + A_t_lat + A_t_lon + 2.5*speed_slow + speed_pen;

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_extra;
}
