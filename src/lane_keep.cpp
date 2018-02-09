/******************************************************************************
| lane_keep.cpp                                                               |
| ...                                                                         |
******************************************************************************/

#include "lane_keep.h"

#include <iostream>
// #define DEBUG
#define DEBUG_COST

using namespace std;

LaneKeep::LaneKeep(){
  dt = 0.05;  // time delta for summing integral costs

  k_j = 25.0; // Coeff for jerk cost
  k_t = 1.0; // Coeff for time cost
  k_s = 50.0; // Coeff for lat movement cost
  k_d = 5.0; // Coeff for lon movement cost

  k_lon = 1.0; // weight of lon costs
  k_lat = 0.3; // weight of lat costs
}
LaneKeep::~LaneKeep(){/* Nothing to do here*/}

std::string LaneKeep::name() const {return "Lane Keeping";}

// Start state: [si, si_d, si_dd] at t0 (t0 = 0)
// End state:   [sf_d, sf_dd] at t1 = t0 + T
// st_d = target velocity.
// Task: generate optimal longitudinal trajectory set of quintic polynomials by
// varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d + ∆s_d[i]], 0, T[j] ]
void LaneKeep::add_trajectories(TrajectorySet &t_set,
                                double si, double si_dot, double si_dot_dot,
                                double di, double di_dot, double di_dot_dot,
                                const int &current_lane, const Road &r,
                                ObstacleTracker &o) const {
  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  #endif

  // Target values
  double speed_limit = r.speed_limit;
  double target_speed = o.lane_speed(current_lane);
  double target_d = r.get_lane_mid_frenet(current_lane);

  // for a "close-ness" cost
  int follow_id = o.vehicle_to_follow();
  double follow_ds = 0.0;
  if(follow_id != -1) follow_ds = o.get_vehicle(follow_id).s;

  // check inputs
  if(target_speed > speed_limit) target_speed = speed_limit;

  // Define constraint ranges
  // NOTE: Right now I'm setting the "target" time at whatever the horizon
  // is for the simulator (TIME_DELTA * m_planning_horizon)
  // NOTE: Its important to think about whats reasonable here. For example,
  // the max comfortable acceleration is 10 m/s^2, which means a target T
  // should _probably_ be based on how long we think the desired speed change
  // should take. 0 -> 22.352 (50MPH) should take at least two seconds, so we
  // have to make sure that that time is represented or we'll probably
  // accelerate super slow. That time window slides up as our speed difference
  // does too.
  // TO-DO: Get a "target" T value sanely that would work in a world thats
  // not the simulator...
  double target_T = 1.5; // seconds
  double dT = 0.25;
  double min_T = target_T - 2.0 * dT;
  double max_T = target_T + 12.0 * dT;

  double min_V = r.speed_limit - 5.5; // m/s -- NOTE: 50MPH -> 22.352
  double max_V = r.speed_limit;
  double dV = (max_V - min_V) / 10.0;

  #ifdef DEBUG
  cout << " [*] Trying " << ((max_V - min_V) / dV) * ((max_T - min_T) / dT) << " combinations" << endl;
  cout << " [-] Varying given:" << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - v = " << min_V << " and v = " << max_V <<  ", dV = " << dV << endl
       << "   - target_speed: " << target_speed << endl
       << "   - max_speed: " << speed_limit << endl
       << "   - target_d: " << target_d << endl;
  #endif

  #ifdef DEBUG_COST
  static double max_c = 0.0;
  static double min_c = 100000000.0;
  static double avg_c = 0.0;
  static int N = 0;
  #endif

  // Vary T and v to generate a bunch of possible s paths
  for(double v = min_V; v <= max_V; v += dV)
  {
    for(double T = min_T; T <= max_T; T += dT)
    {
      // S trajectory will be created given our target start state,
      // target velocity, and a target time horizon
      JMT s_path = JMT({si, si_dot, si_dot_dot}, v, T);

      // Vary just T to generate a matching d path.
      JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

      // Turn our JMTs into a full blown trajectory
      Trajectory traj = Trajectory(name(), s_path, d_path, T);

      // Get the cost of the trajectory, set it
      // cost based on how far off we are from speed limit
      double c = cost(traj, target_speed, speed_limit, target_d, follow_ds);
      traj.cost = c;

      #ifdef DEBUG_COST
      if(c < min_c) min_c = c;
      if(c > max_c) max_c = c;
      N++;
      avg_c = ((avg_c * (N - 1)) + c) / N;
      #endif

      // Add traj to our set of possible trajectories
      insert_traj_sorted(t_set, traj);
    }
  }

  #ifdef DEBUG_COST
  cout << " [*] Behavior '" << name() << "' cost stats:" << endl
       << "   - Average: " << avg_c << endl
       << "   - max: " << max_c << endl
       << "   - min: " << min_c << endl;
  #endif
}

// Calculate a cost for this behavior
double LaneKeep::cost(const Trajectory &traj, const double &target_speed, const double &speed_limit, const double &target_d, const double &follow_ds) const {

  // Difference between target speed and final speed
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double s_dot_delta_2 = (sf_dot - target_speed) * (sf_dot - target_speed);

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
    // Penalize Acceleration oer the trajectories
    A_t_lat += traj.s.get_acceleration_at(t) * traj.s.get_acceleration_at(t);
    A_t_lon += traj.d.get_acceleration_at(t) * traj.d.get_acceleration_at(t);

    // Penalize Jerk over the trajectories
    J_t_lat += traj.s.get_jerk_at(t) * traj.s.get_jerk_at(t);
    J_t_lon += traj.d.get_jerk_at(t) * traj.d.get_jerk_at(t);
  }

  // Penalize driving slower than the speed limit to try to encourage
  // our ego to change lanes
  double C_speed_limit = 30.0 * (sf_dot - speed_limit) * (sf_dot - speed_limit);

  // Penalize choosing to keep lane and getting close to the car in front
  double C_follow_distance = 150000.0 / (follow_ds * follow_ds);

  // Get the total Lateral Trajectory cost
  // s cost is penalizing the magnitude of the distance from target speed
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_s * s_dot_delta_2);

  // Get the total Longitudinal Trajectory cost
  // d cost is chosen as (df - target_d)^2 because we ideally converge on
  // d = target_d and keep that d and we want to punish trajectories that
  // dont converge. Its NOT integral because most trajectories should just
  // be working on converging the whole time. We dont want to punish slow
  // convergence because it actually might be ideal and most comfortable!
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_d * d_delta_2);

  // outside of the algorithm cost values
  double C_extra = 2.0 * (A_t_lat + A_t_lon) + C_speed_limit;

  #ifdef DEBUG
  cout << " [*] Cost Breakdown:" << endl
       << "   - sf_dot: " << sf_dot << endl
       << "   - time: " << traj.T << endl
       << "   - C_lat: " << k_lat * C_lat << endl
       << "     - J_lat: " << (k_j * J_t_lat) << endl
       << "   - C_lon: " << k_lon * C_lon << endl
       << "     - J_lon: " << (k_j * J_t_lon) << endl
       << "   - C_time: " << (k_t * traj.T) << endl
       << "   - C_extra: " << C_extra << endl
       << "     - V_Lat: " << V_t_lat << endl
       << "     - V_Lon: " << V_t_lon << endl
       << "     - A_Lat: " << A_t_lat << endl
       << "     - A_Lon: " << A_t_lon << endl
       << "     - A_comb: " << (A_t_lat + A_t_lon) << endl
       << "     - slow_speed: " << C_speed_limit << endl
       << "     - follow_cost: " << C_follow_distance << endl
       << "   - TOTAL: " << k_lat * C_lat + k_lon * C_lon + C_extra << endl;
  #endif

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_extra;
}
