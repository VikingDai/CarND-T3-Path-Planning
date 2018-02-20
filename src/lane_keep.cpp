/******************************************************************************
| lane_keep.cpp                                                               |
| ...                                                                         |
******************************************************************************/

#include "lane_keep.h"

#include <iostream>
// #define DEBUG
// #define DEBUG_COST

using namespace std;

LaneKeep::LaneKeep()
{
  dt = 0.1;  // time delta for summing integral costs

  k_j = 2.0; // Coeff for jerk cost
  k_t = 1.0; // Coeff for time cost
  k_s = 25.0; // Coeff for lat movement cost
  k_d = 1.0; // Coeff for lon movement cost

  k_lon = 1.0; // weight of lon costs
  k_lat = 0.1; // weight of lat costs
}

LaneKeep::~LaneKeep(){/* Nothing to do here*/}

std::string LaneKeep::name() const {return "Lane Keeping";}

int LaneKeep::add_trajectories(TrajectorySet &t_set,
                                double si, double si_dot, double si_dot_dot,
                                double di, double di_dot, double di_dot_dot,
                                const int &reference_lane, const Road &r,
                                ObstacleTracker &o) const
{
  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  #endif

  // Record how many trajectories we've added
  int added = 0;

  // Target velocity keeping values
  double target_s_dot = r.speed_limit;

  // Target lateral final position based on reference line
  double target_d = r.get_lane_mid_frenet(reference_lane);

  // Our actual target lateral position since keeping is based on
  // our current lane and not the reference lane
  int current_lane = r.get_lane(di);
  double cur_lane_d = r.get_lane_mid_frenet(reference_lane);

  // Who are we following in this lane? How close are we to that guy
  // will depend on the final s value
  int leading_id = o.vehicle_to_follow(current_lane);
  Obstacle leading_vehicle;
  if(leading_id != -1) leading_vehicle = o.get_vehicle(leading_id);

  // Define constraint ranges
  // NOTE: Its important to think about whats reasonable here. For example,
  // the max comfortable acceleration is 10 m/s^2, which means a target T
  // should _probably_ be based on how long we think the desired speed change
  // should take. 0 -> 22.352 (50MPH) should take at least two seconds, so we
  // have to make sure that that time is represented or we'll probably
  // accelerate super slow. That time window slides up as our speed difference
  // does too.
  // NOTE: As such, I'm using naive kinematics to try to pick a target time
  // all while being careful to not have a target time that too low. A short
  // JMT could have weird things happen past the time line

  // Time contraints
  double target_T = max(abs(target_s_dot - si_dot) / 3.0, 1.5);
  double dT = 0.5;
  double min_T = target_T - 0.0 * dT;
  double max_T = target_T + 6.0 * dT;

  // Velocity contraints
  double min_V = target_s_dot - 10.0; // m/s -- NOTE: 50MPH -> 22.352
  double max_V = target_s_dot;
  double dV = (max_V - min_V) / 6.0;

  #ifdef DEBUG
  cout << " [*] Trying " << ((max_V - min_V) / dV) * ((max_T - min_T) / dT) << " combinations" << endl;
  cout << " [-] Varying given:" << endl
       << "   - (T should be: abs(" << target_s_dot << " - " << si_dot << ") / 6.0 = " << target_T << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - v = " << min_V << " and v = " << max_V <<  ", dV = " << dV << endl
       << "   - current_lane: " << current_lane << endl
       << "   - reference_lane: " << reference_lane << endl
       << "   - target_d: " << target_d << endl
       << "   - target_s_dot: " << target_s_dot << endl;
  #endif

  #ifdef DEBUG_COST
  static double max_c = 0.0;
  static double min_c = 100000000.0;
  static double avg_c = 0.0;
  static int N = 0;
  #endif

  // Iterate on possible time frames
  for(double T = min_T; T <= max_T; T += dT)
  {
    // Given this time, calculate a given d_path separately from all the
    // possible velocity choices
    JMT d_path = JMT({di, di_dot, di_dot_dot}, {cur_lane_d, 0.0, 0.0}, T);

    // Vary longitudinal velocities here to get an s_path and final traj
    for(double v = min_V; v <= max_V; v += dV)
    {
      #ifdef DEBUG
      cout << " [-] Trying Keeping Traj:" << endl
           << "   - si: " << si << endl
           << "   - si_d: " << si_dot << endl
           << "   - si_d_d: " << si_dot_dot << endl
           << "   - di: " << di << endl
           << "   - di_d: " << di_dot << endl
           << "   - di_d_d: " << di_dot_dot << endl
           << "   - target_d: " << target_d << endl
           << "   - target_V: " << v << endl
           << "   - T: " << T << endl;
      #endif

      // S trajectory will be created given our target start state,
      // target velocity, and a target time horizon
      JMT s_path = JMT({si, si_dot, si_dot_dot}, v, T);

      // Turn our JMTs into a full blown trajectory
      Trajectory traj = Trajectory(id, s_path, d_path, T);

      // Given this trajectory, how close will we be to the vehicle
      // in front of us, if there is one
      double follow_sf = -1;

      // Get the cost of the trajectory, set it
      // cost based on how far off we are from speed limit
      double c = cost(traj, target_s_dot, target_d, follow_sf);
      traj.cost = c;

      #ifdef DEBUG_COST
      if(c < min_c) min_c = c;
      if(c > max_c) max_c = c;
      N++;
      avg_c = ((avg_c * (N - 1)) + c) / N;
      #endif

      // Add traj to our set of possible trajectories
      insert_traj_sorted(t_set, traj);
      ++added;
    }
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
double LaneKeep::cost(const Trajectory &traj, const double &target_s_dot,
                      const double &target_d, const double &follow_sf) const
{

  // Difference between target speed and final speed
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double s_dot_delta_2 = (sf_dot - target_s_dot) * (sf_dot - target_s_dot);

  // Difference between target lane position and this final position
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

  // Get the total Lateral Trajectory cost
  // s cost is penalizing the magnitude of the distance from target speed
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_s * s_dot_delta_2);

  // Get the total Longitudinal Trajectory cost
  // d cost is chosen as (df - target_d)^2 because we ideally converge on
  // d = target_d and keep that d and we want to punish trajectories that
  // dont converge. Its NOT integral because most trajectories should just
  // be working on converging the whole time. We dont want to punish slow
  // convergence because it actually might be ideal and most comfortable!
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_d * d_delta_2);

  #ifdef DEBUG
  cout << " [*] Cost Breakdown:" << endl
       << "   - sf_dot: " << sf_dot << endl
       << "   - target_s_dot: " << target_s_dot << endl
       << "   - target_d: " << target_d << endl
       << "   - time: " << traj.T << endl
       << "   - C_lon: " << k_lon * C_lon << endl
       << "     - J_lon: " << (k_j * J_t_lon) << endl
       << "   - C_lat: " << k_lat * C_lat << endl
       << "     - J_lat: " << (k_j * J_t_lat) << endl
       << "   - C_time: " << (k_t * traj.T) << endl
       << "   - TOTAL: " << k_lat * C_lat + k_lon * C_lon << endl;
  #endif

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon;
}
