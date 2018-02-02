/******************************************************************************
| lane_stop.cpp                                                               |
| ...                                                                         |
******************************************************************************/

#include "lane_stop.h"

#include <iostream>
#define DEBUG

using namespace std;

LaneStop::LaneStop(){
  dt = 0.5;  // time delta for summing integral costs
  k_j = 1.0; // Coeff for jerk cost
  k_t = 1.0; // Coeff for time cost
  k_s = 1.0; // Coeff for lat movement cost
  k_d = 1.0; // Coeff for lon movement cost

  k_lat = 1.0; // weight of lat costs
  k_lon = 1.0; // weight of lon costs
}
LaneStop::~LaneStop(){/* Nothing to do here*/}

std::string LaneStop::name() const {return "Lane Keeping";}

// Start state: [si, si_d, si_dd] at t0 (t0 = 0)
// End state:   [sf_d, sf_dd] at t1 = t0 + T
// st_d = target velocity.
// Task: generate optimal longitudinal trajectory set of quintic polynomials by
// varying the end constraints by ∆s_d[i] and T[j] according to: [ s1_d, s1+dd, T][ij] = [[st_d + ∆s_d[i]], 0, T[j] ]
void LaneStop::add_trajectories(TrajectorySet &t_set,
                                double si, double si_dot, double si_dot_dot,
                                double di, double di_dot, double di_dot_dot,
                                double target_d, double target_speed,
                                double speed_limit) const {

  // check inputs
  if(target_speed > speed_limit) target_speed = speed_limit;

  // Define constraint ranges
  double min_T = 1.0;
  double max_T = 10.0;
  double dT = 1.0;

  double min_V = 0.0;
  double dV = 1.0;
  double max_V = target_speed;

  #ifdef DEBUG
  cout << "   - Varying between T = " << min_T << " and T = " << max_T
       << ", v = " << min_V << " and v = " << max_V << ", with a target_speed"
       << " of " << target_speed << " (max = " << speed_limit << "), and a"
       << " target_d = " << target_d << endl;
  #endif

  // Vary T and v to generate a bunch of possible s paths
  for(double v = min_V; v < max_V; v += dV)
  {
    for(double T = min_T; T < max_T; T += dT)
    {
      // S trajectory will be created given our target start state,
      // target velocity, and a target time horizon
      JMT s_path = JMT({si, si_dot, si_dot_dot}, v, T);

      // Vary just T to generate a matching d path.
      JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

      // Turn our JMTs into a full blown trajectory
      Trajectory traj = Trajectory(s_path, d_path, T);

      // Get the cost of the trajectory, set it
      double c = cost(traj, target_speed, target_d);
      traj.cost = c;

      // Add traj to our set of possible trajectories
      insert_traj_sorted(t_set, traj);
    }
  }

  return;
}

// Calculate a cost for this behavior
double LaneStop::cost(const Trajectory &traj, const double &target_speed, const double &target_d) const {

  // Difference between target speed and final speed
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double s_dot_delta_2 = (sf_dot - target_speed) * (sf_dot - target_speed);

  // Difference between target lane position and final position
  double df = traj.d.get_position_at(traj.T);
  double d_delta_2 = (df - target_d) * (df - target_d);

  // Jerk cost is summed up over the path for both trajectories
  double J_t_lat = 0.0;
  double J_t_lon = 0.0;
  for(double t = 0.0; t <= traj.T; t += dt)
  {
    J_t_lat += traj.s.get_jerk_at(t);
    J_t_lon += traj.d.get_jerk_at(t);
  }

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

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon;
}
