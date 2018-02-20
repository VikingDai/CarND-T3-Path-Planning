/******************************************************************************
| lane_change.cpp                                                             |
| ...                                                                         |
******************************************************************************/

#include "lane_change.h"

#include <iostream>

#define DEBUG
#define DEBUG_COST

using namespace std;

LaneChange::LaneChange()
{
  distance_buffer = 3.0; // meters, fixed distance of safety past a time gap
  time_gap = 0.5;        // seconds,

  dt = 0.1;   // time delta for summing integral costs

  k_j = 2.0;  // Coeff for jerk cost
  k_t = 1.0;  // Coeff for time cost
  k_s = 25.0; // Coeff for lat movement cost
  k_d = 1.0;  // Coeff for lon movement cost

  k_lon = 1.0; // weight of longitudinal costs
  k_lat = 0.1; // weight of lateral costs
}

LaneChange::~LaneChange(){/* Nothing to do here */}

std::string LaneChange::name() const {return "Lane Changing";}

// This gets called when we (Lane Changing) is the CURRENT behavior and
// the brain wants to know whats okay to do next. In the general sense,
// anything is okay to do next, so default to whatever we've set. However,
// we want our lane change to be completely atomic, so if we haven't yet
// finished the lane change sufficiently, return a set with just our ID
BehaviorSet LaneChange::get_next_behaviors(const double s, const double d,
                                           const Road &r, const int reference_lane) const
{
  if(!current) return next_behaviors;
  double target_d = r.get_lane_mid_frenet(reference_lane);
  if(abs(target_d - d) > 0.1) return {id}; // if we haven't hit a state
  return next_behaviors;
}

// For outputing average, min and max costs
#ifdef DEBUG_COST
static double max_c = 0.0;
static double min_c = 100000000.0;
static double avg_c = 0.0;
static int N = 0;
#endif

int LaneChange::add_trajectories(TrajectorySet &t_set,
                                  double si, double si_dot, double si_dot_dot,
                                  double di, double di_dot, double di_dot_dot,
                                  const int &reference_lane, const Road &r,
                                  ObstacleTracker &o) const
{
  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  if(current) cout << " [*] This is the current Behavior!" << endl;
  #endif

  // Record how many trajectories we've added
  int added = 0;

  // In all cases, the target speed we really want to be going is the
  // ultimate speed limit set for the road.
  double target_s_dot = r.speed_limit;

  // Target lateral final position is the reference lane's position
  // but penalties for staying on that line can cause us to change
  // lanes and move that reference line
  int current_lane = r.get_lane(di);
  double target_d = r.get_lane_mid_frenet(reference_lane);

  // Time contraints for our manuever. Take the max of 2 seconds
  // and however long kinematics tells us a change in s_dot should
  // take given a modest acceleration value
  double target_T = max(abs(target_s_dot - si_dot) / 3.0, 4.0);
  double dT = 0.5;
  double min_T = target_T - 0.0 * dT;
  double max_T = target_T + 6.0 * dT;

  // Velocity contraints for our manuever
  // NOTE: 50MPH -> 22.352 m/s, so keep this in mind
  double min_V = target_s_dot - 10.0; // m/s
  double max_V = target_s_dot;
  double dV = (max_V - min_V) / 6.0;

  // Lateral Movement/Lane constraints
  int min_l = max(0, current_lane - 1);
  int max_l = min(r.lane_count - 1, current_lane + 1);
  int dl = 1;

  // If we're in the middle of a change (i.e we're the current behavior)
  // Then we need to keep seeing out the lane change if we're not close
  // enough to our target.
  if(current)
  {
    #ifdef DEBUG
    cout << " [*] Current Behavior! Need to decide what to do!" << endl;
    #endif

    if(current_lane != reference_lane || abs(target_d - di) > 0.1)
    {
      #ifdef DEBUG
      cout << " [!] Lane change determined not to be complete!\n"
           << "   - Current d: " << di << "\n"
           << "   - Target d: " << target_d << "\n";
      #endif

      min_l = max_l = reference_lane;
    }
  }

  // Add trajectories for each lane we want to change into
  for(int l = min_l; l <= max_l; l += dl)
  {
    // If we're not in a state where we're finishing an existing lane change
    // then dont check our own lane. Thats the same as lane keeping!
    if(!current && l == current_lane) continue;

    // Set the target/final d value
    double df = r.get_lane_mid_frenet(l);

    // Now that we know which lane we're thinking of going into, lets
    // see what we're changing into. Who would we be following? How
    // close are we to that guy will depend on the final s value
    int leading_id = o.vehicle_to_follow(l);
    Obstacle leading_vehicle;
    if(leading_id != -1) leading_vehicle = o.get_vehicle(leading_id);

    #ifdef DEBUG
    cout << " [!] Trying Lane " << l << " -- d = " << target_d << endl;
    #endif

    // Iterate on possible time frames
    for(double T = min_T; T <= max_T; T += dT)
    {
      // Given this time, calculate a given d_path
      JMT d_path = JMT({di, di_dot, di_dot_dot}, {df, 0.0, 0.0}, T);

      // Vary longitudinal velocities here to get an s_path and final traj
      for(double sf_dot = min_V; sf_dot <= max_V; sf_dot += dV)
      {
        #ifdef DEBUG
        cout << " [-] Trying Lane Changing Traj:" << endl
             << "   - si: " << si << endl
             << "   - si_d: " << si_dot << endl
             << "   - si_d_d: " << si_dot_dot << endl
             << "   - di: " << di << endl
             << "   - di_d: " << di_dot << endl
             << "   - di_d_d: " << di_dot_dot << endl
             << "   - df: " << df << endl
             << "   - sf_dot: " << sf_dot << endl
             << "   - T: " << T << endl;
        #endif

        // S trajectory will be created given our target start state,
        // target velocity, and a target time horizon
        JMT s_path = JMT({si, si_dot, si_dot_dot}, sf_dot, T);

        // Turn our JMTs into a full blown trajectory
        Trajectory traj = Trajectory(id, s_path, d_path, T);

        // Given our path and the lane we're going into, where is that
        // leading car going to be reletive to use?
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
  }

  #ifdef DEBUG_COST
  cout << " [*] Behavior '" << name() << "' cost stats:" << endl
       << "   - Average: " << avg_c << endl
       << "   - max: " << max_c << endl
       << "   - min: " << min_c << endl;
  #endif

  return added;
}

// Cost function for trajectories that were created by merging in between two
// cars.
double LaneChange::cost(const Trajectory &traj, const double &target_s_dot,
                        const double &target_d, const double &follow_sf) const
{
  // Difference between target speed and final speed
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

  // Get the total Lateral Trajectory cost
  // s cost is penalizing the magnitude of the distance from target speed
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_s * s_delta_2);

  // Get the total Longitudinal Trajectory cost
  // d cost is chosen as (df - target_d)^2 because we ideally converge on
  // d = target_d and keep that d and we want to punish trajectories that
  // dont converge. Its NOT integral because most trajectories should just
  // be working on converging the whole time. We dont want to punish slow
  // convergence because it actually might be ideal and most comfortable!
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_d * d_delta_2);

  #ifdef DEBUG_COST
  cout << " [*] Cost Breakdown:" << endl
       << "   - sf_dot: " << sf_dot << endl
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
