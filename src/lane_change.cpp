/******************************************************************************
| lane_change.cpp                                                             |
| ...                                                                         |
******************************************************************************/

#include "lane_change.h"

#include <iostream>
#define DEBUG

using namespace std;

LaneChange::LaneChange(){
  distance_buffer = 5.0; // meters, fixed distance of safety past a time gap
  time_gap = 0.5;        // seconds,

  dt = 0.05;  // time delta for summing integral costs

  k_j = 5.0; // Coeff for jerk cost
  k_t = 2.0; // Coeff for time cost
  k_s = 1.0; // Coeff for lat movement cost
  k_d = 0.15; // Coeff for lon movement cost

  k_lon = 1.0; // weight of lon costs
  k_lat = 0.2; // weight of lateral costs
}

LaneChange::~LaneChange(){/* Nothing to do here */}

std::string LaneChange::name() const {return "Lane Changing";}

// Determine possible trajectories for left and right lane changes
// Given our current state (speed and lane), will check the adjacent
// lanes to see if we can merge into them.
void LaneChange::add_trajectories(TrajectorySet &t_set,
                                  double si, double si_dot, double si_dot_dot,
                                  double di, double di_dot, double di_dot_dot,
                                  const int &current_lane, const Road &r,
                                  ObstacleTracker &o) const {

  #ifdef DEBUG
  cout << " [-] Determing trajectories for '" << name() << "'" << endl;
  #endif

  // If our lane is going at the max speed, we probably don't need to change
  double speed_limit = r.speed_limit;
  int follow_id = o.vehicle_to_follow();
  if(follow_id == -1) return;
  Obstacle following = o.get_vehicle(follow_id);
  if(following.s_dot >= speed_limit) return;

  // Define constraint ranges
  double min_T = 1.0;
  double max_T = 5.0;
  double dT = 0.5;
  int left = current_lane - 1;
  int right = current_lane + 1;

  #ifdef DEBUG
  cout << " [-] Varying given:" << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - Right: " << right << endl
       << "   - Left: " << left << endl
       << endl;
  #endif

  // vars for maintaining lane obstacle status
  int cur_id, prev_id;
  Obstacle cur;
  double cur_s, cur_s_dot, cur_s_dot_dot, prev_s, prev_s_dot, prev_s_dot_dot;

  // vars for saving state targets
  double target_d, target_s, target_s_dot, target_s_dot_dot;

  // Generate trajectories for each time, for lane to right and left
  for(double T = min_T; T < max_T; T += dT)
  {
    // Check right
    if(right < r.lane_count) // && !(left >= 0 && o.m_lanes[left].size() == 0))
    {
      prev_id = -1;
      target_d = r.get_lane_mid_frenet(right);

      // iterate on gaps between cars
      for(auto it = o.m_lanes[right].begin(); it != o.m_lanes[right].end(); ++it)
      {
        // Get Obstacle by ID
        cur_id = (*it);
        cur = o.get_vehicle(cur_id);
        cur_s = cur.s;
        cur_s_dot = cur.s_dot;
        cur_s_dot_dot = cur.s_dot_dot;

        // Predict out to the fuuuuture
        cur_s = cur_s + cur_s_dot * T + 0.5 * cur_s_dot_dot * T * T;
        cur_s_dot = cur_s_dot + cur_s_dot_dot * T;
        cur_s_dot_dot = cur_s_dot_dot;

        #ifdef DEBUG
        cout << " [-] Trying T = " << T << ":" << endl
             << "   - Lane: right" << endl
             << "   - Merge in Front of: " << prev_id << endl
             << "   - Merge in Behind of: " << cur_id << endl;
        #endif

        // determine final s values
        if(prev_id == -1)
        {
          // aim to just follow since we dont know whats behind
          target_s = cur_s - (distance_buffer + time_gap * cur_s_dot);
          target_s_dot = cur_s_dot;
          target_s_dot_dot = cur_s_dot_dot;
        }
        else
        {
          // Aim to be half way between the two vehicles going the
          // speed of whats in front
          target_s = 0.5 * (cur_s + prev_s);
          target_s_dot = cur_s_dot;
          target_s_dot_dot = cur_s_dot_dot;
        }

        // update prev_*
        prev_id = cur_id;
        prev_s = cur_s;
        prev_s_dot = cur_s_dot;
        prev_s_dot_dot = cur_s_dot_dot;

        // Adjust for speed limit
        if(target_s_dot > speed_limit) target_s_dot = speed_limit;

        #ifdef DEBUG
        cout << "   - target_s: " << target_s << endl
             << "   - target_d: " << target_d << endl
             << "   - target_s_dot: " << target_s_dot << endl
             << "   - target_s_dot_dot: " << target_s_dot_dot << endl;
        #endif

        // no looking backwards
        if(target_s < si) continue;

        // Vary T and v to generate a bunch of possible s paths
        JMT s_path = JMT({si, si_dot, si_dot_dot}, {target_s, target_s_dot, target_s_dot_dot}, T);

        // Vary just T to generate a matching d path.
        JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

        // Turn our JMTs into a full blown trajectory
        Trajectory traj = Trajectory(name(), s_path, d_path, T);

        // Get the cost of the trajectory, set it
        double c = cost(traj, target_s, target_d, speed_limit);
        traj.cost = c;

        #ifdef DEBUG
        cout << "   - COST: " << c << endl;
        #endif

        // Add traj to our set of possible trajectories
        insert_traj_sorted(t_set, traj);
      }

      // Finally, see if we can pull in front of the leading car in the lane
      JMT s_path = JMT({si, si_dot, si_dot_dot}, speed_limit, T);
      JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);
      Trajectory traj = Trajectory(name(), s_path, d_path, T);
      double c = cost2(traj, speed_limit, target_d, speed_limit);
      traj.cost = c;
      insert_traj_sorted(t_set, traj);

      #ifdef DEBUG
      cout << " [-] Trying T = " << T << ":" << endl
           << "   - Lane: right" << endl
           << "   - Merge Front: " << prev_id << endl
           << "   - Merge Behind: " << -1 << endl
           << "   - target_d: " << target_d << endl
           << "   - target_s_dot: " << speed_limit << endl
           << "   - COST: " << c << endl;
      #endif
    }

    // Check left
    if(left >= 0) // && !(right < r.lane_count && o.m_lanes[right].size() == 0))
    {
      prev_id = -1;
      target_d = r.get_lane_mid_frenet(left);

      // iterate on gaps between cars
      for(auto it = o.m_lanes[left].begin(); it != o.m_lanes[left].end(); ++it)
      {
        // Get Obstacle by ID
        cur_id = (*it);
        cur = o.get_vehicle(cur_id);
        cur_s = cur.s;
        cur_s_dot = cur.s_dot;
        cur_s_dot_dot = cur.s_dot_dot;

        // Predict out to the fuuuuture
        cur_s = cur_s + cur_s_dot * T + 0.5 * cur_s_dot_dot * T * T;
        cur_s_dot = cur_s_dot + cur_s_dot_dot * T;
        cur_s_dot_dot = cur_s_dot_dot;

        #ifdef DEBUG
        cout << " [-] Trying T = " << T << ":" << endl
             << "   - Lane: left" << endl
             << "   - Merge in Front of: " << prev_id << endl
             << "   - Merge in Behind of: " << cur_id << endl;
        #endif

        // determine final s values
        if(prev_id == -1)
        {
          // aim to just follow since we dont know whats behind
          target_s = cur_s - (distance_buffer + time_gap * cur_s_dot);
          target_s_dot = cur_s_dot;
          target_s_dot_dot = cur_s_dot_dot;
        }
        else
        {
          // Aim to be half way between the two vehicles
          target_s = 0.5 * (cur_s + prev_s);
          target_s_dot = cur_s_dot;
          target_s_dot_dot = cur_s_dot_dot;
        }

        // Adjust for speed limit
        if(target_s_dot > speed_limit) target_s_dot = speed_limit;

        // update prev_*
        prev_id = cur_id;
        prev_s = cur_s;
        prev_s_dot = cur_s_dot;
        prev_s_dot_dot = cur_s_dot_dot;

        #ifdef DEBUG
        cout << "   - target_s: " << target_s << endl
             << "   - target_d: " << target_d << endl
             << "   - target_s_dot: " << target_s_dot << endl
             << "   - target_s_dot_dot: " << target_s_dot_dot << endl;
        #endif

        // no looking back
        if(target_s < si) continue;

        // Vary T and v to generate a bunch of possible s paths
        JMT s_path = JMT({si, si_dot, si_dot_dot}, {target_s, target_s_dot, target_s_dot_dot}, T);

        // Vary just T to generate a matching d path.
        JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

        // Turn our JMTs into a full blown trajectory
        Trajectory traj = Trajectory(name(), s_path, d_path, T);

        // Get the cost of the trajectory, set it
        double c = cost(traj, target_s, target_d, speed_limit);
        traj.cost = c;

        #ifdef DEBUG
        cout << "   - COST: " << c << endl;
        #endif

        // Add traj to our set of possible trajectories
        insert_traj_sorted(t_set, traj);
      }

      // Finally, see if we can pull in front of the leading car in the lane
      JMT s_path = JMT({si, si_dot, si_dot_dot}, speed_limit, T);
      JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);
      Trajectory traj = Trajectory(name(), s_path, d_path, T);
      double c = cost2(traj, speed_limit, target_d, speed_limit);
      traj.cost = c;
      insert_traj_sorted(t_set, traj);

      #ifdef DEBUG
      cout << " [-] Trying T = " << T << ":" << endl
           << "   - Lane: left" << endl
           << "   - Merge in Front of: " << prev_id << endl
           << "   - Merge in Behind of: " << -1 << endl
           << "   - target_d: " << target_d << endl
           << "   - target_s_dot: " << speed_limit << endl
           << "   - COST: " << c << endl;
      #endif
    }
  }
}

// derp derp derp
double LaneChange::cost(const Trajectory &traj, const double &target_s, const double &target_d, const double &target_s_dot) const {

  // Difference between target speed and final speed
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

  // Get the total Lateral Trajectory cost
  // s cost is penalizing the magnitude of the distance from target speed
  //             (  JERK COST  )   (  TIME COST )   (      LAT COST     )
  double C_lat = (k_j * J_t_lat) + (k_t * traj.T) + (k_s * s_delta_2);

  // Get the total Longitudinal Trajectory cost
  // d cost is chosen as (df - target_d)^2 because we ideally converge on
  // d = target_d and keep that d and we want to punish trajectories that
  // dont converge. Its NOT integral because most trajectories should just
  // be working on converging the whole time. We dont want to punish slow
  // convergence because it actually might be ideal and most comfortable!
  //             (  JERK COST  )   (  TIME COST )   (    LON COST   )
  double C_lon = (k_j * J_t_lon) + (k_t * traj.T) + (k_d * d_delta_2);

  // Extra costs outside of the algorithm
  double C_extra = V_t_lat + V_t_lon + A_t_lat + A_t_lon + 6.0*speed_pen;

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_extra;
}

// derp derp derp
double LaneChange::cost2(const Trajectory &traj, const double &target_speed, const double &target_d, const double &speed_limit) const {

  // Difference between target speed and final speed
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double s_dot_delta_2 = (sf_dot - target_speed) * (sf_dot - target_speed);

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

  // Penalize driving slower than the speed limit to try to encourage
  // our ego to change lanes
  double speed_slow = (sf_dot - speed_limit) * (sf_dot - speed_limit);

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

  // Extra costs outside of the algorithm
  double C_extra = V_t_lat + V_t_lon + A_t_lat + A_t_lon + 2.0*speed_slow;

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + speed_slow;
}