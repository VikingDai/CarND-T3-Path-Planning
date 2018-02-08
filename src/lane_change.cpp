/******************************************************************************
| lane_change.cpp                                                             |
| ...                                                                         |
******************************************************************************/

#include "lane_change.h"

#include <iostream>

#define DEBUG
#define DEBUG_COST

using namespace std;

LaneChange::LaneChange(){
  distance_buffer = 0.0; // meters, fixed distance of safety past a time gap
  time_gap = 1.0;        // seconds,

  dt = 0.05;  // time delta for summing integral costs

  k_j = 2.0; // Coeff for jerk cost
  k_t = 2.0; // Coeff for time cost
  k_s = 1.0; // Coeff for lat movement cost
  k_d = 0.1; // Coeff for lon movement cost

  k_lon = 0.5; // weight of lon costs
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

  // If our lane is going at the max speed, we really don't need to change
  double speed_limit = r.speed_limit;
  int follow_id = o.vehicle_to_follow();
  if(follow_id == -1) return;

  // Otherwise, grab the vehicle object we're following
  Obstacle following = o.get_vehicle(follow_id);
  if(following.s_dot >= speed_limit) return;

  // Don't even look into changing a lane if we're not that close
  // TO-DO: This is a band-aid, heal this pls
  if((following.s - si) > 40) return;

  // Define constraint ranges
  double target_T = 1.5; // seconds
  double dT = 0.5;
  double min_T = target_T - 1.0 * dT;
  double max_T = target_T + 6.0 * dT;

  int left = current_lane - 1;
  int right = current_lane + 1;

  #ifdef DEBUG
  cout << " [*] Trying " << 2.0 * ((max_T - min_T) / dT) << " combinations" << endl;
  cout << " [-] Varying given:" << endl
       << "   - T = " << min_T << " and T = " << max_T << endl
       << "   - Right: " << right << endl
       << "   - Left: " << left << endl
       << endl;
  #endif

  #ifdef DEBUG
  cout << " [-] Currently following 'Obstacle ID " << follow_id << "' at s = "
       << following.s << " (Ego S = " << si << ")"
       << endl;
  #endif

  #ifdef DEBUG_COST
  static double max_c = 0.0;
  static double min_c = 100000000.0;
  static double avg_c = 0.0;
  static int N = 0;
  #endif

  // vars for maintaining lane obstacle status
  int cur_id, prev_id;
  Obstacle cur;
  double cur_s, cur_s_dot, cur_s_dot_dot, prev_s, prev_s_dot, prev_s_dot_dot;

  // vars for saving state targets
  double target_d, target_s, target_s_dot, target_s_dot_dot;

  // Generate trajectories for each time, for lane to right and left
  for(double T = min_T; T <= max_T; T += dT)
  {
    // Check right
    if(right < r.lane_count) // && !(left >= 0 && o.m_lanes[left].size() == 0))
    {
      #ifdef DEBUG
      cout << " [-] RIGHT LANE CHECKS" << endl;
      cout << " [+] LANE SPEED: " << o.lane_speed(right) << endl;
      #endif

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
             << "   - Lane: Right" << endl
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
        double c = cost(traj, target_s, target_d, speed_limit, cur_s);
        traj.cost = c;

        // Add traj to our set of possible trajectories
        insert_traj_sorted(t_set, traj);

        // no looking very far forwards
        if(target_s >= si) break;

        #ifdef DEBUG_COST
        if(c < min_c) min_c = c;
        if(c > max_c) max_c = c;
        N++;
        avg_c = ((avg_c * (N - 1)) + c) / N;
        #endif
      }

      // Finally, see if we can pull in front of the leading car in the lane
      if(o.m_lanes[right].empty() || o.get_vehicle(o.m_lanes[right].back()).s < si)
      {
        #ifdef DEBUG
        cout << " [-] Trying T = " << T << ":" << endl
             << "   - Lane: Right (to very front)" << endl
             << "   - Merge Front: " << prev_id << endl
             << "   - Merge Behind: " << -1 << endl
             << "   - target_d: " << target_d << endl
             << "   - target_s_dot: " << speed_limit << endl;
        #endif

        JMT s_path = JMT({si, si_dot, si_dot_dot}, speed_limit, T);
        JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);
        Trajectory traj = Trajectory(name(), s_path, d_path, T);
        double c = cost2(traj, speed_limit, target_d, speed_limit);
        traj.cost = c;
        insert_traj_sorted(t_set, traj);

        #ifdef DEBUG_COST
        if(c < min_c) min_c = c;
        if(c > max_c) max_c = c;
        N++;
        avg_c = ((avg_c * (N - 1)) + c) / N;
        #endif
      }
    }

    // Check left
    if(left >= 0) // && !(right < r.lane_count && o.m_lanes[right].size() == 0))
    {
      #ifdef DEBUG
      cout << " [-] LEFT LANE CHECKS" << endl;
      cout << " [+] LANE SPEED: " << o.lane_speed(left) << endl;
      #endif

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
             << "   - Lane: Left" << endl
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

        // no looking backwards
        if(target_s < si) continue;

        // Vary T and v to generate a bunch of possible s paths
        JMT s_path = JMT({si, si_dot, si_dot_dot}, {target_s, target_s_dot, target_s_dot_dot}, T);

        // Vary just T to generate a matching d path.
        JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);

        // Turn our JMTs into a full blown trajectory
        Trajectory traj = Trajectory(name(), s_path, d_path, T);

        // Get the cost of the trajectory, set it
        double c = cost(traj, target_s, target_d, speed_limit, cur_s);
        traj.cost = c;

        // Add traj to our set of possible trajectories
        insert_traj_sorted(t_set, traj);

        // no looking very far forwards
        if(target_s >= si) break;

        #ifdef DEBUG_COST
        if(c < min_c) min_c = c;
        if(c > max_c) max_c = c;
        N++;
        avg_c = ((avg_c * (N - 1)) + c) / N;
        #endif
      }

      // Finally, see if we can pull in front of the leading car in the lane
      if(o.m_lanes[left].empty() || o.get_vehicle(o.m_lanes[left].back()).s < si)
      {
        #ifdef DEBUG
        cout << " [-] Trying T = " << T << ":" << endl
             << "   - Lane: Left (to very front)" << endl
             << "   - Merge in Front of: " << prev_id << endl
             << "   - Merge in Behind of: " << -1 << endl
             << "   - target_d: " << target_d << endl
             << "   - target_s_dot: " << speed_limit << endl;
        #endif

        JMT s_path = JMT({si, si_dot, si_dot_dot}, speed_limit, T);
        JMT d_path = JMT({di, di_dot, di_dot_dot}, {target_d, 0.0, 0.0}, T);
        Trajectory traj = Trajectory(name(), s_path, d_path, T);
        double c = cost2(traj, speed_limit, target_d, speed_limit);
        traj.cost = c;
        insert_traj_sorted(t_set, traj);

        #ifdef DEBUG_COST
        if(c < min_c) min_c = c;
        if(c > max_c) max_c = c;
        N++;
        avg_c = ((avg_c * (N - 1)) + c) / N;
        #endif
      }
    }
  }

  #ifdef DEBUG_COST
  cout << " [*] Behavior '" << name() << "' cost stats:" << endl
       << "   - Average: " << avg_c << endl
       << "   - max: " << max_c << endl
       << "   - min: " << min_c << endl;
  #endif
}

// Cost function for trajectories that were created by merging in between two cars
double LaneChange::cost(const Trajectory &traj, const double &target_s, const double &target_d, const double &target_s_dot, const double &follow_sf) const {

  // Difference between target speed and final speed
  double sf = traj.s.get_position_at(traj.T);
  double s_delta_2 = (sf - target_s) * (sf - target_s);

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
    A_t_lat += 0.02 * traj.s.get_acceleration_at(t) * traj.s.get_acceleration_at(t);
    A_t_lon += 0.02 * traj.d.get_acceleration_at(t) * traj.d.get_acceleration_at(t);

    // Penalize Jerk over the trajectories
    J_t_lat += traj.s.get_jerk_at(t) * traj.s.get_jerk_at(t);
    J_t_lon += traj.d.get_jerk_at(t) * traj.d.get_jerk_at(t);
  }

  // Massive penalty for changing lanes when theres not much of an
  // advantage
  double si_dot = traj.s.get_velocity_at(0); // speed we WERE going
  double sf_dot = traj.s.get_velocity_at(traj.T);
  double C_lane_speed_adv = 0.0;
  if(abs(sf_dot - si_dot) < 2.0) C_lane_speed_adv = 10000.0;
  // else
  //   C_lane_speed_adv = 1.0 / ((sf_dot - si_dot) * (sf_dot - si_dot));

  // Penalty for being far from the target speed
  double C_speed_limit = (sf_dot - target_s_dot) * (sf_dot - target_s_dot);

  // Penalty for merging into someone where the car you're following is
  // close - only for this guy since the other cost assumes theres no
  // one to follow
  double C_follow_dist = 2000.0 / ((sf - follow_sf) * (sf - follow_sf));

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
  double C_extra = (A_t_lat + A_t_lon) + C_speed_limit + C_lane_speed_adv + C_follow_dist;

  #ifdef DEBUG_COST
  cout << " [*] Cost Breakdown:" << endl
       << "   - sf_dot: " << sf_dot << endl
       << "   - time: " << traj.T << endl
       << "   - C_lat: " << k_lat * C_lat << endl
       << "     - J_lat: " << (k_j * J_t_lat) << endl
       << "   - C_lon: " << k_lon * C_lon << endl
       << "     - J_lon: " << (k_j * J_t_lon) << endl
       << "   - C_time: " << (k_t * traj.T) << endl
       << "   - C_extra: " << C_extra << endl
       << "     - A_Lat: " << A_t_lat << endl
       << "     - A_Lon: " << A_t_lon << endl
       << "     - A_comb: " <<(A_t_lat + A_t_lon) << endl
       << "     - C_lane_speed_adv: " << C_lane_speed_adv << endl
       << "     - C_speed_limit: " << C_speed_limit << endl
       << "     - C_follow_dist: " << C_follow_dist << endl
       << "   - TOTAL: " << k_lat * C_lat + k_lon * C_lon + C_extra << endl;
  #endif

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_extra;
}

// Cost function for trajectories that were created by choosing a target
// velocity and time
double LaneChange::cost2(const Trajectory &traj, const double &target_speed, const double &target_d, const double &speed_limit) const {

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
    A_t_lat += 0.02 * traj.s.get_acceleration_at(t) * traj.s.get_acceleration_at(t);
    A_t_lon += 0.02 * traj.d.get_acceleration_at(t) * traj.d.get_acceleration_at(t);

    J_t_lat += traj.s.get_jerk_at(t) * traj.s.get_jerk_at(t);
    J_t_lon += traj.d.get_jerk_at(t) * traj.d.get_jerk_at(t);
  }

  // Massive penalty for changing lanes when theres not much of an
  // advantage - this penalty SHOULD make us want to follow more and
  // change lanes less.
  double si_dot = traj.s.get_velocity_at(0); // speed we WERE going
  double C_lane_speed_adv = 0.0;
  if(abs(sf_dot - si_dot) < 2.0) C_lane_speed_adv = 10000.0;

  // Penalize driving slower than the speed limit to try to encourage
  // our ego to change lanes into the fastest lane possible
  // MIN: 0, MAX: 50*50 --> 2500
  double C_speed_limit = 1.5 * (sf_dot - speed_limit) * (sf_dot - speed_limit);

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
  double C_extra = (A_t_lat + A_t_lon) + C_speed_limit + C_lane_speed_adv;

  #ifdef DEBUG_COST
  cout << " [*] Cost 2 Breakdown:" << endl
       << "   - sf_dot: " << sf_dot << endl
       << "   - time: " << traj.T << endl
       << "   - C_lat: " << k_lat * C_lat << endl
       << "     - J_lat: " << (k_j * J_t_lat) << endl
       << "   - C_lon: " << k_lon * C_lon << endl
       << "     - J_lon: " << (k_j * J_t_lon) << endl
       << "   - C_time: " << (k_t * traj.T) << endl
       << "   - C_extra: " << C_extra << endl
       << "     - A_Lat: " << A_t_lat << endl
       << "     - A_Lon: " << A_t_lon << endl
       << "     - A_comb: " << (A_t_lat + A_t_lon) << endl
       << "     - C_lane_speed_adv: " << C_lane_speed_adv << endl
       << "     - C_speed_limit: " << C_speed_limit << endl
       << "   - TOTAL: " << k_lat * C_lat + k_lon * C_lon + C_extra << endl;
  #endif

  // Return the combined trajectory cost
  return k_lat * C_lat + k_lon * C_lon + C_extra;
}