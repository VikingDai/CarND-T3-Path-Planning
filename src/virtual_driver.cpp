#include "virtual_driver.h"

#include <iostream>
#include <sstream>

using namespace std;

// TO-DO: Define some debug header maybe so it can be cross file?
#define CLI_OUTPUT
#define DEBUG

/******************************************************************************
* Utility Functions and Helpers                                               *
******************************************************************************/

// TO-DO: Define a Util header?
// TO-DO: Define a WayPoint class for dealing with all the frenet stuff

// time between steps
static const double TIME_DELTA = 0.02;

// conversion ratio to/from MPH and M/S
static double MPH_TO_MPS(double mph) {return mph / 2.23694;}
static double MPS_TO_MPH(double ms) {return ms * 2.23694;}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
static double deg2rad(double x) { return x * pi() / 180; }
static double rad2deg(double x) { return x * 180 / pi(); }

static double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

static int ClosestWaypoint(double x, double y, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

  double closestLen = 100000; //large number
  int closestWaypoint = 0;

  for(int i = 0; i < maps_x.size(); i++)
  {
    double map_x = maps_x[i];
    double map_y = maps_y[i];
    double dist = distance(x,y,map_x,map_y);
    if(dist < closestLen)
    {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

static int NextWaypoint(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{

  int closestWaypoint = ClosestWaypoint(x,y,maps_x,maps_y);

  double map_x = maps_x[closestWaypoint];
  double map_y = maps_y[closestWaypoint];

  double heading = atan2((map_y-y),(map_x-x));

  double angle = fabs(theta-heading);
  angle = fmin(2.0*pi() - angle, angle);

  if(angle > pi()/4)
  {
    closestWaypoint++;
    if (closestWaypoint == maps_x.size()) closestWaypoint = 0;
  }

  return closestWaypoint;
}

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
static std::vector<double> getFrenet(double x, double y, double theta, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int next_wp = NextWaypoint(x,y, theta, maps_x, maps_y);

  int prev_wp;
  prev_wp = next_wp - 1;
  if(next_wp == 0) prev_wp  = maps_x.size() - 1;

  double n_x = maps_x[next_wp] - maps_x[prev_wp];
  double n_y = maps_y[next_wp] - maps_y[prev_wp];
  double x_x = x - maps_x[prev_wp];
  double x_y = y - maps_y[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - maps_x[prev_wp];
  double center_y = 2000 - maps_y[prev_wp];
  double centerToPos = distance(center_x, center_y, x_x, x_y);
  double centerToRef = distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) frenet_d *= -1;

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++)
    frenet_s += distance(maps_x[i], maps_y[i], maps_x[i+1], maps_y[i+1]);

  frenet_s += distance(0, 0, proj_x, proj_y);

  return {frenet_s, frenet_d};
}

// Transform from Frenet s,d coordinates to Cartesian x,y
static std::vector<double> getXY(double s, double d, const std::vector<double> &maps_s, const std::vector<double> &maps_x, const std::vector<double> &maps_y)
{
  int prev_wp = -1;

  while(s > maps_s[prev_wp + 1] && (prev_wp < (int)(maps_s.size() - 1) ))
  {
    prev_wp++;
  }

  int wp2 = (prev_wp + 1) % maps_x.size();

  double heading = atan2((maps_y[wp2] - maps_y[prev_wp]), (maps_x[wp2] - maps_x[prev_wp]));

  // the x,y,s along the segment
  double seg_s = (s - maps_s[prev_wp]);
  double seg_x = maps_x[prev_wp] + seg_s * cos(heading);
  double seg_y = maps_y[prev_wp] + seg_s * sin(heading);

  double perp_heading = heading - (pi() / 2.0);

  double x = seg_x + d * cos(perp_heading);
  double y = seg_y + d * sin(perp_heading);

  return {x,y};
}

// Credit to Effendi for suggesting this method and providing a snippet of
// code to make it work. See his full source at:
// https://github.com/edufford/CarND-Path-Planning-Project-P11
std::vector<double> GetHiResXY(double s, double d,
                               const std::vector<double> &map_s,
                               const std::vector<double> &map_x,
                               const std::vector<double> &map_y)
{

  // Wrap around s
  s = std::fmod(s, 6945.554);

  // Find 2 waypoints before s and 2 waypoints after s for angular interpolation
  auto it_wp1_search = std::lower_bound(map_s.begin(), map_s.end(), s);
  int wp1 = (it_wp1_search - map_s.begin() - 1); // wp before s
  int wp2 = (wp1 + 1) % map_s.size(); // wp after s
  int wp3 = (wp2 + 1) % map_s.size(); // wp 2nd after s
  int wp0 = wp1 - 1; // wp 2nd before s
  if (wp0 < 0) { wp0 = map_s.size() - 1; } // wrap around backwards

  // Use angle between wp1-wp2 to derive segment vector at distance s from wp1
  double theta_wp = atan2((map_y[wp2] - map_y[wp1]),
                          (map_x[wp2] - map_x[wp1]));

  // The (x,y,s) along the segment vector between wp1 and wp2
  double seg_s = s - map_s[wp1];
  double seg_x = map_x[wp1] + seg_s * cos(theta_wp);
  double seg_y = map_y[wp1] + seg_s * sin(theta_wp);

  // Interpolate theta at s based on the distance between wp1 (with ave angle
  // from wp0 before and wp2 after) and wp2 (with ave angle from wp1 before
  // and wp3 after)
  double theta_wp1ave = atan2((map_y[wp2] - map_y[wp0]),
                              (map_x[wp2] - map_x[wp0]));

  double theta_wp2ave = atan2((map_y[wp3] - map_y[wp1]),
                              (map_x[wp3] - map_x[wp1]));

  double s_interp = (s - map_s[wp1]) / (map_s[wp2] - map_s[wp1]);

  double cos_interp = ((1-s_interp) * cos(theta_wp1ave)
                         + s_interp * cos(theta_wp2ave));

  double sin_interp = ((1-s_interp) * sin(theta_wp1ave)
                         + s_interp * sin(theta_wp2ave));

  double theta_interp = atan2(sin_interp, cos_interp);

  // Use interpolated theta to calculate final (x,y) at d offset from the
  // segment vector
  double theta_perp = theta_interp - pi()/2;
  double x = seg_x + d * cos(theta_perp);
  double y = seg_y + d * sin(theta_perp);

  return {x, y};
}

/******************************************************************************
* Class Functions                                                             *
******************************************************************************/

// Default Constructor
VirtualDriver::VirtualDriver(){}

// Destructor
VirtualDriver::~VirtualDriver()
{
  for(int i = m_vehicle_behaviors.size() - 1; i >= 0 ; --i)
    delete m_vehicle_behaviors[i];
}

// Non-Default Constructor
VirtualDriver::VirtualDriver(const Vehicle initial_status, const Road &r,
                             const Map &m, const int planning_horizon)
{
  // Map data
  mMap = m;

  // Immediate Road data - set speed limit plus that no one is in the lanes
  mRoad = r;
  mRoad.speed_limit = MPH_TO_MPS(mRoad.speed_limit);

  // Intial pathing status
  m_prev_points_left = 0;
  m_planning_horizon = planning_horizon;
  m_cur_s_coeffs = JMT();
  m_cur_d_coeffs = JMT();
  m_last_followed_window_size = 11;

  // Tracking object
  m_tracker = ObstacleTracker(55, 5);

  // Behaviors this Driver can plan for
  m_vehicle_behaviors = std::vector<Behavior *>(3, NULL);
  m_vehicle_behaviors[0] = new LaneKeep();
  m_vehicle_behaviors[1] = new LaneFollow();
  m_vehicle_behaviors[2] = new LaneChange();

  // Initial Ego car state
  mVeh = initial_status;
}

/******************************************************************************
* Sensor Fusion Updates                                                       *
******************************************************************************/

// Ingest the sensor fusion updates and cascade values to other internal states
// Update First car in front and behind for each lane
void VirtualDriver::sensor_fusion_updates(const long &ts, std::vector<Obstacle> &obs)
{
  m_tracker.update(ts, obs, mVeh, mRoad);
}

/******************************************************************************
* Localization Updates                                                        *
******************************************************************************/

// Localization tells us about where we are. It gives us our vehicle's exact
// location (x/y/s/d) and tells us about our vehicle's state (vx,vy). In a real
// vehicle, the CAN network would be able to report accurate acceleration vals
// for all 6 degrees of freedom as well, but the simulator doesn't report that.
// Localization would also probably be responsible for telling us map updates,
// including landmarks AND road updates (lanes, speed limit, etc). Since these
// things update at different frequencies, I split them up, but they're all
// parts of localization updates

// Update vehicle location and state
// NOTE: Simulator has speeds in MPH, so I convert it here early on
void VirtualDriver::vehicle_update(Vehicle v)
{
  v.speed = MPH_TO_MPS(v.speed);
  v.vx = v.speed*std::cos(v.yaw);
  v.vy = v.speed*std::sin(v.yaw);
  mVeh = v;
}

// Update Road Characteristics
void VirtualDriver::road_update(const Road &r)
{
  mRoad = r;
  mRoad.speed_limit = MPH_TO_MPS(mRoad.speed_limit);
}

// Update the map we're using
void VirtualDriver::map_update(const Map &m){mMap = m;}

// Update the route we we're juuuust following
void VirtualDriver::path_history_update(const Path &prev)
{
  // Grab how many points we have left from our old window
  m_prev_points_left = prev.size();

  #ifdef DEBUG
  cout << " [$] Received path update - We have " << m_prev_points_left
       << " points of the old path left"
       << endl;
  #endif

  // Update our window of followed points given our last path
  int last_ind_followed = m_planning_horizon - m_prev_points_left - 1;

  #ifdef DEBUG
  cout << " [$] From last path we can add on (0," << last_ind_followed << ")"
       << endl;
  #endif

  for(int i = 0; i <= last_ind_followed && i < m_last_path.size(); ++i)
  {
    m_last_followed_points.push_back(Point(m_last_path.x[i], m_last_path.y[i]));
    if(m_last_followed_points.size() > m_last_followed_window_size)
    {
      m_last_followed_points.pop_front();
    }
  }

  #ifdef DEBUG
  cout << " [$] Last " << m_last_followed_window_size << " points followed window:\n";
  for(auto it = m_last_followed_points.begin(); it != m_last_followed_points.end(); ++it)
  {
    cout << "   - (" << (*it).x << ", " << (*it).y << ")\n";
  }
  cout << flush;
  #endif
}

/******************************************************************************
* Trajectory Pool Generation                                                  *
******************************************************************************/

// Given were we are and whats around us (AND where we think those things will
// go), we can choose what we think we should do. This is effectively our
// driving brain thats telling the system what manuevers to make and how to go
// through traffic.

// The Behavior Planner is tightly coupled with the Trajectory planner. Here,
// the trajectory planner does a lot of heavy lifting to generate a bunch of
// candidate trajectories that can be evaluated for optimality later by the
// behavior planner. Its a bit backwards compared to some of the lessons, but
// the lines are super blurred here because of how closely these layers work
// together

// Calculate trajectories for an action, given where we are now and whats
// around us
TrajectorySet VirtualDriver::generate_trajectories()
{
  // Trajectory set to return
  TrajectorySet n_possible_trajectories;

  // Path State Parameters for JMT generation
  double si, si_dot, si_dot_dot;
  double di, di_dot, di_dot_dot;

  // Useful Vehicle State vars cached
  int current_lane = mRoad.get_vehicle_lane(mVeh);

  // Calculate an index based on how many dots have been eaten
  // and how many we've decided to consistently plan out to
  int start_ind = m_planning_horizon - m_prev_points_left - 1;
  double start_time = start_ind * TIME_DELTA;

  // Initial take off state based on current trajectory
  // NOTE: These will be overwritten if there's no existing path
  si = m_cur_s_coeffs.get_position_at(start_time);
  si_dot = m_cur_s_coeffs.get_velocity_at(start_time);
  si_dot_dot = m_cur_s_coeffs.get_acceleration_at(start_time);
  di = m_cur_d_coeffs.get_position_at(start_time);
  di_dot = m_cur_d_coeffs.get_velocity_at(start_time);
  di_dot_dot = m_cur_d_coeffs.get_acceleration_at(start_time);

  // If we don't have a previous state, override the normal default values
  // to something more sensible based on what we know the state of the car
  // to be. Most likely cases of this are 1) start up, or 2) long latency
  // between updates. This may be more accurate than the previous trajectory
  // we know of
  if(m_prev_points_left == 0)
  {
    // Special initial take off state
    si = mVeh.s;
    si_dot = 0;
    si_dot_dot = 0;
    di = mVeh.d;
    di_dot = 0;
    di_dot_dot = 0;
  }

  #ifdef DEBUG
  std::cout << " [*] Starting Parameters:" << std::endl
            << "   - si:    " << si << std::endl
            << "   - si_d:  " << si_dot << std::endl
            << "   - si_dd: " << si_dot_dot << std::endl
            << "   - di:    " << di << std::endl
            << "   - di_d:  " << di_dot << std::endl
            << "   - di_dd: " << di_dot_dot << std::endl
            << "   - current_lane: " << current_lane << std::endl
            << "   - speed_limit: " << mRoad.speed_limit << " m/s" << std::endl;
  #endif

  // With our current set of starting conditions and some final conditions set
  // we can now vary parameters to generate a set of possible trajectories we
  // can follow.
  for(int i = 0; i < m_vehicle_behaviors.size(); ++i)
  {
    #ifdef DEBUG
    std::cout << " [*] Behavior " << i + 1 << ": " << m_vehicle_behaviors[i]->name() << std::endl;
    int t_count = n_possible_trajectories.size();
    #endif

    m_vehicle_behaviors[i]->add_trajectories(n_possible_trajectories,
                                             si, si_dot, si_dot_dot,
                                             di, di_dot, di_dot_dot,
                                             current_lane, mRoad,
                                             m_tracker);

    #ifdef DEBUG
    std::cout << " [+] Added " << n_possible_trajectories.size() - t_count
              << " for '" << m_vehicle_behaviors[i]->name() << "'" << std::endl;
    #endif
  }

  #ifdef DEBUG
  std::cout << " [+] Determined " << n_possible_trajectories.size() << " Possible Trajectories" << std::endl;
  #endif

  return n_possible_trajectories;
}

/******************************************************************************
* Prediction Planning and Behavior Selection                                  *
******************************************************************************/

// The object tracker will maintain the status of the surrounding vehicles and
// lanes. Given our sorted list of possible trajectories, we can probe the
// tracker to tell us the first one that _doesnt_ collide with an obstacle.
// Since behaviors are sorted by lowest cost, we'll take the first one that
// comes back as safe!

// Attempt to hard limit the speeds of the path we choose
bool VirtualDriver::comfortable(Trajectory &traj)
{
  // Get the path from the trajectory that we would be following
  Path p = generate_path(traj);

  // For averaging/gating the acceleration
  double last_x, last_y, last_v;
  int window_size = 10;
  int acc_ins = 0;
  vector<double> accels = vector<double>(window_size, 0.0);
  double avg_a = 0.0;
  double MAX_ACC_MAG = 8.0;

  // Use our last followed points to get acceleration guesses for
  // early points since we're using a rolling, windowed average
  auto it = m_last_followed_points.begin();

  // If we have no points then we can't do this, and if we have only
  // one point then thats not enough to get a velocity to help the
  // start point have an acceleration
  if(it != m_last_followed_points.end() && m_last_followed_points.size() > 1)
  {
    #ifdef DEBUG
    cout << " [&] We have enough points to use to calculate velocities"
         << endl;
    #endif

    // our "last" position is the front of the list/0th point
    last_x = (*it).x;
    last_y = (*it).y;
    last_v = 0.0;
    ++it;

    #ifdef DEBUG
    cout << " [&] Grabbed first point"
         << endl;
    #endif

    // NOTE: I'm using i below to see how many velocities we've seen
    // but the loop terminationo condition is based on the above
    // iterator
    for(int i = 1; it != m_last_followed_points.end(); ++it, ++i)
    {
      double dist = distance((*it).x, (*it).y, last_x, last_y);
      double v =  dist / TIME_DELTA;

      // If we haven't seen 2 velocities then we cant have a real value
      // for acceleration, so only calc acceleration when i >= 2
      if(i >= 2)
      {
        // Insert our acceleration in our window
        accels[acc_ins % window_size] = ((v - last_v) / TIME_DELTA);
        acc_ins++;
      }

      // Update our last know values
      last_x = (*it).x;
      last_y = (*it).y;
      last_v = v;
    }

    // Finally, consider the acceleration at our start point
    accels[acc_ins % window_size] = ((mVeh.speed - last_v) / TIME_DELTA);
    acc_ins++;
  }

  #ifdef DEBUG
  cout << " [&] Filled window with " << acc_ins % window_size << " points"
       << endl;
  std::cout << " [&] Accels Window:\n";
  for(int k = 0; k < window_size; ++k)
    cout << "    - " << accels[k] << endl;
  #endif

  // Once we've potentially looked at the previous points and filled in
  // the acceleration window, we're now at the beginning of the path/our
  // current status
  last_x = mVeh.x;
  last_y = mVeh.y;
  last_v = mVeh.speed;

  #ifdef DEBUG
    cout << " [&] Starting at (" << last_x << ", " << last_y << ") -- v = " << last_v << endl;
    #endif

  // Because we have a reliable last_v, we can start at 0 (our current
  // position) to get another acceleation value to add to the window.
  // However, We don't really want to throw something out using the
  // 0th point because we're already here and that would be silly.
  for(int i = 1; i < p.size(); ++i)
  {
    // Get the instantanious velocity and acceleations
    double dist = distance(p.x[i], p.y[i], last_x, last_y);
    double v =  dist / TIME_DELTA;
    double a = (v - last_v) / TIME_DELTA;

    #ifdef DEBUG
    cout << " [&] Evaluated (" << p.x[i] << ", " << p.y[i] << ") -- v = " << v << " -- a = " << a << endl;
    #endif

    // Always check velocity each step
    if(v >= mRoad.speed_limit)
    {
      #ifdef DEBUG
      std::cout << " [*] Rejected for speed:" << std::endl
                << "   - i = " << i << std::endl
                << "   - last_x = " << last_x << std::endl
                << "   - last_y = " << last_y << std::endl
                << "   - x = " << p.x[i] << std::endl
                << "   - y = " << p.y[i] << std::endl
                << "   - dist = " << dist << std::endl
                << "   - last_v = " << last_v << std::endl
                << "   - v = " << v << std::endl
                << "   - a = " << a << std::endl
                << "   - type: " << traj.behavior << std::endl
                << "   - s: " << traj.s << std::endl
                << "   - d: " << traj.d << std::endl
                << "   - cost: " << traj.cost << std::endl
                << "   - T: " << traj.T << std::endl;
      #endif
      return false;
    }

    // Add the acceleration value to our set of values to avg with
    accels[acc_ins % window_size] = a;
    acc_ins++;

    // If it hasn't been N frames, just keep going, not enough
    // data points yet, otherwise, get that average acceleration
    // and compare to our threshold value
    if(acc_ins >= window_size)
    {
      avg_a = 0;
      for(int j = 0; j < window_size; ++j) avg_a += accels[j];
      avg_a = avg_a / window_size;

      // On initial take off ONLY I get HUGE acceleration outliers
      // so the second half of this is a patch for now :/
      if(abs(avg_a) >= MAX_ACC_MAG && (m_prev_points_left != 0 && acc_ins < window_size))
      {
        #ifdef DEBUG
        std::cout << " [&] Accels Window:\n";
        for(int k = 0; k < window_size; ++k)
          cout << "    - " << accels[k] << std::endl;
        cout << " [&] Accels Sum: " << avg_a * window_size << std::endl;

        std::cout << " [*] Rejected for acceleration:" << std::endl
                  << "   - i = " << i << std::endl
                  << "   - last_x = " << last_x << std::endl
                  << "   - last_y = " << last_y << std::endl
                  << "   - x = " << p.x[i] << std::endl
                  << "   - y = " << p.y[i] << std::endl
                  << "   - dist = " << dist << std::endl
                  << "   - last_v = " << last_v << std::endl
                  << "   - v = " << v << std::endl
                  << "   - a = " << a << std::endl
                  << "   - avg_a = " << avg_a << std::endl
                  << "   - type: " << traj.behavior << std::endl
                  << "   - s: " << traj.s << std::endl
                  << "   - d: " << traj.d << std::endl
                  << "   - cost: " << traj.cost << std::endl
                  << "   - T: " << traj.T << std::endl;
        #endif
        return false;
      }
    }
    last_x = p.x[i]; last_y = p.y[i]; last_v = v;
  }
  return true;
}

// Given a list of sorted (by cost) trajectories, pick the optimal one for us
// to follow
Trajectory VirtualDriver::optimal_trajectory(TrajectorySet &possible_trajectories)
{
  #ifdef DEBUG
  std::cout << " [*] Determining optimal from " << possible_trajectories.size() << " Possible Trajectories" << std::endl;
  #endif

  // find the minimum cost
  Trajectory *opt = NULL;
  for(TrajectorySet::iterator it = possible_trajectories.begin(); it != possible_trajectories.end(); ++it)
  {
    #ifdef DEBUG
    std::cout << " [+] Checking a '" << (*it).behavior << "' of cost " << (*it).cost
              << " and T = " << (*it).T
              << std::endl;
    #endif

    // Make sure the trajectory is safe (no collisions) and
    // comfy (speed, accel and jerk are under requirements)
    if(m_tracker.trajectory_is_safe(*it) && comfortable(*it)) return *it;

    #ifdef DEBUG
    std::cout << " [*] Removed Trajectory:" << std::endl
              << "   - type: " << (*it).behavior << std::endl
              << "   - s: " << (*it).s << std::endl
              << "   - d: " << (*it).d << std::endl
              << "   - cost: " << (*it).cost << std::endl
              << "   - T: " << (*it).T << std::endl;
    #endif
  }

  // Default to the last path we had I guess?
  // TO-DO: What the heck do people do in real life here? Emergency stop?
  Trajectory def = Trajectory(m_cur_s_coeffs, m_cur_d_coeffs, 0);

  return def;
}

/******************************************************************************
* Path Output                                                                 *
******************************************************************************/

// Turn a valid, vetted trajectory into a followable path

// Convert a Trajectory into an actual followable set of way points
Path VirtualDriver::generate_path(const Trajectory &traj)
{
  // Extract JMTs
  JMT jmt_s = traj.s;
  JMT jmt_d = traj.d;

  // How long to plan for and how to step
  double td = TIME_DELTA;
  int n = m_planning_horizon;

  // To contain points for the final path object
  std::vector<double> xpts;
  std::vector<double> ypts;
  double s, d;

  // Generate the path
  for (int i = 0; i < n; ++i)
  {
    s = jmt_s.at(((double) i) * td);
    d = jmt_d.at(((double) i) * td);

    // Check if the (s, _) point falls outside the map, which
    // can happen because the JMT doesn't care!
    if(s > mMap.max_s) s -= mMap.max_s;
    else if(s < 0) s += mMap.max_s;

    std::vector<double> xy = GetHiResXY(s, d, mMap.waypoints_s, mMap.waypoints_x, mMap.waypoints_y);
    xpts.push_back(xy[0]);
    ypts.push_back(xy[1]);
  }

  return Path(xpts, ypts);
}

// Convert a Trajectory into a smoothed, actual followable path for the simulator
Path VirtualDriver::generate_smoothed_path(const Trajectory &traj)
{
  // Get the normal path
  Path p = generate_path(traj);

  // How long to plan for and how to step
  double td = TIME_DELTA;
  int n = m_planning_horizon;

  // Calculate an index based on how many dots have been eaten
  // and how many we've decided to consistently plan out to
  int points_available = m_planning_horizon - m_prev_points_left - 1;
  if(points_available == m_planning_horizon - 1) points_available = 0;
  double t_cur_old_path = points_available * td;

  // Define the window and initial bounds of the smoothing function
  // Note: Smooth window size must be odd
  // Note: By the nature of what we're doing, the first point of our
  // new path should match a point of the old one, so we lose one from
  // the set of possible smoothing points
  // Note: A negative value will dip into the last used points IF they
  // exist. If they don't, oh well. It would look likeeeee:
  // (O = old, N = new, X = point to smooth) --> window = 5
  //           |---X---|
  // O O O O O O O N N N N N N N
  int smooth_size = 15;
  int first = -1 * (smooth_size / 2);
  int nth = first + smooth_size;

  // For perspective shift
  double yaw_rad = deg2rad(mVeh.yaw);

  // Smooth out the beginning of the path
  for(int i = 0; i < m_planning_horizon / 4; ++i)
  {
    // Adjust first/Nth based on existing path points. We have P points
    // and a negative j means dip into the old points. If we dip too far
    // then we just miss a point we can smooth ): Up the ranges
    if(first < 0 && points_available + first < 0)
    {
      ++first;
      ++nth;
      continue;
    }

    // I've gotten weird instances where the path created isn't increasing
    // and this wont work with the spline library. Its fine, we'll just
    // skip over it and smooth out the next point.
    bool inc = true;

    // keep track of the points we want to use to smooth and make the
    // smoothing spline object
    std::vector<double> s_xpts;
    std::vector<double> s_ypts;
    tk::spline smoother;

    // Go over the entire window and all the points except the middle one
    for(int j = first; j < nth; ++j)
    {
      // The point's we'll grab to use
      double __x = 0.0, __y = 0.0;

      // If our first is negative, use the old set of points
      if(j < 0)
      {
        double s = m_cur_s_coeffs.get_position_at((j + points_available) * td);
        double d = m_cur_d_coeffs.get_position_at((j + points_available) * td);

        if(s > mMap.max_s) s -= mMap.max_s;
        else if(s < 0) s += mMap.max_s;

        vector<double> xy = GetHiResXY(s, d, mMap.waypoints_s, mMap.waypoints_x, mMap.waypoints_y);
        __x = xy[0];
        __y = xy[1];
      }
      else if(j != i)
      {
        __x = p.x[j];
        __y = p.y[j];
      }

      // Again, skip the middle point that we're smoothing
      else continue;

      // Covert the points to our perspective, relative to the vehicle
      double _x = __x - mVeh.x;
      double _y = __y - mVeh.y;
      double x = _x * cos(0-yaw_rad) - _y * sin(0-yaw_rad);
      double y = _x * sin(0-yaw_rad) + _y * cos(0-yaw_rad);

      // Here's that weird non-increasing issue...
      if(s_xpts.size() > 0 && x < s_xpts[s_xpts.size() - 1])
      {
        inc = false;
        break;
      }

      // Push them on to our set of points to use with the smoother
      s_xpts.push_back(x);
      s_ypts.push_back(y);
    }

    // If its not increasing, on to the next one.
    if(!inc) continue;

    // Set up the spline
    smoother.set_points(s_xpts, s_ypts);

    // Smooth the middle point out -- convert the x point to be relative
    // to the vehicle again
    double _x = p.x[i] - mVeh.x;
    double _y = p.y[i] - mVeh.y;
    double x = _x * cos(0-yaw_rad) - _y * sin(0-yaw_rad);
    double y = smoother(x);

    // Change y back to proper frame -- relative to the world
    y = x * sin(yaw_rad) + y * cos(yaw_rad) + mVeh.y;

    // Add the "smoothed" point
    p.y[i] = y;
    first++;
    nth++;
  }

  // return our final sets of points as a path
  return p;
}

/******************************************************************************
* Path Planner Pipeline                                                       *
******************************************************************************/

// Path planning is the name used to describe to interaction of the above steps
// and the overall generation of a path given the status of your vehicle, the
// other vehicles, where you are, and what you think the other vehicles will do

// Given the internal state, plan the trajectory
// This will walk through the whole pipeline:
//    - Given sensor fusion data of objects around us, predict where they will
//      be in the future
//    - Use predictions, road status and map waypoints to determine optimal
//      behavior
//    - Given the decided behavior, plan a trajectory to hand to a motion
//      controller
Path VirtualDriver::plan_route()
{
  // Debug state printing
  #ifdef DEBUG
  cout << m_tracker.get_debug_lanes();
  #endif

  #ifdef CLI_OUTPUT
  static int step = 0;
  std::stringstream ss;
  ss << " [+] step: " << step++ << "\n"
     << " [+] road speed limit: " << MPS_TO_MPH(mRoad.speed_limit) << " mph | "
     << mRoad.speed_limit << " m/s\n"
     << " [+] road width: " << mRoad.width << "\n"
     << " [+] vehicle (x, y) / (s, d): (" << mVeh.x << ", "
     << mVeh.y << ") / (" << mVeh.s << ", " << mVeh.d << ")\n"
     << " [+] vehicle lane: " << mRoad.get_vehicle_lane(mVeh) << "\n"
     << " [+] vehicle speed: " << mVeh.speed << " m/s | "
     << MPS_TO_MPH(mVeh.speed) << " mph" << "\n";
  #endif

  // TRAJECTORY POOLING:
  // -------------------
  // Now that we know our lane, and know where the other cars/obstacles are,
  // we can generate a set of possible trajectories to hand over to a
  // behavior/feasiblity checker to weigh and pick from
  std::vector<Trajectory> possible_trajectories = generate_trajectories();

  #ifdef CLI_OUTPUT
  ss << " [*] Looking at top 75:\n";
  for(int i = 0; i < 75 && i < possible_trajectories.size(); ++i)
  {
    ss << "   - " << possible_trajectories[i].behavior << " - "
       << possible_trajectories[i].cost;
    if((i + 1) % 3 == 0) ss << "\n";
  }
  ss << "\n";
  #endif

  // BEHAVIOR PLANNING:
  // ------------------
  // Given our set of possible trajectories, go through it to find the
  // ultimate, optimal trajectory. Then turn that trajectory into a path
  // Note: Since we're working with a simulator, the vehicle updates every .02
  // seconds, and the ultimate distance to be applied needs to be in m/s, so
  // (<dist> / 0.02) * 2.23694 = TARGET_SPEED --> <dist> = (TARGET_SPEED / 2.23694) * 0.02

  // Extract the best one
  Trajectory opt = optimal_trajectory(possible_trajectories);

  #ifdef CLI_OUTPUT
  ss << m_tracker.get_debug_lanes();
  ss << " [*] Behavior: " << opt.behavior << " (" << opt.cost << ")" << "\n"
     << " [*] Target time: " << opt.T << "\n"
     << " [*] Target sf_dot: " << opt.s.get_velocity_at(opt.T) << " m/s | "
     << MPS_TO_MPH(opt.s.get_velocity_at(opt.T)) << " mph\n";

  std::cerr << "\033[?25l" << std::flush; // hide cursor for update
  std::cerr << "\033[1J" << std::flush; // clear entire screen above
  std::cerr << "\033[1;1H" << std::flush; // Move to (1, 1) <-- 1 indexed
  std::cerr << ss.str() << std::flush;
  std::cerr << "\033[?25h" << std::flush; // show cursor again
  #endif

  #ifdef DEBUG
  std::cout << " [*] Optimal Traj Found!" << std::endl
            << "   - type: " << opt.behavior << std::endl
            << "   - s: " << opt.s << std::endl
            << "   - d: " << opt.d << std::endl
            << "   - cost: " << opt.cost << std::endl
            << "   - T: " << opt.T << std::endl;
  #endif

  // Generate the final, follow-able path
  Path p = generate_path(opt);

  #ifdef DEBUG
  cout << " [^] Final output of path:" << endl;
  for(int i = 0; i < p.size(); ++i)
  {
    cout << "    " << i << " | " << "(" << p.x[i] << ", " << p.y[i] << ")" << endl;
  }
  #endif

  // Update our current trajectory and path
  m_last_path = p;
  m_cur_s_coeffs = opt.s;
  m_cur_d_coeffs = opt.d;

  // Return the path we have planned to the motion controller
  return p;
}
