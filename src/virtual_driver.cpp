#include "virtual_driver.h"

#include <iostream>

using namespace std;

// TO-DO: Define some debug header maybe so it can be cross file?
#define DEBUG
#define DEBUG2

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

/******************************************************************************
* Class Functions                                                             *
******************************************************************************/

// Default Constructor
VirtualDriver::VirtualDriver(){}

// Destructor
VirtualDriver::~VirtualDriver(){
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
  #ifdef DEBUG
  std::cout << " [*] Received " << obs.size() << " sensor fusion updates" << std::endl;
  #endif

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
void VirtualDriver::map_update(const Map &m){this->mMap = m;}

// Update the route we we're juuuust following
void VirtualDriver::path_history_update(const Path &prev)
{
  this->m_prev_points_left = prev.size();
  prev_path = prev;
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
  #ifdef DEBUG
  std::cout << " [-] Generating possible trajectories!" << std::endl;
  #endif

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
    #ifdef DEBUG
    std::cout << " [*] Overriding Default Parameters - No points from last known path " << std::endl;
    #endif

    // Special initial take off state
    si = mVeh.s;
    si_dot = 0;
    si_dot_dot = 0;
    di = mVeh.d;
    di_dot = 0;
    di_dot_dot = 0;
  }

  #ifdef DEBUG
  std::cout << " [*] Starting Parameters: " << std::endl
              << "  - si:    " << si << std::endl
              << "  - si_d:  " << si_dot << std::endl
              << "  - si_dd: " << si_dot_dot << std::endl
              << "  - di:    " << di << std::endl
              << "  - di_d:  " << di_dot << std::endl
              << "  - di_dd: " << di_dot_dot << std::endl
              << "  - current_lane: " << current_lane << std::endl
              << "  - speed_limit: " << mRoad.speed_limit << " m/s" << std::endl;
  std::cout << " [*] Determing Possible Trajectories (" << m_vehicle_behaviors.size() << "): " << std::endl;
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
  std::cout << " [+] Determed " << n_possible_trajectories.size() << " Possible Trajectories" << std::endl;
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
  Path p = generate_path(traj);
  double last_x = mVeh.x, last_y = mVeh.y, last_v = mVeh.speed;

  for(int i = 0; i < p.size(); ++i)
  {
    double v = distance(p.x[i], p.y[i], last_x, last_y) / TIME_DELTA;
    if(v >= MPH_TO_MPS(50)) return false;
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
    std::cout << " [+] Checking a '" << (*it).behavior << "' of cost " << (*it).cost << std::endl;
    #endif

    // && comfortable(*it)
    if(m_tracker.trajectory_is_safe(*it)) return *it;
    #ifdef DEBUG
    std::cout << " [*] Removed Trajectory:" << std::endl
              << "   - type: " << (*it).behavior << std::endl
              << "   - s: " << (*it).s << std::endl
              << "   - d: " << (*it).d << std::endl
              << "   - cost: " << (*it).cost << std::endl
              << "   - T: " << (*it).T << std::endl;
    #endif
  }

  // Default to the last path we had I guess
  return Trajectory(m_cur_s_coeffs, m_cur_d_coeffs, 0);
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

  // Calculate an index based on how many dots have been eaten
  // and how many we've decided to consistently plan out to
  int start_ind = m_planning_horizon - m_prev_points_left - 1;
  double start_time = start_ind * TIME_DELTA;

  // To contain points for the final path object
  std::vector<double> xpts;
  std::vector<double> ypts;
  double s, d;

  // Generate the path
  for (int i = 0; i < n; ++i)
  {
    if(start_ind < m_planning_horizon - 1 && i < 5)
    {
      // s = m_cur_s_coeffs.at(start_time + ((double) i) * td);
      // d = m_cur_d_coeffs.at(start_time + ((double) i) * td);
      xpts.push_back(prev_path.x[i]);
      ypts.push_back(prev_path.y[i]);
      continue;
    }
    else
    {
      s = jmt_s.at(((double) i) * td);
      d = jmt_d.at(((double) i) * td);
    }

    // Check if the (s, _) point falls outside the map, which
    // can happen because the JMT doesn't care!
    if(s > mMap.max_s) s -= mMap.max_s;
    else if(s < 0) s += mMap.max_s;

    std::vector<double> xy = getXY(s, d, mMap.waypoints_s, mMap.waypoints_x, mMap.waypoints_y);
    xpts.push_back(xy[0]);
    ypts.push_back(xy[1]);

    #ifdef DEBUG2
    std::cout << "t = " << i*TIME_DELTA << " | (s = " << s << ", d = " << d << ") --> (x = " << xy[0] << ", y = " << xy[1] << ")" << std::endl;
    #endif
  }

  return Path(xpts, ypts);
}

// Convert a Trajectory into a smoothed, actual followable path for the simulator
Path VirtualDriver::generate_smoothed_path(const Trajectory &traj)
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

  // Calculate an index based on how many dots have been eaten
  // and how many we've decided to consistently plan out to
  int start_ind = m_planning_horizon - m_prev_points_left - 1;
  double start_time = start_ind * TIME_DELTA;

  // Generate the smoothed path
  for (int i = 0; i < n; ++i)
  {
    double s = jmt_s.at(((double) i) * td);
    double d = jmt_d.at(((double) i) * td);

    if(start_ind < m_planning_horizon - 1 && i < 5)
    {
      xpts.push_back(prev_path.x[i]);
      ypts.push_back(prev_path.y[i]);
      continue;
    }

    // Check if the (s, _) point falls outside the map, which
    // can happen because the JMT doesn't care!
    if(s > mMap.max_s) s -= mMap.max_s;
    else if(s < 0) s += mMap.max_s;

    std::vector<double> xy = getXY(s, d, mMap.waypoints_s, mMap.waypoints_x, mMap.waypoints_y);
    xpts.push_back(xy[0]);
    ypts.push_back(xy[1]);

    // #ifdef DEBUG2
    // std::cout << "t = " << i*TIME_DELTA << " | (s = " << s << ", d = " << d << ") --> (x = " << xy[0] << ", y = " << xy[1] << ")" << std::endl;
    // #endif
  }

  // Double check the path for smoothness now that we're in the xy space
  int smooth_size = 15;
  int first = 0;
  int nth = smooth_size;

  // For perspective shift
  double yaw_rad = deg2rad(mVeh.yaw);

  for(int i = 0; i < xpts.size() - smooth_size; ++i)
  {
    std::cout << " [-] checking window " << first << " to " << nth << std::endl;

    bool inc = true;
    std::vector<double> s_xpts;
    std::vector<double> s_ypts;
    tk::spline smoother;
    for(int j = first; j < nth; ++j)
    {
      if(j == first + (smooth_size / 2) + 1) continue;
      std::cout << " [-] Adding (" << xpts[j] << ", " << ypts[j] << ")" << std::endl;

      double _x = xpts[j] - mVeh.x;
      double _y = ypts[j] - mVeh.y;
      double x = _x * cos(0-yaw_rad) - _y * sin(0-yaw_rad);
      double y = _x * sin(0-yaw_rad) + _y * cos(0-yaw_rad);

      std::cout << "   - Shifted to (" << x << ", " << y << ")" << std::endl;

      if(x < xpts[xpts.size() - 1]){inc = false; break;}
      s_xpts.push_back(x);
      s_ypts.push_back(y);
    }
    if(!inc) continue;

    // Set up the spline
    smoother.set_points(s_xpts, s_ypts);

    // Smooth the middle point out
    double _x = xpts[first + smooth_size / 2 + 1] - mVeh.x;
    double _y = ypts[first + smooth_size / 2 + 1] - mVeh.y;
    double x = _x * cos(0-yaw_rad) - _y * sin(0-yaw_rad);
    double y = smoother(x);

    // Change y back to proper frame
    y = x * sin(yaw_rad) + y * cos(yaw_rad) + mVeh.y;

    // Apply
    ypts[first + (smooth_size / 2) + 1] = y;
    std::cout << " [-] smoothing to (" << xpts[first + smooth_size / 2 + 1] << ", " << ypts[first + smooth_size / 2 + 1] << ")" << std::endl;

    first++;
    nth++;
  }

  // watch the velocity, acceleration and jerk
  double last_x = mVeh.x, last_y = mVeh.y, last_v = mVeh.speed, last_a = 0;
  double v, a, j;

  #ifdef DEBUG
  for(int i = 0; i < xpts.size(); ++i)
  {
    v = distance(xpts[i], ypts[i], last_x, last_y) / td;
    a = (v - last_v) / td;
    j = (a - last_a) / td;
    std::cout << "t = " << i*TIME_DELTA << " | (x = " << xpts[i] << ", y = " << ypts[0] << ") -- "
              << "v = " << v << ", a = " << a << ", j = " << j << std::endl;
    if(v >= MPH_TO_MPS(50)) std::cout << "HOLY FUCKING SHIT" << std::endl;
    last_x = xpts[i]; last_y = ypts[i]; last_v = v; last_a = a;
  }
  #endif


  return Path(xpts, ypts);
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
  static int step = 0;
  std::cout << " [+] step: " << step++ << std::endl
            << " [+] road width: " << mRoad.width << std::endl
            << " [+] vehicle (x, y) / (s, d): (" << mVeh.x << ", "
            << mVeh.y << ") / (" << mVeh.s << ", " << mVeh.d << ")" << std::endl
            << " [+] vehicle lane: " << mRoad.get_vehicle_lane(mVeh)
            << std::endl
            << " [+] vehicle speed: " << mVeh.speed << " m/s | "
            << MPS_TO_MPH(mVeh.speed) << " mph" << std::endl
            << " [+] road speed limit: " << MPS_TO_MPH(mRoad.speed_limit) << " mph | "
            << mRoad.speed_limit << " m/s"
            << std::endl;
  #endif

  // TRAJECTORY POOLING:
  // -------------------
  // Now that we know our lane, and know where the other cars/obstacles are,
  // we can generate a set of possible trajectories to hand over to a
  // behavior/feasiblity checker to weigh and pick from
  std::vector<Trajectory> possible_trajectories = generate_trajectories();

  // BEHAVIOR PLANNING:
  // ------------------
  // Given our set of possible trajectories, go through it to find the
  // ultimate, optimal trajectory. Then turn that trajectory into a path
  // Note: Since we're working with a simulator, the vehicle updates every .02
  // seconds, and the ultimate distance to be applied needs to be in m/s, so
  // (<dist> / 0.02) * 2.23694 = TARGET_SPEED --> <dist> = (TARGET_SPEED / 2.23694) * 0.02

  // Extract the best one
  Trajectory opt = optimal_trajectory(possible_trajectories);

  #ifdef DEBUG
  std::cout << " [*] Optimal Traj Found! " << std::endl
            << "   - type: " << opt.behavior << std::endl
            << "   - s: " << opt.s << std::endl
            << "   - d: " << opt.d << std::endl
            << "   - cost: " << opt.cost << std::endl
            << "   - T: " << opt.T << std::endl;
  #endif

  // Generate the final, follow-able path
  Path p = generate_path(opt);

  // Update our current trajectory
  m_cur_s_coeffs = opt.s;
  m_cur_d_coeffs = opt.d;

  // Return the path we have planned to the motion controller
  return p;
}
