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

// TO-DO: Define a Util header? Put constants and helper functions in there.
// TO-DO: Define a WayPoint class for dealing with all the frenet stuff? Or,
//        put it in the map class. Might be nicer that way since only the
//        map really cares about the way points. Erryone else just wants points
//        to be translated between Frenet and Cartesian

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
                             const Map &m, const double planning_horizon)
{
  // Map data
  mMap = m;

  // Immediate Road data - set speed limit plus that no one is in the lanes
  mRoad = r;
  mRoad.speed_limit = MPH_TO_MPS(mRoad.speed_limit);

  // Planning timing parameters - convert times in seconds to steps
  m_planning_horizon = planning_horizon / TIME_DELTA;

  // Intial pathing status
  m_prev_points_left = 0;
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

  // Set up our FSM for our behaviors
  //
  //                 +--+  * implementation has
  //                 |  |    logic to sometimes
  //                 v  |    limit this to {2}
  //     (LANE CHANGE)--+
  //      ^         ^
  //      |         |
  //      v         v
  // (KEEP)<------>(FOLLOW)
  //
  m_vehicle_behaviors[0]->id = 0;
  m_vehicle_behaviors[1]->id = 1;
  m_vehicle_behaviors[2]->id = 2;
  m_vehicle_behaviors[0]->next_behaviors = {0, 1, 2};
  m_vehicle_behaviors[1]->next_behaviors = {0, 1, 2};
  m_vehicle_behaviors[2]->next_behaviors = {0, 1, 2};

  // Keeping is current
  m_current_behavior = 0;
  m_vehicle_behaviors[0]->current = true;

  // Initial Ego car state
  mVeh = initial_status;

  // Initial reference lane is whatever we're in
  m_reference_lane = mRoad.get_vehicle_lane(mVeh);
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

// Get the next behavior from our set of behaviors, given our current one
BehaviorSet VirtualDriver::get_next_behaviors()
{
  // If we haven't initialized a behavior, anything goes! <-- shouldnt happen
  if(m_current_behavior == -1) return {0, 1, 2};

  // Otherwise, the behaviors store their own next states
  else return m_vehicle_behaviors[m_current_behavior]->get_next_behaviors(mVeh.s, mVeh.d, mRoad, m_reference_lane);
}

// Calculate trajectories for an action, given where we are now and whats
// around us
TrajectorySet VirtualDriver::generate_trajectories()
{
  // Trajectory set to return
  TrajectorySet n_possible_trajectories;

  // Path State Parameters for JMT generation
  double si, si_dot, si_dot_dot;
  double di, di_dot, di_dot_dot;

  // Calculate an index based on how many dots have been eaten
  // and how many we've decided to consistently plan out to.
  // We can pull our initial status off this
  int start_ind = (m_planning_horizon - m_prev_points_left - 1);

  // Set the start time based on our start index
  double start_time = start_ind * TIME_DELTA;

  // Initial take off state based on current trajectory
  // NOTE: These will be overwritten below if there's no existing path
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

  // Watch out for start states that wrap around the map tile we've got
  // Velocity and acceleration shouldn't matter but position certainly does
  if(si >= mMap.max_s) si -= mMap.max_s;

  // I've seen weird cases where I might collide with a car and get a negative
  // velocity. The simulator doesn't seem to handle this very well at all. I'm
  // gonna cap the speed to positive for now, cant be worse than whats already
  // going on.
  if(si_dot < 0) si_dot = 0;

  #ifdef DEBUG
  std::cout << " [*] Starting Parameters:" << std::endl
            << "   - si:    " << si << std::endl
            << "   - si_d:  " << si_dot << std::endl
            << "   - si_dd: " << si_dot_dot << std::endl
            << "   - di:    " << di << std::endl
            << "   - di_d:  " << di_dot << std::endl
            << "   - di_dd: " << di_dot_dot << std::endl
            << "   - reference_lane: " << m_reference_lane << std::endl
            << "   - speed_limit: " << mRoad.speed_limit << " m/s" << std::endl;
  #endif

  // Get our next possible behaviors from our current one
  BehaviorSet next_behaviors = get_next_behaviors();

  // With our current set of starting conditions and some final conditions set
  // we can now vary parameters to generate a set of possible trajectories we
  // can follow.
  int count = -1;
  for(int i = 0; i < next_behaviors.size(); ++i)
  {
    int j = next_behaviors[i];

    #ifdef DEBUG
    cout << " [*] Behavior (ID " << j << "): "
         << m_vehicle_behaviors[j]->name()
         << endl;
    #endif

    count = m_vehicle_behaviors[j]->add_trajectories(n_possible_trajectories,
                                                   si, si_dot, si_dot_dot,
                                                   di, di_dot, di_dot_dot,
                                                   m_reference_lane, mRoad,
                                                   m_tracker);

    #ifdef DEBUG
    std::cout << " [+] Added " << count << " for '"
              << m_vehicle_behaviors[j]->name() << "'" << std::endl;
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

  // For averaging velocity over a window/gating the acceleration
  double last_x2 = 0.0, last_y2 = 0.0, last_x = 0.0, last_y = 0.0, last_v_avg = 0.0, v_avg = 0.0;
  int window_size = 10;
  vector<double> vels = vector<double>(window_size, 0.0);
  int v_count = 0;
  double MAX_ACC_MAG = 9.9;

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

    // our "last" position is the front of the list/oldest point
    last_x = (*it).x;
    last_y = (*it).y;
    ++it;

    #ifdef DEBUG
    cout << " [&] Visiting old, past points:" << endl;
    cout << "   - (" << last_x << ", " << last_y << ") -- v = ? -- a = ?"
         << endl;
    #endif

    // NOTE: I'm using 'i' below to see how many velocities we've seen
    // but the loop termination condition is based on the above
    // iterator
    for(int i = 1; it != m_last_followed_points.end(); ++it, ++i)
    {
      double dist = distance((*it).x, (*it).y, last_x, last_y);
      double v =  dist / TIME_DELTA;

      #ifdef DEBUG
      cout << "   - (" << (*it).x << ", " << (*it).y << ") -- v = " << v;
      #endif

      // Add to our set of velocities for calculating the average
      // velocity and acceleration
      vels[v_count % window_size] = v;
      ++v_count;

      // If we have enough velocities seen, get the average velocity
      // over the window we're watching
      if(v_count >= window_size)
      {
        last_v_avg = v_avg;
        v_avg = 0.0;
        for(int j = 0; j < window_size; ++j) v_avg += vels[j];
        v_avg = v_avg / window_size;

        #ifdef DEBUG
        cout << " -- v_avg = " << v_avg;
        #endif
      }

      #ifdef DEBUG
      else cout << " -- v_avg = ?";
      #endif


      #ifdef DEBUG
      // If we haven't seen 2 average velocities then we cant have a real value
      // for acceleration, so only calc acceleration when i >= 2
      if(v_count > window_size)
      {
        // Get the acceleration in the direction of the vehicle
        double accT = (v_avg - last_v_avg) / TIME_DELTA;

        // Get an estimate of the radius of curvature for the acceleration
        // created due to turning
        // roc = ((1 + (y'(x))^2)^1.5) / | y''(x) |
        // y'(x) --> velocity at x
        // y'(x) --> acceleration at x
        // double r_o_c = pow((1 + (v_avg * v_Avg)), 1.5) / abs(accT);
        Circle c = Circle({last_x2, last_y2}, {last_x, last_y}, {p.x[i], p.y[i]});
        double r_o_c = c.radius();
        double accN = (v_avg * v_avg) / r_o_c;
        double a = sqrt(accN*accN + accT*accT);
        cout << " -- accT = " << accT << "-- accN = " << accN << " -- a = " << a << endl;
      }
      else cout << " accT = ? -- accN = ? -- a = ?" << endl;
      #endif

      // Update our last know values
      last_x2 = last_x;
      last_y2 = last_y;
      last_x = (*it).x;
      last_y = (*it).y;
    }
  }
  else
  {
    // Once we've potentially looked at the previous points and filled in
    // the acceleration window, we're now at the beginning of the path/our
    // current status
    last_x2 = last_x;
    last_y2 = last_y;
    last_x = mVeh.x;
    last_y = mVeh.y;
  }

  #ifdef DEBUG
  cout << " [&] Velocities Seen: " << v_count << endl;
  cout << " [&] Last Avg V: " << last_v_avg << endl;
  cout << " [&] Avg V: " << v_avg << endl;
  #endif

  #ifdef DEBUG
  cout << " [&] Starting at (" << last_x << ", " << last_y << ")" << endl;
  #endif

  // Because we have a reliable last_v, we can start at 0 (our current
  // position) to get another acceleation value to add to the window.
  // However, We don't really want to throw something out using the
  // 0th point because we're already here and that would be silly.
  for(int i = 1; i < p.size(); ++i)
  {
    // Get the instantanious velocity and accelerations
    double dist = distance(p.x[i], p.y[i], last_x, last_y);
    double v =  dist / TIME_DELTA;

    #ifdef DEBUG
    cout << " [&] Evaluated " << i -1 << " to " << i << "(" << p.x[i] << ", " << p.y[i] << ") -- v = " << v << endl;
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
                << "   - v = " << v << std::endl
                << "   - type: " << traj.behavior << std::endl
                << "   - s: " << traj.s << std::endl
                << "   - d: " << traj.d << std::endl
                << "   - cost: " << traj.cost << std::endl
                << "   - T: " << traj.T << std::endl;
      #endif
      return false;
    }

    // Add to our set of velocities for calculating the average
    // velocity and acceleration
    vels[v_count % window_size] = v;
    ++v_count;

    // If we have enough velocities seen, get the average velocity
    // over the window we're watching
    if(v_count >= window_size)
    {
      last_v_avg = v_avg;
      v_avg = 0.0;
      for(int j = 0; j < window_size; ++j) v_avg += vels[j];
      v_avg = v_avg / window_size;
    }

    // If we've seen more than one averaged velocity then we can
    // get a acceleration
    if(v_count > window_size)
    {
      // Get the acceleration in the direction of the vehicle
      double accT = (v_avg - last_v_avg) / (TIME_DELTA * window_size);

      // Get the acceleration due to turning
      double accN = 0.0;

      // Get an estimate of the radius of curvature for the acceleration
      // created due to turning
      // OLD METHOD:
      // roc = ((1 + (y'(x))^2)^1.5) / | y''(x) |
      // y'(x) --> velocity at x
      // y'(x) --> acceleration at x
      // double r_o_c = pow((1 + (v_avg * v_avg)), 1.5) / abs(accT);
      // NEW METHOD: Fit a circle and take the radius
      Circle c = Circle({last_x2, last_y2}, {last_x, last_y}, {p.x[i], p.y[i]});
      double r_o_c = c.radius();
      if(r_o_c < 0) r_o_c = 1000000000000.0; // really big number

      #ifdef DEBUG
      cout << " [&] ROC: " << r_o_c << endl;
      #endif

      accN = (v_avg * v_avg) / r_o_c;

      // Total acceleration
      double a = sqrt(accN*accN + accT*accT);

      #ifdef DEBUG
      cout << " [&] Avg V: " << v_avg << endl;
      cout << " [&] Last Avg V: " << last_v_avg << endl;
      cout << " [&] AccT: " << accT << endl;
      cout << " [&] AccN: " << accN << endl;
      cout << " [&] AccTotal: " << a << endl;
      #endif

      // See if we violate the max acceleration
      if(abs(a) >= MAX_ACC_MAG)
      {
        #ifdef DEBUG
        std::cout << " [*] Rejected for acceleration:" << std::endl
                  << "   - i = " << i << std::endl
                  << "   - last_x = " << last_x << std::endl
                  << "   - last_y = " << last_y << std::endl
                  << "   - x = " << p.x[i] << std::endl
                  << "   - y = " << p.y[i] << std::endl
                  << "   - dist = " << dist << std::endl
                  << "   - v = " << v << std::endl
                  << "   - v_avg = " << v_avg << std::endl
                  << "   - last_v_avg = " << last_v_avg << std::endl
                  << "   - accT = " << accT << std::endl
                  << "   - accN = " << accN << std::endl
                  << "   - acc_total = " << a << std::endl
                  << "   - type: " << traj.behavior << std::endl
                  << "   - s: " << traj.s << std::endl
                  << "   - d: " << traj.d << std::endl
                  << "   - cost: " << traj.cost << std::endl
                  << "   - T: " << traj.T << std::endl;
        #endif
        return false;
      }
    }

    // Update our last used points
    last_x = p.x[i]; last_y = p.y[i];
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
    //if(comfortable(*it)) return *it;
    if(m_tracker.trajectory_is_safe(*it) && comfortable(*it)) return *it;

    #ifdef DEBUG
    std::cout << " [*] Removed Trajectory:" << std::endl
              << "   - type: " << m_vehicle_behaviors[(*it).behavior]->name() << std::endl
              << "   - s: " << (*it).s << std::endl
              << "   - d: " << (*it).d << std::endl
              << "   - cost: " << (*it).cost << std::endl
              << "   - T: " << (*it).T << std::endl;
    #endif
  }

  #ifdef DEBUG
  cout << "SHIT FUCK AHHHH DEFAULT TO THE LAST ONE?" << endl;
  #endif

  // Default to the last path we had I guess?
  // TO-DO: What the heck do people do in real life here? Emergency stop?
  Trajectory def = Trajectory(m_current_behavior, m_cur_s_coeffs, m_cur_d_coeffs, 0);

  return def;
}

/******************************************************************************
* Path Output                                                                 *
******************************************************************************/

// Turn a valid, vetted trajectory into a followable path

// Convert a Trajectory into an actual followable set of way points
Path VirtualDriver::generate_path(const Trajectory &traj)
{
  // To contain points for the final path object
  std::vector<double> xpts;
  std::vector<double> ypts;
  double s, d;

  for(int i = 0; i < m_planning_horizon; ++i)
  {
    s = traj.s.at(((double) i) * TIME_DELTA);
    d = traj.d.at(((double) i) * TIME_DELTA);

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
// NOTE: Currently Unused
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

// Convert a Trajectory into a smoothed, actual followable path for the simulator
// NOTE: Currently Unused
Path VirtualDriver::generate_smoothed_path_2(const Trajectory &traj)
{
  // Get the normal path - sets of x and y points
  Path p = generate_path(traj);

  // Set of coresponding time values -- We're going to parameterize the X and Y
  // equations and smooth with splines
  std::vector<double> time_set;

  // For perspective shift of points
  double yaw_rad = deg2rad(mVeh.yaw);

  // Define our smoother splines and smoothing parameters
  std::vector<double> xpts, ypts;
  int smooth_interval = 5;               // Rate to add points to the spline
  int smooth_window = 0;                  // Window (+/-) the point to add
  int smooth_window_step = smooth_window; // Increment size to step through
                                          // the window
  // Add points to our spline smoothers
  for(int i = 0; i < p.size(); ++i)
  {
    if(i % smooth_interval == 0)
    {
      for(int j = i - smooth_window; j <= i + smooth_window; j += smooth_interval)
      {
        // Handle points that wouldn't be in the path already generated
        if(j < 0 || j >= p.size()) continue;

        // Add points in window to splines
        xpts.push_back(p.x[i]);
        ypts.push_back(p.y[i]);
        time_set.push_back((double) j * TIME_DELTA);
      }
    }
  }

  // Create spline smoothers from data sets made above
  tk::spline x_t, y_t;
  x_t.set_points(time_set, xpts);
  y_t.set_points(time_set, ypts);

  // Extract smoothed points for our path
  for(int i = 0; i < p.size(); ++i)
  {
    double t = ((double) i * TIME_DELTA);
    p.x[i] = x_t(t);
    p.y[i] = y_t(t);
  }

  // Return final path object
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
  cout << " [+] Our speed: " << mVeh.speed << " m/s" << endl;
  cout << m_tracker.get_debug_lanes();
  #endif

  #ifdef CLI_OUTPUT
  static int step = 0;
  std::stringstream ss;
  ss << " [+] step: " << step++ << "\n"
     << " [+] planning_horizon: " << m_planning_horizon << "\n"
     << " [+] road speed limit: " << MPS_TO_MPH(mRoad.speed_limit) << " mph | "
     << mRoad.speed_limit << " m/s\n"
     << " [+] road width: " << mRoad.width << "\n"
     << " [+] vehicle (x, y) / (s, d): (" << mVeh.x << ", "
     << mVeh.y << ") / (" << mVeh.s << ", " << mVeh.d << ")\n"
     << " [+] vehicle lane: " << mRoad.get_vehicle_lane(mVeh) << "\n"
     << " [+] vehicle yaw: " << mVeh.yaw << " degrees | "
     << deg2rad(mVeh.yaw) << " rad/s\n"
     << " [+] vehicle speed: " << mVeh.speed << " m/s | "
     << MPS_TO_MPH(mVeh.speed) << " mph" << "\n"
     << " [+] reference lane: " << m_reference_lane << "\n"
     << " [+] m_current_behavior: (" << m_current_behavior << ") "
     << m_vehicle_behaviors[m_current_behavior]->name() << "\n"
     << " [+] Possible Next behaviors:\n";
    BehaviorSet next_behaviors = get_next_behaviors();
    for(int i = 0; i < next_behaviors.size(); ++i)
    {
      if(m_vehicle_behaviors[next_behaviors[i]]->current) ss << "\033[0;32m";
      ss << "    - " << m_vehicle_behaviors[next_behaviors[i]]->name() << "\t";
      if(m_vehicle_behaviors[next_behaviors[i]]->current) ss << "\033[0m";
    }
    ss << endl;
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

  #ifdef CLI_OUTPUT
  bool found = false;
  ss << " [*] Looking at top 96 (of " << possible_trajectories.size() << "):\n";
  ss << "\033[0;31m";
  for(int i = 0; i < 96 && i < possible_trajectories.size(); ++i)
  {
    if(trajs_are_same(possible_trajectories[i], opt))
    {
      ss << "\033[0;32m";
      found = true;
    }
    ss << "   - " << m_vehicle_behaviors[possible_trajectories[i].behavior]->name() << " - "
       << possible_trajectories[i].cost;
    if(found) ss << "\033[0m";
    if((i + 1) % 3 == 0) ss << "\n";
  }
  if(!found) ss << "\033[0m";
  ss << "\n";
  #endif

  #ifdef CLI_OUTPUT
  ss << m_tracker.get_debug_lanes();
  ss << " [*] Behavior: " << m_vehicle_behaviors[opt.behavior]->name() << " (" << opt.cost << ")" << "\n"
     << " [*] Target time: " << opt.T << "\n"
     << " [*] Target Lane: " << mRoad.get_lane(opt.d.at(opt.T)) << "\n"
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
            << "   - type: " << m_vehicle_behaviors[opt.behavior]->name() << std::endl
            << "   - s: " << opt.s << std::endl
            << "   - d: " << opt.d << std::endl
            << "   - cost: " << opt.cost << std::endl
            << "   - T: " << opt.T << std::endl;
  if(mRoad.get_lane(opt.d.at(opt.T)) != m_reference_lane)
  {
    std::cout << "[*] Beginning a lane change from lane "
              << m_reference_lane << " to "
              << mRoad.get_lane(opt.d.at(opt.T)) << endl;
  }
  #endif

  // Clear/Set current behavior
  if(opt.behavior != m_current_behavior)
  {
    for(int i = 0; i < m_vehicle_behaviors.size(); ++i)
    {
      if(i == opt.behavior) m_vehicle_behaviors[i]->current = true;
      else m_vehicle_behaviors[i]->current = false;
    }
    m_current_behavior = opt.behavior;
    m_reference_lane = mRoad.get_lane(opt.d.at(opt.T));
  }

  // Generate the final, follow-able path
  Path p = generate_path(opt);

  #ifdef DEBUG_PATH
  cout << " [^] Final output of path:" << endl;
  for(int i = 0; i < p.size(); ++i)
  {
    cout << "    " << i << " | " << "(" << p.x[i] << ", " << p.y[i] << ")";
    if(i != 0) cout << " -- v = " << (distance(p.x[i-1], p.y[i-1], p.x[i], p.y[i]) / TIME_DELTA) << "m/s";
    else cout << " -- v = ??.???? m/s";
    cout << endl;
  }
  #endif

  // Update our current trajectory, path, and reference lane
  m_last_path = p;
  m_cur_s_coeffs = opt.s;
  m_cur_d_coeffs = opt.d;

  // Return the path we have planned to the motion controller
  return p;
}
