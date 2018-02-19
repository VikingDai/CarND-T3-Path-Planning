#include "obstacle_tracker.h"

#include <iostream>
using namespace std;

// #define DEBUG
#define DEBUG_LITE

// Receive Sensor Fusion Updates
// These come in with an ID for the object and an (x,y), (s,d) and (vx, vy)
// value update. Matching IDs to track objects over time can help our
// predictions greatly. Here I have a map<ID, list<Obstacle> >.
// NOTE: THE @#$% SIMULATOR?! It just gives everything on your side of the road
// and ONLY things on your side of the road, ALWAYS, no matter how close it is.
// It's like I'm a fucking driving god and can see everything, forever and
// always. Plus the car ID's repeat, presumably because the car's wrap around
// the track, in and out of view. What I really need to do is gate myself with
// a "view distance" and just pretend things are coming in and out of view,
// since the sensor fusion system doesn't really do that. THIS WASN'T CLEAR AT
// ALL WHAT THE HECK. <\rant>

/******************************************************************************
| TrackedObstacle                                                             |
******************************************************************************/

// Defaults
TrackedObstacle::TrackedObstacle(){}
TrackedObstacle::~TrackedObstacle(){}

// non-Defaults
TrackedObstacle::TrackedObstacle(Obstacle &obs, const int history_buffer_size)
{
  // Set Tracked Obstacle Id
  this->id = obs.id;

  // Update History with first object
  this->history_buffer_size = history_buffer_size;
  update(obs);
}

// current status off the front
Obstacle TrackedObstacle::current() const {return history.front();}

// Getters for current/instantaneous status
double TrackedObstacle::s() const {return history.front().s;}
double TrackedObstacle::d() const {return history.front().d;}
double TrackedObstacle::s_dot() const {return history.front().s_dot;}
double TrackedObstacle::d_dot() const {return history.front().d_dot;}
double TrackedObstacle::s_dot_dot() const {return history.front().s_dot_dot;}
double TrackedObstacle::d_dot_dot() const {return history.front().d_dot_dot;}

// Update Interface
void TrackedObstacle::update(Obstacle &obs)
{
  // Don't want to mismatch data
  if(obs.id != id) return;

  // If we have older data, calculate instantaneous v for frenet frame
  if(history.size() != 0)
  {
    // Calculate time delta
    double dt = (((double)(obs.t - history.front().t)) / 1000.0);

    // velocities
    obs.s_dot = (obs.s - history.front().s) / dt;
    obs.d_dot = (obs.d - history.front().d) / dt;

    // acceleration
    // if(history.size() > 1)
    //{
    //  obs.s_dot_dot = (obs.s_dot - history.front().s_dot) / dt;
    //  obs.d_dot_dot = (obs.d_dot - history.front().d_dot) / dt;
    //}

    #ifdef DEBUG
    cout << " [+] Updating Object " << id << endl
         << "   - dt: " << dt << endl
         << "   - last: " << history.front().t << endl
         << "   - curr: " << obs.t << endl
         << "   - s: " << obs.s << endl
         << "   - d: " << obs.d << endl
         << "   - s_dot: " << obs.s_dot << endl
         << "   - d_dot: " << obs.d_dot << endl
         << "   - s_dot_dot: " << obs.s_dot_dot << endl
         << "   - d_dot_dot: " << obs.d_dot_dot << endl;
    #endif
  }

  // Add the obstacle status onto the history chain
  history.push_front(obs);
}

/*****************************************************************************\
| Prediction interface                                                        |
\*****************************************************************************/

double TrackedObstacle::s_at(double t) const
{
  if(t == 0) return history.front().s;
  return s() + (s_dot() * t) + (0.5 * s_dot_dot() * t * t);
}

double TrackedObstacle::d_at(double t) const
{
  if(t == 0) return history.front().d;
  return d() + (d_dot() * t) + (0.5 * d_dot_dot() * t * t);
}

double TrackedObstacle::s_dot_at(double t) const
{
  if(t == 0) return history.front().s;
  return s_dot() + s_dot_dot() * t;
}

double TrackedObstacle::d_dot_at(double t) const
{
  if(t == 0) return history.front().d;
  return d_dot() + d_dot_dot() * t;
}

// Debug printing
std::string TrackedObstacle::print() const {return "<TrackedObstacle>";}

/******************************************************************************
| ObstacleTracker                                                             |
******************************************************************************/

// Default Constructor/Destructor
ObstacleTracker::ObstacleTracker(){}
ObstacleTracker::~ObstacleTracker(){}

// Non-Default Constructor/Destructor
ObstacleTracker::ObstacleTracker(const double view_distance, const int tracking_buffer_size = 25)
{
  m_view_distance = view_distance;
  m_tracking_buffer_max = tracking_buffer_size;
}

// Get lane speed
double ObstacleTracker::lane_speed(const int &lane_num)
{
  int follow_id = vehicle_to_follow(lane_num);
  if(follow_id == -1) return m_road.speed_limit;
  double l_s = m_obstacles[follow_id].current().speed; // this is in m/s
  return (l_s > m_road.speed_limit ? m_road.speed_limit : l_s);
}

void ObstacleTracker::update(const long &ts, std::vector<Obstacle> &obs, const Vehicle &veh, const Road &road)
{
  #ifdef DEBUG
  cout << " [*] Ingesting Sensor Fusion Updates" << endl;
  #endif

  // Update internal status
  m_veh = veh;
  m_road = road;

  // Clear out our lane status
  m_lanes.clear();
  m_lanes = std::vector<std::list<int>>(m_road.lane_count, std::list<int>());

  // Stack subsequent tracking points in order based on ID, incoming values
  // are "fresh" from the constructor
  for(int i = 0; i < obs.size(); ++i)
  {
    // Should the object be in view? Just does s
    // TO-DO: Maybe make this a radius?
    if(obs[i].s < (m_veh.s - m_view_distance) || obs[i].s > (m_veh.s + m_view_distance)) continue;

    #ifdef DEBUG
    std::cout << " [+] Update Object ID = " << obs[i].id << " - (s = " << obs[i].s
              << ", d = " << obs[i].d << ", speed = " << obs[i].speed << ")"
              << std::endl;
    #endif

    // Whats this car's lane?
    int olane = m_road.get_vehicle_lane(obs[i]);

    // Insert the car's ID into our lane map
    // if there's no cars yet, add it right awa
    if(m_lanes[olane].size() == 0)
    {
      m_lanes[olane].push_front(obs[i].id);
    }

    // if we're more than the end, add ourselves to the back
    else if(obs[i].s > m_obstacles[m_lanes[olane].back()].s())
    {
      m_lanes[olane].push_back(obs[i].id);
    }

    // otherwise, iterate and find our spot
    else
    {
      for(auto it = m_lanes[olane].begin(); it != m_lanes[olane].end(); ++it)
      {
        if(obs[i].s < m_obstacles[(*it)].s())
        {
          m_lanes[olane].insert(it, obs[i].id);
          break;
        }
      }
    }

    // Update tracked obstacles
    if(m_obstacles.count(obs[i].id) == 0) m_obstacles[obs[i].id] = TrackedObstacle(obs[i], m_tracking_buffer_max);
    else m_obstacles[obs[i].id].update(obs[i]);
  }

  // Remove data thats no longer fresh -- means the car has left view
  for(auto it = m_obstacles.cbegin(); it != m_obstacles.cend();)
  {
    if((*it).second.current().t != ts){
      m_obstacles.erase(it++);
    }
    else {
      #ifdef DEBUG_LITE
      std::cout << " [+] Object ID = " << (*it).second.id << " - ("
                << "s = " << (*it).second.s()
                << ", d = " << (*it).second.d()
                << ", s_dot = " << (*it).second.s_dot()
                << ", d_dot = " << (*it).second.d_dot()
                << ", s_dot_dot = " << (*it).second.s_dot_dot()
                << ", d_dot_dot = " << (*it).second.d_dot_dot()
                << ")" << std::endl;
      #endif

      ++it;
    }
  }
}

std::string ObstacleTracker::get_debug_lanes()
{
  double p_res = 2.0;

  vector<stringstream> lane_strs;
  lane_strs.resize(m_lanes.size());

  int veh_lane = m_road.get_vehicle_lane(m_veh);
  for(int i = 0; i < m_lanes.size(); ++i)
  {
    // keep track of the next veh to draw for the lane
    auto it = m_lanes[i].begin();
    double o_s = 0.0;
    if(it != m_lanes[i].end())
    o_s = m_obstacles[*it].s();

    for(double s = m_veh.s - m_view_distance; s <= m_veh.s + m_view_distance + p_res; s += p_res)
    {
      if(it == m_lanes[i].end() || m_veh.s < o_s)
      {
        if(i == veh_lane && (m_veh.s >= s && m_veh.s < s + p_res))
        {
          lane_strs[i] << "(US)";
        }
        else if(it != m_lanes[i].end() && (o_s >= s && o_s < s + p_res))
        {
          lane_strs[i] << "(" << *it << ")";
          ++it;
          if(it != m_lanes[i].end())
            o_s = m_obstacles[*it].s();
        }
        else
        {
          lane_strs[i] << ".";
        }
      }
      else
      {
        if(it != m_lanes[i].end() && (o_s >= s && o_s < s + p_res))
        {
          lane_strs[i] << "(" << *it << ")";
          ++it;
          if(it != m_lanes[i].end())
            o_s = m_obstacles[*it].s();
        }
        else if(i == veh_lane && (m_veh.s >= s && m_veh.s < s + p_res))
        {
          lane_strs[i] << "(US)";
        }
        else
        {
          lane_strs[i] << ".";
        }
      }
    }
  }

  // Build return value into this
  stringstream s;

  // Obstacle status out
  s << " [*] Obstacle Status:\n";
  for(auto it = m_obstacles.cbegin(); it != m_obstacles.cend(); ++it)
  {
    s << "   - ID: " << (*it).second.id
      << " -- s = " << (*it).second.s()
      << " -- d = " << (*it).second.d()
      << " -- s_dot = " << (*it).second.s_dot()
      << " -- d_dot = " << (*it).second.d_dot() << "\n";
  }

  // Lanes status out
  s << " [*] Lane Status:\n";
  for(int i = 0; i < lane_strs.size(); ++i)
  {
    s << "   - [" << i << ", spd: " << std::setprecision(6) << lane_speed(i) << "]\t"
      << lane_strs[i].str() << "\n";
  }
  return s.str();
}

int ObstacleTracker::vehicle_to_follow()
{
  int lane = m_road.get_vehicle_lane(m_veh);
  if(m_lanes[lane].size() == 0) return -1;
  if(m_veh.s > m_obstacles[m_lanes[lane].back()].s()) return -1;
  for(auto it = m_lanes[lane].begin(); it != m_lanes[lane].end(); ++it)
  {
    if(m_veh.s < m_obstacles[(*it)].s()) return (*it);
  }
  return -1;
}

int ObstacleTracker::vehicle_to_follow(const int &lane_num)
{
  if(m_lanes[lane_num].size() == 0) return -1;
  if(m_veh.s > m_obstacles[m_lanes[lane_num].back()].s()) return -1;
  for(auto it = m_lanes[lane_num].begin(); it != m_lanes[lane_num].end(); ++it)
  {
    if(m_veh.s < m_obstacles[(*it)].s()) return (*it);
  }
  return -1;
}

Obstacle ObstacleTracker::get_vehicle(const int &id)
{
  return m_obstacles[id].current();
}

bool ObstacleTracker::trajectory_is_safe(const Trajectory &traj)
{
  // Cushy safe buffer space so we can keep extra careful
  double BUFFER_S = 4.0;
  double BUFFER_D = 0.75;

  // Get the trajectory d bounds
  double initial_lane = m_road.get_lane(traj.d.at(0));
  double final_lane = m_road.get_lane(traj.d.at(traj.T));
  double min_d = 0.0;
  double max_d = m_road.width;

  // Update d bounds based on the trajectory taking place
  // If initial <= final we can do the following
  // if(initial_lane <= final_lane)
  // {
  //   for(int i = 0; i < initial_lane; ++i)
  //   {
  //     min_d += m_road.lanes[i].width;
  //     max_d += m_road.lanes[i].width;
  //   }
  //   for(int i = initial_lane; i <= final_lane; ++i)
  //   {
  //     max_d += m_road.lanes[i].width;
  //   }
  // }
  //
  // otherwise, we have to do this
  // else
  // {
  //   for(int i = 0; i < final_lane; ++i)
  //   {
  //     min_d += m_road.lanes[i].width;
  //     max_d += m_road.lanes[i].width;
  //   }
  //   for(int i = final_lane; i <= initial_lane; ++i)
  //   {
  //     max_d += m_road.lanes[i].width;
  //   }
  // }

  for(auto it = m_obstacles.begin(); it != m_obstacles.end(); ++it)
  {
    // Get the obstacle
    TrackedObstacle obs_t = (*it).second;

    #ifdef DEBUG
    std::cout << " [+] Checking Trajectory against Obstacle " << obs_t.id
              << ", s = " << obs_t.s()
              << ", d = " << obs_t.d()
              << ", s_dot = " << obs_t.s_dot()
              << ", d_dot = " << obs_t.d_dot()
              << endl;
    #endif

    // Ignore the guy keeping their lane behind us for now
    // TO-DO: Handle this case better
    if(m_road.get_lane(obs_t.d()) == m_road.get_lane(traj.d.at(0)) && obs_t.s() <= traj.s.at(0))
    {
      #ifdef DEBUG
      std::cout << " [+] Its this guys job to follow us... " << endl;
      #endif
      continue;
    }

    // Iterate on the time steps to see if the trajectory is safe
    double dT = 0.1;
    for(double T = 0.0; T <= traj.T; T += dT)
    {
      // Get ego vehicle's trajectory position at T
      double s = traj.s.at(T);
      double d = traj.d.at(T);

      // Dont leave our final destination lane or the sides of the road

      if((d - m_veh.width / 2.0) < (min_d + BUFFER_D) ||
         (d + m_veh.width / 2.0) > (max_d - BUFFER_D))
      {
        #ifdef DEBUG_LITE
        std::cout << "   - LEFT OUR SIDE OF THE ROAD!" << endl;
        #endif
        return false;
      }

      // Get the obstacles position at T
      // ASSUME: constant velocity, and no lane changes
      double obs_s = obs_t.s_at(T);
      double obs_d = obs_t.d();

      #ifdef DEBUG
      std::cout << "  - T = " << T << ": us - "
                << "(" << s << ", " << d << ") | obs - "
                << "(" << obs_s << ", " << obs_d << ")" << endl;
      #endif

      // Do they collide?
      // NOTE: I've seen some weird things were the car behind
      // us going fast might suggest it would rear end us. I'm
      // going to throw out this idea though because thats not
      // reaally our fault if we're already at the speed limit
      // for example.
      if(obs_s >= (s - m_veh.length / 2.0) - BUFFER_S &&
         obs_s <= (s + m_veh.length / 2.0) + BUFFER_S &&
         obs_d >= (d - m_veh.width / 2.0) - BUFFER_D  &&
         obs_d <= (d + m_veh.width / 2.0) + BUFFER_D)
      {
        #ifdef DEBUG_LITE
        std::cout << "  - COLLISION with Object ID " << obs_t.id << endl;
        #endif

        return false;
      }
    }
  }
  return true;
}
