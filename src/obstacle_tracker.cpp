#include "obstacle_tracker.h"

#include <iostream>
using namespace std;
#define DEBUG

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

// Getters for averaged status values
double TrackedObstacle::avg_s() const {return s_avg;}
double TrackedObstacle::avg_d() const {return d_avg;}
double TrackedObstacle::avg_s_dot() const {return s_dot_avg;}
double TrackedObstacle::avg_d_dot() const {return d_dot_avg;}
double TrackedObstacle::avg_s_dot_dot() const {return s_dot_dot_avg;}
double TrackedObstacle::avg_d_dot_dot() const {return d_dot_dot_avg;}

// Update Interface
void TrackedObstacle::update(Obstacle &obs)
{
  // Don't want to mismatch data
  if(obs.id != id) return;

  // If we have older data, calculate instantaneous v and a for frenet frame
  if(history.size() != 0)
  {
    double dt = (((double)(obs.t - history.front().t)) / 1000.0);
    obs.s_dot = (obs.s - history.front().s) / dt;
    obs.d_dot = (obs.d - history.front().d) / dt;
    //obs.s_dot_dot = (obs.s_dot - history.front().s_dot) / dt;
    //obs.d_dot_dot = (obs.d_dot - history.front().d_dot) / dt;

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

  // Adjust running averages
  double N = history.size();
  if(history.size() >= history_buffer_size)
  {
    s_avg = ((s_avg * N) - history.back().s + obs.s) / N;
    d_avg = ((d_avg * N) - history.back().d + obs.s) / N;
    s_dot_avg = ((s_dot_avg * N) - history.back().s_dot + obs.s_dot) / N;
    d_dot_avg = ((d_dot_avg * N) - history.back().d_dot + obs.d_dot) / N;
    s_dot_dot_avg = ((s_dot_dot_avg * N) - history.back().s_dot_dot + obs.s_dot_dot) / N;
    d_dot_dot_avg = ((d_dot_dot_avg * N) - history.back().d_dot_dot + obs.d_dot_dot) / N;
    history.pop_back();
  }
  else
  {
    s_avg = ((s_avg * N) + obs.s) / N;
    d_avg = ((d_avg * N) + obs.d) / N;
    s_dot_avg = ((s_dot_avg * N) + obs.s_dot) / N;
    d_dot_avg = ((d_dot_avg * N) + obs.d_dot) / N;
    s_dot_dot_avg = ((s_dot_dot_avg * N) + obs.s_dot_dot) / N;
    d_dot_dot_avg = ((d_dot_dot_avg * N) + obs.d_dot_dot) / N;
  }

  #ifdef DEBUG
  cout << " [+] Updating Object " << id << " averages" << endl
       << "   - avg_s: " << s_avg << endl
       << "   - avg_d: " << d_avg << endl
       << "   - avg_s_dot: " << s_dot_avg << endl
       << "   - avg_d_dot: " << d_dot_avg << endl
       << "   - avg_s_dot_dot: " << s_dot_dot_avg << endl
       << "   - avg_d_dot_dot: " << d_dot_dot_avg << endl;
  #endif

}

/*****************************************************************************\
| Prediction interface                                                        |
\*****************************************************************************/

double TrackedObstacle::s_at(double t) const
{
  //if(-1.0 * t > history.size()) return history.back().s;
  // if(t < 0) ...
  if(t == 0) return history.front().s;
  return s() + (s_dot() * t) + (0.5 * s_dot_dot() * t * t);
}

double TrackedObstacle::d_at(double t) const
{
  // if(-1.0 * t > history.size()) return history.back().d;
  // if(t < 0) ...
  if(t == 0) return history.front().d;
  return d() + (d_dot() * t) + (0.5 * d_dot_dot() * t * t);
}

double TrackedObstacle::s_dot_at(double t) const
{
  //if(-1.0 * t > history.size()) return history.back().s;
  // if(t < 0) ...
  if(t == 0) return history.front().s;
  return s_dot() + s_dot_dot() * t;
}

double TrackedObstacle::d_dot_at(double t) const
{
  // if(-1.0 * t > history.size()) return history.back().d;
  // if(t < 0) ...
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
      #ifdef DEBUG
      std::cout << " [+] Object ID = " << (*it).second.current().id << " - ("
                << "s = " << (*it).second.current().s
                << ", d = " << (*it).second.current().d
                << ", s_dot = " << (*it).second.current().s_dot
                << ", d_dot = " << (*it).second.current().d_dot
                << ", s_dot_dot = " << (*it).second.current().s_dot_dot
                << ", d_dot_dot = " << (*it).second.current().d_dot_dot
                << ")" << std::endl;
      #endif

      ++it;
    }
  }

  // debug lane status
  #ifdef DEBUG
  std::cout << " [*] Lane Status:" << std::endl;
  int veh_lane = m_road.get_vehicle_lane(m_veh);
  for(int i = 0; i < m_lanes.size(); ++i)
  {
    bool saw = false;
    std::cout << "   - [" << i << ", s: " << lane_speed(i) << "]: . . . ";
    for(auto it = m_lanes[i].begin(); it != m_lanes[i].end(); ++it)
    {
      if(i == veh_lane && !saw && m_veh.s < m_obstacles[*it].s())
      {
        saw = true;
        std::cout << "(E) . . . ";
      }
      std::cout << "(" << *it << ") . . . ";
    }
    if(i == veh_lane && !saw)
        std::cout << "(E) . . . ";
    std::cout << std::endl;
  }
  #endif
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
  double BUFFER_S = 4.0;
  double BUFFER_D = 0.5;

  for(auto it = m_obstacles.begin(); it != m_obstacles.end(); ++it)
  {
    // Get the obstacle
    TrackedObstacle obs_t = (*it).second;

    #ifdef DEBUG
    std::cout << " [+] Checking Trajectory against Obstacle " << obs_t.id << endl;
    #endif

    // Iterate on the time steps to see if the trajectory is safe
    double dT = 0.1;
    for(double T = 0.0; T <= traj.T; T += dT)
    {
      // Get trajectory position at T
      double s = traj.s.at(T);
      double d = traj.d.at(T);

      // Dont leave our side of the road...
      // TO-DO: Un-hardcode 12
      if(d < 0.1 || d > m_road.width - 0.1)
      {
        #ifdef DEBUG
        std::cout << "   - LEFT OUR SIDE OF THE ROAD!" << endl;
        #endif
        return false;
      }

      // Get the obstacles position at T
      double obs_s = obs_t.s_at(T);
      double obs_d = obs_t.d_at(T);

      #ifdef DEBUG
      std::cout << "  - T = " << T << ": "
                << "(" << s << ", " << d << ") | "
                << "(" << obs_s << ", " << obs_d << ")" << endl;
      #endif

      // do they collide?
      if(obs_s >= (s - m_veh.length / 2.0) - BUFFER_S &&
         obs_s <= (s + m_veh.length / 2.0) + BUFFER_S &&
         obs_d >= (d - m_veh.width / 2.0) - BUFFER_D  &&
         obs_d <= (d + m_veh.width / 2.0) + BUFFER_D)
      {
        #ifdef DEBUG
        std::cout << "  - COLLISION!" << endl;
        #endif

        return false;
      }
    }
  }
  return true;
}
