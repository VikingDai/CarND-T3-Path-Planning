#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include "jmt.h"
#include <list>

class Trajectory {
  public:

    // A trajectory is a set of two JMTs, a time horizon and a cost
    std::string behavior;
    double T;
    JMT s;
    JMT d;
    double cost;

    Trajectory(){};
    Trajectory(const JMT &_s, const JMT &_d, const double &_t, const double &_c = 0){behavior = "?"; s = _s; d = _d; T = _t; cost = _c;};
    Trajectory(const std::string &_b, const JMT &_s, const JMT &_d, const double &_t, const double &_c = 0){behavior = _b; s = _s; d = _d; T = _t; cost = _c;};
    ~Trajectory(){};
};

typedef std::vector<Trajectory> TrajectorySet;

inline void insert_traj_sorted(TrajectorySet &t_set, const Trajectory &traj)
{
  // Iterate on the set and insert it in a sorted order
  // and return. If we make it to the end and break the
  // loop, it'll just go on the end. This also works for
  // the empty case.
  for(auto it = t_set.begin(); it != t_set.end(); ++it)
  {
    if(traj.cost < it->cost)
    {
      t_set.insert(it, traj);
      return;
    }
  }
  t_set.push_back(traj);
};

#endif // __TRAJECTORY_H__