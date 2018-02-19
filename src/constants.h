/******************************************************************************
| Constants                                                                   |
| ...                                                                         |
******************************************************************************/

#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants
{
  //-----------------------------------------------------------------------//
  // Vehicle Attribute Constants                                           //
  //-----------------------------------------------------------------------//

  // Use for determining if theres a car to the side, also used to determine
  // where "front" starts
  constexpr double CAR_LENGTH = 4.47;

  // Use for determining if theres a car to the side, also used to determine
  // where "front" starts
  constexpr double CAR_WIDTH = 2.43;

  //-----------------------------------------------------------------------//
  // Sensor Fusion System Constants                                        //
  //-----------------------------------------------------------------------//

  // The point at which I care about things that are around me.
  // NOTE: I don't really need a 'd' value since theres only a couple lanes and the
  // simulator only hands me things on my side of the road anyways
  constexpr double VIEW_RANGE_S = 30.0; // meters

  // How many data points to in the past to size while tracking a vehicle. Can be
  // tuned based on how well we predict
  constexpr int OBS_TRACK_SIZE = 25;

  //-----------------------------------------------------------------------//
  // Trajectory Planner Constants                                          //
  //-----------------------------------------------------------------------//

  // When I'm keeping lane and following, OR changing lanes and wanted to follow
  // after, this is how close to keep. In real life, this is probably determined
  // by the current speed and how long it would take to come to a complete stop
  // Roughly v / A_MAX --> t to stop?
  constexpr double FOLLOWING_DISTANCE = 10.0; // meters

  // Acceleration amounts for a "comfortable" ride, not necessarily I.C.E.
  constexpr double A_MIN = -10.0; // m/s
  constexpr double A_MAX = 10.0; // m/s

  //-----------------------------------------------------------------------//
  // Path Generation Constants                                             //
  //-----------------------------------------------------------------------//

  // Total number of steps to plan for
  // NOTE: I could base this on time too? Might be easier to
  // set or more meaningful? idk.
  constexpr double PLANNING_HORIZON = 50;

  // time between steps
  constexpr double TIME_DELTA = 0.02;
}

#endif // __CONSTANTS_H__