#include <iostream>
#include <uWS/uWS.h>
#include <chrono>
#include <thread>
#include <vector>
#include <cmath>
#include "json.hpp"
#include "virtual_driver.h"
#include "map.h"
#include "lane.h"
#include "road.h"
#include "vehicle.h"
#include "obstacle.h"
#include "path.h"

// TO-DO Have a Debug header that can encapsulate the debug modes and
// pretty print options
// #define DEBUG

// Our initial speed limit. In theory, localization would help update this
// as we move around our world
#define SPEED_LIMIT 48.0

// Constants representing our vehicle. In the real world each car would
// have its own config space that a class would probably look at to set
// itself up
#define VEH_LENGTH 4.47
#define VEH_WIDTH  2.43

using namespace std;
using namespace std::chrono;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s)
{
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) return "";
  else if (b1 != string::npos && b2 != string::npos) return s.substr(b1, b2 - b1 + 2);
  return "";
}

void getSimulatorUpdates(const long &ts, const json &j, Vehicle &v, vector<Obstacle> &o, Path &prev)
{
  // Note: 'j[1]' is the data JSON object
  // Ego car's localization Data
  double car_x = j[1]["x"];
  double car_y = j[1]["y"];
  double car_s = j[1]["s"];
  double car_d = j[1]["d"];
  double car_yaw = j[1]["yaw"];
  double car_speed = j[1]["speed"];

  // Previous Trajectory data given to the Planner
  // This is WHATS LEFT of the path given
  vector<double> prev_x = j[1]["previous_path_x"];
  vector<double> prev_y = j[1]["previous_path_y"];
  prev.x = prev_x;
  prev.y = prev_y;

  // Previous path's end s and d values
  double end_path_s = j[1]["end_path_s"];
  double end_path_d = j[1]["end_path_d"];

  cout << " [?] End (s, d) - (" << end_path_s << ", " << end_path_d << ")" << endl;
  cout << " [?] Car (s, d) - (" << car_s << ", " << car_d << ")" << endl;
  cout << " [?] Car (x, y) - (" << car_x << ", " << car_y << ")" << endl;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];
  vector<Obstacle> obs;
  for(int i = 0; i < sensor_fusion.size(); ++i)
  {
    int id;
    double x, y, s, d, vx, vy;
    id = sensor_fusion[i][0];
    x = sensor_fusion[i][1];
    y = sensor_fusion[i][2];
    vx = sensor_fusion[i][3];
    vy = sensor_fusion[i][4];
    s = sensor_fusion[i][5];
    d = sensor_fusion[i][6];
    obs.push_back(Obstacle(id, ts, s, d, sqrt(vx*vx+vy*vy)));
  }

  // Update those references, woo!
  v = Vehicle(VEH_LENGTH, VEH_WIDTH, car_x, car_y, car_s, car_d, car_speed, car_yaw);
  o = obs;
}

void sendTrajectory(uWS::WebSocket<uWS::SERVER> &ws, const Path &p)
{
  // Encode the response to client
  json msgJson;
  msgJson["next_x"] = p.x;
  msgJson["next_y"] = p.y;
  auto msg = "42[\"control\"," + msgJson.dump() + "]";

  //this_thread::sleep_for(chrono::milliseconds(1000));
  ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
}

int main()
{
  // Web Socket Boiler Plate
  uWS::Hub h;

  // Load the map from file, given our length
  Map m = Map("../data/highway_map.csv", 6945.554);

  // Load our initial road. Our current (only...) settings have
  // each Lane as 4m wide, with 3 lanes in total. The speed limit
  // on this road is 50MPH
  Road r = Road({Lane(4.0), Lane(4.0), Lane(4.0)}, SPEED_LIMIT);

  // Create our vehicle object
  Vehicle v = Vehicle();
  v.length = VEH_LENGTH; // 4.47 meters
  v.width = VEH_WIDTH;   // 2.43 meters

  // We can make an educated guess at initial status since the sim
  // starts the same every time.
  v.d = 6.0;
  v.s = 124.834;
  v.d = 6.16483;
  v.yaw = 0.0;
  v.x = 909.48;
  v.y = 1128.67;

  // Virtual Driver options and object
  double prediction_horizon = 1.50; // 75 steps
  VirtualDriver driver = VirtualDriver(v, r, m, prediction_horizon);

  // What we're gonna do on each message
  h.onMessage([&driver, &m, &r](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode)
  {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2')
    {
      // Check for data
      string s = hasData(data);

      if (s != "")
      {
        // Parse it
        json j = json::parse(s);
        string event = j[0].get<string>();

        // handle it
        if (event == "telemetry")
        {
          // Get the data from the simulator payload
          //   - Updated Vehicle state
          //   - All objects around us
          auto now = time_point_cast<milliseconds>(system_clock::now());
          auto value = now.time_since_epoch();
          long ts = value.count();
          Vehicle v;
          vector<Obstacle> obs;
          Path prev_path;
          getSimulatorUpdates(ts, j, v, obs, prev_path);

          #ifdef DEBUG
          std::cerr << "---------------------------------------------------------" << endl
                    << " [-] Timestep: " << ts << endl;
          #endif

          // Ingest all the incoming data into the Virtual Driver
          //    Sensor Fusion --> What is around me right now?
          //    Localization --> Where am I in the world now?
          // These two things will feed the prediction and behavior planning,
          // which will feed the trajectory planning. We also grab the previous
          // path points to help our algorithm smoothly transition onward
          driver.vehicle_update(v);
          driver.path_history_update(prev_path);
          driver.sensor_fusion_updates(ts, obs);

          // Get the path plan/trajectory from the driver
          // The path is a set of (x,y) points that the car will visit sequentially every .02 seconds
          Path plan = driver.plan_route();

          // Send the response
          sendTrajectory(ws, plan);
        }
      }
      else
      {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data, size_t, size_t)
  {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) res->end(s.data(), s.length());
    else res->end(nullptr, 0);
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,char *message, size_t length)
  {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  // Bind and run
  int port = 4567;
  if (h.listen(port)) std::cout << "Listening to port " << port << std::endl;
  else
  {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
