#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "vehicle.h"
#include "utility.h"

using namespace std;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_first_of("}");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}


// initialization
int lane = 1;
int iterate = 0;
int prev_lane = 1;
int lane_change_elapsed_time = 0;
int deceleration_at = 0;

double ref_val = 0.0;
double prev_ref_val = 0.0;

const double LANE_CHANGE_COST = 600;
const double LOW_SPEED_COST = 100;
const double OVER_SPEED_LIMIT_COST = 300;
const double COLLISON_ACC_COST = 300; // Penalty for acceleration when collision
const double SPEED_LIMIT = 49.5;
const double SPEED_LOWER_LIMIT = 40.0;
bool is_lane_changed = false;
bool first_implement = true;

// Keep Lane, Lane Change Left, Lane Change Right
// Lane Change Left velocity down, Lane Change Right velocity down
vector<string> states = {"KL", "LCL", "LCLd", "LCR", "LCRd"};


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                                                                                                       uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    //auto sdata = string(data).substr(0, length);
    //cout << sdata << endl;
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
          // j[1] is the data JSON object

          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          int prev_size = previous_path_x.size();

          if (prev_size > 0) {
            car_s = end_path_s;
          }

          vector<vector<double>> next_trajectory;
          double cost = 1000000;
          double next_ref_val;

          cout << "### " << iterate << " ###" << endl; // logging
          cout << "### LANE: " << prev_lane << " ###" << endl; // logging

          for (string state: states) {
            vector<vector<double>> tmp_trajectory;
            int next_lane;
            double next_trajectory_cost = 0.0;
            double acceleration;

            // keep velocity
            if (state == "KL") {
              acceleration = 0.224;
              next_lane = prev_lane;

            } else if (state == "LCL" && prev_lane > 0) {
              acceleration = 0.0;
              next_lane = prev_lane - 1;
              next_trajectory_cost += LANE_CHANGE_COST;

            } else if (state == "LCR" && prev_lane < 2) {
              acceleration = 0.0;
              next_lane = prev_lane + 1;
              next_trajectory_cost += LANE_CHANGE_COST;

            // velocity down
            } else if (state == "LCLd" && prev_lane > 0) {
              acceleration = -0.224;
              next_lane = prev_lane - 1;
              next_trajectory_cost += LANE_CHANGE_COST;
              next_trajectory_cost += LOW_SPEED_COST;

            } else if (state == "LCRd" && prev_lane < 2) {
              acceleration = -0.224;
              next_lane = prev_lane + 1;
              next_trajectory_cost += LANE_CHANGE_COST;
              next_trajectory_cost += LOW_SPEED_COST;

            } else {
              continue;

            }

            double distance = getClosestVehicleDistance(car_s, next_lane, sensor_fusion);
            // cout << " ### distance: " << distance << endl; // logging

            // Calculate the acceleration when state is KL.
            if ((state == "KL") && (distance < 15.0)) {
              if (distance < 0.5) {
                distance = 0.5;
              }

              acceleration = -4 / distance;
              deceleration_at = 0;

            } else if ((state == "KL") && (distance < 30.0) && (prev_ref_val > SPEED_LOWER_LIMIT)) {
              // acceleration = 0.0112*distance - 0.56;
              acceleration = 0.0224*distance - 0.896;

              if (deceleration_at == 0) {
                cout << "### deceleration init ###" << endl; // logging
                deceleration_at = iterate;
              }

            } else if ((state == "KL") && (40.0 < distance) && (distance < 50.0) && (SPEED_LIMIT > prev_ref_val) && (prev_ref_val > SPEED_LOWER_LIMIT)) {
              acceleration = 0.0;

            } else if ((state == "KL") && (distance <= 80.0) && (SPEED_LIMIT > prev_ref_val) && (prev_ref_val > SPEED_LOWER_LIMIT)) {
              // acceleration = 0.0112*distance - 0.672;
              acceleration = (0.224/30)*distance - (5*0.224/3);

            } else if ((state == "KL") && (prev_ref_val >= SPEED_LIMIT)) {
              acceleration = 0.0;

            }

            // Increase cost when continuously decelerating
            if ((state == "KL") && (acceleration < 0.0) && (deceleration_at > 0)) {
              int deceleration_time = iterate - deceleration_at;
              if (deceleration_time > 10) {
                deceleration_time = 10;
              }
              cout << "### continuous deceleration penalty ### " << 100 * deceleration_time << endl; // logging
              next_trajectory_cost += 100*deceleration_time;

            } else if ((state == "KL") && (acceleration > 0.0) && (deceleration_at > 0)) {
              deceleration_at = 0;

            } else if (is_lane_changed) {
              deceleration_at = 0;

            }

            next_ref_val = prev_ref_val + acceleration;

            cout << state << "\t"; // logging

            // Generate a trajectory using parameters according to state
            tmp_trajectory = getTrajectory(car_x, car_y, car_yaw, car_s, next_ref_val, next_lane, prev_size,
                                           previous_path_x, previous_path_y,
                                           map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // Convert to xy trajectory to sd trajectory
            vector<vector<double>> sd_trajectory = getSDTrajectory(tmp_trajectory[0], tmp_trajectory[1], car_yaw, map_waypoints_x, map_waypoints_y);
            double forward_collision_cost = getForwardCollisionCost(next_lane, car_s, sd_trajectory, sensor_fusion);
            next_trajectory_cost += forward_collision_cost;
            cout << "Fc cost: " << forward_collision_cost << ", "; // logging

            // Calculate the collision cost of car coming from behind when lane changing only
            if (next_lane != prev_lane) {
              double backward_collision_cost = getBackwardCollisionCost(next_lane, car_s, sd_trajectory, sensor_fusion);

              cout << "Bc cost: " << backward_collision_cost << ", "; // logging
              next_trajectory_cost += backward_collision_cost;

            } else {

              cout << "Bc cost: 0, "; // logging
            }

            // When speed limit exceeded
            if ((prev_ref_val > SPEED_LIMIT) && (acceleration > 0.0)) {
              next_trajectory_cost += OVER_SPEED_LIMIT_COST;
            }

            // Add the cost of lane changing
            if ((next_lane == prev_lane) && (forward_collision_cost > 0.0) && (acceleration >= 0.0)) {
              next_trajectory_cost += COLLISON_ACC_COST;
            }

            // Add cost for continuous lane change
            if ((is_lane_changed) && (next_lane != prev_lane) && (lane_change_elapsed_time == 0)) {
              next_trajectory_cost += LANE_CHANGE_COST*5;
              lane_change_elapsed_time = iterate + 5;

            } else if ((lane_change_elapsed_time > iterate) && (state != "KL")) {
              next_trajectory_cost += LANE_CHANGE_COST*5;

            } else {
              lane_change_elapsed_time = 0;

            }

            // logging
            cout << "total_cost " << next_trajectory_cost;
            cout << endl;

            if (cost > next_trajectory_cost) {
              cost = next_trajectory_cost;
              next_trajectory = tmp_trajectory;
              lane = next_lane;
              ref_val = next_ref_val;
            }

          }

          // logging
          if (prev_lane != lane) {
            is_lane_changed = true;
            cout << " ======================= LANE CHANGED ======================= " << endl; // logging

          } else if (prev_lane == lane) {
            is_lane_changed = false;
          }

          prev_lane = lane;
          prev_ref_val = ref_val;

          iterate += 1;

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          msgJson["next_x"] = next_trajectory[0];
          msgJson["next_y"] = next_trajectory[1];

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

          if (first_implement) {
            first_implement = false;
          }
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  h.run();
}
