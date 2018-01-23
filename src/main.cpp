#include <fstream>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "vehicle.h"
#include "util.h"

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

  Vehicle vehicle;

  h.onMessage([&vehicle, &map_waypoints_x, &map_waypoints_y, &map_waypoints_s, &map_waypoints_dx, &map_waypoints_dy](
      uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
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
          vector<double> previous_path_x = j[1]["previous_path_x"];
          vector<double> previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side of the road.
          // [id, x, y, vx, vy, s, d]
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          // cout << sensor_fusion << endl;

          json msgJson;

          // TODO: define a path made up of (x,y) points that the car will visit sequentially every .02 seconds
          int prev_size = previous_path_x.size();

          cout << "----------------------------------" << endl;
          cout << "prev_size: " << prev_size << endl;

          // plan every 50 way points (1sec)
          if (prev_size > 75) {
            msgJson["next_x"] = previous_path_x;
            msgJson["next_y"] = previous_path_y;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
          } else {

            // reference of vehicle position and speed, for trajectory generation
            prev_size = prev_size > 10 ? 10 : prev_size; // keep first 3 way points of previous path
            double ref_s {car_s};
            double ref_d {car_d};
            double ref_x {car_x};
            double ref_y {car_y};
            double ref_yaw {deg2rad(car_yaw)}; // degree to radian(pi based)
            double ref_speed {car_speed * mph2ms};

            // anchor points to fit spline
            // 5 anchor points, 2 on current lane, 3 on target lane
            vector<double> ptsx;
            vector<double> ptsy;

            if (prev_size == 0) {

              // use two points that make the path tangent to the car
              double prev_ref_x = ref_x - cos(ref_yaw); // x - 1 * cos(yaw)
              double prev_ref_y = ref_y - sin(ref_yaw); // y - 1 * sin(yaw)

              ptsx.push_back(prev_ref_x);
              ptsx.push_back(ref_x);

              ptsy.push_back(prev_ref_y);
              ptsy.push_back(ref_y);

            } else {

              // reset ref points to last points of previous path
              // as the start point to new path way points
              ref_x = previous_path_x[prev_size - 1];
              ref_y = previous_path_y[prev_size - 1];

              double ref_x_prev = previous_path_x[prev_size - 2];
              double ref_y_prev = previous_path_y[prev_size - 2];

              // car's yaw angle after execute all remain previous way points
              ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

              ref_speed = distance(ref_x_prev, ref_y_prev, ref_x, ref_y) / 0.02; // ms

              vector<double> sd = getFrenet(ref_x, ref_y, ref_yaw, map_waypoints_x, map_waypoints_y);

              ref_s = sd[0];
              ref_d = sd[1];

              ptsx.push_back(ref_x_prev);
              ptsx.push_back(ref_x);

              ptsy.push_back(ref_y_prev);
              ptsy.push_back(ref_y);

            }

            vehicle.update_config(ref_s, ref_d, ref_speed, sensor_fusion);

            Behavior behavior = vehicle.choose_next_state();
            // print_behavior(behavior);
            double target_lane = behavior.target_lane;
            double target_speed = behavior.target_speed;
            vector<double> target_vehicle = behavior.target_vehicle;

            // In Frenet add evenly 30m spaced points ahead of the starting reference
            vector<double> next_wp0 = getXY((ref_s + 30), (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp1 = getXY((ref_s + 60), (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_wp2 = getXY((ref_s + 90), (2 + 4 * target_lane), map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // add 3 points above
            ptsx.push_back(next_wp0[0]);
            ptsx.push_back(next_wp1[0]);
            ptsx.push_back(next_wp2[0]);

            ptsy.push_back(next_wp0[1]);
            ptsy.push_back(next_wp1[1]);
            ptsy.push_back(next_wp2[1]);

            // shift points in ptsx and ptsy from xy coordinate to car coordinate, which then use to fit a spline
            // to avoid deal with vertical line
            for (int i = 0; i < ptsx.size(); i++) {

              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;

              ptsx[i] = (shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw));
              ptsy[i] = (shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw));

            }

            // create a spline
            tk::spline s;

            // set(x, y) points to the spline
            s.set_points(ptsx, ptsy);


            // points send to simulator
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            // push all remaining of previous path points
            for (int i = 0; i < prev_size; i++) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            double x_add_on = 0;

            for (int i = 1; i <= 100 - prev_size; i++) {
              if (ref_speed < target_speed - 2) {
                ref_speed += max_acc * 0.02;
              } else if (ref_speed + 1 > target_speed) {
                ref_speed -= max_acc * 0.02;
              }

              double x_point = x_add_on + ref_speed * 0.02;
              double y_point = s(x_point);

              x_add_on = x_point;


              double x_ref = x_point;
              double y_ref = y_point;

              // rotate back to normal after rotating it earlier
              x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
              y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
              // shift
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);

            }
//
//          double target_x = 30.0;
//          double target_y = s(target_x);
//          double target_dist = sqrt(target_x * target_x + target_y * target_y);
//
//          double x_add_on = 0;
//
//          for (int i = 1; i <= 50 - previous_path_x.size(); i++) {
//            double N = (target_dist / (0.02 * ref_speed / 2.24));
//            double x_point = x_add_on + (target_x) / N;
//            double y_point = s(x_point);
//
//            x_add_on = x_point;
//
//            double x_ref = x_point;
//            double y_ref = y_point;
//
//            // rotate back to normal after rotating it earlier
//            x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
//            y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));
//            // shift
//            x_point += ref_x;
//            y_point += ref_y;
//
//            next_x_vals.push_back(x_point);
//            next_y_vals.push_back(y_point);
//
//          }

            msgJson["next_x"] = next_x_vals;
            msgJson["next_y"] = next_y_vals;

            auto msg = "42[\"control\"," + msgJson.dump() + "]";

            //this_thread::sleep_for(chrono::milliseconds(1000));
            ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
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
