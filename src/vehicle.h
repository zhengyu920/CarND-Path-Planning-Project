//
// Created by Yu Zheng on 1/22/18.
//

#ifndef PATH_PLANNING_VEHICLE_H
#define PATH_PLANNING_VEHICLE_H

#include <iostream>
#include <vector>
#include <string>
#include <math.h>
#include <limits>

using namespace std;

enum class States {
  KL,
  LCL,
  LCR
};

struct Behavior {
  int target_lane;
  double target_speed; // m/s
  vector<double> target_vehicle;
};

class Vehicle {
  public:

  int target_lane;

  States state;

  // Main car's localization Data

  double ref_s;
  double ref_d;
  double ref_speed;

//  // Previous path data given to the Planner
//  vector<double> previous_path_x;
//  vector<double> previous_path_y;
//  // Previous path's end s and d values
//  double end_path_s;
//  double end_path_d;

  // Sensor Fusion Data, a list of all other cars on the same side of the road.
  vector<vector<double>> sensor_fusion;


  /**
  * Constructor
  */
  Vehicle();

  /**
  * Destructor
  */
  virtual ~Vehicle();

  void update_config(double ref_s, double ref_d, double ref_speed, vector<vector<double>>& sensor_fusion);

  vector<States> successor_states();

  Behavior choose_next_state();

  Behavior generate_behavior(States state);

  Behavior keep_lane();

  Behavior lane_change_left();

  Behavior lane_change_right();

  bool check_lane(int lane);

  bool get_vehicle_behind(int lane, vector<double> & rVehicle);

  bool get_vehicle_ahead(int lane, vector<double> & rVehicle);

  bool within_lane(int lane, double d);

  vector<double> get_lane_speed();

  bool car_in_lane(int lane, double d);

  void print_vehicle();

};

#endif //PATH_PLANNING_VEHICLE_H
