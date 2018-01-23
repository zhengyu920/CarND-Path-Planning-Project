//
// Created by Yu Zheng on 1/22/18.
//

#include "vehicle.h"
#include "util.h"


Vehicle::Vehicle() {
  this->target_lane = 1;
  this->ref_speed = 0;
}

Vehicle::~Vehicle() {}

bool Vehicle::within_lane(int lane, double d) {
  return d > (2 + 4 * lane - 2) && d < (2 + 4 * lane + 2);
}

bool Vehicle::car_in_lane(int lane, double d) {
  return abs(d  - (2 + 4 * lane)) <= 1;
}

bool Vehicle::check_lane(int lane) {
  // find ref_v to use
  for (int i = 0; i < this->sensor_fusion.size(); i++) {
    // car is in neighbor lane
    float d = this->sensor_fusion[i][6];
    if (within_lane(lane, d)) {
      double vx = this->sensor_fusion[i][3];
      double vy = this->sensor_fusion[i][4];
      // double check_speed = sqrt(vx * vx + vy * vy); // meter/h
      double check_car_s = this->sensor_fusion[i][5];

      // check_car_s += ((double) prev_size * 0.02 * check_speed); // if using previous points can project s value outwards
      // because we push all previous path points into next path points

      // check s values greater than mine and s gap
      if (check_car_s > (this->ref_s - 20) && (check_car_s - this->ref_s) < 20) {
        return false;
      }
    }
  }
  return true;
}

void Vehicle::update_config(double ref_s, double ref_d, double ref_speed, vector<vector<double>>& sensor_fusion) {
  this->ref_s = ref_s;
  this->ref_d = ref_d;
  this->ref_speed = ref_speed;
  vector<vector<double>> sf;
  for (int i = 0; i < sensor_fusion.size(); i++) {
    double s = sensor_fusion[i][5];
    if (s >= (this->ref_s - 30) && (s - this->ref_s) <= 30) {
      sf.push_back(sensor_fusion[i]);
    }
  }
  this->sensor_fusion = sf;
}

bool Vehicle::get_vehicle_ahead(int lane, vector<double> &rVehicle) {
  double min_s = max_s;
  bool found_vehicle = false;
  for (int i = 0; i < this->sensor_fusion.size(); i++) {
    double d = this->sensor_fusion[i][6];
    double s = this->sensor_fusion[i][5];
    if (within_lane(lane, d) && s < min_s) {
      min_s = s;
      rVehicle = this->sensor_fusion[i];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

bool Vehicle::get_vehicle_behind(int lane, vector<double> &rVehicle) {
  double max_s = -1;
  bool found_vehicle = false;
  for (int i = 0; i < this->sensor_fusion.size(); i++) {
    double d = this->sensor_fusion[i][6];
    double s = this->sensor_fusion[i][5];
    if (within_lane(lane, d) && s > max_s) {
      max_s = s;
      rVehicle = this->sensor_fusion[i];
      found_vehicle = true;
    }
  }
  return found_vehicle;
}

vector<States> Vehicle::successor_states() {
  vector<States> states;
  states.push_back(States::KL);

  if (this->state != States::KL && !car_in_lane(this->target_lane, this->ref_d)) {  // in the middle of lane changing
    return states;
  }

  if (this->target_lane > 0) { // if not at the left most lane
    states.push_back(States::LCL);
  }

  if (this->target_lane < lanes_available - 1) { // if not at the right most lane
    states.push_back(States::LCR);
  }

  return states;
}

Behavior Vehicle::generate_behavior(States state) {
  if (state == States::LCL && check_lane(this->target_lane - 1)) {
    return lane_change_left();
  }
  if (state == States::LCR && check_lane(this->target_lane + 1)) {
    return lane_change_right();
  }
  return keep_lane();
}

Behavior Vehicle::keep_lane() {
  vector<double> vAhead;
  Behavior behavior {this->target_lane, max_speed * mph2ms};
  if (get_vehicle_ahead(this->target_lane, vAhead)) {
    behavior.target_vehicle = vAhead;
    behavior.target_speed = min(max_speed * mph2ms, get_speed(vAhead[3], vAhead[4]));
  }
  return behavior;
}

Behavior Vehicle::lane_change_left() {
  vector<double> vAhead;
  Behavior behavior {this->target_lane - 1, max_speed * mph2ms};
  if (get_vehicle_ahead(this->target_lane - 1, vAhead)) {
    behavior.target_vehicle = vAhead;
    behavior.target_speed = min(max_speed * mph2ms, get_speed(vAhead[3], vAhead[4]));
  }
  return behavior;
}

Behavior Vehicle::lane_change_right() {
  vector<double> vAhead;
  Behavior behavior {this->target_lane + 1, max_speed * mph2ms};
  if (get_vehicle_ahead(this->target_lane + 1, vAhead)) {
    behavior.target_vehicle = vAhead;
    behavior.target_speed = min(max_speed * mph2ms, get_speed(vAhead[3], vAhead[4]));
  }
  return behavior;
}

Behavior Vehicle::choose_next_state() {
  vector<States> states = successor_states();
  double min_cost = 100000000000.0;
  double cost;
  Behavior best_behavior;
  //vector<double> lane_speed = get_lane_speed();
  print_vehicle();
  cout << "* searching best move..." << endl;
  for (int i = 0; i < states.size(); i++) {

    States next_state = states[i];
    Behavior behavior = generate_behavior(next_state);
    cost = 1000 * (abs(max_speed - behavior.target_speed) / max_speed);
    if (target_lane != behavior.target_lane) {
      cost += 0.3;
    }
    if (cost < min_cost) {
      best_behavior = behavior;
      min_cost = cost;
    }
    cout << "* candidate state " << " " << endl;
    cout << "cost : " << cost << endl;
    print_behavior(behavior);
  }
  this->target_lane = best_behavior.target_lane;
  cout << "* best behavior" << endl;
  print_behavior(best_behavior);
  return best_behavior;
}

vector<double> Vehicle::get_lane_speed() {
  vector<double> lane_speed;
  vector<double> rVehicle;
  for (int lane = 0; lane < lanes_available; lane++) {
    if (get_vehicle_ahead(lane, rVehicle)) {
      lane_speed.push_back(get_speed(rVehicle[3], rVehicle[4]));
    } else {
      lane_speed.push_back(max_speed);
    }
  }
  return lane_speed;
}

void Vehicle::print_vehicle() {
  cout << "ref_s: " << this->ref_s << endl;
  cout << "ref_d: " << this->ref_d << endl;
  cout << "ref_speed: " << this->ref_speed << endl;
}