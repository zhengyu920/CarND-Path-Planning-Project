//
// Created by Yu Zheng on 1/22/18.
//

#ifndef PATH_PLANNING_UTIL_H
#define PATH_PLANNING_UTIL_H

#include <cmath>
#include <vector>
#include "vehicle.h"

using namespace std;

const int lanes_available = 3;

const double max_s = 6945.554;

const double mph2ms = 0.447;

const double max_speed = 48; // mph

const double max_acc = 8; // m/s^2

double get_speed(double vx, double vy);

// For converting back and forth between radians and degrees.
constexpr double pi();

double deg2rad(double x);

double rad2deg(double x);

// geometry distance between two points
double distance(double x1, double y1, double x2, double y2);

// given a point, and way points in map, find the cloest waypoint in map to our given point
int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);


// find next way point, that is gurrentee in front of us
int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
// take x and y, theta??
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);

void print_vector(vector<double>& v);

void print_behavior(Behavior & behavior);

#endif //PATH_PLANNING_UTIL_H
