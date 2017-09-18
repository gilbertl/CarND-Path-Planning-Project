#include "Planner.h"
#include "utility.h"
#include <math.h>

using std::vector;

Planner::Planner(vector<double> map_waypoints_x,
                 vector<double> map_waypoints_y,
                 vector<double> map_waypoints_s,
                 vector<double> map_waypoints_dx,
                 vector<double> map_waypoints_dy):
    map_waypoints_x(map_waypoints_x),
    map_waypoints_y(map_waypoints_y), 
    map_waypoints_s(map_waypoints_s),
    map_waypoints_dx(map_waypoints_dx),
    map_waypoints_dy(map_waypoints_dy) {
}

Planner::~Planner() {}

void Planner::PlanPath(double car_x,
                       double car_y,
                       double car_s,
                       double car_d,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       vector<double>& next_x_vals, 
                       vector<double>& next_y_vals) {
  next_x_vals.clear();
  next_y_vals.clear();

  int path_size = previous_path_x.size();
  for(int i = 0; i < path_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }

  double s, d;
  if(path_size == 0) {
    s = car_s;
    d = car_d;
  } else {
    double pos_x = previous_path_x[path_size-1];
    double pos_y = previous_path_y[path_size-1];
    double pos_x2 = previous_path_x[path_size-2];
    double pos_y2 = previous_path_y[path_size-2];
    double angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
    vector<double> frenet = utility::getFrenet(pos_x, pos_y, angle, 
        map_waypoints_x, map_waypoints_y);
    s = frenet[0];
    d = frenet[1];
  }

  double dist_inc = 0.5;  // ~50mph
  for(int i = 0; i < 50 - path_size; i++) {
    vector<double> xy = utility::getXY(s + (i + 1) * dist_inc,
                                       //d,
                                       2, 
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
    next_x_vals.push_back(xy[0]);
    next_y_vals.push_back(xy[1]);
  }
}
