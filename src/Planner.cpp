#include "Planner.h"
#include "utility.h"
#include "spline.h"
#include <math.h>
#include <iostream>

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

const int Planner::X_IDX = 1;
const int Planner::Y_IDX = 2;
const int Planner::VX_IDX = 3;
const int Planner::VY_IDX = 4;
const int Planner::S_IDX = 5;
const int Planner::D_IDX = 6;

const double Planner::SECS_PER_FRAME = 0.02;
const double Planner::IDEAL_SPEED_M_PER_S = 22;

Behavior Planner::NextBehavior(
    double seconds_ahead, double car_s, double car_d, 
    vector<vector<double>> sensor_fusion) {
  // TODO(gilbertleung): see if car ahead is driving at our ideal speed
  if (!TooCloseToCarAhead(seconds_ahead, car_s, car_d, sensor_fusion)) {
    return STAY_IDEAL_SPEED;
  } else if (CanSwitchLeft(seconds_ahead, car_s, car_d, sensor_fusion)) {
    return SWITCH_LEFT;
  } else if (CanSwitchRight(seconds_ahead, car_s, car_d, sensor_fusion)) {
    return SWITCH_RIGHT;
  } else {
    return SLOW_TO_SPEED_AHEAD;
  }
}

void Planner::PredictCarSD(vector<double> sensor_fusion_datum, double num_secs_ahead, double* s, double* d) {
  double x = sensor_fusion_datum[X_IDX];
  double y = sensor_fusion_datum[Y_IDX];
  double vx = sensor_fusion_datum[VX_IDX];
  double vy = sensor_fusion_datum[VY_IDX];
  double new_x = x + vx * num_secs_ahead;
  double new_y = y + vy * num_secs_ahead;
  double angle = atan2(new_y - y, new_x - x);
  vector<double> frenet = utility::getFrenet(new_x, new_y, angle, map_waypoints_x, map_waypoints_y);
  *s = frenet[0];
  *d = frenet[1];
}

bool Planner::TooCloseToCarAhead(
    double seconds_ahead, double car_s, double car_d, 
    vector<vector<double>> sensor_fusion) {
  const int BUFFER = 40;
  for (const auto& sensor_datum : sensor_fusion) {
    double other_car_s, other_car_d;
    PredictCarSD(sensor_datum, seconds_ahead, &other_car_s, &other_car_d);

    double distance_to_car = other_car_s - car_s;
    if (distance_to_car > 0 && distance_to_car < BUFFER) {
      std::cout << "nearby... car_d: " << car_d << ", other car's d: " << other_car_d
          << ", car_s: " << car_s << "other car's s: " << other_car_s << std::endl;
    }
    
    if (fabs(other_car_d - car_d) < 1.5 
        && distance_to_car > 0
        && distance_to_car < BUFFER) {
      std::cout << "car_d: " << car_d << ", other car's d: " << other_car_d
          << ", car_s: " << car_s << "other car's s: " << other_car_s << std::endl;
      return true;
    }
  } 
  return false;
}

bool Planner::CanSwitchLeft(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion) {
  if (car_d < 6) {
    return false;
  }
  const int BUFFER_BEHIND = 5;
  const int BUFFER_AHEAD = 80;
  int next_car_d = car_d - 4;
  for (const auto& sensor_datum : sensor_fusion) {
    double other_car_s, other_car_d;
    PredictCarSD(sensor_datum, seconds_ahead, &other_car_s, &other_car_d);
    double distance_to_car = other_car_s - car_s;
    if (distance_to_car > -BUFFER_BEHIND
        && distance_to_car < BUFFER_AHEAD
        && fabs(other_car_d - next_car_d) < 1.5) {
      return false;
    }
  }
  return true;
}

bool Planner::CanSwitchRight(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion) {
  if (car_d > 6) {
    return false;
  }
  const int BUFFER_BEHIND = 5;
  const int BUFFER_AHEAD = 80;
  int next_car_d = car_d + 4;
  for (const auto& sensor_datum : sensor_fusion) {
    double other_car_s, other_car_d;
    PredictCarSD(sensor_datum, seconds_ahead, &other_car_s, &other_car_d);
    double distance_to_car = other_car_s - car_s;
    if (distance_to_car > -BUFFER_BEHIND
        && distance_to_car < BUFFER_AHEAD
        && fabs(other_car_d - next_car_d) < 1.5) {
      return false;
    }
  }
  return true;
}

void Planner::PlanPath(double car_x,
                       double car_y,
                       double car_s,
                       double car_d,
                       double car_yaw,
                       const vector<double>& previous_path_x,
                       const vector<double>& previous_path_y,
                       const vector<vector<double>>& sensor_fusion,
                       vector<double>* next_x_vals, 
                       vector<double>* next_y_vals) {
  next_x_vals->clear();
  next_y_vals->clear();

  int path_size = previous_path_x.size();
  for(int i = 0; i < path_size; i++) {
    next_x_vals->push_back(previous_path_x[i]);
    next_y_vals->push_back(previous_path_y[i]);
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

  Behavior behavior = NextBehavior(path_size * SECS_PER_FRAME, s, d, sensor_fusion);
  std::cout << "Next behavior should be: " << behavior << std::endl;

  double desired_speed, desired_d;
  switch (behavior) {
    case STAY_IDEAL_SPEED:
      desired_speed = IDEAL_SPEED_M_PER_S;
      desired_d = ClosestCenter(d);
      break;
    case SLOW_TO_SPEED_AHEAD:
      // TODO(gilbertleung): actually slow to speed of car ahead
      desired_speed = IDEAL_SPEED_M_PER_S / 2; 
      desired_d = ClosestCenter(d);
      break;
    case SWITCH_LEFT:
      desired_speed = IDEAL_SPEED_M_PER_S;
      desired_d = ClosestCenter(d) - 4;
      break;
    case SWITCH_RIGHT:
      desired_speed = IDEAL_SPEED_M_PER_S;
      desired_d = ClosestCenter(d) + 4;
      break;
  }

  double ref_x, ref_y, ref_x_prev, ref_y_prev, ref_yaw;
  vector<double> ptsx, ptsy;
  if (path_size < 2) {
    ref_x = car_x;
    ref_y = car_y;
    ref_yaw = utility::deg2rad(car_yaw);
    // Inteperloate one meter back.
    ref_x_prev = car_x - cos(car_yaw);
    ref_y_prev = car_y - sin(car_yaw);
  } else {
    ref_x = (*next_x_vals)[path_size-1];
    ref_y = (*next_y_vals)[path_size-1];
    ref_x_prev = (*next_x_vals)[path_size-2];
    ref_y_prev = (*next_y_vals)[path_size-2];
    ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);
  }
  ptsx.push_back(ref_x_prev);
  ptsx.push_back(ref_x);
  ptsy.push_back(ref_y_prev);
  ptsy.push_back(ref_y);
  // Add 3 more reference points on the desired lane, some distance apart.
  for (int i = 0; i < 3; i++) {
    vector<double> xy = utility::getXY(s + (i + 1) * 30,
                                       desired_d,
                                       map_waypoints_s,
                                       map_waypoints_x,
                                       map_waypoints_y);
    ptsx.push_back(xy[0]);
    ptsy.push_back(xy[1]);
  }
  utility::globalToLocal(ref_x, ref_y, ref_yaw, &ptsx, &ptsy);

  tk::spline spline;
  spline.set_points(ptsx, ptsy);

  double end_x = 30;
  double end_y = spline(end_x);
  double total_dist = sqrt(end_x * end_x + end_y * end_y);
  int num_increments = total_dist / (SECS_PER_FRAME * desired_speed);
  double x_increment = end_x / num_increments;
  vector<double> new_xs, new_ys;
  
  for(int i = 0; i < 50 - path_size; i++) {
    double x_local = (i + 1) * x_increment;
    new_xs.push_back(x_local);
    new_ys.push_back(spline(x_local));
  }
  utility::localToGlobal(ref_x, ref_y, ref_yaw, &new_xs, &new_ys);

  next_x_vals->insert(std::end(*next_x_vals), std::begin(new_xs), std::end(new_xs));
  next_y_vals->insert(std::end(*next_y_vals), std::begin(new_ys), std::end(new_ys));
}

double Planner::ClosestCenter(double car_d) {
  vector<int> CENTERS = {2, 6, 10};
  int closest_center = CENTERS[0];
  for (int center : CENTERS) {
    if (fabs(center - car_d) < fabs(closest_center - car_d)) {
      closest_center = center;
    }
  }
  return closest_center;
}
