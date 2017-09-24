#include <vector>

#ifndef PLANNER_H_
#define PLANNER_H_

using std::vector;

enum Behavior { STAY_IDEAL_SPEED, SLOW_TO_SPEED_AHEAD, SWITCH_LEFT, SWITCH_RIGHT };

class Planner {
 public:
  Planner(vector<double> map_waypoints_x,
         vector<double> map_waypoints_y,
         vector<double> map_waypoints_s,
         vector<double> map_waypoints_dx,
         vector<double> map_waypoints_dy);
  virtual ~Planner();
  
  void PlanPath(double car_x,
                double car_y,
                double car_s,
                double car_d,
                double car_yaw,
                const vector<double>& previous_path_x,
                const vector<double>& previous_path_y,
                const vector<vector<double>>& sensor_fusion,
                vector<double>* next_x_vals, 
                vector<double>* next_y_vals);
 private:
  static const int X_IDX;
  static const int Y_IDX;
  static const int VX_IDX;
  static const int VY_IDX;
  static const int S_IDX;
  static const int D_IDX;
  static const double SECS_PER_FRAME;
  static const double IDEAL_SPEED_M_PER_S;

  Behavior NextBehavior(
      double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion);
  bool TooCloseToCarAhead(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion);
  bool CanSwitchLeft(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion);
  bool CanSwitchRight(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion);
  double ClosestCenter(double car_d);
  void PredictCarSD(vector<double> sensor_fusion_datum, double num_secs_ahead, double* s, double* d);

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

#endif
