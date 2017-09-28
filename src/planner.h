#include <vector>

#ifndef PLANNER_H_
#define PLANNER_H_

using std::vector;

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
  static const double MAX_ACCELERATION_M_PER_S2;
  static const double MAX_JERK_M_PER_S3;

  void MaxSpeedsWithoutJerk(double *min_speed, double *max_speed);

  void NextBehavior(
      double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion, double* next_speed, double* next_d);
  bool TooCloseToCarAhead(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion, double* car_ahead_speed);
  bool SlowCarAhead(
    double seconds_ahead, double car_s, double car_d, 
    vector<vector<double>> sensor_fusion);
  bool CanSwitchToD(double seconds_ahead, double car_s, double car_d, vector<vector<double>> sensor_fusion, double target_d);
  double ClosestCenter(double car_d);
  void PredictCarSD(vector<double> sensor_fusion_datum, double num_secs_ahead, double* s, double* d);

  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  double last_target_d;
  double last_target_speed;
  double last_last_target_speed;
};

#endif
