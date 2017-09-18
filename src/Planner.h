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
                vector<double> previous_path_x,
                vector<double> previous_path_y,
                vector<double>& next_x_vals, 
                vector<double>& next_y_vals);
 private:
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;
};

#endif
