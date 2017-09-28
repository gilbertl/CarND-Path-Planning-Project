#ifndef UTILITY_H_
#define UTILITY_H_

#include <vector>

namespace utility {
  using std::vector;

  vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);
  int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);
  vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);
  double distance(double x1, double y1, double x2, double y2);

  double deg2rad(double deg);

  void globalToLocal(double ref_x, double ref_y, double ref_psi, vector<double>* xs, vector<double>* ys);

  void localToGlobal(double car_x, double car_y, double car_psi, vector<double>* xs, vector<double>* ys);

  bool inBetween(double target, double a, double b, double buffer);

  void applyLimits(double lower_limit, double upper_limit, double* curr_lower_limit, double* curr_upper_limit);
}

#endif
