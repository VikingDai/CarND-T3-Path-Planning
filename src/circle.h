#ifndef CIRCLE_H
#define CIRCLE_H

#include <vector>
#include <cmath>

class Circle{
public:
  double radius();
  std::vector<double> center();
  Circle(const std::vector<double> &p1, const std::vector<double> &p2, const std::vector<double> &p3);
  Circle();
  virtual ~Circle();

private:
  void __calc_circle(const std::vector<double> &p1, const std::vector<double> &p2, const std::vector<double> &p3);
  bool is_perpendicular(const std::vector<double> &p1, const std::vector<double> &p2, const std::vector<double> &p3);

  double m_radius;
  double m_center_x;
  double m_center_y;
};

#endif