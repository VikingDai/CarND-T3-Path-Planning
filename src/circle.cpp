// https://web.archive.org/web/20060908234034/http://local.wasp.uwa.edu.au/~pbourke/geometry/circlefrom3/Circle.cpp

#include "circle.h"

static double distance(double x1, double y1, double x2, double y2)
{
  return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

Circle::Circle()
{
  // Initialize Radius to something non-sensical
  this->m_radius = -1.0;
}

Circle::~Circle(){}

Circle::Circle(const std::vector<double> &pt1, const std::vector<double> &pt2, const std::vector<double> &pt3)
{
  // Initialize Radius to something non-sensical
  this->m_radius = -1.0;

  // Check the points that have come in to make sure we consume them in the right order
  if(!this->is_perpendicular(pt1, pt2, pt3)) this->__calc_circle(pt1, pt2, pt3);
  else if(!this->is_perpendicular(pt1, pt3, pt2)) this->__calc_circle(pt1, pt3, pt2);
  else if(!this->is_perpendicular(pt2, pt1, pt3)) this->__calc_circle(pt2, pt1, pt3);
  else if(!this->is_perpendicular(pt2, pt3, pt1)) this->__calc_circle(pt2, pt3, pt1);
  else if(!this->is_perpendicular(pt3, pt2, pt1)) this->__calc_circle(pt3, pt2, pt1);
  else if(!this->is_perpendicular(pt3, pt1, pt2)) this->__calc_circle(pt3, pt1, pt2);
}

// Check the given point are perpendicular to x or y axis
bool Circle::is_perpendicular(const std::vector<double> &pt1, const std::vector<double> &pt2, const std::vector<double> &pt3)
{
  double yDelta_a = pt2[1] - pt1[1];
  double xDelta_a = pt2[0] - pt1[0];
  double yDelta_b = pt3[1] - pt2[1];
  double xDelta_b = pt3[0] - pt2[0];

  // checking whether the line of the two pts are vertical
  if(fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001) return false;
  if(fabs(yDelta_a) <= 0.0000001) return true;
  else if(fabs(yDelta_b) <= 0.0000001) return true;
  else if(fabs(xDelta_a)<= 0.000000001) return true;
  else if(fabs(xDelta_b)<= 0.000000001) return true;
  return false;
}

void Circle::__calc_circle(const std::vector<double> &pt1, const std::vector<double> &pt2, const std::vector<double> &pt3)
{
  double yDelta_a = pt2[1] - pt1[1];
  double xDelta_a = pt2[0] - pt1[0];
  double yDelta_b = pt3[1] - pt2[1];
  double xDelta_b = pt3[0] - pt2[0];

  if (fabs(xDelta_a) <= 0.000000001 && fabs(yDelta_b) <= 0.000000001)
  {
      this->m_center_x = 0.5 * (pt2[0] + pt3[0]);
      this->m_center_y = 0.5 * (pt1[1] + pt2[1]);
      this->m_radius = distance(m_center_x, m_center_y, pt1[0], pt1[1]);
      return;
  }

  // IsPerpendicular() assure that xDelta(s) are not zero
  double aSlope = yDelta_a / xDelta_a;
  double bSlope = yDelta_b / xDelta_b;

  // checking whether the given points are colinear.
  if (fabs(aSlope-bSlope) <= 0.000000001) return;

  // calc center
  this->m_center_x= (aSlope * bSlope * (pt1[1] - pt3[1]) + bSlope * (pt1[0] + pt2[0]) - aSlope * (pt2[0] + pt3[0])) / (2.0 * (bSlope - aSlope));
  this->m_center_y = -1.0 * (m_center_x - (pt1[0] + pt2[0]) / 2.0) / aSlope + (pt1[1] + pt2[1]) / 2.0;
  this->m_radius = distance(m_center_x, m_center_y, pt1[0], pt1[1]);
  return;
}

std::vector<double> Circle::center(){return {m_center_x, m_center_y};}
double Circle::radius(){return this->m_radius;}