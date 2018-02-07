#include "jmt.h"

// #include <iostream>
// #define DEBUG
// using namespace std;

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector2d;

JMT::JMT(){this->coeffs = VectorXd::Zero(6);}
JMT::~JMT(){}

// Create a JMT given both the known start and end states and the over which you want the
// states to change
JMT::JMT(const std::vector<double> &start, const std::vector<double> &end, const double t)
{
  #ifdef DEBUG
  cout << "        + CREATING JMT(1)" << endl;
  #endif

  MatrixXd A = MatrixXd(3,3);
  VectorXd b = VectorXd(3);
  VectorXd x = VectorXd(3);
  this->coeffs = VectorXd(6);

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  const double pi = start[0];
  const double vi = start[1];
  const double ai = start[2];
  const double pf = end[0];
  const double vf = end[1];
  const double af = end[2];

  A <<   t3,     t4,    t5,
       3*t2,   4*t3,  5*t4,
       6*t ,  12*t2, 20*t3;

  b << pf - (pi + vi * t + 0.5 * ai * t2),
       vf - (vi + ai * t),
       af - ai;

  x = A.inverse() * b;

  this->coeffs << pi, vi, (ai / 2.0), x[0], x[1], x[2];
}

// Create a JMT given just the starting state, a target velocity, and a time to reach
// that velocity
JMT::JMT(const std::vector<double> &start, const double target_velocity, const double t)
{
  #ifdef DEBUG
  cout << "        + CREATING JMT(2)" << endl;
  #endif

  const double t2 = t * t;
  const double t3 = t2 * t;

  const double pi = start[0];
  const double vi = start[1];
  const double ai = start[2];

  #ifdef DEBUG
  cout << "        + target v: " << target_velocity << endl;
  cout << "        + pi: " << pi << endl;
  cout << "        + vi: " << vi << endl;
  cout << "        + ai: " << ai << endl;
  #endif

  MatrixXd A(2, 2);
  A << 3*t2,  4*t3,
        6*t, 12*t2;

  #ifdef DEBUG
  cout << "          + Made A(2, 2)" << endl;
  #endif

  VectorXd b(2);
  b << target_velocity - vi - ai * t,
       0. - ai;

  #ifdef DEBUG
  cout << "          + Made B(2)" << endl;
  #endif

  Vector2d x = A.colPivHouseholderQr().solve(b);

  #ifdef DEBUG
  cout << "          + Solved -> x(2)" << endl;
  #endif

  this->coeffs = VectorXd(6);
  this->coeffs << pi, vi, (ai / 2.0), x[0], x[1], 0.0;
}

// -----

double JMT::at(const double t) const{

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;
  const double t5 = t * t4;

  Eigen::VectorXd T = VectorXd(6);
  T << 1.0, t, t2, t3, t4, t5;

  return T.transpose() * this->coeffs;
}

double JMT::get_position_at(const double t) const {
  return at(t);
}

double JMT::get_velocity_at(const double t) const {

  const double t2 = t * t;
  const double t3 = t * t2;
  const double t4 = t * t3;

  Eigen::VectorXd T = VectorXd(6);
  T << 0, 1.0, 2.0 * t, 3.0 * t2, 4.0 * t3, 5.0 * t4;

  return T.transpose() * this->coeffs;
}

double JMT::get_acceleration_at(const double t) const {

  const double t2 = t * t;
  const double t3 = t * t2;

  Eigen::VectorXd T = VectorXd(6);
  T << 0, 0, 2.0, 6.0 * t, 12.0 * t2, 20.0 * t3;

  return T.transpose() * this->coeffs;
}

double JMT::get_jerk_at(const double t) const {

  Eigen::VectorXd T = VectorXd(6);
  T << 0, 0, 0, 6.0, 24.0 * t, 60.0 * t * t;

  return T.transpose() * this->coeffs;
}

std::ostream& operator<<(std::ostream &os, const JMT &val)
{
  bool written_first = false;
  int z_count = 0;
  for(int i = 5; i >= 0; --i)
  {
    if(val.coeffs[i] == 0)
    {
      z_count++;
      continue;
    }

    if(!written_first){
      if(val.coeffs[i] < 0) os << "-";
      written_first = true;
    }
    else os << (val.coeffs[i] < 0 ? " - " : " + ");

    if(i != 0){
      os << (val.coeffs[i] < 0 ? val.coeffs[i] * -1.0 : val.coeffs[i]) << "x";
      if(i != 1) os << "^" << i;
    }
    else os << val.coeffs[i];
  }
  if(z_count == 6) os << "0";
  return os;
}
