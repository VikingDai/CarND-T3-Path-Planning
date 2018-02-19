#ifndef JMT_H
#define JMT_H

#include <iostream>
#include <vector>
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/QR"
#include "eigen3/Eigen/Dense"
// #include <Eigen/Core>
// #include <Eigen/QR>
// #include <Eigen/Dense>

class JMT {
  public:

  	// A JMT is just a polynomial, this holds the coefficients
    Eigen::VectorXd coeffs;

    // Constructor / Destructor
    JMT();
    ~JMT();

    // Non-Defaults
    JMT(const std::vector<double> &start, const std::vector<double> &end, const double t);
    JMT(const std::vector<double> &start, const double target_velocity, const double t);

    // Function to get instantanious values along the JMT
    double at(const double t) const;

    // Derivative functions to get instantanious component values along the JMT
    double get_position_at(const double t) const;
    double get_velocity_at(const double t) const;
    double get_acceleration_at(const double t) const;
    double get_jerk_at(const double t) const;

    // Pretty Print for debug
    friend std::ostream& operator<<(std::ostream &os, const JMT &val);
};

#endif // JMT_H