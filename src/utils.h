#ifndef UTILS_H
#define UTILS_H

#include <math.h>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"

// For converting back and forth between radians and degrees.
constexpr double pi();
double deg2rad(double x);
double rad2deg(double x);

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x);

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(const Eigen::VectorXd &xvals,
                        const Eigen::VectorXd &yvals,
                        int order);

#endif /* UTILS_H */
