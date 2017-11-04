#include <iostream>
#include "RoadGeometry.h"

void RoadGeometry::setCenter(std::vector<double> &x, std::vector<double> &y, int maxFitOrder, double maxFitError) {
  this->x = x;
  this->y = y;
  fit(maxFitOrder, maxFitError);
}

double RoadGeometry::centerY(double x) {
  return polyeval(polynomial, x);
}

CppAD::AD<double> RoadGeometry::centerY(const CppAD::AD<double> &x) {
  return polyeval(polynomial, x);
}

void RoadGeometry::fit(int maxFitOrder, double maxFitError) {
  Eigen::VectorXd xVals(x.size());
  Eigen::VectorXd yVals(y.size());
  for (size_t i = 0; i < x.size(); i++) {
    xVals[i] = x[i];
    yVals[i] = y[i];
  }

  double fiterr;
  int order = 2;
  do { // Compute the polynomial coefficients to an order that has small fitting error
    polynomial = polyfit(xVals, yVals, order++);
    fiterr = 0.0;
    for (size_t i = 0; i < x.size(); i++) {
      fiterr += square<double>(y[i] - centerY(x[i]));
    }
  } while (fiterr > maxFitError && order < maxFitOrder);

#ifdef VERBOSE_OUT
  std::cout << "Polynominal: " << polynomial << ", error: " << fiterr << std::endl;
#endif
}

double RoadGeometry::orientation(double px, double dir) {
  double psi = std::atan(polyder(polynomial, px));
  if (dir < 0) {  // adjust the angle if pointing to negative x direction
    psi = normalizeAngle(psi + M_PI);
  }
  return psi;
}

CppAD::AD<double> RoadGeometry::orientation(const CppAD::AD<double> &px, double dir) {
  CppAD::AD<double> psi = CppAD::atan(polyder(polynomial, px));
  if (dir < 0) {  // adjust the angle if pointing to negative x direction
    psi = normalizeAngle(psi + M_PI);
  }
  return psi;
}

double RoadGeometry::computeOrientationChange(double x0, double x1) {
  double psi0 = orientation(x0, x1 - x0);
  double psi1 = orientation(x1, x1 - x0);
  return psi1 - psi0;
}

double RoadGeometry::cte(double px, double py) {
  return centerY(px) - py;
}

CppAD::AD<double> RoadGeometry::cte(const CppAD::AD<double> &px, const CppAD::AD<double> &py) {
  CppAD::AD<double> y = polyeval(polynomial, px);
  return y - py;
}