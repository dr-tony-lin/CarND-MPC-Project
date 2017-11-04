#ifndef _MODEL_ROAD_H_
#define _MODEL_ROAD_H_

#include <math.h>
#include <vector>
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen/Core"
#include "../utils/utils.h"

class RoadGeometry {
  /**
   * x coordinates of the center trajectory
   */ 
  std::vector<double> x;

  /**
   * y coordinates of the center trajectory
   */ 
  std::vector<double> y;

  /**
   * Fitted polynomial coefficients
   */ 
  Eigen::VectorXd polynomial;

  /**
   * Fit the trajectory to a polynomial
   * 
   * @param maxFitOrder maximal polynomial fit order default is 5
   * @param maxFitError maximal polynomial fit error default is 0.5
   */ 
  void fit(int maxFitOrder = 5, double maxFitError=0.5);

public:
  /**
   * Constructor
   */ 
  RoadGeometry() {};

  /**
   * Return the fitted polynomial coefficients
   */ 
  Eigen::VectorXd &getPolynomial() { return polynomial;}

  /**
   * Set the center trajectory
   * @param x the x trajectory
   * @param y the y trajectory
   * @param maxFitOrder maximal polynomial fit order default is 5
   * @param maxFitError maximal polynomial fit error default is 0.5
   */ 
  void setCenter(std::vector<double> &x, std::vector<double> &y, int maxFitOrder = 5, double maxFitError=0.5);

  /**
   * Return the y coordinate of the center at x
   * @param x the X coordinate
   */ 
  double centerY(double x);

  /**
   * Return the y coordinate of the center at x
   * @param x the X coordinate
   */ 
  CppAD::AD<double> centerY(const CppAD::AD<double> &x);

  /**
   * Compute the orientation of the road at x coordinate px
   * @param px the x coordinate
   * @param dir the moving direction, >= 0 for moving along the positive x, < 0 for the negative direction
   */ 
  double orientation(double px, double dir=1);

  /**
   * Compute the orientation of the road at x coordinate px
   * @param px the x coordinate
   * @param dir the moving direction, >= 0 for moving along the positive x, < 0 for the negative direction
   */ 
  CppAD::AD<double> orientation(const CppAD::AD<double> &px, double dir=1);

  /**
   * Compute the change in orientation of the road between two points
   * @param x0 the x coordinate of the first point
   * @param x1 the x coordinate of the second point
   */ 
  double computeOrientationChange(double x0, double x1);
  
  /**
   * Compute the cross track error
   * @param px the x coordinate
   * @param py the y coordinate
   */ 
  double cte(double px, double py);

  /**
   * Compute the cross track error
   * @param px the x coordinate
   * @param py the y coordinate
   */ 
  CppAD::AD<double> cte(const CppAD::AD<double> &px, const CppAD::AD<double> &py);
};

#endif