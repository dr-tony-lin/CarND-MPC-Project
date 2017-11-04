#ifndef _UTILS_H_
#define _UTILS_H_
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen/Core"

/**
 * Mile per hour to meters per second
 * @param mph the value to convert
 */ 
inline double MpH2MpS(double mph) {
  return mph * 1609.34 / 3600.0;
}

/**
 * Meter per second to mile per hour
 * @param mps the value to convert
 */ 
inline double MpS2MpH(double mps) {
  return mps * 3600.0 / 1609.34;
}

/**
 * Evaluate a polynomial.
 * @param coeffs the coefficients
 * @param x the variable
 */ 
template<typename T> T polyeval(Eigen::VectorXd coeffs, T x) {
  T result = 0;
  for (int i = coeffs.size() - 1; i >= 0; i--) {
    result = result * x + coeffs[i];
  }
  return result;
}

/**
 * Evaluate the derivative of polynomial.
 * @param coeffs the coefficients
 * @param x the variable
 */ 
 template<typename T> T polyder(const Eigen::VectorXd coeffs, const T &x) {
  T result = 0;
  for (int i = coeffs.size() - 1; i >= 1; i--) {
    result = result * x + i * coeffs[i];
  }
  return result;
}

/**
 * Clamp a to min and max range
 * @param a the value to clamp
 * @param min the minimal value
 * @param max the maximal value
 * @retur the clampped value
 */
template<typename T> T clamp(const T a, const T min, const T max) {
  return a < min? min: (a > max? max: a);
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
extern Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order);

/**
 * Convert degree to radian
 * @param x degree to convert
 */ 
inline double deg2rad(double x) { return x * M_PI / 180; }

/**
 * Convert radian to degree
 * @param x radian to convert
 */
inline double rad2deg(double x) { return x * 180 / M_PI; }

/**
 * Compute square, as not sure if pow if optimized
 * @param a the value to square
 */ 
template<typename T> T square(const T &a) {return a * a;}

/**
 * Normalize the given angle to [-PI, PI) range
 * @param a the angle
 */                      
template<typename T> T normalizeAngle(const T &a) {
  T result = a;
  while (result >= M_PI) result -= 2. * M_PI;
  while (result < -M_PI) result += 2. * M_PI;
  return result;
}

// SMall number
const double EPSILON = 1E-6;

#endif