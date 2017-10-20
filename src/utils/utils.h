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
 template<typename T> T polyder(Eigen::VectorXd coeffs, T &x) {
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
 * Compute the slop in radian of the polynomial at x in direction x
 * @param poly the polynomial
 * @param x the x value
 * @param dir the direction, only the sign is important, +x or -x direction
 */ 
extern double polypsi(Eigen::VectorXd poly, double x, double dir);

/**
 * Compute the slop in radian of the polynomial at x in direction x
 * @param poly the polynomial
 * @param x the x value of type CppAD::AD<double>
 * @param dir the direction, only the sign is important, +x or -x direction
 * @return the radian in type CppAD::AD<double>
 */
extern CppAD::AD<double> polypsi(Eigen::VectorXd poly, CppAD::AD<double> x, double dir);

/**
 * Update the vehicle position and psi after dt seconds
 * @param dt the time to move (in seconds)
 * @param x the x coordinate
 * @param y the y coordinate
 * @param psi the current orientation
 * @param velocity the current velocity (meter per second)
 * @param steering the current steering angle
 * @param acceleration the current acceleration (meter per second per second)
 * @param length the Lf of the vehicle
 */ 
extern void moveVehicle(double dt, double &x, double &y, double &psi, double &velocity,
  double steering, double acceleration, double length);

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
double computeSpeedTarget(double angle, double max);

/**
 * Simple logic to adjust speed according to steering angle,
 * assuming the polynomial is convex
 * @param poly the polynomial
 * @param x0 the starting position
 * @param x1 the ending position
 */ 
double computeYawChange(Eigen::VectorXd poly, double x0, double x1);

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
double computeYawChangeSpeedLimit(double yawChange, double max);

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
CppAD::AD<double> computeSpeedTarget(CppAD::AD<double> angle, double max);

/**
 * Simple logic to compute throttle from acceleration.
 * @param accel the acceleration to reach
 * @param target the target speed
 * @param max_accel the maximal acceleration
 * @param max_decel the maximal deceleration
 */ 
double computeThrottle(double accel, double target, double max_accel, double max_decel);

/**
 * Convert the global trajectory to the vehicle's coordinate
 * @param x_trajectory the x trajectory
 * @param y_trajectory the y trajectory
 * @param px x coordinate of the vehicle
 * @param py y coordinate of the vehicle
 * @param psi orientation of the vehicle
 */ 
extern void globalToVehicle(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory, 
                          double px, double py, double psi);

/**
 * Convert the global coordinate to the vehicle's coordinate
 * @param x the x coordinate
 * @param y the y coordinate
 * @param px x coordinate of the vehicle
 * @param py y coordinate of the vehicle
 * @param psi orientation of the vehicle
 */ 
void globalToVehicle(double &x, double &y, double px, double py, double psi);
                            
/**
 * Convert the trajectory in the vehicle's coordinate to the global coordinate
 * @param x_trajectory the x trajectory
 * @param y_trajectory the y trajectory
 * @param px x coordinate of the vehicle
 * @param py y coordinate of the vehicle
 * @param psi orientation of the vehicle
 */ 
void vehicleToGlobal(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory,
                          double px, double py, double psi);
/**
 * Convert the point from the vehicle's coordinate to the global coordinate
 * @param x the x coordinate
 * @param y the y coordinate
 * @param px x coordinate of the vehicle
 * @param py y coordinate of the vehicle
 * @param psi orientation of the vehicle
 */ 
extern void vehicleToGlobal(double &x, double &y, double px, double py, double psi);

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