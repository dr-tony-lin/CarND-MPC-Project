#include <math.h>
#include <iostream>
#include "Eigen/QR"
#include "utils.h"
#include "Config.h"

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

double polypsi(Eigen::VectorXd poly, double x, double dir) {
  double psi = -atan(polyder(poly, x));
  if (dir < 0) {
    psi += M_PI;
  }
  return normalizeAngle(psi);
}

CppAD::AD<double> polypsi(Eigen::VectorXd poly, CppAD::AD<double> x, double dir) {
  CppAD::AD<double> psi = -CppAD::atan(polyder(poly, x));
  if (dir < 0) {
    psi += M_PI;
  }
  return normalizeAngle(psi);
}

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
double computeSpeedTarget(double angle, double max) {
  double y = fabs(angle);
  for (int i = 0; i < Config::steers.size(); i++) {
    if (y <= Config::steers[i]) {
      if (Config::steerSpeeds.size() > i) {
        return Config::steerSpeeds[i];
      }
      else {
        return Config::steerSpeeds.back();
      }
    }
  }

  return Config::steerSpeeds.back();
}

/**
 * Simple logic to adjust speed according to steering angle
 * @param angle the angle
 * @param max the maximal speed permitted
 */ 
CppAD::AD<double>  computeSpeedTarget(CppAD::AD<double>  angle, double max) {
  CppAD::AD<double>  y = CppAD::fabs(angle);
  for (int i = 0; i < Config::steers.size(); i++) {
    if (y <= Config::steers[i]) {
      if (Config::steerSpeeds.size() > i) {
        return Config::steerSpeeds[i];
      }
      else {
        Config::steerSpeeds.back();
      }
    }
  }

  return Config::steerSpeeds.back();
}

/**
 * Simple logic to compute throttle from acceleration.
 * @param accel the acceleration to reach
 * @param target the target speed
 * @param max_accel the maximal acceleration
 * @param max_decel the maximal deceleration
 */ 
double computeThrottle(double accel, double target, double max_accel, double max_decel) {
  // the throttle to keep if no accel, divide by Config::maxSpeed is a rough estimate of target speed,
  // and throttle to keep for the speed
  double keep = target / Config::maxSpeed;
  if (accel >= 0) { // acceleration
    if (accel < 0.001) { // small acceleration, keep the mimimal throttle
      return keep;
    }
    else { // otherwise compute the throttle
      return std::fmin(1, keep + (1 - keep) * accel / max_accel);
    }
  }
  else {
    if (accel <= -15) { // deceleration
      return -1;
    } else if (accel < -10) { // deceleration
      return -0.95 - (1 - 0.95) * accel / max_decel;
    } else if (accel < -5) { // deceleration
      return -0.9 - (1 - 0.9) * accel / max_decel;
    }
    return -0.85 - (1 - 0.85) * accel / max_decel;
  }
}

void globalToVehicle(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory, 
                      double px, double py, double psi) {
  double cosine = std::cos(psi);
  double sine = std::sin(psi);
  for (int i = 0; i < x_trajectory.size(); i++) {
    double vx = x_trajectory[i] - px;
    double vy = y_trajectory[i] - py;
    x_trajectory[i] = vx * cosine + vy * sine;
    y_trajectory[i] = vy * cosine - vx * sine;
  }
}

void globalToVehicle(double &x, double &y, double px, double py, double psi) {
  double cosine = std::cos(psi);
  double sine = std::sin(psi);
  double vx = x - px;
  double vy = y - py;
  x = vx * cosine + vy * sine;
  y = vy * cosine - vx * sine;
}

void vehicleToGlobal(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory,
                      double px, double py, double psi) {
  double cosine = std::cos(psi);
  double sine = std::sin(psi);
  for (int i = 0; i < x_trajectory.size(); i++) {
    double gx = px + x_trajectory[i] * cosine - y_trajectory[i] * sine;
    double gy = py + x_trajectory[i] * sine + y_trajectory[i] * cosine;
    x_trajectory[i] = gx;
    y_trajectory[i] = gy;
  }
}

void vehicleToGlobal(double &x, double &y, double px, double py, double psi) {
  double cosine = std::cos(psi);
  double sine = std::sin(psi);
  double gx = px + x * cosine - y * sine;
  double gy = py + x * sine + y * cosine;
  x = gx;
  y = gy;
}

// Implements a simple car motion model
void moveVehicle(double dt, double &x, double &y, double &psi, double &velocity,
  double steering, double acceleration, double length) {
  // perturb the acceleration and steering angle with gaussian noise
  
  // clamp the steering angle
  if (steering > Config::maxSteering) steering = Config::maxSteering;
  if (steering < -Config::maxSteering) steering = -Config::maxSteering;

  if (acceleration > Config::maxAcceleration) acceleration = Config::maxAcceleration;
  if (acceleration < Config::maxDeceleration) acceleration = Config::maxDeceleration;

  double dist = (velocity + acceleration * dt / 2) * dt;
  // Compute turing angle from steering angle
  double turn = tan(steering) * dist / length;
  double new_psi = normalizeAngle(psi + turn);
  double radius = 0;
  if (fabs(turn) > EPSILON) {  // turn is not 0
    // Compute the turn radius
    radius = dist / turn; // del psi = turn / dt
    // update x and y
    x += radius * (sin(new_psi) - sin(psi));
    y += radius * (cos(psi) - cos(new_psi));
  } else {  // turn is 0
    // update x and y
    x += dist * cos(psi);
    y += dist * sin(psi);
  }

  // update velocity
  velocity += acceleration * dt;
  if (velocity > Config::maxSpeed) velocity = Config::maxSpeed;
  std::cout << "Move: " << turn << " " << psi << " " << new_psi << " " << dist << " " << steering 
  << " " << radius <<  " " << acceleration << " " << velocity << std::endl;
  // update yaw
  psi = new_psi;
}
