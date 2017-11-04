#include <math.h>
#include <iostream>
#include "../utils/Config.h"
#include "Vehicle.h"

Vehicle::Vehicle(const Vehicle &another) {
  this->x = another.x;
  this->y = another.y;
  this->orientation = another.orientation;
  this->velocity = another.velocity;
  this->steering = another.steering;
  this->acceleration = another.acceleration;
}

Vehicle &Vehicle::operator=(const Vehicle &another) {
  this->x = another.x;
  this->y = another.y;
  this->orientation = another.orientation;
  this->velocity = another.velocity;
  this->steering = another.steering;
  this->acceleration = another.acceleration;
  return *this;
}

void Vehicle::update(double x, double y, double orientation, double velocity, double steering, double acceleration) {
  this->x = x;
  this->y = y;
  this->orientation = orientation;
  this->velocity = velocity;
  this->steering = steering;
  this->acceleration = acceleration;
}

double Vehicle::computeSpeedTarget(double angle, double max) const {
  double y = fabs(angle);
  for (size_t i = 0; i < Config::steers.size(); i++) {
    if (y <= Config::steers[i]) {
      if (Config::steerSpeeds.size() > i) {
        return std::fmin(Config::steerSpeeds[i], max);
      }
      else {
        return std::fmin(Config::steerSpeeds.back(), max);
      }
    }
  }

  return std::fmin(Config::steerSpeeds.back(), max);
}

CppAD::AD<double>  Vehicle::computeSpeedTarget(CppAD::AD<double>  angle, double max) const {
  CppAD::AD<double>  y = CppAD::fabs(angle);
  for (size_t i = 0; i < Config::steers.size(); i++) {
    if (y <= Config::steers[i]) {
      if (Config::steerSpeeds.size() > i) {
        return Config::steerSpeeds[i] < max? Config::steerSpeeds[i]: max;
      }
      else {
        return Config::steerSpeeds.back() < max? Config::steerSpeeds.back(): max;
      }
    }
  }

  return Config::steerSpeeds.back() < max? Config::steerSpeeds.back(): max;
}

double Vehicle::computeYawChangeSpeedLimit(double yawChange, double max) const {
  double y = fabs(yawChange);
  for (size_t i = 0; i < Config::yawChanges.size(); i++) {
    if (y <= Config::yawChanges[i]) {
      if (Config::yawChangeSpeeds.size() > i) {
        return std::fmin(Config::yawChangeSpeeds[i], max);
      }
      else {
        return std::fmin(Config::yawChangeSpeeds.back(), max);
      }
    }
  }
  return std::fmin(Config::yawChangeSpeeds.back(), max);
}

double Vehicle::computeThrottle(double accel, double target, double max_accel, double max_decel) const {
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

void Vehicle::globalToVehicle(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const {
  double cosine = std::cos(this->orientation);
  double sine = std::sin(this->orientation);
  for (size_t i = 0; i < x_trajectory.size(); i++) {
    double vx = x_trajectory[i] - this->x;
    double vy = y_trajectory[i] - this->y;
    x_trajectory[i] = vx * cosine + vy * sine;
    y_trajectory[i] = vy * cosine - vx * sine;
  }
}

void Vehicle::globalToVehicle(double &x, double &y) {
  double cosine = std::cos(this->orientation);
  double sine = std::sin(this->orientation);
  double vx = x - this->x;
  double vy = y - this->y;
  x = vx * cosine + vy * sine;
  y = vy * cosine - vx * sine;
}

void Vehicle::vehicleToGlobal(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const {
  double cosine = std::cos(this->orientation);
  double sine = std::sin(this->orientation);
  for (size_t i = 0; i < x_trajectory.size(); i++) {
    double gx = this->x + x_trajectory[i] * cosine - y_trajectory[i] * sine;
    double gy =this->y + x_trajectory[i] * sine + y_trajectory[i] * cosine;
    x_trajectory[i] = gx;
    y_trajectory[i] = gy;
  }
}

void Vehicle::vehicleToGlobal(double &x, double &y) const {
  double cosine = std::cos(this->orientation);
  double sine = std::sin(this->orientation);
  double gx = this->x + x * cosine - y * sine;
  double gy = this->y + x * sine + y * cosine;
  x = gx;
  y = gy;
}

void Vehicle::move(double dt) {
    double dist = this->velocity * dt;
    // COmpute delta psi
    double delta_psi = this->steering * dist / this->length;
    // New psi
    double new_orientation = this->orientation + delta_psi;
    // New position
    double px = x + dist * cos(this->orientation);
    double py = y + dist * sin(this->orientation);
    // new velocity
    double v = this->velocity + this->acceleration * dt;
    if (this->velocity > Config::maxSpeed) this->velocity = Config::maxSpeed;
#ifdef VERBOSE_OUT
    std::cout << "Move: " << dt << ", pos: " << this->x << ", " << this->y << ", psi:" << this->orientation
              << ", steering: " << this->steering <<  ", accel: " << this->acceleration << ", velocity: "
              << this->velocity << ", delta psi: " << delta_psi << ", to: " << px << ", " << py 
              << ", psi: " << new_orientation << ", velocity: " << v << ", dist: " << dist << std::endl;
#endif
    // update yaw
    this->x = px;
    this->y = py;
    this->orientation = new_orientation;
    this->velocity = v;
}