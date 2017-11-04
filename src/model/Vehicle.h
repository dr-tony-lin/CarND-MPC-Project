#ifndef _MODEL_CAR_H_
#define _MODEL_CAR_H_
#include <vector>
#include <cppad/cppad.hpp>
#include "Eigen/QR"

class Vehicle {
  /**
   * X coordinate
   */ 
  double x;
  
  /**
   * Y coordinate
   */ 
  double y;

  /**
   * Orientation
   */ 
  double orientation;

  /**
   * Velocity
   */ 
  double velocity;

  /**
   * Steering angle
   */ 
  double steering;

  /**
   * Acceleration
   */ 
  double acceleration;

  /**
   * Length of the vehicle netween the front and back wheels
   */ 
  double length;

public:
  /**
   * Constructor
   * @param length Length of the vehicle netween the front and back wheels
   */ 
  Vehicle() {}

  /** Copy constructor
   * @param another another vehicle to copy
   */ 
  Vehicle(const Vehicle &another);

  /**
   * Assignment operator
   * @param another another vehicle to copy
   */ 
  Vehicle &operator=(const Vehicle &another);

  /**
   * Set length of the vehicle between the front and back wheels
   * @param length the length
   */ 
  void setLength(double length) { this->length = length;}

  /**
   * Get length of the vehicle between the front and back wheels
   */  
  double getLength() const { return length;}

  /**
   * Get the x coordinate of the vehicle
   */  
  double getX() const { return x;}

  /**
   * Get the y coordinate of the vehicle
   */  
  double getY() const { return y;}

  /**
   * Get the orientation of the vehicle
   */  
  double getOrientation() const { return orientation;}

  /**
   * Get the velocity of the vehicle
   */  
  double getVelocity() const { return velocity;}

  /**
   * Get the steering angle of the vehicle
   */  
  double getSteering() const { return steering;}

  /**
   * Get the acceleration of the vehicle
   */  
  double getAcceleration() const { return acceleration;}

  /**
   * Simple logic to adjust speed according to steering angle
   * @param angle the angle
   * @param max the maximal speed permitted
   */ 
  double computeYawChangeSpeedLimit(double yawChange, double max) const;

  /**
   * Simple logic to adjust speed according to steering angle
   * @param angle the steering angle
   * @param max the maximal speed permitted
   */ 
  double computeSpeedTarget(double angle, double max) const;

  /**
   * Simple logic to adjust speed according to steering angle
   * @param angle the steering angle
   * @param max the maximal speed permitted
   */ 
  CppAD::AD<double> computeSpeedTarget(CppAD::AD<double> angle, double max) const;

  /**
   * Simple logic to compute throttle from acceleration.
   * @param accel the acceleration to reach
   * @param target the target speed
   * @param max_accel the maximal acceleration
   * @param max_decel the maximal deceleration
   */ 
  double computeThrottle(double accel, double target, double max_accel, double max_decel) const;

  /**
   * Convert the global trajectory to the vehicle's coordinate
   * @param x_trajectory the x trajectory
   * @param y_trajectory the y trajectory
   */ 
  void globalToVehicle(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const;

  /**
   * Convert the global coordinate to the vehicle's coordinate
   * @param x the x coordinate
   * @param y the y coordinate
   */ 
  void globalToVehicle(double &x, double &y);
                              
  /**
   * Convert the trajectory in the vehicle's coordinate to the global coordinate
   * @param x_trajectory the x trajectory
   * @param y_trajectory the y trajectory
   */ 
  void vehicleToGlobal(std::vector<double> &x_trajectory, std::vector<double> &y_trajectory) const;
  /**
   * Convert the point from the vehicle's coordinate to the global coordinate
   * @param x the x coordinate
   * @param y the y coordinate
   */ 
  void vehicleToGlobal(double &x, double &y) const;

  /**
   * Move the vehicle for dt seconds
   */ 
  void move(double dt);

  /**
   * Update the vehicle's position, orientation, velocity, steering, and acceleration
   * @param x the x coordinate
   * @param y the y coordinate
   * @param orientation the current orientation
   * @param velocity the current velocity (meter per second)
   * @param steering the current steering angle
   * @param acceleration the current acceleration (meter per second per second)
   */ 
  void update(double x, double y, double orientation, double velocity, double steering, double acceleration);
};

#endif