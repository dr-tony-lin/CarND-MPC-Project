#ifndef _UTILS_CONFIG_H_
#define _UTILS_CONFIG_H_
#include <string>
#include <vector>
#include <fstream>
#include <streambuf>
#include "json.hpp"

struct Config {
public:
  /**
   * Index of for CTE in the cost weight array
   */ 
  static const int WEIGHT_CTE = 0;

  /**
   * Index for epsi in the cost weight array
   */ 
  static const int WEIGHT_EPSI = 1;

  /**
   * Index for velocity in the cost weight array
   */ 
  static const int WEIGHT_V = 2;

  /**
   * Index for delta in the cost weight array
   */ 
  static const int WEIGHT_DELTA = 3;

  /**
   * Index for delta change in the cost weight array
   */ 
  static const int WEIGHT_DDELTA = 4;

  /**
   * Index for acceleration in the cost weight array
   */ 
  static const int WEIGHT_A = 6;
  /**
   * Index for acceleration change in the cost weight array
   */ 
  static const int WEIGHT_DA = 7;
  /**
   * Index for large deceleration at low velocity in the cost weight array
   */ 
  static const int WEIGHT_DECEL_LOW_V = 8;
  /**
   * Index for negative velocity in the cost weight array
   */ 
  static const int WEIGHT_NEG_V = 9;
  
  /**
   * Index for large epsi in the cost weight array
   */ 
  static const int WEIGHT_LARGE_EPSI = 10;

  /**
   * Index for large CTE in the cost weight array
   */ 
  static const int WEIGHT_LARGE_CTE = 11;

  /**
   * The timestep length
   */
  static size_t N;

  /**
   * The duration (in second)
   */ 
  static double dt;

  /**
   * The IPOPT timeout (in second)
   */ 
  static double ipoptTimeout;

  /**
   * Control latency in millies
   */ 
  static long latency;

  /**
   * look ahead prediction time in seconds
   */ 
  static double lookahead;
  
  /**
   * The maximal polynomial fitting order
   */
  static int maxFitOrder;

  /**
   * The maximal polynomial fit error
   */
  static double maxFitError;

  /** 
   * Maximal steering angle
   */ 
  static double maxSteering;

  /**
   * Maximal acceleration of vehicle
   */ 
  static double maxAcceleration;

  /**
   * Maximal deceleration of vehicle
   * */
  static double maxDeceleration;

  /**
   * Maximal speed of vehicle
   */ 
  static double maxSpeed;
  
  /**
   * Minimal yaw
   */ 
  static double yawLow;
  
  /**
   * Maximal yaw
   */ 
  static double yawHigh;

  /**
   * Steering angle adjustment threshold
   */ 
  static double steerAdjustmentThresh;

  /**
   * Steering angle adjustment ratio with respect to yaw change
   */ 
  static double steerAdjustmentRatio;

  /**
   * Length from the front wheels to the center of the back wheels
   */ 
  static double Lf;

  /**
   * CTE's panic threshold
   */ 
  static double ctePanic;

  /**
   * epsi's panic threshold
   */ 
  static double epsiPanic;

  /**
   * Cost weights: 0: cte, 1: epsi, 2: v, 3: delta, 4: delta delta, 5: not used,
   * 6: a, 7: delta a, 8: large deceleration low velocity, 9: negative speed, 10: out of range epsi
   */ 
  static std::vector<double> weights;
  
  /**
   * Steer to map to speed target
   */ 
  static std::vector<double> steers;

  /**
   * Steer to map to throttle, the last one is for the rest
   */ 
  static std::vector<double> steerSpeeds;
  
  /**
   * Yaw change to speed maximum
   */ 
  static std::vector<double> yawChanges;
  
  /**
   * Yaw change to speed maximum, the last one is for the rest
   */ 
  static std::vector<double> yawChangeSpeeds;

  /**
   * Load configuration from file
   * @param fileName name of the config file 
   */ 
  static void load(std::string fileName);
};

#endif