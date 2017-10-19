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
   * The maximal polynominal order
   */
  static int maxPolyOrder;

  /** 
   * Maximal change in steering
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
   * Length from the front wheels to the center of the back wheels
   */ 
  static double Lf;

  /**
   * epsi's reference value
   */ 
  static double epsiRef;

  /**
   * epsi's panic threshold
   */ 
  static double epsiPanic;

  /**
   * Cost weights: 0: cte, 1: epsi, 2: v, 3: delta delta, 4: delta, 5: a,
   * 6: delta a, 7: large deceleration low velocity, 8: negative speed,
   * 9: out of range psi [-PI, PI], 10: out of range epsi
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
   * Load configuration from file
   * @param fileName name of the config file 
   */ 
  static void load(std::string fileName);
};

#endif