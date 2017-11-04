#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"
#include "../model/RoadGeometry.h"
#include "../model/Vehicle.h"

using namespace std;
using Eigen::VectorXd;

class MPC {
  /**
   * Ipopt options
   */ 
  string options;

  /**
   * The vehicle
   */ 
  Vehicle vehicle;

  /**
   * The current road geometry
   */ 
  RoadGeometry roadGeometry;

 public:
  MPC();

  virtual ~MPC();

  /** Solve the model given an initial state, polynomial coefficients, and target velocity
   * Return the first actuations
   * @param state the initial state
   * @param target_velocity the target velocity
   * @param x_trajectory control trjectory for x axis, default NULL
   * @param y_trajectory control trjectory for y axis, default NULL
   * @param dir moving direction, should be any positive number
   * @return a vector of x, y, psi, v, cte, epsi, delta, a, and cost
   */ 
  vector<double> solve(VectorXd &state, double target_velocity,
    vector<double> *x_trajectory=NULL, vector<double> *y_trajectory=NULL, double dir=1);
  
  /**
   * Run the controller
   * @param vehicle the vehicle
   * @param ptsx the x trajectory of the center line
   * @param ptsy the y trajectory of the center line
   * @param x_trajectory the x trajectory of the center line in the target's coordinate
   * @param y_trajectory the y trajectory of the center line in the target's coordinate
   * @return a vector of x, y, psi, v, steer (normalized to [-1, 1]), a, cte, and epsi
   */
  vector<double> run(Vehicle &vehicle, vector<double> &ptsx, 
    vector<double> &ptsy, vector<double> *x_trajectory = NULL, vector<double> *y_trajectory = NULL);
};

#endif /* MPC_H */
