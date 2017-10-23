#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;
using Eigen::VectorXd;

class MPC {
  /**
   * Ipopt options
   */ 
  string options;

  /**
   * The polynomial coefficient of the target trajectory
   */ 
  VectorXd poly;

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
   */ 
  vector<double> Solve(VectorXd &state, double target_velocity, vector<double> *x_trajectory=NULL,
    vector<double> *y_trajectory=NULL, double dir=1);
  
  /**
   * Run the controller
   * @param px the x coordinate of the target
   * @param py the y coordinate of the target
   * @param psi the orientation of the target
   * @param v the velocity of the target
   * @param steer thesteering angle of the target
   * @param ptsx the x trajectory of the center line
   * @param ptsy the y trajectory of the center line
   * @param x_trajectory the x trajectory of the center line in the target's coordinate
   * @param y_trajectory the y trajectory of the center line in the target's coordinate
   */
  vector<double> run(double px, double py, double psi, double v, double steer, vector<double> &ptsx, 
    vector<double> &ptsy, vector<double> *x_trajectory = NULL, vector<double> *y_trajectory = NULL);
};

#endif /* MPC_H */
