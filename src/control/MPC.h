#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen/Core"

using namespace std;

class MPC {
  /**
   * Ipopt options
   */ 
  std::string options;

 public:
  MPC();

  virtual ~MPC();

  /** Solve the model given an initial state, polynomial coefficients, and target velocity
   * Return the first actuations
   * @param state the initial state
   * @param coeffs the polynomial coefficients for the road trajectory
   * @param target_velocity the target velocity
   * @param dir moving direction
   * @param x_trajectory control trjectory for x axis
   * @param y_trajectory control trjectory for y axis
   */ 
  vector<double> Solve(Eigen::VectorXd &state, Eigen::VectorXd &coeffs, double target_velocity,
        double dir, std::vector<double> *x_trajectory=NULL, std::vector<double> *y_trajectory=NULL);
};

#endif /* MPC_H */
