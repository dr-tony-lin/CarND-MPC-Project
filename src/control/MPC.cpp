#include <iostream>
#include "MPC.h"
#include <cppad/cppad.hpp>
#include <cppad/ipopt/solve.hpp>
#include "Eigen/Core"
#include "../utils/utils.h"
#include "../utils/Config.h"

using CppAD::AD;
using namespace std;
using Eigen::VectorXd;

class FG_eval {
  /**
   * The target velocity to reach or maintain
   */ 
  double target_velocity;

  /**
   * Moving direction, should be positive
   */ 
  double dir;

  /**
   * Fitted polynomial coefficients
   */ 
  VectorXd &coeffs;

 public:
  /**
   * Constructor
   * @param coeffs the fitted polynomial coefficients
   * @param target_velocity the target velocity to reach or maintain
   * @param dir the moving direction, should be any positive number
   */ 
  FG_eval(VectorXd &coeffs, double target_velocity, double dir=1): coeffs(coeffs) {
    this->target_velocity = target_velocity;
    this->dir = dir;
  }

  typedef CPPAD_TESTVECTOR(AD<double>) ADvector;
  void operator()(ADvector& fg, const ADvector& vars) {
    size_t N = Config::N;
    double dt = Config::dt;
    vector<double> cost_weights = Config::weights;
    
    // Starting indices
    size_t x_start = 0;
    size_t y_start = x_start + N;
    size_t psi_start = y_start + N;
    size_t v_start = psi_start + N;
    size_t cte_start = v_start + N;
    size_t epsi_start = cte_start + N;
    size_t delta_start = epsi_start + N;
    size_t a_start = delta_start + N - 1;

    // `fg` a vector of the cost constraints, `vars` is a vector of variable values (state & actuators)
    
    // Compute costs
    fg[0] = 0;

    // Penalize large  CTE, epsi, and v error
    for (size_t i = 0; i < N; i++) {
      fg[0] += square(vars[cte_start + i]) * cost_weights[0];
      // Penalize large psi more
      if (CppAD::fabs(vars[epsi_start + i]) > Config::epsiPanic) { // large epsi
          fg[0] += square(vars[epsi_start + i]) * cost_weights[10];
      }
      else {
        fg[0] += square(vars[epsi_start + i]) * cost_weights[1];
      }

      // Penalize speed with the target speed
      fg[0] += square(vars[v_start + i]-computeSpeedTarget(vars[psi_start + i], Config::maxSpeed))*cost_weights[2];
      if (vars[v_start + i] < 0) { // Penalize negative speed
        fg[0] += square(vars[v_start + i]) * cost_weights[9];
      }
    }

    // Minimize the use of acceleration actuators.
    for (size_t i = 0; i < N - 1; i++) {
      fg[0] += square(vars[delta_start + i]) * cost_weights[3];

      if (vars[a_start + i] > 0) { // penalize large acceleration, but not deceleration
        fg[0] += square(vars[a_start + i]) * cost_weights[6];
      }

      // Penalize large deceleration while speed is low 
      if (vars[a_start + i] < 0 && vars[v_start + i] < vars[a_start + i] ) {
        fg[0] += square(vars[v_start + i] - vars[a_start + i]) * cost_weights[8];
      }
    }

    // Penalize large delta and acceleration changes
    for (size_t i = 0; i < N-2; i++) { 
      fg[0] += square(vars[delta_start + i + 1] - vars[delta_start + i]) * cost_weights[4];
      fg[0] += square(vars[a_start + i + 1] - vars[a_start + i]) * cost_weights[7];
    }

    fg[1 + x_start] = vars[x_start];
    fg[1 + y_start] = vars[y_start];
    fg[1 + psi_start] = vars[psi_start];
    fg[1 + v_start] = vars[v_start];
    fg[1 + cte_start] = vars[cte_start];
    fg[1 + epsi_start] = vars[epsi_start];
    for (size_t i = 1; i < N; i++) {
      // The state at time i+1 .
      AD<double> x1 = vars[x_start + i];
      AD<double> y1 = vars[y_start + i];
      AD<double> psi1 = vars[psi_start + i];
      AD<double> v1 = vars[v_start + i];
      AD<double> cte1 = vars[cte_start + i];
      AD<double> epsi1 = vars[epsi_start + i];

      // The state at time i.
      AD<double> x0 = vars[x_start + i - 1];
      AD<double> y0 = vars[y_start + i - 1];
      AD<double> psi0 = vars[psi_start + i - 1];
      AD<double> v0 = vars[v_start + i - 1];
      AD<double> cte0 = vars[cte_start + i - 1];
      AD<double> epsi0 = vars[epsi_start + i - 1];
      AD<double> delta0 = vars[delta_start + i - 1];
      AD<double> a0 = vars[a_start + i - 1];

      // v0 * dt
      AD<double> vdt = v0 * dt;
      // new psi
      AD<double> psi = psi0 + delta0 * vdt / Config::Lf;
      fg[1 + x_start + i] = x1 - (x0 + CppAD::cos(psi0) * vdt);
      fg[1 + y_start + i] = y1 - (y0 + CppAD::sin(psi0) * vdt);
      fg[1 + psi_start + i] = psi1 - psi;
      AD<double> v = v0 + a0 * dt;
      if (v < 0) { // no negative speed!
        v = 0;
      }
      fg[1 + v_start + i] = v1 - v;
      // we want the errors to be close to 0
      fg[1 + cte_start + i] = cte1 - ((polyeval(coeffs, x0) - y0) + CppAD::sin(epsi0) * vdt);
      fg[1 + epsi_start + i] = epsi1 - (psi - polypsi(coeffs, x0, 1.0));
    }
  }
};

//
// MPC class definition implementation.
//
MPC::MPC() {
  // options for IPOPT solver
  // Uncomment this if you'd like more print information
#ifdef TRACE_IPOPT
  options += "Integer print_level  5\n";
#else
  options += "Integer print_level  0\n";
#endif
  // NOTE: Setting sparse to true allows the solver to take advantage
  // of sparse routines, this makes the computation MUCH FASTER. If you
  // can uncomment 1 of these and see if it makes a difference or not but
  // if you uncomment both the computation time should go up in orders of
  // magnitude.
  options += "Sparse  true        forward\n";
  options += "Sparse  true        reverse\n";
  options += "String  linear_solver  mumps\n";
  // NOTE: Currently the solver has a maximum time limit of 0.5 seconds.
  // Value below 0.25 might not converge well
  options += "Numeric max_cpu_time          " + std::to_string(Config::ipoptTimeout) + "\n";

}
MPC::~MPC() {}

vector<double> MPC::solve(VectorXd &state, double target_velocity, vector<double> *x_trajectory,
  vector<double> *y_trajectory, double dir) {
  typedef CPPAD_TESTVECTOR(double) Dvector;
  size_t N = Config::N;

  // Starting indices
  size_t x_start = 0;
  size_t y_start = x_start + N;
  size_t psi_start = y_start + N;
  size_t v_start = psi_start + N;
  size_t cte_start = v_start + N;
  size_t epsi_start = cte_start + N;
  size_t delta_start = epsi_start + N;
  size_t a_start = delta_start + N - 1;

  // Set the number of model variables (includes both states and inputs).
  size_t n_vars = 6 * N + 2 * (N-1);

  // Set the number of constraints
  size_t n_constraints = N * 6;

  Dvector vars(n_vars);
  Dvector vars_lowerbound(n_vars);
  Dvector vars_upperbound(n_vars);
  for (size_t i = 0; i < n_vars; i++) {
    // Initial value of the independent variables, SHOULD BE 0 besides initial state.
    vars[i] = 0.0;
  }

  // Set the initial variable values
  vars[x_start] = state[0];
  vars[y_start] = state[1];
  vars[psi_start] = state[2];
  vars[v_start] = state[3];
  vars[cte_start] = state[4];
  vars[epsi_start] = state[5];

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = 0; i < psi_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = psi_start; i < v_start; i++) {
    vars_lowerbound[i] = Config::yawLow;
    vars_upperbound[i] = Config::yawHigh;
  }
  
  // Set all non-actuators upper and lowerlimits
  // to the max negative and positive values.
  for (size_t i = v_start; i < cte_start; i++) {
    vars_lowerbound[i] = -Config::maxSpeed;
    vars_upperbound[i] = Config::maxSpeed;
  }

  // Set CTE upper bound and lower bound
  for (size_t i = cte_start; i < delta_start; i++) {
    vars_lowerbound[i] = -1.0e19;
    vars_upperbound[i] = 1.0e19;
  }

  // The upper and lower limits of steering angle
  for (size_t i = delta_start; i < a_start; i++) {
    vars_lowerbound[i] = -Config::maxSteering;
    vars_upperbound[i] = Config::maxSteering;
  }

  // Acceleration/decceleration upper and lower limits.
  for (size_t i = a_start; i < n_vars; i++) {
    vars_lowerbound[i] = Config::maxDeceleration;
    vars_upperbound[i] = Config::maxAcceleration;
  }

  // Lower and upper limits for the constraints
  // Should be 0 besides initial state.
  Dvector constraints_lowerbound(n_constraints);
  Dvector constraints_upperbound(n_constraints);
  for (size_t i = 0; i < n_constraints; i++) {
    constraints_lowerbound[i] = 0;
    constraints_upperbound[i] = 0;
  }

  // Set constraint's initial value to be the state value
  constraints_lowerbound[x_start] = state[0];
  constraints_lowerbound[y_start] = state[1];
  constraints_lowerbound[psi_start] = state[2];
  constraints_lowerbound[v_start] = state[3];
  constraints_lowerbound[cte_start] = state[4];
  constraints_lowerbound[epsi_start] = state[5];

  constraints_upperbound[x_start] = state[0];
  constraints_upperbound[y_start] = state[1];
  constraints_upperbound[psi_start] = state[2];
  constraints_upperbound[v_start] = state[3];
  constraints_upperbound[cte_start] = state[4];
  constraints_upperbound[epsi_start] = state[5];

  // object that computes objective and constraints
  FG_eval fg_eval(poly, target_velocity, dir);

  // place to return solution
  CppAD::ipopt::solve_result<Dvector> solution;

  // solve the problem
  CppAD::ipopt::solve<Dvector, FG_eval>(
      options, vars, vars_lowerbound, vars_upperbound, constraints_lowerbound,
      constraints_upperbound, fg_eval, solution);

  // Check some of the solution values
  bool ok = solution.status == CppAD::ipopt::solve_result<Dvector>::success;

  if (!ok) {
#ifdef EXIT_ON_IPOPT_FAILURE
    throw std::string("Ipopt failed with ") + std::to_string(solution.status);
#else
    std::cout << "Ipopt failed with " + std::to_string(solution.status) << std::endl;
#endif
  }

  // Update x, y trajectory if provided
  if (x_trajectory) {
    for (size_t i = 0; i < N; i++) {
      x_trajectory->push_back(solution.x[x_start+i]);
      y_trajectory->push_back(solution.x[y_start+i]);
    }
  }

  #ifdef VERBOSE_OUT
  std::cout << "Solution: " << solution.x << std::endl;
#endif

  // Return the first actuator values. The variables can be accessed with
  // `solution.x[i]`.
  // Ther first element of each state varaible is the initial state value, therefore, the first actuator
  // values will be their second elements, delta and a don't have initial state value, and the first
  // actuator values will be their first element. Also return the cost as the last element of the result
  return {solution.x[x_start+1], solution.x[y_start+1], solution.x[psi_start+1], solution.x[v_start+1],
          solution.x[cte_start+1], solution.x[epsi_start+1], solution.x[delta_start], solution.x[a_start],
          solution.obj_value};
}

vector<double> MPC::run(double px, double py, double psi, double v, double steer, vector<double> &ptsx,
    vector<double> &ptsy,std::vector<double> *x_trajectory, std::vector<double> *y_trajectory) {
  globalToVehicle(ptsx, ptsy, px, py, psi);
  // Compute the polynomial coefficients
  VectorXd xVals(ptsx.size());
  VectorXd yVals(ptsx.size());
  for (int i = 0; i < ptsx.size(); i++) {
    xVals[i] = ptsx[i];
    yVals[i] = ptsy[i];
  }

  double fiterr = 0;
  int order = 2;
  do { // Compute the polynomial coefficients to an order that has small fitting error
    poly = polyfit(xVals, yVals, order++);
    for (int i = 0; i < ptsx.size(); i++) {
      fiterr = square(ptsy[i] - polyeval(poly, ptsx[i]));
    }
  } while (fiterr > 1 && order < Config::maxPolyOrder);
#ifdef VERBOSE_OUT
  cout << "Polynominal: " << poly << endl;
#endif

  // Calculate the CTE, origin is now 0, 0
  double cte = polyeval(poly, 0);
  // Calculate the psi error, origin and psi are now all 0
  double epsi = -polypsi(poly, 0, 1.0);

  // Compute the maximum speed according to the total psi change on the current center line curve
  double max_yaw_change = computeYawChange(poly, 0, ptsx[ptsx.size() - 1]);
  double max_speed = computeYawChangeSpeedLimit(max_yaw_change, Config::maxSpeed);
  // Compute the speed target according to steering
  double target_speed = computeSpeedTarget(steer, max_speed);
  double steer_value;

  // Set the yaw constraints to MPC
  if (max_yaw_change < 0) {
    Config::yawLow = max_yaw_change;
    Config::yawHigh = 0.1;
  }
  else {
    Config::yawLow = -0.1;
    Config::yawHigh = max_yaw_change;
  }

#ifdef VERBOSE_OUT
  cout << "Psi: " << psi<< ", ePsi: " << epsi << ", cte: " << cte << ", fit err: " << fiterr 
  << ", Yaw range: " << Config::yawLow << ", " << Config::yawHigh << ", order: " << order << endl;
#endif

  // Set the initial state, origin and psi are all 0
  VectorXd state(6);
  state << 0, 0, 0, v, cte, epsi;

  // Solve MPC
  auto result = solve(state, target_speed, x_trajectory, y_trajectory);
  double steer_angle = result[6];
  double accel = std::min(result[7], target_speed - v);
  steer_value = clamp(steer_angle / Config::maxSteering, -1.0, 1.0);
#ifdef VERBOSE_OUT
  cout << "Steering: " << steer_angle << ", " << result[9] << ", " << result[10] 
  << ", accel: " << result[7] << ", speed: " << result[3] << ", steer: " << steer_value
  << ", target speed: " << target_speed << ", CTE: " << result[4] << ", epsi: " << result[5] 
  << ", cost: " << result[8] << ", max speed: " << max_speed << endl;
#endif

  return {result[0], result[1], result[2], result[3], steer_value, accel, result[4], result[5]};
}

