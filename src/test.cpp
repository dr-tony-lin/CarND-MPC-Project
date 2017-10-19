#include <math.h>
#include "control/MPC.h"
#include "utils/utils.h"
#include "utils/Config.h"
#include "Eigen/Core"
#include "Eigen/QR"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main() {
  MPC mpc;
  int iters = 100;

  Config::load("../config.json");

  Eigen::VectorXd ptsx(6);
  Eigen::VectorXd ptsy(6);
  // ptsx << -61.09,-78.29172,-93.05002,-107.7717,-123.3917,-134.97;
  // ptsy << 92.88499,78.73102,65.34102,50.57938,33.37102,18.404;

  ptsx << -134.97,-145.1165,-158.3417,-164.3164,-169.3365,-175.4917;
  ptsy << 18.404,4.339378,-17.42898,-30.18062,-42.84062,-66.52898;

  ptsx << -145.1165,-158.3417,-164.3164,-169.3365,-175.4917,-176.9617;
  ptsy << 4.339378,-17.42898,-30.18062,-42.84062,-66.52898,-76.85062;
  
  ptsx << -164.3164,-169.3365,-175.4917,-176.9617,-176.8864,-175.0817;
  ptsy << -30.18062,-42.84062,-66.52898,-76.85062,-90.64063,-100.3206;

  // NOTE: free feel to play around with these
  // double x = -61.97283;
  // double y = 93.53992;
  // double psi = 3.857562;
  // double v = 33.06046;

  // double x = -146.7283;
  // double y = 1.660802;
  // double psi = 4.125825;
  // double v = 26.6806;
  // double x = -144.7913;
  // double y = 3.767814;
  // double psi = 0.03732295;
  // double v = 10.32361;
  // double x = -146.8912;
  // double y = 2.129487;
  // double psi = 0.4009452;
  // double v = 13.87815;

  double x = -166.0726;
  double y = -29.59644;
  double psi = 4.088;
  double v = 10.62756;

  double fiterr = 0;
  int order = 2;
  Eigen::VectorXd coeffs;
  do {
    coeffs = polyfit(ptsx, ptsy, order++);
    for (int i = 0; i < ptsx.size(); i++) {
      fiterr = square(ptsy[i] - polyeval(coeffs, ptsx[i]));
    }
  } while (fiterr > 1 && order < 10);

  std::cout << "Fit error: " << fiterr << std::endl;

  // TODO: calculate the cross track error
  double cte = polyeval(coeffs, x) - y;
  // TODO: calculate the orientation error
  std::cout << " polypsi: " << polypsi(coeffs, x, ptsx[1] - ptsx[0]) << ", ";
  double epsi = psi - polypsi(coeffs, x, ptsx[1] - ptsx[0]);

  Eigen::VectorXd state(8);
  state << x, y, psi, v, cte, epsi, 0, 0;

  std::vector<double> x_vals = {state[0]};
  std::vector<double> y_vals = {state[1]};
  std::vector<double> psi_vals = {state[2]};
  std::vector<double> v_vals = {state[3]};
  std::vector<double> cte_vals = {state[4]};
  std::vector<double> epsi_vals = {state[5]};
  std::vector<double> delta_vals = {};
  std::vector<double> a_vals = {};

  for (size_t i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;

    try {
      auto vars = mpc.Solve(state, coeffs, 40, ptsx[1] - ptsx[0]);

      x_vals.push_back(vars[0]);
      y_vals.push_back(vars[1]);
      psi_vals.push_back(vars[2]);
      v_vals.push_back(vars[3]);
      cte_vals.push_back(vars[4]);
      epsi_vals.push_back(vars[5]);

      delta_vals.push_back(vars[6]);
      a_vals.push_back(vars[7]);

      state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5], vars[6], vars[7];
      std::cout << "Cost " << vars[8] << std::endl;
      std::cout << "x = " << vars[0] << std::endl;
      std::cout << "y = " << vars[1] << std::endl;
      std::cout << "psi = " << vars[2] << std::endl;
      std::cout << "v = " << vars[3] << std::endl;
      std::cout << "cte = " << vars[4] << std::endl;
      std::cout << "epsi = " << vars[5] << std::endl;
      std::cout << "delta = " << vars[6] << ", " << vars[9] << ", " << vars[10] << std::endl;
      std::cout << "a = " << vars[7] << ", " << vars[11] << ", " << vars[12] << std::endl;
      std::cout << std::endl;
    } catch(std::string e) {
      std::cout << "Ipopt failed: " << e << std::endl;
    }
  }

  // Plot values
  // NOTE: feel free to play around with this.
  // It's useful for debugging!
  plt::subplot(5, 1, 1);
  plt::title("CTE");
  plt::plot(cte_vals);
  plt::subplot(5, 1, 2);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(5, 1, 3);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::subplot(5, 1, 4);
  plt::title("Route");
  plt::plot(std::vector<double>(ptsx.data(), ptsx.data() + ptsx.size()),
            std::vector<double>(ptsy.data(), ptsy.data() + ptsy.size()), "r");
  plt::plot(x_vals, y_vals);
  
  plt::subplot(5, 1, 5);
  plt::title("Psi");
  plt::plot(psi_vals);
  plt::show();
}