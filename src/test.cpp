#include <math.h>
#include "control/MPC.h"
#include "utils/utils.h"
#include "utils/Config.h"
#include "Eigen/Core"
#include "Eigen/QR"
#include "matplotlibcpp.h"
#include "model/Vehicle.h"

namespace plt = matplotlibcpp;

int main() {
  MPC mpc;
  int iters = 25;

  Config::load("../config-stable.json");

  // vector<double> ptsx = {-134.97,-145.1165,-158.3417,-164.3164,-169.3365,-175.4917}; 
  // vector<double> ptsy = {18.404,4.339378,-17.42898,-30.18062,-42.84062,-66.52898};
  // double x = -146.8912;
  // double y = 2.129487;
  // double psi = 0.4009452;
  // double v = 13.87815;

  // vector<double> ptsx = {-164.3164,-169.3365,-175.4917,-176.9617,-176.8864,-175.0817};
  // vector<double> ptsy = {-30.18062,-42.84062,-66.52898,-76.85062,-90.64063,-100.3206};

  // double x = -166.0726;
  // double y = -29.59644;
  // double psi = 4.088;
  // double v = 30.62756;

  // double x = -144.7913;
  // double y = 3.767814;
  // double psi = 0.03732295;
  // double v = 10.32361;

  // vector<double> ptsx = {-61.09,-78.29172,-93.05002,-107.7717,-123.3917,-134.97};
  // vector<double> ptsy = {92.88499,78.73102,65.34102,50.57938,33.37102,18.404};
  // double x = -61.97283;
  // double y = 93.53992;
  // double psi = 3.857562;
  // double v = 33.06046;

  vector<double> ptsx = {-145.1165,-158.3417,-164.3164,-169.3365,-175.4917,-176.9617};
  vector<double> ptsy = {4.339378,-17.42898,-30.18062,-42.84062,-66.52898,-76.85062};
  double x = -146.7283;
  double y = 1.660802;
  double psi = 4.125825;
  double v = 26.6806;

  std::vector<double> x_vals;
  std::vector<double> y_vals;
  std::vector<double> psi_vals;
  std::vector<double> v_vals;
  std::vector<double> cte_vals;
  std::vector<double> epsi_vals;
  std::vector<double> delta_vals;
  std::vector<double> a_vals;

  Vehicle vehicle;
  RoadGeometry roadGeometry;

  vehicle.setLength(Config::Lf);
  vehicle.update(x, y, psi, v, 0, 0);

  vector<double> vars = mpc.run(vehicle, ptsx, ptsy);

  x_vals.push_back(vars[0]);
  y_vals.push_back(vars[1]);
  psi_vals.push_back(vars[2]);
  v_vals.push_back(vars[3]);
  cte_vals.push_back(vars[6]);
  epsi_vals.push_back(vars[7]);

  delta_vals.push_back(vars[4]*Config::maxSteering);
  a_vals.push_back(vars[5]);

  Eigen::VectorXd state(6);
  state << vars[0], vars[1], vars[2], vars[3], vars[6], vars[7];

  for (int i = 0; i < iters; i++) {
    std::cout << "Iteration " << i << std::endl;
    try {
      auto vars = mpc.solve(state, 40);
      
      x_vals.push_back(vars[0]);
      y_vals.push_back(vars[1]);
      psi_vals.push_back(vars[2]);
      v_vals.push_back(vars[3]);
      cte_vals.push_back(vars[4]);
      epsi_vals.push_back(vars[5]);

      delta_vals.push_back(vars[6]);
      a_vals.push_back(vars[7]);

      state << vars[0], vars[1], vars[2], vars[3], vars[4], vars[5];
      std::cout << "Cost " << vars[8] << std::endl;
      std::cout << "x = " << vars[0] << std::endl;
      std::cout << "y = " << vars[1] << std::endl;
      std::cout << "psi = " << vars[2] << std::endl;
      std::cout << "v = " << vars[3] << std::endl;
      std::cout << "cte = " << vars[4] << std::endl;
      std::cout << "epsi = " << vars[5] << std::endl;
      std::cout << "delta = " << vars[6] << std::endl;
      std::cout << "a = " << vars[7] << std::endl;
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
  plt::title("ePsi");
  plt::plot(epsi_vals);
  plt::subplot(5, 1, 3);
  plt::title("Delta (Radians)");
  plt::plot(delta_vals);
  plt::subplot(5, 1, 4);
  plt::title("Velocity");
  plt::plot(v_vals);

  plt::subplot(5, 1, 5);
  plt::title("Route");

  plt::plot(std::vector<double>(ptsx.data(), ptsx.data() + ptsx.size()),
            std::vector<double>(ptsy.data(), ptsy.data() + ptsy.size()), "r");
  plt::plot(x_vals, y_vals);
  plt::show();
}