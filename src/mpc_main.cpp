#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <utility>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "control/MPC.h"
#include "utils/utils.h"
#include "utils/Config.h"
#include "json.hpp"

using namespace std;
using namespace chrono;

// for convenience
using json = nlohmann::json;

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

/**
 * Run MPC
 * @param px the x coordinate of the vehicle
 * @param py the y coordinate of the vehicle
 * @param psi the orientation of the vehicle
 * @param v the velocity of the vehicle
 * @param steer thesteering angle of the vehicle
 * @param ptsx the x trajectory of the center line
 * @param ptsy the y trajectory of the center line
 * @param x_trajectory the x trajectory of the center line in the vehicle's coordinate
 * @param y_trajectory the y trajectory of the center line in the vehicle's coordinate
 */
vector<double> run_mpc(double px, double py, double psi, double v, double steer, vector<double> &ptsx, vector<double> &ptsy,
         std::vector<double> *x_trajectory = NULL, std::vector<double> *y_trajectory = NULL) {
  globalToVehicle(ptsx, ptsy, px, py, psi);
  // Compute the polynomial coefficients
  Eigen::VectorXd xVals(ptsx.size());
  Eigen::VectorXd yVals(ptsx.size());
  for (int i = 0; i < ptsx.size(); i++) {
    xVals[i] = ptsx[i];
    yVals[i] = ptsy[i];
  }

  double fiterr = 0;
  int order = 2;
  static Eigen::VectorXd poly;
  do { // Compute the polynomial coefficients to an order that has small fitting error
    poly = polyfit(xVals, yVals, order++);
    for (int i = 0; i < ptsx.size(); i++) {
      fiterr = square(ptsy[i] - polyeval(poly, ptsx[i]));
    }
  } while (fiterr > 1 && order < Config::maxPolyOrder);
#ifdef VERBOSE_OUT
    cout << "Polynominal: " << poly << endl;
#endif

  // Calculate the CTE
  double cte = polyeval(poly, 0);
  // Calculate the psi error
  double px_psi = polypsi(poly, 0, ptsx[1] - ptsx[0]);
  double epsi = -px_psi;
#ifdef VERBOSE_OUT
  cout << "Psi: " << psi<< ", ePsi: " << epsi<< ", psi at px: " << px_psi << ", cte: " << cte << ", fit err: " 
       << fiterr << ", order: " << order << endl;
#endif   
  Eigen::VectorXd state(6);
  state << 0, 0, 0, v, cte, epsi;

  MPC mpc;
  
  // Compute the target speed using the estimated future psi
  double max_yaw_change = computeYawChange(poly, px, ptsx[ptsx.size() - 1]);
  double max_speed = computeYawChangeSpeedLimit(max_yaw_change, Config::maxSpeed);
  double target_speed = computeSpeedTarget(steer, max_speed);
  double steer_value;

  if (max_yaw_change < 0) {
    Config::yawLow = max_yaw_change * 0.5;
    Config::yawHigh = 0.1;
  }
  else {
    Config::yawLow = -0.1;
    Config::yawHigh = max_yaw_change * 0.5;
  }
  // Solve MPC
  auto result = mpc.Solve(state, poly, target_speed, ptsx[1] - ptsx[0], x_trajectory, y_trajectory);
  double steer_angle = result[6];
  double accel = std::min(result[7], target_speed - v);
  steer_value = clamp(steer_angle / Config::maxSteering, -1.0, 1.0);
#ifdef VERBOSE_OUT
  cout << "Steering: " << steer_angle << ", " << result[9] << ", " << result[10] 
        << ", accel: " << result[7] << ", speed: " << result[3] << ", steer: " << steer_value << ", target speed: "
        << target_speed << ", CTE: " << result[4] << ", epsi: " << result[5] 
        << ", cost: " << result[8] << ", max yar change: " << max_yaw_change << ", max speed: " << max_speed << endl;
#endif
  
  return {result[0], result[1], result[2], result[3], steer_value, accel};
}

int main() {
  uWS::Hub h;
  // MPC is initialized here!
  MPC mpc;
  static high_resolution_clock::time_point prev_time = high_resolution_clock::now();

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    high_resolution_clock::time_point stime = high_resolution_clock::now();

    string sdata = string(data).substr(0, length);
#ifdef VERBOSE_OUT   
    cout << "Message: " << ((stime - prev_time).count() * 10.0e-9) << "s, " << sdata << endl;
#endif    
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      static double steer_value = 0;
      static double throttle_value = 0;
      static double steer_angle = 0;
      static double accel = 0;
      static double prev_v = 0;

      string s = hasData(sdata);
      if (s != "") {
        high_resolution_clock::time_point stime = high_resolution_clock::now();
        double dt = (stime - prev_time).count() * 10.0e-9;

        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double throttle = j[1]["throttle"];
          psi = normalizeAngle(psi);
          v = MpH2MpS(v); // speed in meters/second
          double steer = j[1]["steering_angle"];
          steer = -steer; // Convert to normal radian
          // Simulate car move to compensate the latency, but there is still a gap with the
          // Car model implemented in the simulator
          if (Config::latency) {
            double ahead = Config::lookahead;
            double dt = 0.01; // 10 millie seconds
            while(ahead >= dt) {
              moveVehicle(dt, px, py, psi, v, steer, throttle > 0? 
                          accel*throttle: throttle>-0.5? -10: throttle<-0.75? -15: Config::maxDeceleration,
                          Config::Lf);
              ahead -= dt;
            }
          }

#ifdef PLOT_TRAJECTORY //Display the MPC predicted trajectory 
          // Trajectories points to display on the simulator, points are in reference to the vehicle's
          // coordinate system the points in the simulator are connected by a Yellow line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          vector<double> result = run_mpc(px, py, psi, v, steer, ptsx, ptsy, &mpc_x_vals, &mpc_y_vals);
#else
          vector<double> result = run_mpc(px, py, psi, v, steer, ptsx, ptsy);
#endif
          steer_angle = result[4];
          accel = result[5];
          steer_value = -clamp(steer_angle / Config::maxSteering, -1.0, 1.0);
          throttle_value = computeThrottle(accel, result[3], Config::maxAcceleration, Config::maxDeceleration);

          nanoseconds elapsed = duration_cast<nanoseconds> (high_resolution_clock::now() - stime);

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;
          
#ifdef PLOT_TRAJECTORY //Display the MPC predicted trajectory 
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;
          msgJson["next_x"] = ptsx;
          msgJson["next_y"] = ptsy;
#endif
          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
#ifdef VERBOSE_OUT          
          cout << "Time: " << elapsed.count()/1000000 << "ms, response: " << msg << endl;
#endif
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          if (Config::latency) {
            this_thread::sleep_for(chrono::milliseconds(Config::latency));
          }
          
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
      res->end(nullptr, 0);
    }
  });

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    cout << "Connected!!!" << endl;
    Config::load("../config.json");
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    // ws.close(); // will crash on Windows if close
    cout << "Disconnected" << endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    cout << "Listening to port " << port << endl;
  } else {
    cerr << "Failed to listen to port" << endl;
    return -1;
  }
  h.run();
}
