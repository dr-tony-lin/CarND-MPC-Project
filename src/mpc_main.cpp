#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
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

int main() {
  uWS::Hub h;
  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    
    // MPC is initialized here!
    MPC mpc;
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    high_resolution_clock::time_point stime = high_resolution_clock::now();

    string sdata = string(data).substr(0, length);
#ifdef VERBOSE_OUT   
    cout << "Message: " << sdata << endl;
#endif    
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          static vector<double> eval_ptsx;

          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          double steer = j[1]["steering_angle"];

          globalToVehicle(ptsx, ptsy, px, py, psi);

          // Compute the polynominal coefficients
          Eigen::VectorXd xVals(ptsx.size());
          Eigen::VectorXd yVals(ptsx.size());
          for (int i = 0; i < ptsx.size(); i++) {
            xVals[i] = ptsx[i];
            yVals[i] = ptsy[i];
          }

          double fiterr = 0;
          int order = 2;
          static Eigen::VectorXd poly;
          if (eval_ptsx.size() == 0 || eval_ptsx[0] != ptsx[0]) { // new, compute the polynominal coefficients
            do { // Compute the polynominal coefficients to an order that has small fitting error
              poly = polyfit(xVals, yVals, order++);
              for (int i = 0; i < ptsx.size(); i++) {
                fiterr = square(ptsy[i] - polyeval(poly, ptsx[i]));
              }
            } while (fiterr > 1 && order < Config::maxPolyOrder);
            eval_ptsx = ptsx;
#ifdef VERBOSE_OUT
            cout << "Polynominal: " << poly << endl;
#endif
          }

          // Calculate the CTE
          double cte = polyeval(poly, 0);
          // Calculate the psi error
          double px_psi = polypsi(poly, 0, ptsx[1] - ptsx[0]);
          double epsi = -px_psi;
#ifdef VERBOSE_OUT
          cout << "Psi: " << psi<< ", ePsi: " << epsi<< ", psi at px: " << px_psi << ", cte: " << cte << ", fit err: " 
               << fiterr << ", order: " << order << endl;
#endif   
          Eigen::VectorXd state(8);
          state << 0, 0, 0, v, cte, epsi, steer, 
                  clamp(computeSpeedTarget(steer, Config::maxSpeed) - v, Config::maxDeceleration, Config::maxAcceleration);

          MPC mpc;
          
#ifdef PLOT_TRAJECTORY //Display the MPC predicted trajectory 
          // Trajectories points to display on the simulator, points are in reference to the vehicle's
          // coordinate system the points in the simulator are connected by a Yellow line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
#endif
          // Compute the target speed using the estimated future psi
          double target_speed = computeSpeedTarget(steer, Config::maxSpeed);
          double steer_value = 0;
          double throttle_value = 0;
          try {
            // Solve MPC
            auto result = mpc.Solve(state, poly, target_speed, ptsx[1] - ptsx[0], &mpc_x_vals, &mpc_y_vals);
            double steer_angle = result[6];
            double accel = std::min(result[7], computeSpeedTarget(steer_angle, Config::maxSpeed) - v);
            steer_value = -clamp(steer_angle / Config::maxSteering, -1.0, 1.0);
            throttle_value = computeThrottle(accel, result[3], Config::maxAcceleration, Config::maxDeceleration);
#ifdef VERBOSE_OUT
            cout << "Steering: " << steer_angle << ", " << result[9] << ", " << result[10] 
                 << ", accel: " << result[7] << ", speed: " << result[3] 
                 << ", throttle: " << throttle_value  << ", steer: " << steer_value << ", target speed: "
                 << target_speed << ", CTE: " << result[4] << ", epsi: " << result[5] 
                 << ", cost: " << result[8] << endl;
#endif
#ifdef PLOT_TRAJECTORY
            // Convert to vehicle coordinate
            //globalToVehicle(mpc_x_vals, mpc_y_vals, px, py, psi);
            
#endif
          } catch(std::string e) {
            std::cout << "Ipopt failed! " << e << std::endl;
            steer_value = steer;
            throttle_value = computeThrottle(0, v, Config::maxAcceleration, Config::maxDeceleration);
          }

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
