#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <utility>
#include <thread>
#include <vector>
#include "Eigen/Core"
#include "Eigen/QR"
#include "utils/utils.h"
#include "utils/Config.h"
#include "utils/Reducer.h"
#include "control/MPC.h"
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

#if defined(COLLECT_DATA) || defined(VERBOSE_OUT)
static high_resolution_clock::time_point prev_time = high_resolution_clock::now();
#endif
#if defined(COLLECT_DATA)
static bool is_first = true;
#endif

static Reducer<double> latencyReducer(5);

int main() {
  uWS::Hub h;
  
  h.onMessage([](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      static double steer_angle = 0; // steering angle from MPC
      static double steer_value = 0; // steering to sent to the simulator
      static double accel = 0;  // acceleration from MPC
      static double throttle_value = 0; // throttle to sent to the simulator

      string s = hasData(sdata);
      if (s != "") {
        high_resolution_clock::time_point stime = high_resolution_clock::now();
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // The controller
          MPC mpc;

#if defined(COLLECT_DATA) || defined(VERBOSE_OUT)
          // Compute elapsed time since last message
          double dt = (stime - prev_time).count() * 1.0e-9;
          prev_time = stime;
#ifdef VERBOSE_OUT
          cout << "Message: " << dt << "s, " << j.dump() << endl;
#endif // VERBOSE_OUT
#endif // defined(COLLECT_DATA) || defined(VERBOSE_OUT)

          // j[1] is the data JSON object
          // X, and Y trajectory of the center line
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];

          // Current vehicle position
          double px = j[1]["x"];
          double py = j[1]["y"];
          // Current psi
          double psi = j[1]["psi"];
          // Current speed
          double v = j[1]["speed"];
          // Current steering angle
          double steer = j[1]["steering_angle"];

          // Normalize psi to [-PI, PI]
          psi = normalizeAngle(psi);
          // speed in meters/second
          v = MpH2MpS(v);
          // Convert to normal radial coordinate
          steer = -steer;

#ifdef COLLECT_DATA
          // collect data, for analysis purpose
          static double last_psi = 0;
          static double last_v = 0;
          // Current throttle
          double throttle = j[1]["throttle"]; // Simulator will return 0 if the value is negative, may be a bug
          
          if (!is_first) {
            cerr << dt << "," << psi << "," << v << "," << throttle << "," << throttle_value
                << "," << steer << "," << steer_value*Config::maxSteering << "," << (psi - last_psi) / dt 
                << "," << (v - last_v) / dt << endl;
          } else {
            is_first = false;
          }
          last_psi = psi;
          last_v = v;
#endif
          // Simulate car move to compensate the latency, the latency is the average MPC time + actuator latency
          // The real acceleration depends on the throttle, the current speed, the vehicle and the road
          // Here we simply use the throttle value from the previous MPC times by a factor, here 4 is picked,
          // But it does not matter much.
          if (Config::latency) {
            moveVehicle(Config::lookahead + latencyReducer.mean<double>(), px, py, psi, v, steer,
                        (throttle_value - v / Config::maxSpeed) * 8, Config::Lf);
          }

#ifdef PLOT_TRAJECTORY
          //Display the MPC predicted trajectory 
          // Trajectories points to display on the simulator, points are in reference to the vehicle's
          // coordinate system the points in the simulator are connected by a Yellow line
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          vector<double> result = mpc.run(px, py, psi, v, steer, ptsx, ptsy, &mpc_x_vals, &mpc_y_vals);
#else
          vector<double> result = mpc.run(px, py, psi, v, steer, ptsx, ptsy);
#endif
          steer_angle = result[4] * Config::maxSteering; // the steering angle in radian
          accel = result[5]; // the acceleration in meter/s
          steer_value = -result[4]; // the steering in simulator's radial coordinate
          // Estimate the throttle value from acceleration and velocity from MPC
          throttle_value = computeThrottle(accel, result[3], Config::maxAcceleration, Config::maxDeceleration);

          // For average MPC time to set for compensating the latency
          nanoseconds elapsed = duration_cast<nanoseconds> (high_resolution_clock::now() - stime);
          latencyReducer.push(elapsed.count() * 1.0E-9);
          
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
#else
          msgJson["mpc_x"] = NULL;
          msgJson["mpc_y"] = NULL;
          msgJson["next_x"] = NULL;
          msgJson["next_y"] = NULL;
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
    
    latencyReducer.clear();
#if defined(COLLECT_DATA) || defined(VERBOSE_OUT)
    prev_time = high_resolution_clock::now();
#endif
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
