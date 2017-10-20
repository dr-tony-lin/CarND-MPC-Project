#include <math.h>
#include "utils.h"
#include "Config.h"

size_t Config::N = 25;

int Config::maxPolyOrder = 4;
long Config::latency = 100;
double Config::lookahead = 0;
double Config::ipoptTimeout = 0.5;
double Config::dt = 0.025;
double Config::maxSteering = deg2rad(25.0);
double Config::maxAcceleration = MpH2MpS(8);
double Config::maxDeceleration = MpH2MpS(-20);
double Config::maxSpeed = MpH2MpS(100);
double Config::Lf = 2.67;
double Config::epsiRef = 0.1;
double Config::epsiPanic = 1;
std::vector<double> Config::weights = {100, 100, 1, 1, 1, 5000, 1, 1000};
std::vector<double> Config::steers = {0.1, 0.2, 0.3};
std::vector<double> Config::steerSpeeds = {MpH2MpS(80), MpH2MpS(65), MpH2MpS(30), MpH2MpS(25)};
std::vector<double> Config::yawChanges;
std::vector<double> Config::yawChangeSpeeds;

double Config::yawLow;
double Config::yawHigh;

void Config::load(std::string fileName) {
  std::ifstream in(fileName);

  nlohmann::json js;
  js << in;

  N = js["N"];
  dt = js["dt"];
  maxAcceleration = js["max acceleration"];
  maxAcceleration = MpH2MpS(maxAcceleration);
  maxDeceleration = js["max deceleration"];
  maxDeceleration = MpH2MpS(maxDeceleration);
  maxSteering = deg2rad(js["max steering"]);
  maxSpeed = js["max speed"];
  maxSpeed = MpH2MpS(maxSpeed);
  latency = js["latency"];
  lookahead = js["look ahead"];
  lookahead /= 1000.0;
  maxPolyOrder= js["max poly order"];
  ipoptTimeout = js["ipopt timeout"];
  Lf = js["Lf"];
  epsiRef = js["epsi ref"];
  epsiPanic = js["epsi panic"];
  std::vector<double> w = js["weights"];
  weights = w;
  std::vector<double> st = js["steers"];
  steers = st;
  std::vector<double> sts = js["steer speeds"];
  for (int i = 0; i < sts.size(); i++) {
    sts[i] = MpH2MpS(sts[i]);
  }
  steerSpeeds = sts;
  std::vector<double> yc = js["yaw change"];
  yawChanges = yc;
  std::vector<double> ycs = js["yaw change speed"];
  for (int i = 0; i < ycs.size(); i++) {
    ycs[i] = MpH2MpS(ycs[i]);
  }
  yawChangeSpeeds = ycs;
}