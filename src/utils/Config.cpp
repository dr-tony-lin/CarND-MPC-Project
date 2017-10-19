#include <math.h>
#include "utils.h"
#include "Config.h"

size_t Config::N = 25;

int Config::maxPolyOrder = 4;
long Config::latency = 100;
double Config::ipoptTimeout = 0.5;
double Config::dt = 0.025;
double Config::maxSteering = deg2rad(25.0);
double Config::maxAcceleration = 8;
double Config::maxDeceleration = -20;
double Config::maxSpeed = 100;
double Config::Lf = 2.67;
double Config::epsiRef = 0.1;
double Config::epsiPanic = 1;
std::vector<double> Config::weights = {100, 100, 1, 1, 1, 5000, 1, 1000};
std::vector<double> Config::steers = {0.1, 0.2, 0.3};
std::vector<double> Config::steerSpeeds = {80, 65, 30, 25};

void Config::load(std::string fileName) {
  std::ifstream in(fileName);

  nlohmann::json js;
  js << in;

  N = js["N"];
  dt = js["dt"];
  maxAcceleration = js["max acceleration"];
  maxDeceleration = js["max deceleration"];
  maxSteering = deg2rad(js["max steering"]);
  maxSpeed = js["max speed"];
  latency = js["latency"];
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
  steerSpeeds = sts;
}