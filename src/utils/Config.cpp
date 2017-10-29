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
double Config::epsiPanic = 1;
double Config::ctePanic = 0.6;
double Config::cteOvershotRatio = 0.96;
double Config::steerAdjustmentThresh = 0.6;
double Config::steerAdjustmentRatio = 0.025;
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

  double speedScale;

  N = js["N"];
  dt = js["dt"];
  maxAcceleration = js["max acceleration"];
  maxAcceleration = MpH2MpS(maxAcceleration);
  maxDeceleration = js["max deceleration"];
  maxDeceleration = MpH2MpS(maxDeceleration);
  maxSteering = deg2rad(js["max steering"]);
  maxSpeed = js["max speed"];
  maxSpeed = MpH2MpS(maxSpeed);
  speedScale = maxSpeed/MpH2MpS(100.0);
  latency = js["latency"];
  lookahead = latency * 1.0E-3;
  maxPolyOrder= js["max poly order"];
  ipoptTimeout = js["ipopt timeout"];
  Lf = js["Lf"];
  epsiPanic = js["epsi panic"];
  ctePanic = js["cte panic"];
  cteOvershotRatio = js["cte overshot reduction ratio"];
  steerAdjustmentThresh = js["steer adjustment threshold"];
  steerAdjustmentRatio = js["steer adjustment ratio"];
  steerAdjustmentRatio = clamp(steerAdjustmentRatio, 0.0, 0.1);
  std::vector<double> w = js["weights"];
  weights = w;
  std::vector<double> st = js["steers"];
  steers = st;
  std::vector<double> sts = js["steer speeds"];
  for (int i = 0; i < sts.size(); i++) {
    if (speedScale <= 1) {
      sts[i] = std::fmin(MpH2MpS(sts[i]), maxSpeed);
    }
    else {
      sts[i] = MpH2MpS(sts[i]) * speedScale;
    }
  }
  steerSpeeds = sts;
  std::vector<double> yc = js["yaw changes"];
  yawChanges = yc;
  std::vector<double> ycs = js["yaw change speeds"];
  for (int i = 0; i < ycs.size(); i++) {
    if (speedScale <= 1) {
      ycs[i] = std::fmin(MpH2MpS(ycs[i]), maxSpeed);
    }
    else {
      ycs[i] = MpH2MpS(ycs[i]) * speedScale;
    }
  }
  yawChangeSpeeds = ycs;
}