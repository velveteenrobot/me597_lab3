#include "SensorModel.h"

#include <random>

using namespace std;

static const double POSITION_STD = 0.1;

static std::default_random_engine gen;
static std::normal_distribution<double> position_noise(0, POSITION_STD);

double SensorModel::getDistStd() {
  return POSITION_STD;
}

void SensorModel::setPosition(double x, double y, double heading) {
  mX = x + position_noise(gen);
  mY = y + position_noise(gen);
  mHeading = heading;
}
