#include "PoseParticle.h"
#include "marker.h"

#include <cstdlib>
#include <climits>
#include <cmath>

#define PI 3.14159

/* PoseParticle::PoseParticle(const PoseParticle& other)
    :mX(other.mX),
     mY(other.mY),
     mHeading(other.mHeading),
     mSensors(other.mSensors) {} */

PoseParticle::PoseParticle(
    double x,
    double y,
    double heading,
    SensorModel& sensors)
    :mX(x), mY(y), mHeading(heading), mSensors(sensors) {}

double randRange(double min, double max) {
  double range = max - min;
  int randInt = rand();
  return min + (double)randInt * range / (double)INT_MAX;
}

void makeRandomParticles(
    vector<PoseParticle*>& parts,
    unsigned int num,
    SensorModel& sensors,
    double minX,
    double maxX,
    double minY,
    double maxY) {
  for (unsigned int i = 0; i < num; ++i) {
    parts.push_back(new PoseParticle(
        randRange(minX, maxX),
        randRange(minY, maxY),
        randRange(-PI, PI),
        sensors));
  }
}

void PoseParticle::updateForOdom(double speed, double turnRate, double deltaTime) {
  double prevHeading = mHeading;
  mHeading = mHeading + turnRate * deltaTime;
  double avgHeading = (prevHeading + mHeading) / 2;

  mX += speed * cos(avgHeading) * deltaTime;
  mY += speed * sin(avgHeading) * deltaTime;
}

static double normalProb(double error, double std) {
  double x = error / std;
  return (1 / std) * (1 / sqrt(2.0*PI)) * exp(-(x*x)/2.0);
}

double PoseParticle::getProbability() {
  double xError = mSensors.getX() - mX;
  double yError = mSensors.getY() - mY;

  double xProb = normalProb(xError, mSensors.getDistStd());
  double yProb = normalProb(yError, mSensors.getDistStd());

  double headingError = mSensors.getHeading() - mHeading;
  // normalize heading error
  headingError = fmod(headingError, PI * 2);
  if (headingError > PI) {
    headingError -= PI * 2;
  }

  return xProb * 0.5 + yProb * 0.5;
}

void PoseParticle::drawMarker() {
  drawPoint(mX, mY);
}
