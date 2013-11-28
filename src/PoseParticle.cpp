#include "PoseParticle.h"
#include "marker.h"

#include <cstdlib>
#include <climits>
#include <cmath>
#include <random>

#define PI 3.14159

using namespace std;

static std::default_random_engine gen;
static std::normal_distribution<double> distance_distribution(0, 0.1);
static std::normal_distribution<double> heading_distribution(0, 0.1);

static double randRange(double min, double max) {
  double range = max - min;
  int randInt = rand();
  return min + (double)randInt * range / (double)INT_MAX;
}

PoseParticle::PoseParticle(const PoseParticle& other, bool addVarience)
    :mX(other.mX),
     mY(other.mY),
     mHeading(other.mHeading),
     mSensors(other.mSensors) {
  if (addVarience) {
    double distError = distance_distribution(gen);
    double distErrorTheta = randRange(-PI, PI);
    mX += distError * cos(distErrorTheta);
    mY += distError * sin(distErrorTheta);;
    mHeading += heading_distribution(gen);
  }
}

PoseParticle::PoseParticle(
    double x,
    double y,
    double heading,
    SensorModel& sensors)
    :mX(x), mY(y), mHeading(heading), mSensors(sensors) {}

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

  double distError = sqrt(xError*xError + yError*yError);
  double distProb = normalProb(distError, mSensors.getDistStd());

  double headingError = mSensors.getHeading() - mHeading;
  // normalize heading error
  headingError = fmod(headingError, PI * 2);
  if (headingError > PI) {
    headingError -= PI * 2;
  }

  double headingProb = normalProb(headingError, mSensors.getHeadingStd());

  // TODO: emulate what a scan at this particle would see on the map
  // TODO: compare the emulated scan with the most recent scan,
  // TODO: produce a scan probability

  return 0.8 * distProb + 0.2 * headingProb;
}

void PoseParticle::drawMarker() {
  drawPoint(mX, mY);
}

void updateParticleFilter(vector<PoseParticle*>& parts) {
  // find the net probability
  double totalProb = 0;
  for (int i = 0; i < parts.size(); i++) {
    totalProb += parts[i]->getProbability();
  }
  // make the new particles
  vector<PoseParticle*> oldParts(parts);
  parts.clear();
  for (int i = 0; i < oldParts.size(); ++i) {
    double weightedIndex = randRange(0, totalProb);
    int j = 0;
    do {
      weightedIndex -= oldParts[j]->getProbability();
    } while (weightedIndex > 0 && j++ < (oldParts.size() - 1));
    parts.push_back(new PoseParticle(*oldParts[j], true));
  }

  // Delete the old particles
  for (int i = 0; i < oldParts.size(); ++i) {
    delete oldParts[i];
  }
}
