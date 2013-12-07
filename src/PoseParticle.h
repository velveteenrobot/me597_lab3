#pragma once
#include "SensorModel.h"

#include <vector>

using namespace std;

class PoseParticle {
 public:
  // has a boolean so that c++ doesn't try to get smart with this ctor
  PoseParticle(const PoseParticle& other, bool addVarience);
  PoseParticle(double x, double y, double heading, SensorModel& sensors);
  void updateForOdom(double speed, double turnRate, double deltaTime);

  double getProbability();

  void drawMarker();
  double getX() { return mX; }
  double getY() { return mY; }
 private:
  double mX, mY, mHeading;
  SensorModel& mSensors;
};

void makeRandomParticles(
    vector<PoseParticle*>& parts,
    unsigned int num,
    SensorModel& sensors,
    double minX,
    double maxX,
    double minY,
    double maxY);

void updateParticleFilter(vector<PoseParticle*>& parts);
void drawParticleFilter(vector<PoseParticle*>& parts);
