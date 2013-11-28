#pragma once
#include "SensorModel.h"

#include <vector>

using namespace std;

class PoseParticle {
 public:
  // PoseParticle(const PoseParticle& other);
  PoseParticle(double x, double y, double heading, SensorModel& sensors);
  void updateForOdom(double speed, double turnRate, double deltaTime);

  double getProbability();

  void drawMarker();
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
