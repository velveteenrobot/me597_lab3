#pragma once

class SensorModel {
 public:
  double getX() { return mX; }
  double getY() { return mY; }
  double getDistStd();

  double getHeading() { return mHeading; }
  double getHeadingStd() { return 0.3; }
  void setPosition(double x, double y, double heading);

 private:
  double mX, mY, mHeading;
};

