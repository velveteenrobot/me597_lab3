#pragma once

class SensorModel {
 public:
  double getX() { return mX; }
  double getY() { return mY; }
  double getDistStd() { return 0.1; }

  double getHeading() { return 0.0; } // TODO

  void setPosition(double x, double y) {
    mX = x;
    mY = y;
  }
 private:
  double mX, mY;
};

