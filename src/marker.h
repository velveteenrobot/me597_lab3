#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;
using namespace geometry_msgs;

void markerInit(ros::NodeHandle& n);
void drawPoint(double x, double y);
void addEstimate(double x, double y);
void addPosition(double x, double y);


void flushPoints();
typedef enum {
  RANDOM_TREE,
  SELECTED_TREE,
  CARROT,
} MarkerType;

void drawLine(MarkerType type, vector<Point>& points);