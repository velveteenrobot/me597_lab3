#pragma once

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <vector>

using namespace std;
using namespace geometry_msgs;

void markerInit(ros::NodeHandle& n);
void drawPoint(double x, double y);

void flushPoints();
