#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher particlePub;
static visualization_msgs::Marker parts;

static ros::Publisher estimatePub;
static visualization_msgs::Marker estimates;

static ros::Publisher positionPub;
static visualization_msgs::Marker positions;

void markerInit(ros::NodeHandle& n) {
  particlePub = n.advertise<visualization_msgs::Marker>(
      "/particle_marker",
      1,
      true);

  parts.header.frame_id = "/map";
  parts.id = 1;
  parts.type = visualization_msgs::Marker::POINTS;
  parts.action = visualization_msgs::Marker::ADD;
  parts.scale.x = 0.1;
  parts.color.r = 1.0;
  parts.color.a = 1.0;
  parts.lifetime = ros::Duration(0.5);

  estimatePub = n.advertise<visualization_msgs::Marker>(
      "/estimate_markers",
      1,
      true);
  estimates.header.frame_id = "/map";
  estimates.id = 1;
  estimates.type = visualization_msgs::Marker::POINTS;
  estimates.action = visualization_msgs::Marker::ADD;
  estimates.scale.x = 0.1;
  estimates.color.b = 1.0;
  estimates.color.a = 1.0;

  positionPub = n.advertise<visualization_msgs::Marker>(
      "/position_markers",
      1,
      true);
  positions.header.frame_id = "/map";
  positions.id = 1;
  positions.type = visualization_msgs::Marker::POINTS;
  positions.action = visualization_msgs::Marker::ADD;
  positions.scale.x = 0.1;
  positions.color.g = 1.0;
  positions.color.a = 1.0;
}

void flushPoints() {
  particlePub.publish(parts);
  parts.id += 1;
  parts.points.clear();

  estimatePub.publish(estimates);
  positionPub.publish(positions);
}

void drawPoint(double x, double y) {
  Point p = Point();
  p.x = x;
  p.y = y;
  parts.points.push_back(p);
}

void addEstimate(double x, double y) {
  Point p = Point();
  p.x = x;
  p.y = y;
  estimates.points.push_back(p);
}

void addPosition(double x, double y) {
  Point p = Point();
  p.x = x;
  p.y = y;
  positions.points.push_back(p);
}
