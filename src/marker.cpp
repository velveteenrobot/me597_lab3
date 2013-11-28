#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher particlePub;
static visualization_msgs::Marker lines;

void markerInit(ros::NodeHandle& n) {
  particlePub = n.advertise<visualization_msgs::Marker>(
      "/particle_marker",
      1,
      true);

  lines.header.frame_id = "/map";
  lines.id = 1;
  lines.type = visualization_msgs::Marker::POINTS;
  lines.action = visualization_msgs::Marker::ADD;
  lines.scale.x = 0.1;
  lines.color.r = 1.0;
  lines.color.a = 1.0;
  lines.lifetime = ros::Duration(0.1);
}

void flushPoints() {
  particlePub.publish(lines);
  lines.id += 1;
  lines.points.clear();
}

void drawPoint(double x, double y) {
  Point p = Point();
  p.x = x;
  p.y = y;
  lines.points.push_back(p);
}
