#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher particlePub;
static visualization_msgs::Marker lines;
void markerInit(ros::NodeHandle& n) {
  particlePub = n.advertise<visualization_msgs::Marker>(
      "/particle_marker",
      1000,
      true);

  lines.header.frame_id = "/map";
  lines.id = 1;
  lines.type = visualization_msgs::Marker::POINTS;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.05;
  lines.color.r = 1.0;
  lines.color.a = 1.0;
}

void flushPoints() {
  particlePub.publish(lines);
}

void drawPoint(double x, double y) {
  Point p = Point();
  p.x = x;
  p.y = y;
  lines.points.push_back(p);

  cout<<"Point: "<<x<<", "<<y<<endl;
}
