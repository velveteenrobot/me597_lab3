#include "marker.h"

#include <visualization_msgs/Marker.h>

static ros::Publisher particlePub;
static visualization_msgs::Marker parts;

static ros::Publisher estimatePub;
static visualization_msgs::Marker estimates;

static ros::Publisher positionPub;
static visualization_msgs::Marker positions;

static ros::Publisher randomTree;
static ros::Publisher selectedPath;
static ros::Publisher carrotPath;

void markerInit(ros::NodeHandle& n) {
  randomTree = n.advertise<visualization_msgs::Marker>(
      "random_tree",
      1,
      true);
  selectedPath = n.advertise<visualization_msgs::Marker>(
      "selected_path",
      1,
      true);
  carrotPath = n.advertise<visualization_msgs::Marker>(
      "carrot",
      1,
      true);

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

static int lastId = 1;

void drawLine(MarkerType type, vector<Point>& points) {
  double x = 0;
  double y = 0;
  double steps = 50;

  visualization_msgs::Marker lines;
  lines.header.frame_id = "/map";
  lines.id = lastId;
  lastId++;
  lines.type = visualization_msgs::Marker::LINE_STRIP;
  lines.action = visualization_msgs::Marker::ADD;
  lines.ns = "curves";
  lines.scale.x = 0.05;

  for (int i = 0; i < points.size(); ++i) {
    lines.points.push_back(points[i]);
  }

  switch (type) {
    case RANDOM_TREE:
      lines.color.r = 1.0;
      lines.color.b = 1.0;
      lines.color.a = 1.0;
      randomTree.publish(lines);
      break;
    case SELECTED_TREE:
      lines.color.r = 1.0;
      lines.color.a = 1.0;
      selectedPath.publish(lines);
      break;
    case CARROT:
      lines.color.b = 1.0;
      lines.color.a = 1.0;
      carrotPath.publish(lines);
      break;
    default:
      break;
  }
}