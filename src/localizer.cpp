#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <me597_lab3/ips_msg.h>

#include <iostream>

#include "SensorModel.h"
#include "PoseParticle.h"
#include "marker.h"

using namespace std;
using namespace ros;

#define CYCLE_TIME 0.1

SensorModel* sensor_model;
vector<PoseParticle*> parts;

void odom_callback(const nav_msgs::Odometry& msg) {
  // update all the particles
  for (int i = 0; i < parts.size(); ++i) {
    parts[i]->updateForOdom(
        msg.twist.twist.linear.x,
        msg.twist.twist.angular.z,
        CYCLE_TIME);
    parts[i]->drawMarker();
  }
  flushPoints();
}

void pose_callback(const me597_lab3::ips_msg& msg) {
  // cout<<"Pose: "<<msg.X<<", "<<msg.Y<<endl;
  sensor_model->setPosition(msg.X, msg.Y, msg.Yaw);
  addPosition(msg.X, msg.Y);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_localizer");
  ros::NodeHandle n;

  SensorModel model;
  sensor_model = &model;

  makeRandomParticles(parts, 500, model, -5, 5, -1, 9);

  markerInit(n);

  ros::Subscriber odom_sub = n.subscribe("/turtlebot4/odom", 1, odom_callback);
  ros::Subscriber ips_sub = n.subscribe("/indoor_pos", 1, pose_callback);

  ros::Publisher pose_estimate_publisher = n.advertise<me597_lab3::ips_msg>(
      "/pose_esitmate",
      1000);

  //Set the loop rate
  ros::Rate loop_rate(1/CYCLE_TIME); // 10 Hz update time

  for (int i = 0; i < parts.size(); ++i) {
    parts[i]->drawMarker();
  }

  flushPoints();

  int framesToUpdate = 0;

  while (ros::ok()) {
    // cout<<"OK"<<endl;
    loop_rate.sleep();
    ros::spinOnce();

    drawParticleFilter(parts);
    framesToUpdate--;
    if (framesToUpdate <= 0) {
      updateParticleFilter(parts);
      framesToUpdate = 10;
    }
  }
}
