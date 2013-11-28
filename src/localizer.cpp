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

using namespace std;
using namespace ros;

SensorModel* sensor_model;

void odom_callback(const nav_msgs::Odometry& msg) {
  cout<<"Odom: "
      <<msg.twist.twist.linear.x<<", "
      <<msg.twist.twist.angular.z<<endl;
  // TODO: update all the particles
}

void pose_callback(const me597_lab3::ips_msg& msg) {
  cout<<"Pose: "<<msg.X<<", "<<msg.Y<<endl;
  sensor_model->setPosition(msg.X, msg.Y);
}

void scan_callback(const sensor_msgs::LaserScan& msg) {
  cout<<"Ranges:";
  for (unsigned int i = 0; i < msg.ranges.size(); ++i) {
    cout<<" "<<msg.ranges[i];
  }
  cout<<endl;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "turtlebot_localizer");
  ros::NodeHandle n;

  SensorModel model;
  sensor_model = &model;

  vector<PoseParticle*> parts;
  makeRandomParticles(parts, 200, model, 0, 5, 0, 5);

  ros::Subscriber odom_sub = n.subscribe("/odom", 1, odom_callback);
  ros::Subscriber ips_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber scan_sub = n.subscribe("/scan", 1, scan_callback);

  ros::Publisher pose_estimate_publisher = n.advertise<me597_lab3::ips_msg>(
      "/pose_esitmate",
      1000);

  //Set the loop rate
  ros::Rate loop_rate(10); // 10 Hz update time

  while (ros::ok()) {
    // cout<<"OK"<<endl;

    loop_rate.sleep();
    ros::spinOnce();
  }
}
