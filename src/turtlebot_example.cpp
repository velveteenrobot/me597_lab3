//  ///////////////////////////////////////////////////////////
//
// turtlebot_example.cpp
// This file contains example code for use with ME 597 lab 2
// It outlines the basic setup of a ros node and the various 
// inputs and outputs needed for this lab
// 
// Author: James Servos 
//
// //////////////////////////////////////////////////////////

#include "turtlebot_example.h"
#include "Map.h"
#include "marker.h"
#include "RRT.h"
#include "tracking.h"

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/OccupancyGrid.h>
#include <me597_lab3/ips_msg.h>

#include <vector>
#include <math.h>       // sqrt, pow 
#include <algorithm>    // min

#define PI 3.14159265
#define MAP_IDX(width, i, j) ((width) * (j) + (i))

using namespace std;

//Callback function for the Position topic (LIVE)

static bool poseReady = false;
static Pose pose;

double mapRes = 0.1;
double mapWidth = 10;
double mapHeight = 10;
double robotMotion [3] = {0.5, 0, -0.5};
nav_msgs::OccupancyGrid knownMapMsg;
std::vector< std::vector<double> > knownMap(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));

std::vector< std::vector<double> > LO(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));
std::vector< std::vector<double> > L(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));
ros::Publisher map_pub; 

std::vector< std::vector<double> > get_inverse_m_m(sensor_msgs::LaserScan scan)
{
  std::vector< std::vector<double> > invMod(int(mapHeight/mapRes), std::vector<double>(int(mapWidth/mapRes),0));

  //Range finder inverse measurement model
  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      double alpha = 0.1;
      double beta = 0.05;
      tf::Quaternion q;
      tf::quaternionMsgToTF(pose.orientation, q);
      double roll, pitch, yaw;
      tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
      
      //Find range and bearing to the current cell
      double r = sqrt( pow((i*mapRes-pose.position.x),2) + pow((j*mapRes-pose.position.y),2));
      double phi = fmod((atan2(j*mapRes-pose.position.y,i*mapRes-pose.position.x) - yaw + PI),(2*PI))-PI;
      //phi = mod(atan2(j-y,i-x)-theta+pi,2*pi)-pi;
          
      //Find the applicable range measurement 
      //[meas_cur,k] = min(abs(phi-meas_phi));
      int k = round((scan.angle_min - phi)/scan.angle_increment);
      double range_cur = scan.ranges[k];

      //If out of range, or behind range measurement, or outside of field
      // of view, no new information is available
      if ((r > scan.range_max) || (r > range_cur+alpha/2) || (abs(phi-(scan.angle_min + k*scan.angle_increment))>beta/2))
      {
        invMod[i][j] = 0.5;
        cout<<"don't know"<<endl;

      }
      //If the range measurement was in this cell, likely to be an object
      else if ((range_cur < scan.range_max) && (abs(r-range_cur)<alpha/2))
      {
        invMod[i][j] = 0.7;
        cout<<"occupied"<<endl;
      }   
      //If the cell is in front of the range measurement, likely to be
      // empty
      else if (r < range_cur)
      { 
        invMod[i][j] = 0.3;
        cout<<"free"<<endl;
      }
    }
  }

  return invMod;
}

void pose_callback(const me597_lab3::ips_msg& msg)
{
  //This function is called when a new position message is received
  if(msg.tag_id != TAGID) {
    return;
  }

  pose.position.x = msg.X;
  pose.position.y = msg.Y;

  quaternionTFToMsg(
      tf::createQuaternionFromRPY(0, 0, msg.Yaw),
      pose.orientation);
  poseReady = true;
}

//Callback function for the map
void scan_callback(const sensor_msgs::LaserScan& msg)
{
  std::vector< std::vector<double> > invMod = get_inverse_m_m(msg);
  

  //Calculate updated log odds
  //L = L +log(invmod./(1-invmod))-L0;
  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      L[i][j] = L[i][j] + log(invMod[i][j]/(1-invMod[i][j]));
    }
  }

  //Calculate probabilities
  //m = exp(L)./(1+exp(L));
  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      knownMap[i][j] = exp(L[i][j])/(1+exp(L[i][j]));
    }
  }
  
  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      if (knownMap[i][j] == 0.5)
        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, j, i)] = -1;
      else if (knownMap[i][j] >= 0.5)
        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, j, i)] = 100;
      else 
        knownMapMsg.data[MAP_IDX(knownMapMsg.info.width, j, i)] = 0;
    }
  }

  map_pub.publish(knownMapMsg);
  cout<<"publishing map"<<endl;

}

void spinOnce(ros::Rate& loopRate) {
  loopRate.sleep(); //Maintain the loop rate
  ros::spinOnce();   //Check for new messages
}

int main(int argc, char **argv)
{
  //Initialize the ROS framework
  ros::init(argc,argv,"main_control");
  ros::NodeHandle n;

  //Subscribe to the desired topics and assign callbacks
  ros::Subscriber pose_sub = n.subscribe("/indoor_pos", 1, pose_callback);
  ros::Subscriber scan_pub = n.subscribe("/scan_throttle", 1, scan_callback);
  map_pub = n.advertise<nav_msgs::OccupancyGrid>("/map", 1000);

  //Setup topics to Publish from this node
  markerInit(n);

  //Set the loop rate
  ros::Rate loopRate(1/CYCLE_TIME);    //20Hz update rate

  cout<<"wait for the position"<<endl;
  while (!poseReady) {
    spinOnce(loopRate);
  }

  //inialise map
  knownMapMsg.info.resolution = mapRes;
  knownMapMsg.info.width = int(mapWidth/mapRes);
  knownMapMsg.info.height = int(mapHeight/mapRes);
  knownMapMsg.data.resize(int (mapWidth/mapRes * mapHeight/mapRes));

  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      knownMap[i][j] = 0.5;
    }
  }

  //L0 = log(m./(1-m));

  for (int i = 0; i < int (mapHeight/mapRes); i++)
  {
    for (int j = 0; j < int (mapWidth/mapRes); j++)
    {
      LO[i][j] = log(knownMap[i][j]/(1-knownMap[i][j]));
    }
  }

  L = LO;
  

  while (ros::ok()) {
    cout<<"OK"<<endl;
  
    spinOnce(loopRate);
  }
  // TODO: free memory
  return 0;
}
