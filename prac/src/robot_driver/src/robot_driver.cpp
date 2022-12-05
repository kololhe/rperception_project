#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Point.h"
#include <math.h>
#include <string>
#include <iostream>
#include <iterator>
#include <random>

double x0 = 2.5;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "robot_driver");

  ros::NodeHandle n;
  ros::Publisher vel_pub = n.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 1000);
  ros::Publisher imu_pub = n.advertise<geometry_msgs::Point>("/bot_imu", 1000);
  ros::Publisher gt_pub = n.advertise<geometry_msgs::Point>("/bot_gt", 1000);

  ros::Rate loop_rate(250);
  int count = 0;
  ros::Time begin = ros::Time::now();

  while (ros::ok())
  {
    gazebo_msgs::ModelState msg;
    geometry_msgs::Point imu_msg;
    geometry_msgs::Point gt_msg;
    
    ros::Duration elapsed = ros::Time::now() - begin;
    float time = elapsed.sec + elapsed.nsec*(10e-10);
    //Write the robot motion controller message
    char model_name[] = "tb3_1";
    msg.model_name = model_name;
    msg.pose.position.x = x0;
    msg.pose.position.y = cos(time);
    msg.pose.position.z = 0;
    ROS_INFO("pose %f", time);//msg.pose.position.x);
    //Write the simulated IMU message
    imu_msg.x = -cos(time);// + meas_noise;
    //Write the simulated ground truth data message
    gt_msg.x = cos(time);
    //Publish the finished messages
    vel_pub.publish(msg);
    imu_pub.publish(imu_msg);
    gt_pub.publish(gt_msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}