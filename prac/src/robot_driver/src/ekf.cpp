#include "ros/ros.h"
#include "gazebo_msgs/ModelState.h"
#include "geometry_msgs/Point.h"
#include <math.h>

ros::Publisher ekf_pub;
ros::Subscriber imu_sub; 
ros::Subscriber cam_sub; 

float x[3][1];
float P[3][3];

void predict(float x, float P, float dt)
{


}


int main (int argc, char** argv) {
  ros::init(argc, argv, "ekf");
  ros::NodeHandle n;
  
  // get measurements
  imu_sub = n.subscribe("/bot_imu", 10, imu);
  cam_sub = n.subscribe("/central_pixel", 10, cam);

  ekf_pub = n.advertise<geometry_msgs::Point>("//ekf_estimate", 10);
  
  ros::spin();
}
