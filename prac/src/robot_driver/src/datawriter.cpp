// This code creates a txt file with columns as follows:
// time    ground_truth_x     ekf_est_x  ekf_err  model_only_x   model_only_err
#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include <fstream>
#include <string>
using namespace std;

// Subscribers
ros::Subscriber gt_sub;
ros::Subscriber ekf_sub; 
ros::Subscriber model_only_sub; 

int init = 0;
double t_start; 

double x_gt = 0;
double x_ekf = 0;
double x_model_only = 0;

void write_data()
{
  ROS_INFO("ground truth: %f",x_gt);
  ROS_INFO("ekf: %f", x_ekf);
  ROS_INFO("model only: %f",x_model_only);
  // find time elapsed since first message received.
  double t_now = ros::Time::now().toSec();
  double t_elapsed = t_now - t_start;
  //ROS_INFO("writing data to file");
  double model_only_err = x_gt - x_model_only;
  double ekf_err = x_gt - x_ekf;
  // write data to file
  fstream datafile;
  datafile.open("Data1.txt", ios_base::app);
  datafile << setprecision(8) << t_elapsed << ", " << x_gt << ", " << x_ekf << ", " << ekf_err << ", " << x_model_only << ", " << model_only_err << endl;
  datafile.close();
}

void gt_info(const geometry_msgs::Point &msg1)
{
  if (init == 0)
  {
    t_start = ros::Time::now().toSec();
    ROS_INFO("first message received - timer start");
    init = 1;
  }
  x_gt = msg1.x;
  //ROS_INFO("ground truth: %f",x_gt);
  write_data();
}

void ekf_info(const geometry_msgs::Point &msg2)
{
  if (init == 0)
  {
    t_start = ros::Time::now().toSec();
    ROS_INFO("first message received - timer start");
    init = 1;
  }
  x_ekf = msg2.x;
  //ROS_INFO("ekf: %f", x_ekf);
  write_data();
}

void model_only_info(const geometry_msgs::Point &msg3)
{
  if (init == 0)
  {
    t_start = ros::Time::now().toSec();
    ROS_INFO("first message received - timer start");
    init = 1;
  }
  x_model_only = msg3.x;
  //ROS_INFO("model only: %f",x_model_only);
  write_data();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "datawriter");
  ros::NodeHandle n;
  // Get estimates for current lat, long, and heading, and call function
  gt_sub = n.subscribe("/bot_gt", 500, gt_info);
  // Get path gps path information
  ekf_sub = n.subscribe("ekf_estimate", 500, ekf_info);
  // Get path gps path information
  model_only_sub = n.subscribe("model_only_est", 500, model_only_info);
  
  ros::spin();
}
