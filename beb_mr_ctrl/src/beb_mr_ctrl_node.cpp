#include <ros/ros.h>

#include "beb_mr_ctrl/beb_mr_ctrl.h"

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "beb_mr_ctrl_node");

  ros::NodeHandle nh;

  ROS_INFO("Starting beb_mr_ctrl_node ...");
  beb_mr_ctrl::BebMrCtrl pos_ctrl(nh);

  pos_ctrl.Spin();
  return 0;
}
