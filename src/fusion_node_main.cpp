#include <ros/ros.h>

// Cob messages
#include <cob_perception_msgs/Detection.h>
#include <cob_perception_msgs/DetectionArray.h>

// Own
#include <people_fusion_node/fusion_node.h>

// System includes
#include <iostream>



// Command line parameters
int g_argc;
char** g_argv;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "people_fusion_node");
  g_argc = argc;
  g_argv = argv;

  // Create the node handle
  ros::NodeHandle nh;

  FusionNode fn(nh);

  cob_perception_msgs::Detection detection;

  ROS_INFO("people_fusion_node started!");

  ros::spin();

  return 0;
}

