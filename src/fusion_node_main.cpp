#include <ros/ros.h>

// Command line parameters
int g_argc;
char** g_argv;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "people_fusion_node");
  g_argc = argc;
  g_argv = argv;
  ros::NodeHandle nh;
  //PeopleDetectionFusionNode (nh); // TODO define this node

  ROS_INFO("people_fusion_node started!");

  ros::spin();

  return 0;
}

