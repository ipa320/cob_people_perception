#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_

// Ros includes
#include <ros/ros.h>

// System includes
#include <iostream>
#include <string>

// Cob includes
#include <cob_perception_msgs/Detection.h>
#include <cob_perception_msgs/DetectionArray.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// Default variables
static std::string fixed_frame              = "odom_combined";  // The fixed frame

using namespace ros;

class FusionNode{

  private:
    NodeHandle nh_; /**< The node handle */

    tf::TransformListener tfl_; /**< The transform listener */

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_;
    //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    //tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;


  public:
    FusionNode(ros::NodeHandle nh); /**< Constructor */

    void detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

};




#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_ */
