#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_

// Ros includes
#include <ros/ros.h>

// System includes
#include <iostream>
#include <string>
#include <vector>

// Cob includes
#include <cob_perception_msgs/Detection.h>
#include <cob_perception_msgs/DetectionArray.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_sequencer.h>

// Own includes
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/visualization/visualization_helper.h>
#include <people_fusion_node/detection_types.h>

// Default variables
static std::string fixed_frame              = "odom_combined";  // The fixed frame
static unsigned int trackerIdCounter = 0;


using namespace ros;

class FusionNode{

  private:
    NodeHandle nh_; /**< The node handle */

    tf::TransformListener tfl_; /**< The transform listener */

    ros::Publisher pub_internal_; /**< The internal */


    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_all_;
    message_filters::TimeSequencer<cob_perception_msgs::DetectionArray> sequencer;

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_0_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_0_;

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_1_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_1_;

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_2_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_2_;


    //message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
    //tf::MessageFilter<people_msgs::PositionMeasurement> people_notifier_;

    std::vector<TrackerPtr> trackerList_;

    // Has own visualizationHelper
    VisualizationHelper vh_;

    const std::string topic0_, topic1_, topic2_;

    ros::Publisher internal_pub_; /**< The internal publisher */


  public:
    FusionNode(ros::NodeHandle nh); /**< Constructor */

    void detectionCallback0(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

    void detectionCallback1(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

    void detectionCallback2(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

    void detectionCallbackAll(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

};




#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_ */
