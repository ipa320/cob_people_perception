
// Own includes
#include <people_fusion_node/fusion_node.h>

// Debug defines
#define FUSION_NODE_DEBUG 1         // Debug the leg detector
#define FUSION_NODE_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

// Constructor
FusionNode::FusionNode(ros::NodeHandle nh) :
      nh_(nh),  // Node Handle
      detections_sub_(nh_, "people_detections", 10), //Subscriber
      detection_notifier_(detections_sub_, tfl_, fixed_frame, 10)
      {

        // Set the laserCallback
        detection_notifier_.registerCallback(boost::bind(&FusionNode::detectionCallback, this, _1));
        detection_notifier_.setTolerance(ros::Duration(0.01));

      };

void FusionNode::detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
}
