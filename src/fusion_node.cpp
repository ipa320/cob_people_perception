
// Own includes
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>
#include <people_fusion_node/visualization/visualization_helper.h>

// ROS includes
#include <ros/ros.h>
#include <ros/console.h>

// Debug defines
#define FUSION_NODE_DEBUG 1         // Debug the leg detector
#define FUSION_NODE_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

// Constructor
FusionNode::FusionNode(ros::NodeHandle nh) :
      nh_(nh),  // Node Handle
      topic0_("people_detections"),
      //topic1_("people_detections/pcl_detections"),
      //topic2_("people_detections/face_detections"),
      detections_sub_0_(nh_, "people_detections/pcl_detections", 10), //Subscriber
      detection_notifier_0_(detections_sub_0_, tfl_, fixed_frame, 10),
      detections_sub_1_(nh_, "people_detections", 10), //Subscriber
      detection_notifier_1_(detections_sub_1_, tfl_, fixed_frame, 10),
      detections_sub_2_(nh_, "people_detections/face_detections", 10), //Subscriber
      detection_notifier_2_(detections_sub_2_, tfl_, fixed_frame, 10),
      vh_(nh)
      {
        std::cout << "Created Fusion Node!" << std::endl;


        // Set the laserCallback
        detection_notifier_0_.registerCallback(boost::bind(&FusionNode::detectionCallback0, this, _1));
        detection_notifier_0_.setTolerance(ros::Duration(0.01));

        // Set the laserCallback
        detection_notifier_1_.registerCallback(boost::bind(&FusionNode::detectionCallback1, this, _1));
        detection_notifier_1_.setTolerance(ros::Duration(0.01));

        // Set the laserCallback
        detection_notifier_2_.registerCallback(boost::bind(&FusionNode::detectionCallback2, this, _1));
        detection_notifier_2_.setTolerance(ros::Duration(0.01));

      };

void FusionNode::detectionCallback0(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());


  // Development: Clear Tracker list
  trackerList_.clear();

  for(int i = 0; i < detectionArray->detections.size(); i++){

	  // Create temp Tracker for every detection
	  cob_perception_msgs::Detection det = detectionArray->detections[i];

	  // Initial State
	  StatePosVel initialState(det.pose.pose.position.x, det.pose.pose.position.y);

	  TrackerPtr t = TrackerPtr(new Tracker(initialState, detectionArray->header.stamp));

	  std::cout << *t << std::endl;

	  trackerList_.push_back(t);

	  //std::cout << "Tracker" << "<" << detectionArray->detections[i].pose.pose.position.x << ", " << detectionArray->detections[i].pose.pose.position.y << ">" << std::endl;
  }

  vh_.publishDetectionArray(detectionArray,0);

}

void FusionNode::detectionCallback1(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "detectionCallback1" << std::endl;

  vh_.publishDetectionArray(detectionArray,1);

}

void FusionNode::detectionCallback2(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "detectionCallback2" << std::endl;

  vh_.publishDetectionArray(detectionArray,2);

}
