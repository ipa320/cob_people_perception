
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
      detections_sub_(nh_, "people_detections", 10), //Subscriber
      detection_notifier_(detections_sub_, tfl_, fixed_frame, 10),
	  vh_(nh)
      {

        // Set the laserCallback
        detection_notifier_.registerCallback(boost::bind(&FusionNode::detectionCallback, this, _1));
        detection_notifier_.setTolerance(ros::Duration(0.01));

      };

void FusionNode::detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
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

  vh_.publishTracker(trackerList_);

}
