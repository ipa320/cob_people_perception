
// Own includes
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>
#include <people_fusion_node/visualization/visualization_helper.h>
#include <people_fusion_node/visualization/color_definitions.h>
#include <people_fusion_node/detection/detection.h>
#include <people_fusion_node/association/association.h>
#include <people_fusion_node/association/associatorGNN.h>

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
      detections_sub_all_(nh_, "people_detections/internal/all_detections", 50), //Subscriber

      detections_sub_0_(nh_, "people_detections/laser_detections", 25), //Subscriber
      detection_notifier_0_(detections_sub_0_, tfl_, fixed_frame, 25),
      detections_sub_1_(nh_, "people_detections/body_detections", 25), //Subscriber
      detection_notifier_1_(detections_sub_1_, tfl_, fixed_frame, 25),
      detections_sub_2_(nh_, "people_detections/face_detections", 25), //Subscriber
      detection_notifier_2_(detections_sub_2_, tfl_, fixed_frame, 25),
      vh_(nh),
      sequencer(detections_sub_all_, ros::Duration(1), ros::Duration(0.01), 25)
      {
        std::cout << "Created Fusion Node!" << std::endl;


        // Set the laserCallback
        sequencer.registerCallback(boost::bind(&FusionNode::detectionCallbackAll, this, _1));
        //seq.setTolerance(ros::Duration(0.01));

        detection_notifier_0_.registerCallback(boost::bind(&FusionNode::detectionCallback0, this, _1));
        detection_notifier_0_.setTolerance(ros::Duration(0.01));

        // Set the laserCallback
        detection_notifier_1_.registerCallback(boost::bind(&FusionNode::detectionCallback1, this, _1));
        detection_notifier_1_.setTolerance(ros::Duration(0.01));

        // Set the laserCallback
        detection_notifier_2_.registerCallback(boost::bind(&FusionNode::detectionCallback2, this, _1));
        detection_notifier_2_.setTolerance(ros::Duration(0.01));


        internal_pub_= nh_.advertise<cob_perception_msgs::DetectionArray>("people_detections/internal/all_detections", 0);

      };

void FusionNode::detectionCallback0(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "Received on people_detections/laser_detections" << std::endl;


  vh_.publishDetectionArray(detectionArray,0);
  internal_pub_.publish(detectionArray);
}

void FusionNode::detectionCallback1(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "Received on people_detections/body_detections" << std::endl;


  vh_.publishDetectionArray(detectionArray,1);
  internal_pub_.publish(detectionArray);

}
void FusionNode::detectionCallback2(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "Received on people_detections/face_detections" << std::endl;

  vh_.publishDetectionArray(detectionArray,2);
  internal_pub_.publish(detectionArray);

}

void FusionNode::detectionCallbackAll(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  //std::cout << BOLDYELLOW << "Received " << detectionArray->detections.size() << ". Time: " << detectionArray->header.stamp << std::endl;

  if(detectionArray->detections.size() == 0){
    std::cout << "No detections -> Abort" << std::endl;
    return;
  }

  Type detectionTyp;

  // Get the detection type
  if(detectionArray->detections.size() > 0){
    std::string typeString = detectionArray->detections[0].detector;
    if(typeString == "laser") {detectionTyp = laser;}
    else if(typeString == "body"){detectionTyp = body;}
    else if(typeString == "face"){detectionTyp = face;}
    else {detectionTyp = unkown; }
  }

  switch(detectionTyp){
    case laser:
      std::cout << BOLDRED << "[" << detectionArray->detections.size() << "]" << " Laser Detections" << RESET << std::endl;
      break;
    case body:
      std::cout << BOLDGREEN << "[" << detectionArray->detections.size() << "]" << " Body Detections" << RESET << std::endl;
      break;
    case face:
      std::cout << BOLDBLUE << "[" << detectionArray->detections.size() << "]" << " Face Detections" << RESET << std::endl;
      break;
  }

  for(size_t i = 0; i < detectionArray->detections.size(); i++){
    std::cout << "\t Detection: " << detectionArray->detections[i].label << std::endl;
  }



  // Development: Clear Tracker list
  //trackerList_.clear();

  ///////////////////////////////////////////////////////////
  /// Delete old Trackers
  ///////////////////////////////////////////////////////////
  std::cout << BOLDWHITE << "Removing old trackers" << RESET << std::endl;
  std::vector<TrackerPtr> clearedTrackers;
  ros::Time currentTime = detectionArray->header.stamp;
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin();
      trackerIt < trackerList_.end();
      trackerIt++)
  {
    double ageSec = (currentTime - (*trackerIt)->getCurrentTime()).toSec();

    if(ageSec < 1.5){
      clearedTrackers.push_back(*trackerIt);
    }else{
      std::cout << RED << " deleted age: " << ageSec << RESET << std::endl;
    }
  }

  trackerList_ = clearedTrackers;


  // Print trackers
  std::cout << "Remaining trackers:" << std::endl;
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin(); trackerIt < trackerList_.end(); trackerIt++){
    std::cout << "[" << (*trackerIt)->getId() << "]";
  }
  std::cout << std::endl;


  // Store the detections in a list
  std::vector<DetectionPtr> detections;
  for(int i = 0; i < detectionArray->detections.size(); i++){
    // Create temp Tracker for every detection
    cob_perception_msgs::Detection det = detectionArray->detections[i];
    detections.push_back(DetectionPtr(new Detection(det.pose.pose.position.x, det.pose.pose.position.y, det.header.stamp, i)));
  }


  ///////////////////////////////////////////////////////////
  /// ASSOCIATION
  ///////////////////////////////////////////////////////////
  std::cout << BOLDWHITE << "Association" << RESET << std::endl;


  // Make the associations
  AssociatorGNN associatiorGNN;
  std::vector<AssociationPtr> associations;

  std::vector<DetectionPtr> notAssociatedDetections; // Detections that were not used in the association

  associations = associatiorGNN.associateGNN(detections, trackerList_, notAssociatedDetections);



  ///////////////////////////////////////////////////////////
  /// APPLY THE ASSOCIATED DETECTIONS
  ///////////////////////////////////////////////////////////
  std::cout << BOLDWHITE << "Update" << RESET << std::endl;

  for(std::vector<AssociationPtr>::iterator assoIt = associations.begin();
      assoIt < associations.end();
      assoIt++)
  {
    std::cout << "Updating " << *((*assoIt)->getTracker()) << " with: " << *((*assoIt)->getDetection()) << " Distance: " << (*assoIt)->getDistance() << std::endl;
    (*assoIt)->getTracker()->update((*assoIt)->getDetection());
  }

  ///////////////////////////////////////////////////////////
  /// TRACKER CREATION
  ///////////////////////////////////////////////////////////

  std::cout << BOLDWHITE << "Creation:" << RESET << std::endl;


  // Create a tracker for the unused detections
  for(std::vector<DetectionPtr>::iterator detectionIt = notAssociatedDetections.begin();
      detectionIt < notAssociatedDetections.end();
      detectionIt++)
  {
    TrackerPtr t = TrackerPtr(new Tracker((*detectionIt)->getState(), detectionArray->header.stamp));
    std::cout << "\tCreated Tracker: " << *t << std::endl;
    trackerList_.push_back(t);
  }

  vh_.publishTracker(trackerList_);

}


