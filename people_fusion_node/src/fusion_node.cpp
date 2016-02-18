
// ROS includes
#include <ros/ros.h>
#include <ros/console.h>

// Own includes
#include <people_fusion_node/fusion_node.h>
#include <people_fusion_node/tracker.h>
#include <people_fusion_node/state_pos_vel.h>
#include <people_fusion_node/visualization/visualization_helper.h>
#include <people_fusion_node/visualization/color_definitions.h>
#include <people_fusion_node/detection/detection.h>
#include <people_fusion_node/association/association.h>
#include <people_fusion_node/association/associatorGNN.h>

#include <people_fusion_node/detector_config.h>



// Debug defines
#define FUSION_NODE_DEBUG 1         // Debug the leg detector
#define FUSION_NODE_TIME_DEBUG 1    // Debug the calculation time inside the leg_detector

// Constructor
FusionNode::FusionNode(ros::NodeHandle nh, std::vector<detector_config> detector_configs, double timeHorizon) :
      nh_(nh),  // Node Handle
      detector_configs_(detector_configs),
      detections_sub_all_(nh_, "people_detections/internal/all_detections", 50), //Subscriber
      vh_(nh, detector_configs.size()),
      sequencer(detections_sub_all_, ros::Duration(1), ros::Duration(0.01), 25),
      totalDetectorWeight_(0),
      timeHorizon_(timeHorizon)
      {
        std::cout << "Created Fusion Node! TimeHorizon: " << timeHorizon_ << std::endl;

        // Create detector
        for(size_t i = 0; i < detector_configs_.size(); i++){
          Detector* detector_ptr = new Detector(nh_, detector_configs[i], i, detector_configs.size());

          // Insert into vector
          detectors_.push_back(detector_ptr);

          // Insert into map
          if ( detectors_map_.find(detector_ptr->getName()) == detectors_map_.end() ) {
            detectors_map_.insert ( std::pair<std::string,Detector*>(detector_ptr->getName(),detector_ptr) );

            // Calculate the total weight
            totalDetectorWeight_ += detector_ptr->getWeight();

          } else {
            ROS_ERROR("You have multiple detectors named \"%s\", please use unique names!", detector_ptr->getName().c_str());
            ROS_BREAK();
          }

          // Initialize the detections count
          if ( detectionCounts_.find(detector_ptr->getName()) == detectionCounts_.end() ) {
            detectionCounts_.insert ( std::pair<std::string, size_t>(detector_ptr->getName(),0) );
          }

        }

        // Set the laserCallback
        sequencer.registerCallback(boost::bind(&FusionNode::detectionCallbackAll, this, _1));
        //seq.setTolerance(ros::Duration(0.01));

        people_pub_= nh_.advertise<cob_perception_msgs::DetectionArray>("people_detections/fused_detections", 0);

      };

FusionNode::~FusionNode(){
  // Remove the detectors
  for (std::vector<Detector*>::iterator it = detectors_.begin() ; it != detectors_.end(); ++it)
    {
      delete (*it);
    }
  detectors_.clear();
}


void FusionNode::detectionCallbackAll(const people_fusion_node::DetectionExt::ConstPtr& detectionMsg)
{
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  //std::cout << BOLDYELLOW << "Received " << detectionArray->detections.size() << ". Time: " << detectionArray->header.stamp << std::endl;
  std::string detectionTyp = detectionMsg->detector;
  ROS_INFO_STREAM(WHITE << "[" << detectionMsg->header.stamp << "] " << " Received " << detectionMsg->detections.detections.size() << " detections on " << detectionMsg->detector << RESET);
  cob_perception_msgs::DetectionArray::ConstPtr detectionArray(new cob_perception_msgs::DetectionArray(detectionMsg->detections));


  ros::Time currentTime = detectionMsg->header.stamp;

  // Care about the history
  detection_hist_element hist_el;
  hist_el.time_ = detectionMsg->header.stamp;
  hist_el.type_ = detectionMsg->detector;

  // Trim the history to the horizon
  this->detectionHistory_.push_back(hist_el);
  detection_hist_element oldest_hist_el = this->detectionHistory_.front();
  detection_hist_element newest_hist_el = this->detectionHistory_.back();

  double currentHorizon = (newest_hist_el.time_ - oldest_hist_el.time_).toSec();

  while(currentHorizon > timeHorizon_){

      // Remove first element
      this->detectionHistory_.erase( this->detectionHistory_.begin() );

      // Set the new oldest element
      oldest_hist_el = this->detectionHistory_.front();

      currentHorizon = (newest_hist_el.time_ - oldest_hist_el.time_).toSec();
  }

  // Update the detection count
  this->updateDetectionsCount();


  if(detectionArray->detections.size() == 0){
    ROS_DEBUG("No detections -> Abort");
    return;
  }
  else
  {
    ROS_DEBUG_STREAM("Received message from detector " << detectionArray->detections[0].detector << std::endl);
  }


  ///////////////////////////////////////////////////////////
  /// Delete old Trackers
  ///////////////////////////////////////////////////////////
  ROS_DEBUG_STREAM(BOLDWHITE << "Removing old trackers" << RESET << std::endl);
  std::vector<TrackerPtr> clearedTrackers;
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin();
      trackerIt < trackerList_.end();
      trackerIt++)
  {
    double ageSec = (currentTime - (*trackerIt)->getCurrentTime()).toSec();

    if(ageSec < 1.5){
      clearedTrackers.push_back(*trackerIt);
    }else{
      ROS_DEBUG_STREAM(" deleted age: " << ageSec << RESET << std::endl);
    }
  }

  trackerList_ = clearedTrackers;


  // Print trackers
  ROS_DEBUG_STREAM("Remaining trackers:" << std::endl);
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin(); trackerIt < trackerList_.end(); trackerIt++){
    ROS_DEBUG_STREAM("[" << (*trackerIt)->getId() << "]");
  }
  ROS_DEBUG_STREAM(std::cout << std::endl);


  // Store the detections in a list
  // TODO transform into fixed frame
  std::vector<DetectionPtr> detections;
  for(int i = 0; i < detectionArray->detections.size(); i++){
    // Create temp Tracker for every detection
    cob_perception_msgs::Detection det = detectionArray->detections[i];
    detections.push_back(DetectionPtr(new Detection(det.pose.pose.position.x, det.pose.pose.position.y, det.header.stamp, i, detectionTyp)));
  }


  ///////////////////////////////////////////////////////////
  /// ASSOCIATION
  ///////////////////////////////////////////////////////////
  ROS_DEBUG_STREAM(BOLDWHITE << "Association" << RESET << std::endl);


  // Make the associations
  AssociatorGNN associatiorGNN;
  std::vector<AssociationPtr> associations;

  std::vector<DetectionPtr> notAssociatedDetections; // Detections that were not used in the association

  associations = associatiorGNN.associateGNN(detections, trackerList_, notAssociatedDetections);

  ///////////////////////////////////////////////////////////
  /// APPLY THE ASSOCIATED DETECTIONS
  ///////////////////////////////////////////////////////////
  ROS_DEBUG_STREAM(BOLDWHITE << "Update" << RESET << std::endl);

  for(std::vector<AssociationPtr>::iterator assoIt = associations.begin();
      assoIt < associations.end();
      assoIt++)
  {
    (*assoIt)->getTracker()->update((*assoIt)->getDetection());
  }

  ///////////////////////////////////////////////////////////
  /// Calculate the tracker score
  ///////////////////////////////////////////////////////////
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin(); trackerIt < trackerList_.end(); trackerIt++){
    double score = 0;

    std::map<std::string, size_t> counts = (*trackerIt)->getUpdateCounts();
    std::map<std::string, size_t> totalCounts = this->getDetectionsCounts();
    std::map<std::string, double> frequencies = (*trackerIt)->getUpdateFrequencies();

    // Calculate the time correction factor for tracker existing shorter than timeHorizon
    double timeCorrectionFactor = 1;
    double trackerLifetime = (*trackerIt)->getLifetimeSec(currentTime);
    if(trackerLifetime < timeHorizon_)
      timeCorrectionFactor = timeHorizon_ / trackerLifetime;


    for(std::map<std::string, Detector*>::iterator mapIt = detectors_map_.begin(); mapIt != detectors_map_.end(); mapIt++) {

      Detector* detector = mapIt->second;

      // First check if min frequency and min counts hold
      if(frequencies[mapIt->first] > detector->getMinFrequency() &&
         counts[mapIt->first] > detector->getMinDetections()){


        if(totalCounts[mapIt->first] > 0)
        score += detector->getWeight() * counts[mapIt->first] / ((double) totalCounts[mapIt->first]) * timeCorrectionFactor;
      }


    }

    double scoreNormed = score/this->totalDetectorWeight_;

    (*trackerIt)->setScore(score/this->totalDetectorWeight_);

    // Iterate the detector map
  }

  ///////////////////////////////////////////////////////////
  /// TRACKER CREATION
  ///////////////////////////////////////////////////////////
  ROS_DEBUG_STREAM(BOLDWHITE << "Creation:" << RESET << std::endl);

  // Create a tracker for the unused detections
  for(std::vector<DetectionPtr>::iterator detectionIt = notAssociatedDetections.begin();
      detectionIt < notAssociatedDetections.end();
      detectionIt++)
  {
    TrackerPtr t = TrackerPtr(new Tracker((*detectionIt)->getState(), detectionArray->header.stamp, detector_configs_, timeHorizon_));
    trackerList_.push_back(t);
  }

  vh_.publishTracker(trackerList_);

  ///////////////////////////////////////////////////////////
  /// Publish results
  ///////////////////////////////////////////////////////////
  cob_perception_msgs::DetectionArray resultDetectionArray;
  resultDetectionArray.header = detectionArray->header;
  for(std::vector<TrackerPtr>::iterator trackerIt = trackerList_.begin(); trackerIt < trackerList_.end(); trackerIt++){
    cob_perception_msgs::Detection detection;
    detection.header = detectionArray->header;
    detection.pose.pose.position.x = (*trackerIt)->getCurrentState().getPos().getX();
    detection.pose.pose.position.y = (*trackerIt)->getCurrentState().getPos().getY();
    detection.pose.pose.position.z = 0;
    detection.label = (*trackerIt)->getIdStr();
    detection.score = (*trackerIt)->getScore();

    resultDetectionArray.detections.push_back(detection);
  }

  people_pub_.publish(resultDetectionArray);


}

// TODO would be more efficient to update directly on trimming!
void FusionNode::updateDetectionsCount(){

  // First reset
  for(std::map<std::string, size_t>::iterator histCountIt = detectionCounts_.begin(); histCountIt != detectionCounts_.end(); histCountIt++) {
    detectionCounts_[histCountIt->first] = 0;
  }

  // Then count
  for(std::vector<detection_hist_element>::iterator histIt = this->detectionHistory_.begin(); histIt < this->detectionHistory_.end(); histIt++){
    detectionCounts_[histIt->type_]++;
  }
}

std::map<std::string, size_t> FusionNode::getDetectionsCounts() const{
  return this->detectionCounts_;
}


