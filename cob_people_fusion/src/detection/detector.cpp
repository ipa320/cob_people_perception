#include <cob_people_fusion/detection/detector.h>
#include <cob_people_fusion/visualization/color_definitions.h>
#include <cob_people_fusion/DetectionExt.h>

Detector::Detector(ros::NodeHandle nh, detector_config detector_cfg, int id, size_t totalNumberDetectors):
  id_(id),
  nh_(nh),
  topic_(detector_cfg.topic),
  name_(detector_cfg.name),
  weight_(detector_cfg.weight),
  min_frequency_(detector_cfg.min_frequency),
  min_detections_(detector_cfg.min_detections),
  detections_sub_(nh_, topic_, 25), //Subscriber
  detection_notifier_(detections_sub_, tfl_, fixed_frame, 25),
  vh_(nh, totalNumberDetectors)
{
  // Subscribe
  detection_notifier_.registerCallback(boost::bind(&Detector::detectionCallback, this, _1));
  detection_notifier_.setTolerance(ros::Duration(0.01));

  internal_pub_= nh_.advertise<cob_people_fusion::DetectionExt>("all_detections", 0);
}

void Detector::detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray){
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());


  // Check if the detector field in the message is correct
  bool detectionError = false;
  for(size_t i = 0; i < detectionArray->detections.size(); i++){
    if(this->name_ != detectionArray->detections[i].detector){
      ROS_ERROR("The name of the detector \"%s\" defined in the message does not match \"%s\" given in the config file", detectionArray->detections[i].detector.c_str(), this->name_.c_str());
      //ROS_BREAK();
    }
  }

  cob_people_fusion::DetectionExt detectionMsg;
  detectionMsg.detections = *detectionArray;
  detectionMsg.header = detectionArray->header;
  detectionMsg.detector = this->name_;

  if(!detectionError){
    vh_.publishDetectionArray(detectionArray, this->getId());
    internal_pub_.publish(detectionMsg);
    ROS_INFO_STREAM(WHITE << "Received on " << this->topic_ << " forwarded to all_detections" << RESET);

  }
  else{
    ROS_INFO_STREAM(WHITE << "Received on " << this->topic_ << RED << " NOT FORWARDED DUE TO ERROR" << RESET);
  }

}
