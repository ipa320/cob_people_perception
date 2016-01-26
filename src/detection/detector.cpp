#include <people_fusion_node/detection/detector.h>
#include <people_fusion_node/DetectionExt.h>

Detector::Detector(ros::NodeHandle nh, detector_config detector_cfg):
  nh_(nh),
  topic_(detector_cfg.topic),
  name_(detector_cfg.name),
  weight_(detector_cfg.weight),
  min_frequency_(detector_cfg.min_frequency),
  min_detections_(detector_cfg.min_detections),
  detections_sub_(nh_, topic_, 25), //Subscriber
  detection_notifier_(detections_sub_, tfl_, fixed_frame, 25),
  vh_(nh)
{
  // Subscribe
  detection_notifier_.registerCallback(boost::bind(&Detector::detectionCallback, this, _1));
  detection_notifier_.setTolerance(ros::Duration(0.01));

  internal_pub_= nh_.advertise<people_fusion_node::DetectionExt>("people_detections/internal/all_detections", 0);
}

void Detector::detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray){
  //ROS_DEBUG_COND(FUSION_NODE_DEBUG, "FusionNode::%s - Number of detections: %i", __func__, (int) detectionArray->detections.size());
  std::cout << "Received on " << this->topic_ << std::endl;

  // Check if the detector field in the message is correct
  bool detectionError = false;
  for(size_t i = 0; i < detectionArray->detections.size(); i++){
    if(this->name_ != detectionArray->detections[i].detector){
      ROS_ERROR("The name of the detector \"%s\" defined in the message does not match \"%s\" given in the config file", detectionArray->detections[i].detector.c_str(), this->name_.c_str());
      ROS_BREAK();
    }
  }

  people_fusion_node::DetectionExt detectionMsg;
  detectionMsg.detections = *detectionArray;
  detectionMsg.header = detectionArray->header;
  detectionMsg.detector = this->name_;

  if(!detectionError){
    vh_.publishDetectionArray(detectionArray,0);
    internal_pub_.publish(detectionMsg);
  }
  else{
    ROS_ERROR("Due to errors no detections are forwarded by %s", this->name_.c_str());
  }

}
