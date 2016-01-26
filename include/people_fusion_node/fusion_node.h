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
#include <people_fusion_node/detector_config.h>
#include <people_fusion_node/detection/detector.h>


#include <people_fusion_node/DetectionExt.h>
#include <people_fusion_node/consts.h>

// Default variables
static unsigned int trackerIdCounter = 0;

struct detection_hist_element{
    ros::Time time_;
    std::string type_;
};


using namespace ros;

class FusionNode{

  private:
    NodeHandle nh_; /**< The node handle */

    tf::TransformListener tfl_; /**< The transform listener */

    ros::Publisher pub_internal_; /**< The internal */

    std::vector<detector_config> detector_configs_; /**< Configured detectors */
    std::vector<Detector*> detectors_;
    std::map<std::string, Detector*> detectors_map_;

    message_filters::Subscriber<people_fusion_node::DetectionExt> detections_sub_all_;
    message_filters::TimeSequencer<people_fusion_node::DetectionExt> sequencer;

    std::vector<detection_hist_element> detectionHistory_;
    std::map<std::string, size_t> detectionCounts_;

    double totalDetectorWeight_;

    /*
    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_0_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_0_;

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_1_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_1_;

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_2_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_2_;
*/
    std::vector<TrackerPtr> trackerList_;

    VisualizationHelper vh_;

    //ros::Publisher internal_pub_; /**< The internal publisher */

    ros::Publisher people_pub_; /**< The people publisher */



  public:
    FusionNode(ros::NodeHandle nh, std::vector<detector_config> detectors); /**< Constructor */

    ~FusionNode(); /**< Destructor */

    void detectionCallbackAll(const people_fusion_node::DetectionExt::ConstPtr& detectionMsg);

    void updateDetectionsCount();

    std::map<std::string, size_t> getDetectionsCounts() const;

};




#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_ */
