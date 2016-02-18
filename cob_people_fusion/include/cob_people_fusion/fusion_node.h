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
#include <cob_people_fusion/tracker.h>
#include <cob_people_fusion/visualization/visualization_helper.h>
#include <cob_people_fusion/detector_config.h>
#include <cob_people_fusion/detection/detector.h>


#include <cob_people_fusion/DetectionExt.h>
#include <cob_people_fusion/consts.h>

// Default variables
static unsigned int trackerIdCounter = 0;

struct detection_hist_element{
    ros::Time time_;
    std::string type_;
};


using namespace ros;

class FusionNode{

  private:
    NodeHandle nh_;                                 /**< The node handle */

    tf::TransformListener tfl_;                     /**< The transform listener */

    ros::Publisher pub_internal_;                   /**< The internal */

    std::vector<detector_config> detector_configs_; /**< Configured detectors */
    std::vector<Detector*> detectors_;              /**< List of all detectors */
    std::map<std::string, Detector*> detectors_map_;/**< Mapping from detector names to detectors */

    message_filters::Subscriber<cob_people_fusion::DetectionExt> detections_sub_all_; /**< Internal subscriber */
    message_filters::TimeSequencer<cob_people_fusion::DetectionExt> sequencer;        /**< Time sequencer needed to keep incoming message in chronological order */

    std::vector<detection_hist_element> detectionHistory_; /**< history of incoming detections (trimmed to timeHorizon) */
    std::map<std::string, size_t> detectionCounts_;        /**< counts the detections of each type */

    double totalDetectorWeight_;                           /**< weight sum of all detectors */

    std::vector<TrackerPtr> trackerList_;                  /**< list of all trackers */

    VisualizationHelper vh_;                               /**< Assigned visualization helper */

    ros::Publisher people_pub_; /**< The people publisher */

    double timeHorizon_;


  private:

    void updateDetectionsCount(); /**< Recount the detections, needed because of trimming */

    std::map<std::string, size_t> getDetectionsCounts() const;

  public:
    FusionNode(ros::NodeHandle nh, std::vector<detector_config> detectors, double timeHorizon); /**< Constructor */

    ~FusionNode(); /**< Destructor */

    void detectionCallbackAll(const cob_people_fusion::DetectionExt::ConstPtr& detectionMsg);

};




#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_FUSION_NODE_H_ */
