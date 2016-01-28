#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_

// System includes
#include <vector>

// Ros includes
#include <ros/ros.h>

// Own includes
#include <people_fusion_node/tracker.h>

// Cob includes
#include <cob_perception_msgs/Detection.h>
#include <cob_perception_msgs/DetectionArray.h>

using namespace ros;

/**
 * Visualizes existing trackers and detection arrays using markers
 */
class VisualizationHelper{

  private:
    NodeHandle nh_; /**< The node handle */

    ros::Publisher visualization_pub_; /**< The visualization publisher */

    size_t totalNumberDetectors_; /**< Total number of detectors */


  public:

    VisualizationHelper(ros::NodeHandle nh, size_t totalNumberDetectors);

    /**
     * Publish trackers
     * @param trackerList
     */
    void publishTracker(std::vector<TrackerPtr> &trackerList);

    /**
     * Publish detections
     * @param detectionArray
     * @param id The detection id - same id same color - different id different color
     */
    void publishDetectionArray(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray, int id);
};



#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_ */
