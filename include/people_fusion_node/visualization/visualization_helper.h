#ifndef PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_
#define PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_

// System includes
#include <vector>

// Ros includes
#include <ros/ros.h>

// Own includes
#include <people_fusion_node/tracker.h>

using namespace ros;

class VisualizationHelper{
  private:
    NodeHandle nh_; /**< The node handle */

    ros::Publisher visualization_pub_; /**< The visualization publisher */

  public:

    VisualizationHelper(ros::NodeHandle nh);

    void publishTracker(std::vector<TrackerPtr> &trackerList);
};



#endif /* PEOPLEFUSIONNODE_INCLUDE_PEOPLE_FUSION_NODE_VISUALIZATION_VISUALIZATION_HELPER_H_ */
