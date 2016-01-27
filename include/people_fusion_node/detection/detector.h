#ifndef PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTOR_H_
#define PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTOR_H_

#include <ros/ros.h>

#include <people_fusion_node/detector_config.h>
#include <people_fusion_node/visualization/visualization_helper.h>

#include <people_fusion_node/consts.h>

// Transforms
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

class Detector{

  private:

    int id_;

    ros::NodeHandle nh_; /**< The node handle */
    ros::Publisher internal_pub_; /**< The internal publisher */
    std::string name_;

    double weight_;
    double min_frequency_;

    size_t min_detections_;

    const std::string topic_;

    VisualizationHelper vh_;

    tf::TransformListener tfl_; /**< The transform listener */

    message_filters::Subscriber<cob_perception_msgs::DetectionArray> detections_sub_;
    tf::MessageFilter<cob_perception_msgs::DetectionArray> detection_notifier_;

  public:

    Detector(ros::NodeHandle nh, detector_config detector_cfg, int id, size_t totalNumberDetectors);

    int getId() const { return this->id_; };

    void detectionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detectionArray);

    std::string getName() const { return this->name_; };

    double getWeight() const { return this->weight_; };

    double getMinFrequency() const { return this->min_frequency_; };

    size_t getMinDetections() const { return this->min_detections_; };

};




#endif /* PEOPLE_FUSION_NODE_INCLUDE_PEOPLE_FUSION_NODE_DETECTION_DETECTOR_H_ */
