/*
 * occlusion_model.cpp
 *
 *  Created on: May 8, 2015
 *      Author: frm-ag
 */

#include <ros/console.h>
#include <sensor_msgs/LaserScan.h>

#include <tf/transform_listener.h>
#include <tf/tf.h>

#ifndef PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_
#define PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_

#define DEBUG_OCCLUSION_MODEL 1

class OcclusionModel{
  public:
    /**
     * The currently used laserscan
     */
    sensor_msgs::LaserScan scan_;

    /**
     * The used transformation listener
     */
    tf::TransformListener& tfl_;

    /**
     * Constructor
     * @param tf
     */
    OcclusionModel(tf::TransformListener& tfl);

    /**
     * Set the scan to use
     * @param scan
     */
    void updateScan(const sensor_msgs::LaserScan& scan);

    double getOcclusionProbability(tf::Stamped<tf::Point>);
};

typedef boost::shared_ptr<OcclusionModel> OcclusionModelPtr;

#endif /* PEOPLE_PEOPLE_TRACKING_FILTER_INCLUDE_PEOPLE_TRACKING_FILTER_OCCLUSION_MODEL_H_ */
