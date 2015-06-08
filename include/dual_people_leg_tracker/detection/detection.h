/*
 * detection.h
 *
 *  Created on: Jun 8, 2015
 *      Author: frm-ag
 */

#ifndef PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_DETECTION_DETECTION_H_
#define PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_DETECTION_DETECTION_H_

#include <tf/transform_datatypes.h>
#include <leg_detector/laser_processor.h>

class Detection{
  public:
    tf::Stamped<tf::Point> point;
    laser_processor::SampleSet* cluster;
};

typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_DETECTION_DETECTION_H_ */
