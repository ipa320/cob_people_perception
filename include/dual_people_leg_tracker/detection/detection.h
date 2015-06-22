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
    tf::Stamped<tf::Point> point_;          /**< Location of the detection, given within fixed frame */
    laser_processor::SampleSet* cluster_;
    unsigned int id_;                       /**< Unique id (within cycle)*/
};

typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_DETECTION_DETECTION_H_ */
