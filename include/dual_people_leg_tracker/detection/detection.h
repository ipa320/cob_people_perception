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
    bool is_real_; /**< Needed for 'fake' measurements */
    double fake_probability_;

    tf::Stamped<tf::Point> point_;          /**< Location of the detection, given within fixed frame */
    laser_processor::SampleSet* cluster_;
    unsigned int id_;                       /**< Unique id (within cycle)*/

  public:
    // Detection for real
    Detection(int id, tf::Stamped<tf::Point> point, laser_processor::SampleSet* cluster):
      id_(id),
      point_(point),
      cluster_(cluster),
      is_real_(true)
    {

    }
    // Fake Detection
    Detection(int id, tf::Stamped<tf::Point> point, double probability):
      id_(id),
      point_(point),
      fake_probability_(probability),
      is_real_(false)
    {

    }

    bool isReal(){
      return is_real_;
    }

    double getProbability(){
      if(isReal()){
        return cluster_->getProbability();
      }
      return fake_probability_;
    }

};

typedef boost::shared_ptr<Detection> DetectionPtr;


#endif /* PEOPLE_DUAL_PEOPLE_LEG_TRACKER_INCLUDE_DUAL_PEOPLE_LEG_TRACKER_DETECTION_DETECTION_H_ */
