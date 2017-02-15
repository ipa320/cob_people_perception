/*
 * features.h
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */

#ifndef SAVED_FEATURE_H_
#define SAVED_FEATURE_H_

#include <ros/ros.h>

// Own includes

// Transforms
#include <tf/transform_listener.h>

// People tracking
#include <cob_people_tracking_filter/tracker_kalman.h>
#include <cob_people_tracking_filter/state_pos_vel.h>
#include <cob_people_tracking_filter/rgb.h>

//using namespace std;
//using namespace ros;
//using namespace tf;
//using namespace estimation;
//using namespace BFL;
//using namespace MatrixWrapper;

// Default variables
#define DEBUG_SAVED_FEATURE 1

/**
 *  \brief
 */
class SavedFeature
{
public:
  static int nextid;
  tf::TransformListener& tfl_;
  std::string fixed_frame_;

  BFL::StatePosVel sys_sigma_;
  estimation::TrackerKalman filter_;

  std::string id_;
  std::string object_id;
  int int_id_;
  ros::Time time_;
  ros::Time meas_time_;

  double reliability, p;

  bool use_filter_;



  tf::Stamped<tf::Point> position_;
  SavedFeature* other;
  float dist_to_person_;


  SavedFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl);

  void propagate(ros::Time time);

  void update(tf::Stamped<tf::Point> loc, double probability);

  double getLifetime()
  {
    return filter_.getLifetime();
  }

  double getReliability()
  {
    return reliability;
  }

private:
  void updatePosition();
};

#endif /* SAVED_FEATURE_H_ */
