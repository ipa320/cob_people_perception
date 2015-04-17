/*
 * features.h
 *
 *  Created on: Apr 15, 2015
 *      Author: frm-ag
 */

#ifndef LEG_FEATURE_H_
#define LEG_FEATURE_H_

#include <ros/ros.h>

// Own includes

// Transforms
#include <tf/transform_listener.h>

// People tracking
#include <people_tracking_filter/advanced_tracker_particle.h>
#include <people_tracking_filter/state_pos_vel.h>
#include <people_tracking_filter/rgb.h>

//using namespace std;
//using namespace ros;
//using namespace tf;
//using namespace estimation;
//using namespace BFL;
//using namespace MatrixWrapper;

// Default variables
#define DEBUG_LEG_TRACKER 1



/**
 *  \brief The low level tracker to track each leg
 */
class LegFeature
{
public:
  static int nextid;
  tf::TransformListener& tfl_;
  std::string fixed_frame_;

  BFL::StatePosVel sys_sigma_;
  estimation::AdvancedTrackerParticle filter_;

  std::string id_;
  std::string object_id;
  int int_id_;
  ros::Time time_;
  ros::Time meas_time_;

  double reliability, p;

  bool use_filter_;



  tf::Stamped<tf::Point> position_;
  LegFeature* other;
  float dist_to_person_;


  LegFeature(tf::Stamped<tf::Point> loc, tf::TransformListener& tfl);

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

#endif /* LEG_FEATURE_H_ */
