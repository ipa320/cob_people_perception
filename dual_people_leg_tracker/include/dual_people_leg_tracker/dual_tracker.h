/*
 * leg_detector.h
 *
 *  Created on: Mar 19, 2015
 *      Author: frm-ag
 */

#ifndef DUAL_TRACKER_H_
#define DUAL_TRACKER_H_

// ROS includes
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

// ROS Messages
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

// Own includes
#include <dual_people_leg_tracker/DualTrackerConfig.h>

// OpenCV includes
#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/ml.h>

// ROS Messages
#include <people_msgs/PositionMeasurement.h>
#include <people_msgs/PositionMeasurementArray.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/Marker.h>

// People Stack
#include <people_tracking_filter/state_pos_vel.h>
#include <leg_detector/laser_processor.h>

// OMP
#include <omp.h>

using namespace std;
using namespace laser_processor;
using namespace ros;
using namespace tf;
using namespace BFL;

static double no_observation_timeout_s = 0.5;
static double max_meas_jump_           = 0.75; // 1.0


#endif /* DUAL_TRACKER_H_ */
