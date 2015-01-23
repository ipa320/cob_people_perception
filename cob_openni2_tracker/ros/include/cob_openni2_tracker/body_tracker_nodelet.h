/*
 * body_tracker_nodelet.h
 *
 *  Created on: Jan 15, 2015
 *      Author: rmb-om
 */

#ifndef BODY_TRACKER_NODELET_H_
#define BODY_TRACKER_NODELET_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "cob_openni2_tracker/body_tracker.h"

//PLUGINLIB_DECLARE_CLASS(cob_openni2_tracker, BodyTrackerNodelet, cob_openni2_tracker::BodyTrackerNodelet, nodelet::Nodelet);

class BodyTrackerNodelet : public nodelet::Nodelet
{
public:
	BodyTrackerNodelet();
	virtual ~BodyTrackerNodelet();
	virtual void onInit();

protected:
	BodyTracker *bt_listener;
};


#endif /* BODY_TRACKER_NODELET_H_ */
