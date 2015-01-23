/*
 * body_tracker_nodelet.cpp
 *
 *  Created on: Jan 15, 2015
 *      Author: rmb-om
 */


#include <cob_openni2_tracker/body_tracker_nodelet.h>


//PLUGINLIB_DECLARE_CLASS(cob_openni2_tracker, body_tracker_nodelet, body_tracker_nodelet, nodelet::Nodelet)

PLUGINLIB_EXPORT_CLASS(BodyTrackerNodelet, nodelet::Nodelet)

BodyTrackerNodelet::BodyTrackerNodelet(): Nodelet(), bt_listener(0)
{
	ROS_INFO("Init BodyTrackerNodelet.");
}

BodyTrackerNodelet::~BodyTrackerNodelet()
{
	if (bt_listener != 0) {
		delete bt_listener;
	}
}

void BodyTrackerNodelet::onInit()
{
	if(bt_listener == 0)
		bt_listener = new BodyTracker(getPrivateNodeHandle());
}


