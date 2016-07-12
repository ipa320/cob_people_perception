/*
 *****************************************************************
 * Copyright (c) 2015 \n
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA) \n\n
 *
 *****************************************************************
 *
 * \note
 * Project name: Care-O-bot
 * \note
 * ROS stack name: cob_people_perception
 * \note
 * ROS package name: cob_openni2_tracker
 *
 * \author
 * Author: Olha Meyer
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: Jan 15, 2015
 *
 * \brief
 * functions for detecting people using camera data
 * current approach: start a nodelet to launch the BodyTracker node and
 * camera driver simultaneously.
 *
 *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the Fraunhofer Institute for Manufacturing
 * Engineering and Automation (IPA) nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/


#ifndef BODY_TRACKER_NODELET_H_
#define BODY_TRACKER_NODELET_H_

#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>
#include "cob_openni2_tracker/body_tracker.h"
#include "openni2_camera/openni2_device_manager.h"

//PLUGINLIB_DECLARE_CLASS(cob_openni2_tracker, body_tracker_nodelet, body_tracker_nodelet, nodelet::Nodelet)
class BodyTrackerNodelet : public nodelet::Nodelet
{
public:
	BodyTrackerNodelet();
	virtual ~BodyTrackerNodelet();
	virtual void onInit();
	void run();
	//boost::shared_ptr<BodyTracker> inst_;

protected:
	BodyTracker *bt_listener;
	boost::shared_ptr<boost::thread> tracker_thread_;
	volatile bool running_;
	std::string nodelet_manager_;
};

#endif /* BODY_TRACKER_NODELET_H_ */
