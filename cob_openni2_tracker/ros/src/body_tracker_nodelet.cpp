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

#include <cob_openni2_tracker/body_tracker_nodelet.h>
#include "nodelet/loader.h"
#include "nodelet/NodeletList.h"
#include "nodelet/NodeletLoad.h"
#include "nodelet/NodeletUnload.h"
#include <boost/thread.hpp>

PLUGINLIB_EXPORT_CLASS(BodyTrackerNodelet, nodelet::Nodelet);

BodyTrackerNodelet::BodyTrackerNodelet(): Nodelet(), bt_listener(NULL)
{
}

BodyTrackerNodelet::~BodyTrackerNodelet()
{
	if (bt_listener != NULL)
	{
		bt_listener->shutdown_ = true;
		tracker_thread_->join();
		delete bt_listener;
	}
}

void BodyTrackerNodelet::onInit()
{
	NODELET_INFO("Init BodyTrackerNodelet.");

	getPrivateNodeHandle().param<std::string>("nodelet_manager", nodelet_manager_, "");
	std::cout << "nodelet_manager = " << nodelet_manager_ << "\n";

	if(bt_listener == 0)
		bt_listener = new BodyTracker(getPrivateNodeHandle());

	if (bt_listener->shutdown_ == true)
	{
		delete bt_listener;
		bt_listener = 0;
//		nodelet::NodeletUnload req;
//		req.request.name = this->getPrivateNodeHandle().getNamespace();
//		NODELET_INFO("name: %s", this->getPrivateNodeHandle().getNamespace().c_str());
//		ros::ServiceClient unload = this->getNodeHandle().serviceClient<nodelet::NodeletUnload>(nodelet_manager_ + "/unload_nodelet");
//		unload.call(req);
	}
	else
	{
		tracker_thread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&BodyTrackerNodelet::run, this)));
	}
}

void BodyTrackerNodelet::run()
{
	bt_listener->runTracker(); // tracking loop is stopped by setting bt_listener->shutdown_ = true;
}
