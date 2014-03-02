/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2012 \n
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
 * ROS package name: cob_people_detection
 *
 * \author
 * Author: Richard Bormann
 * \author
 * Supervised by:
 *
 * \date Date of creation: 07.08.2012
 *
 * \brief
 * gateway for sensor messages
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

#include "cob_people_detection/sensor_message_gateway_node.h"
#include "cob_vision_utils/GlobalDefines.h"

// ros
#include <nodelet/nodelet.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// boost
#include <boost/bind.hpp>

// this should really be in the implementation (.cpp file)
#include <pluginlib/class_list_macros.h>

namespace cob_people_detection
{
class SensorMessageGatewayNodelet: public nodelet::Nodelet
{
protected:
	ros::NodeHandle node_handle_;
	SensorMessageGatewayNode* sensor_message_gateway_;

public:
	SensorMessageGatewayNodelet()
	{
		sensor_message_gateway_ = 0;
	}

	~SensorMessageGatewayNodelet()
	{
		if (sensor_message_gateway_ != 0)
			delete sensor_message_gateway_;
	}

	virtual void onInit()
	{
		node_handle_ = getNodeHandle();

		sensor_message_gateway_ = new SensorMessageGatewayNode(node_handle_);
	}
};

}
// watch the capitalization carefully
PLUGINLIB_DECLARE_CLASS(cob_people_detection, SensorMessageGatewayNodelet, cob_people_detection::SensorMessageGatewayNodelet, nodelet::Nodelet)
