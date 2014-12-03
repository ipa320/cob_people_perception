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

#ifndef __SENSOR_MESSAGE_GATEWAY_NODE_H__
#define __SENSOR_MESSAGE_GATEWAY_NODE_H__

#ifdef __LINUX__
#else
#endif

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>		// use as: directory_ = ros::package::getPath("cob_people_detection") + "/common/files/windows/";
// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// image transport
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// dynamic reconfigure
#include <dynamic_reconfigure/server.h>
#include <cob_people_detection/sensor_message_gatewayConfig.h>

// boost
#include <boost/thread/mutex.hpp>

namespace cob_people_detection
{
class SensorMessageGatewayNode
{
public:

	/// Constructor
	/// @param nh ROS node handle
	SensorMessageGatewayNode(ros::NodeHandle nh);
	~SensorMessageGatewayNode(void); ///< Destructor

protected:

	/// Callback for incoming point clouds
	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);

	void imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg);

	void reconfigureCallback(cob_people_detection::sensor_message_gatewayConfig &config, uint32_t level);

	dynamic_reconfigure::Server<cob_people_detection::sensor_message_gatewayConfig> reconfigure_server_;

	ros::NodeHandle node_handle_;

	ros::Subscriber pointcloud_sub_; ///< subscribes to a colored point cloud
	ros::Publisher pointcloud_pub_; ///< publisher for the colored point cloud

	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image input topic
	image_transport::Publisher color_image_pub_; ///< Color camera image output topic

	ros::Duration target_publishing_delay_; ///< computed from target_publishing_rate_

	// image message buffer
	sensor_msgs::Image image_buffer_; // stores the received color image until the corresponding pointcloud is received and published

	// mutex
	boost::mutex image_buffer_mutex_; ///< secures the access to the image buffer

	// parameters
	double target_publishing_rate_; ///< rate at which the input messages are published (in Hz)
	ros::Time last_publishing_time_pcl_; ///< time of the last publishing activity
	ros::Time last_publishing_time_image_; ///< time of the last publishing activity
	bool display_timing_; ///< displays runtimes
};

}

#endif // __SENSOR_MESSAGE_GATEWAY_NODE_H__
