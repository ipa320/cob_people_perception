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

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// boost
#include <boost/bind.hpp>

namespace cob_people_detection
{
SensorMessageGatewayNode::SensorMessageGatewayNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	ros::Duration(5.0).sleep();

	it_ = 0;

	last_publishing_time_pcl_ = ros::Time::now();
	last_publishing_time_image_ = ros::Time::now();

	// Parameters
	std::cout << "\n--------------------------\nSensor Message Gateway Parameters:\n--------------------------\n";
	node_handle_.param("target_publishing_rate", target_publishing_rate_, 100.0);
	std::cout << "target_publishing_rate = " << target_publishing_rate_ << std::endl;
	target_publishing_delay_ = ros::Duration(1.0 / target_publishing_rate_);
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << std::endl;

	// reconfigure server
	//	dynamic_reconfigure::Server<cob_people_detection::sensorMessageGatewayConfig>::CallbackType f;
	//	f = boost::bind(&SensorMessageGatewayNode::pointcloudCallback, this, _1, _2);
	reconfigure_server_.setCallback(boost::bind(&SensorMessageGatewayNode::reconfigureCallback, this, _1, _2));

	it_ = new image_transport::ImageTransport(node_handle_);

	// advertise topics
	pointcloud_pub_ = node_handle_.advertise<sensor_msgs::PointCloud2>("pointcloud_rgb_out", 1);
	color_image_pub_ = it_->advertise("colorimage_out", 1);

	// subscribe to sensor topic
	pointcloud_sub_ = node_handle_.subscribe("pointcloud_rgb_in", 1, &SensorMessageGatewayNode::pointcloudCallback, this);
	color_image_sub_.subscribe(*it_, "colorimage_in", 1);
	color_image_sub_.registerCallback(boost::bind(&SensorMessageGatewayNode::imageCallback, this, _1));

	std::cout << "SensorMessageGatewayNode initialized." << std::endl;
}

SensorMessageGatewayNode::~SensorMessageGatewayNode(void)
{
	if (it_ != 0)
		delete it_;
}

void SensorMessageGatewayNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
	//	ROS_INFO("%d MessageGateway: Time stamp of pointcloud message: %f. Delay: %f.", pointcloud->header.seq, pointcloud->header.stamp.toSec(), ros::Time::now().toSec()-pointcloud->header.stamp.toSec());

	//	// secure this access with a mutex
	//	boost::lock_guard<boost::mutex> lock(image_buffer_mutex_);

	// forward incoming message with desired rate
	//std::cout << "Time delay: " << time_delay.toSec() << std::endl;
	if (target_publishing_rate_ != 0.0 && (ros::Time::now() - last_publishing_time_pcl_) > target_publishing_delay_)
	{
		pointcloud_pub_.publish(*pointcloud);
		//color_image_pub_.publish(image_buffer_);
		if (display_timing_ == true)
			ROS_INFO("%d MessageGateway: Time stamp of pointcloud message: %f. Delay: %f.", pointcloud->header.seq, pointcloud->header.stamp.toSec(), ros::Time::now().toSec()-pointcloud->header.stamp.toSec());
		//std::cout << "published: " << (ros::Time::now()-last_publishing_time_).toSec() << std::endl;
		last_publishing_time_pcl_ = ros::Time::now();
	}
}

void SensorMessageGatewayNode::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	//	// secure this access with a mutex
	//	boost::lock_guard<boost::mutex> lock(image_buffer_mutex_);

	// store image message
	//image_buffer_ = *color_image_msg;

	// forward incoming message with desired rate
	if (target_publishing_rate_ != 0.0 && (ros::Time::now() - last_publishing_time_image_) > target_publishing_delay_)
	{
		color_image_pub_.publish(*color_image_msg);
		//ROS_INFO("%d MessageGateway: Time stamp of image message: %f. Delay: %f.", color_image_msg->header.seq, color_image_msg->header.stamp.toSec(), ros::Time::now().toSec()-color_image_msg->header.stamp.toSec());
		last_publishing_time_image_ = ros::Time::now();
	}
}

void SensorMessageGatewayNode::reconfigureCallback(cob_people_detection::sensor_message_gatewayConfig &config, uint32_t level)
{
	target_publishing_rate_ = config.target_publishing_rate;
	target_publishing_delay_ = ros::Duration(1.0 / target_publishing_rate_);
	ROS_INFO("Reconfigure request accomplished. New target publishing rate: %f", config.target_publishing_rate);
}
}
