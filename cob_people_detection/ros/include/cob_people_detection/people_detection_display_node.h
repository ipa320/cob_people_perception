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
 * functions for display of people detections
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

#ifndef _PEOPLE_DETECTION_DISPLAY_
#define _PEOPLE_DETECTION_DISPLAY_

// standard includes
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// external includes
#include "cob_vision_utils/GlobalDefines.h"

namespace ipa_PeopleDetector
{

class PeopleDetectionDisplayNode
{
protected:
	//message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_; ///< Shared xyz image and color image topic
	//	image_transport::SubscriberFilter people_segmentation_image_sub_; ///< Color camera image topic

	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter colorimage_sub_; ///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray, cob_perception_msgs::ColorDepthImageArray,
			sensor_msgs::Image> >* sync_input_3_;
	//	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray, cob_perception_msgs::ColorDepthImageArray, sensor_msgs::PointCloud2> >* sync_input_3_;
	//	message_filters::Subscriber<sensor_msgs::PointCloud2> pointcloud_sub_;
	message_filters::Subscriber<cob_perception_msgs::ColorDepthImageArray> face_detection_subscriber_; ///< receives the face messages from the face detector
	message_filters::Subscriber<cob_perception_msgs::DetectionArray> face_recognition_subscriber_; ///< receives the face messages from the detection tracker

	image_transport::Publisher people_detection_image_pub_; ///< topic for publishing the image containing the people positions

	ros::NodeHandle node_handle_; ///< ROS node handle

	// parameters
	bool display_; ///< if on, several debug outputs are activated
	bool display_timing_;

public:

	PeopleDetectionDisplayNode(ros::NodeHandle nh);
	~PeopleDetectionDisplayNode();

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_recognition_msg,
			const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& colorimage_msg);
	//	void inputCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_recognition_msg, const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg);
};

}
;

#endif // _PEOPLE_DETECTION_DISPLAY_
