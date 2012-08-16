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
#include <cob_people_detection_msgs/DetectionArray.h>
#include <cob_people_detection_msgs/ColorDepthImageArray.h>

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

// external includes
#include "cob_vision_utils/GlobalDefines.h"


namespace ipa_PeopleDetector {

class PeopleDetectionDisplayNode
{
protected:
	//message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_; ///< Shared xyz image and color image topic
	image_transport::ImageTransport* it_;
//	image_transport::SubscriberFilter people_segmentation_image_sub_; ///< Color camera image topic
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::DetectionArray, cob_people_detection_msgs::ColorDepthImageArray, sensor_msgs::Image> >* sync_input_3_;
	message_filters::Subscriber<cob_people_detection_msgs::ColorDepthImageArray> face_detection_subscriber_; ///< receives the face messages from the face detector
	message_filters::Subscriber<cob_people_detection_msgs::DetectionArray> face_recognition_subscriber_; ///< receives the face messages from the detection tracker

	image_transport::Publisher people_detection_image_pub_; ///< topic for publishing the image containing the people positions

	ros::NodeHandle node_handle_; ///< ROS node handle

//	// parameters
//	bool display_; ///< if on, several debug outputs are activated
//	bool use_people_segmentation_; ///< enables the combination of face detections with the openni people segmentation
//	double face_redetection_time_; ///< timespan during which a face is preserved in the list of tracked faces although it is currently not visible
//	double min_segmented_people_ratio_color_; ///< the minimum area inside the face rectangle found in the color image that has to be covered with positive people segmentation results (from openni_tracker)
//	double min_segmented_people_ratio_range_; ///< the minimum area inside the face rectangle found in the range image that has to be covered with positive people segmentation results (from openni_tracker)
//	double tracking_range_m_; ///< maximum tracking manhattan distance for a face (in meters), i.e. a face can move this distance between two images and can still be tracked
//	double face_identification_score_decay_rate_; ///< face identification score decay rate (0 < x < 1), i.e. the score for each label at a certain detection location is multiplied by this factor
//	double min_face_identification_score_to_publish_; ///< minimum face identification score to publish (0 <= x < max_score), i.e. this score must be exceeded by a label at a detection location before the person detection is published (higher values increase robustness against short misdetections, but consider the maximum possible score max_score w.r.t. the face_identification_score_decay_rate: new_score = (old_score+1)*face_identification_score_decay_rate --> max_score = face_identification_score_decay_rate/(1-face_identification_score_decay_rate))
//	bool fall_back_to_unknown_identification_; ///< if this is true, the unknown label will be assigned for the identification of a person if it has the highest score, otherwise, the last detection of a name will display as label even if there has been a detection of Unknown recently for that face

public:

	PeopleDetectionDisplayNode(ros::NodeHandle nh);
	~PeopleDetectionDisplayNode();

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_people_detection_msgs::DetectionArray::ConstPtr& face_recognition_msg, const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg);
};

};

#endif // _PEOPLE_DETECTION_DISPLAY_
