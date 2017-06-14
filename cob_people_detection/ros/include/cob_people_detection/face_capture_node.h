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
 * functions for capturing face images and storing them to a common database
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

#ifndef _FACE_CAPTURE_NODE_
#define _FACE_CAPTURE_NODE_

// standard includes
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
//#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/addDataAction.h>
#include <cob_people_detection/updateDataAction.h>
#include <cob_people_detection/deleteDataAction.h>

// services
#include <cob_people_detection/captureImage.h>
#include <cob_people_detection/finishRecording.h>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// external includes
#include "cob_vision_utils/GlobalDefines.h"

#include "cob_people_detection/face_recognizer.h"

namespace ipa_PeopleDetector
{

typedef actionlib::SimpleActionServer<cob_people_detection::addDataAction> AddDataServer;
typedef actionlib::SimpleActionServer<cob_people_detection::updateDataAction> UpdateDataServer;
typedef actionlib::SimpleActionServer<cob_people_detection::deleteDataAction> DeleteDataServer;

class FaceCaptureNode
{
protected:

	ros::NodeHandle node_handle_; ///< ROS node handle

	// mutex
	boost::mutex active_action_mutex_; ///< facilitates that only one action server can be active at the same time

	// face recognizer trainer
	FaceRecognizer face_recognizer_trainer_;
	std::vector<cv::Mat> face_images_; ///< Vector of face images
	std::vector<cv::Mat> face_depthmaps_; ///< Vector of face depthmaps
	std::string current_label_; ///< Label of currently captured images
	bool capture_image_; ///<
	int number_captured_images_; ///<
	bool finish_image_capture_; ///<
	enum CaptureMode
	{
		MANUAL = 0, CONTINUOUS
	};
	enum UpdateMode
	{
		BY_INDEX = 1, BY_LABEL
	};
	//enum DeleteMode {BY_INDEX=1, BY_LABEL};

	image_transport::ImageTransport* it_;
	//	image_transport::SubscriberFilter people_segmentation_image_sub_; ///< Color camera image topic
	//	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray, cob_perception_msgs::ColorDepthImageArray, sensor_msgs::Image> >* sync_input_3_;
	//	message_filters::Subscriber<cob_perception_msgs::DetectionArray> face_recognition_subscriber_; ///< receives the face messages from the detection tracker
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::ColorDepthImageArray, sensor_msgs::Image> >* sync_input_2_;
	message_filters::Subscriber<cob_perception_msgs::ColorDepthImageArray> face_detection_subscriber_; ///< receives the face messages from the face detector
	image_transport::SubscriberFilter color_image_sub_; ///< Color camera image topic

	// actions
	AddDataServer* add_data_server_; ///< Action server that handles add data requests
	UpdateDataServer* update_data_server_; ///< Action server that handles update data requests
	DeleteDataServer* delete_data_server_; ///< Action server that handles delete data requests

	// services
	ros::ServiceServer service_server_capture_image_; ///< Service server that triggers an image recording
	ros::ServiceServer service_server_finish_recording_; ///< Service server that finishes image recording

	// parameters
	std::string data_directory_; ///< path to the classifier model
	//	bool display_; ///< if on, several debug outputs are activated
	//	bool use_people_segmentation_; ///< enables the combination of face detections with the openni people segmentation
	//	double face_redetection_time_; ///< timespan during which a face is preserved in the list of tracked faces although it is currently not visible
	//	double min_segmented_people_ratio_color_; ///< the minimum area inside the face rectangle found in the color image that has to be covered with positive people segmentation results (from openni_tracker)
	//	double min_segmented_people_ratio_range_; ///< the minimum area inside the face rectangle found in the range image that has to be covered with positive people segmentation results (from openni_tracker)
	//	double tracking_range_m_; ///< maximum tracking manhattan distance for a face (in meters), i.e. a face can move this distance between two images and can still be tracked
	//	double face_identification_score_decay_rate_; ///< face identification score decay rate (0 < x < 1), i.e. the score for each label at a certain detection location is multiplied by this factor
	//	double min_face_identification_score_to_publish_; ///< minimum face identification score to publish (0 <= x < max_score), i.e. this score must be exceeded by a label at a detection location before the person detection is published (higher values increase robustness against short misdetections, but consider the maximum possible score max_score w.r.t. the face_identification_score_decay_rate: new_score = (old_score+1)*face_identification_score_decay_rate --> max_score = face_identification_score_decay_rate/(1-face_identification_score_decay_rate))
	//	bool fall_back_to_unknown_identification_; ///< if this is true, the unknown label will be assigned for the identification of a person if it has the highest score, otherwise, the last detection of a name will display as label even if there has been a detection of Unknown recently for that face


	void addDataServerCallback(const cob_people_detection::addDataGoalConstPtr& goal);

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg);//, const sensor_msgs::Image::ConstPtr& color_image_msg);

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);
	unsigned long convertDepthImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	bool captureImageCallback(cob_people_detection::captureImage::Request &req, cob_people_detection::captureImage::Response &res);

	bool finishRecordingCallback(cob_people_detection::finishRecording::Request &req, cob_people_detection::finishRecording::Response &res);

	void updateDataServerCallback(const cob_people_detection::updateDataGoalConstPtr& goal);

	void deleteDataServerCallback(const cob_people_detection::deleteDataGoalConstPtr& goal);

public:

	FaceCaptureNode(ros::NodeHandle nh);
	~FaceCaptureNode();
};

}
;

#endif // _FACE_CAPTURE_NODE_
