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
 * functions for detecting a face within a color image (patch)
 * current approach: haar detector on color image
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

#ifdef __LINUX__
#include "cob_people_detection/face_detector_node.h"
#include "cob_vision_utils/GlobalDefines.h"
#include "cob_people_detection/face_detection_message_helper.h"
#else
#endif

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/shared_ptr.hpp>

// timer
#include <cob_people_detection/timer.h>

using namespace ipa_PeopleDetector;

FaceDetectorNode::FaceDetectorNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";

	// Parameters
	double faces_increase_search_scale; // The factor by which the search window is scaled between the subsequent scans
	int faces_drop_groups; // Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int faces_min_search_scale_x; // Minimum search scale x
	int faces_min_search_scale_y; // Minimum search scale y
	bool reason_about_3dface_size; // if true, the 3d face size is determined and only faces with reasonable size are accepted
	double face_size_max_m; // the maximum feasible face diameter [m] if reason_about_3dface_size is enabled
	double face_size_min_m; // the minimum feasible face diameter [m] if reason_about_3dface_size is enabled
	double max_face_z_m; // maximum distance [m] of detected faces to the sensor
	bool debug; // enables some debug outputs
	std::cout << "\n--------------------------\nFace Detector Parameters:\n--------------------------\n";
	node_handle_.param("data_directory", data_directory_, data_directory_);
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("faces_increase_search_scale", faces_increase_search_scale, 1.1);
	std::cout << "faces_increase_search_scale = " << faces_increase_search_scale << "\n";
	node_handle_.param("faces_drop_groups", faces_drop_groups, 68);
	std::cout << "faces_drop_groups = " << faces_drop_groups << "\n";
	node_handle_.param("faces_min_search_scale_x", faces_min_search_scale_x, 20);
	std::cout << "faces_min_search_scale_x = " << faces_min_search_scale_x << "\n";
	node_handle_.param("faces_min_search_scale_y", faces_min_search_scale_y, 20);
	std::cout << "faces_min_search_scale_y = " << faces_min_search_scale_y << "\n";
	node_handle_.param("reason_about_3dface_size", reason_about_3dface_size, true);
	std::cout << "reason_about_3dface_size = " << reason_about_3dface_size << "\n";
	node_handle_.param("face_size_max_m", face_size_max_m, 0.35);
	std::cout << "face_size_max_m = " << face_size_max_m << "\n";
	node_handle_.param("face_size_min_m", face_size_min_m, 0.1);
	std::cout << "face_size_min_m = " << face_size_min_m << "\n";
	node_handle_.param("max_face_z_m", max_face_z_m, 8.0);
	std::cout << "max_face_z_m = " << max_face_z_m << "\n";
	node_handle_.param("debug", debug, false);
	std::cout << "debug = " << debug << "\n";
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << "\n";

	// initialize face detector
	face_detector_.init(data_directory_, faces_increase_search_scale, faces_drop_groups, faces_min_search_scale_x, faces_min_search_scale_y, reason_about_3dface_size,
			face_size_max_m, face_size_min_m, max_face_z_m, debug);

	// advertise topics
	face_position_publisher_ = node_handle_.advertise<cob_perception_msgs::ColorDepthImageArray>("face_positions", 1);
	face_position_publisher_cartesian_ = node_handle_.advertise<cob_perception_msgs::DetectionArray>("face_detections_cartesian", 1);

	// subscribe to head detection topic
	head_position_subscriber_ = nh.subscribe("head_positions", 1, &FaceDetectorNode::head_positions_callback, this);

	std::cout << "FaceDetectorNode initialized." << std::endl;
}

FaceDetectorNode::~FaceDetectorNode(void)
{
}

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(const sensor_msgs::Image* const )
{
}

void FaceDetectorNode::head_positions_callback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& head_positions)
{
	//	Timer tim;
	//	tim.start();

	// receive head positions and detect faces in the head region, finally publish detected faces

	// convert color and depth image patches of head regions
	std::vector<cv::Mat> heads_color_images(head_positions->head_detections.size());
	std::vector<cv::Mat> heads_depth_images(head_positions->head_detections.size());
	std::vector<cv::Rect> head_bounding_boxes(head_positions->head_detections.size());
	cv_bridge::CvImageConstPtr cv_cptr;
	cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);
	for (unsigned int i = 0; i < head_positions->head_detections.size(); i++)
	{
		// color image
		sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(head_positions->head_detections[i].color_image), voidDeleter);
		try
		{
			cv_cptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::RGB8);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		heads_color_images[i] = cv_cptr->image.clone();

		// depth image
		msgPtr = boost::shared_ptr<const sensor_msgs::Image>(&(head_positions->head_detections[i].depth_image), voidDeleter);
		try
		{
			cv_cptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::TYPE_32FC3);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		heads_depth_images[i] = cv_cptr->image;

		// head bounding box
		const cob_perception_msgs::Rect& source_rect = head_positions->head_detections[i].head_detection;
		cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
		head_bounding_boxes[i] = rect;
	}
	std::vector < std::vector<cv::Rect> > face_bounding_boxes;
	face_detector_.detectColorFaces(heads_color_images, heads_depth_images, face_bounding_boxes);
	// face_normalizer_.normalizeFaces(heads_color_images, heads_depth_images, face_coordinates);

	// prepare the face position message for publication
	if (face_position_publisher_.getNumSubscribers() > 0)
	{
		cob_perception_msgs::ColorDepthImageArray image_array;
		image_array = *head_positions;
		for (unsigned int i = 0; i < face_bounding_boxes.size(); i++)
		{
			for (unsigned int j = 0; j < face_bounding_boxes[i].size(); j++)
			{
				// face rectangle
				cob_perception_msgs::Rect rect;
				rect.x = face_bounding_boxes[i][j].x;
				rect.y = face_bounding_boxes[i][j].y;
				rect.width = face_bounding_boxes[i][j].width;
				rect.height = face_bounding_boxes[i][j].height;
				image_array.head_detections[i].face_detections.push_back(rect);
			}
			// processed color image
			cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
			cv_ptr->image = heads_color_images[i];
			cv_ptr->toImageMsg(image_array.head_detections[i].color_image);
			image_array.head_detections[i].color_image.header = head_positions->head_detections[i].color_image.header;
		}
		face_position_publisher_.publish(image_array);
	}
	
	// prepare the cartesian face detection message for publication
	if (face_position_publisher_cartesian_.getNumSubscribers() > 0)
	{
		FaceDetectionMessageHelper face_detection_message_helper;
		cob_perception_msgs::DetectionArray detection_msg;
		face_detection_message_helper.prepareCartesionDetectionMessage(detection_msg, head_positions->header, heads_depth_images, head_bounding_boxes, face_bounding_boxes, 0);
		face_position_publisher_cartesian_.publish(detection_msg);
	}

	if (display_timing_ == true)
		ROS_INFO("%d FaceDetection: Time stamp of pointcloud message: %f. Delay: %f.", head_positions->header.seq, head_positions->header.stamp.toSec(),
				ros::Time::now().toSec() - head_positions->header.stamp.toSec());
	//	ROS_INFO("Face detection took %f ms.", tim.getElapsedTimeInMilliSec());
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh("~");

	// Create FaceDetectorNode class instance
	FaceDetectorNode face_detector_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
