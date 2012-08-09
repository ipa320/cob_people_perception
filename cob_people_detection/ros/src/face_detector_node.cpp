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
#else
#endif

// OpenCV
#include "opencv/cv.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

using namespace ipa_PeopleDetector;

FaceDetectorNode::FaceDetectorNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/windows/";

	// Parameters
	double faces_increase_search_scale;		// The factor by which the search window is scaled between the subsequent scans
	int faces_drop_groups;					// Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int faces_min_search_scale_x;				// Minimum search scale x
	int faces_min_search_scale_y;				// Minimum search scale y
	std::cout << "\n--------------------------\nHead Detector Parameters:\n--------------------------\n";
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

	// initialize face detector
	face_detector_.init(data_directory_, faces_increase_search_scale, faces_drop_groups, faces_min_search_scale_x, faces_min_search_scale_y);

	// advertise topics
	face_position_publisher_ = node_handle_.advertise<cob_people_detection_msgs::ColorDepthImageArray>("face_positions", 1);

	// subscribe to head detection topic
	head_position_subscriber_ = nh.subscribe("head_positions", 1, &FaceDetectorNode::head_positions_callback, this);

}

FaceDetectorNode::~FaceDetectorNode(void)
{
}

void FaceDetectorNode::head_positions_callback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& head_positions)
{
	// publish image patches from head region
	cob_people_detection_msgs::ColorDepthImageArray image_array;
	cv_bridge::CvImage cv_ptr;
//	for (unsigned int i=0; i<head_bounding_boxes.size(); i++)
//	{
//		cv::Mat color_patch = color_image(head_bounding_boxes[i]);
//		cv_ptr.image = color_image;
//		cv_ptr.encoding = "bgr8";
//		image_array.color_images.push_back(*(cv_ptr.toImageMsg()));
//		cv::Mat depth_patch = depth_image(head_bounding_boxes[i]);
//		cv_ptr.image = depth_image;
//		cv_ptr.encoding = "bgr8";
//		image_array.depth_images.push_back(*(cv_ptr.toImageMsg()));
//	}
//	head_position_publisher_.publish(image_array);
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceDetectorNode class instance
	FaceDetectorNode face_detector_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
