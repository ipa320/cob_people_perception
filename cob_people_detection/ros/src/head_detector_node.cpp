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
 * functions for detecting a head within a point cloud/depth image
 * current approach: haar detector on depth image
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
#include "cob_people_detection/head_detector_node.h"
#include "cob_vision_utils/GlobalDefines.h"
#else
#endif

// OpenCV
#include "opencv/cv.h"
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// timer
#include <cob_people_detection/timer.h>

using namespace ipa_PeopleDetector;

HeadDetectorNode::HeadDetectorNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";

	// Parameters
	double depth_increase_search_scale; // The factor by which the search window is scaled between the subsequent scans
	int depth_drop_groups; // Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int depth_min_search_scale_x; // Minimum search scale x
	int depth_min_search_scale_y; // Minimum search scale y
	std::cout << "\n--------------------------\nHead Detector Parameters:\n--------------------------\n";
	node_handle_.param("data_directory", data_directory_, data_directory_);
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("fill_unassigned_depth_values", fill_unassigned_depth_values_, true);
	std::cout << "fill_unassigned_depth_values = " << fill_unassigned_depth_values_ << "\n";
	node_handle_.param("depth_increase_search_scale", depth_increase_search_scale, 1.1);
	std::cout << "depth_increase_search_scale = " << depth_increase_search_scale << "\n";
	node_handle_.param("depth_drop_groups", depth_drop_groups, 68);
	std::cout << "depth_drop_groups = " << depth_drop_groups << "\n";
	node_handle_.param("depth_min_search_scale_x", depth_min_search_scale_x, 20);
	std::cout << "depth_min_search_scale_x = " << depth_min_search_scale_x << "\n";
	node_handle_.param("depth_min_search_scale_y", depth_min_search_scale_y, 20);
	std::cout << "depth_min_search_scale_y = " << depth_min_search_scale_y << "\n";
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << "\n";

	// initialize head detector
	head_detector_.init(data_directory_, depth_increase_search_scale, depth_drop_groups, depth_min_search_scale_x, depth_min_search_scale_y);

	// advertise topics
	head_position_publisher_ = node_handle_.advertise<cob_perception_msgs::ColorDepthImageArray>("head_positions", 1);

	// subscribe to sensor topic
	pointcloud_sub_ = nh.subscribe("pointcloud_rgb", 1, &HeadDetectorNode::pointcloud_callback, this);

	std::cout << "HeadDetectorNode initialized." << std::endl;
}

HeadDetectorNode::~HeadDetectorNode(void)
{
}

void HeadDetectorNode::pointcloud_callback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{

	//	Timer tim;
	//	tim.start();

	// convert incoming colored point cloud to cv::Mat images
	cv::Mat depth_image;
	cv::Mat color_image;
	convertPclMessageToMat(pointcloud, depth_image, color_image);

//		cv::Mat gray_depth(depth_image.rows, depth_image.cols, CV_32FC1);
//		for (int v=0; v<depth_image.rows; ++v)
//			for (int u=0; u<depth_image.cols; ++u)
//				gray_depth.at<float>(v,u) = depth_image.at<cv::Vec3f>(v,u).val[2];
//		cv::normalize(gray_depth, gray_depth, 0.f, 1.f, cv::NORM_MINMAX);
//		cv::imshow("depth image", gray_depth);
//		char key = cv::waitKey(1000);
//		if (key == 's')
//		{
//			cv::normalize(gray_depth, gray_depth, 0.f, 255.f, cv::NORM_MINMAX);
//			cv::imwrite("depth_image.png", gray_depth);
//		}

	// detect heads in the depth image
	std::vector<cv::Rect> head_bounding_boxes;
	head_detector_.detectRangeFace(depth_image, head_bounding_boxes, fill_unassigned_depth_values_);

	// publish image patches from head region
	cob_perception_msgs::ColorDepthImageArray image_array;
	image_array.header = pointcloud->header;
	image_array.head_detections.resize(head_bounding_boxes.size());
	for (unsigned int i = 0; i < head_bounding_boxes.size(); i++)
	{
		cv_bridge::CvImage cv_ptr;
		image_array.head_detections[i].head_detection.x = head_bounding_boxes[i].x;
		image_array.head_detections[i].head_detection.y = head_bounding_boxes[i].y;
		image_array.head_detections[i].head_detection.width = head_bounding_boxes[i].width;
		image_array.head_detections[i].head_detection.height = head_bounding_boxes[i].height;
		cv::Mat depth_patch = depth_image(head_bounding_boxes[i]);
		cv_ptr.image = depth_patch;
		cv_ptr.encoding = sensor_msgs::image_encodings::TYPE_32FC3; // CV32FC3
		image_array.head_detections[i].depth_image = *(cv_ptr.toImageMsg());
		image_array.head_detections[i].depth_image.header = pointcloud->header;
		cv::Mat color_patch = color_image(head_bounding_boxes[i]);
		cv_ptr.image = color_patch;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		image_array.head_detections[i].color_image = *(cv_ptr.toImageMsg());
		image_array.head_detections[i].color_image.header = pointcloud->header;
	}
	head_position_publisher_.publish(image_array);

	if (display_timing_ == true)
		ROS_INFO("%d HeadDetection: Time stamp of pointcloud message: %f. Delay: %f.", pointcloud->header.seq, pointcloud->header.stamp.toSec(),
				ros::Time::now().toSec() - pointcloud->header.stamp.toSec());
	//	ROS_INFO("Head Detection took %f ms.", tim.getElapsedTimeInMilliSec());
}

unsigned long HeadDetectorNode::convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& pointcloud, cv::Mat& depth_image, cv::Mat& color_image)
{
	pcl::PointCloud < pcl::PointXYZRGB > depth_cloud; // point cloud
	pcl::fromROSMsg(*pointcloud, depth_cloud);
	depth_image.create(depth_cloud.height, depth_cloud.width, CV_32FC3);
	color_image.create(depth_cloud.height, depth_cloud.width, CV_8UC3);
	uchar* depth_image_ptr = (uchar*)depth_image.data;
	uchar* color_image_ptr = (uchar*)color_image.data;
	for (int v = 0; v < (int)depth_cloud.height; v++)
	{
		int depth_base_index = depth_image.step * v;
		int color_base_index = color_image.step * v;
		for (int u = 0; u < (int)depth_cloud.width; u++)
		{
			int depth_index = depth_base_index + 3 * u * sizeof(float);
			float* depth_data_ptr = (float*)(depth_image_ptr + depth_index);
			int color_index = color_base_index + 3 * u * sizeof(uchar);
			uchar* color_data_ptr = (uchar*)(color_image_ptr + color_index);
			pcl::PointXYZRGB point_xyz = depth_cloud(u, v);
			depth_data_ptr[0] = point_xyz.x;
			depth_data_ptr[1] = point_xyz.y;
			depth_data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
			color_data_ptr[0] = point_xyz.r;
			color_data_ptr[1] = point_xyz.g;
			color_data_ptr[2] = point_xyz.b;
			//if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
		}
	}
	return ipa_Utils::RET_OK;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "head_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh("~");

	// Create HeadDetectorNode class instance
	HeadDetectorNode head_detector_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
