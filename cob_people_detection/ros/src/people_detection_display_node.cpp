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

#include <cob_people_detection/people_detection_display_node.h>

using namespace ipa_PeopleDetector;

inline bool isnan_(double num)
{
	return (num != num);
}
;

PeopleDetectionDisplayNode::PeopleDetectionDisplayNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	it_ = 0;
	sync_input_3_ = 0;

	// parameters
	std::cout << "\n---------------------------\nPeople Detection Display Parameters:\n---------------------------\n";
	node_handle_.param("display", display_, false);
	std::cout << "display = " << display_ << "\n";
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << "\n";

	// subscribers
	//	people_segmentation_image_sub_.subscribe(*it_, "people_segmentation_image", 1);
	it_ = new image_transport::ImageTransport(node_handle_);
	colorimage_sub_.subscribe(*it_, "colorimage_in", 1);
	face_recognition_subscriber_.subscribe(node_handle_, "face_position_array", 1);
	face_detection_subscriber_.subscribe(node_handle_, "face_detections", 1);
	//	pointcloud_sub_.subscribe(node_handle_, "pointcloud_in", 1);

	// input synchronization
	//	sync_input_3_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray, cob_perception_msgs::ColorDepthImageArray, sensor_msgs::PointCloud2> >(30);
	//	sync_input_3_->connectInput(face_recognition_subscriber_, face_detection_subscriber_, pointcloud_sub_);
	sync_input_3_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray,
			cob_perception_msgs::ColorDepthImageArray, sensor_msgs::Image> >(60);
	sync_input_3_->connectInput(face_recognition_subscriber_, face_detection_subscriber_, colorimage_sub_);
	sync_input_3_->registerCallback(boost::bind(&PeopleDetectionDisplayNode::inputCallback, this, _1, _2, _3));

	// publishers
	people_detection_image_pub_ = it_->advertise("face_position_image", 1);

	std::cout << "PeopleDetectionDisplay initialized." << std::endl;
}

PeopleDetectionDisplayNode::~PeopleDetectionDisplayNode()
{
	if (it_ != 0)
		delete it_;
	if (sync_input_3_ != 0)
		delete sync_input_3_;
}

/// Converts a color image message to cv::Mat format.
unsigned long PeopleDetectionDisplayNode::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return ipa_Utils::RET_FAILED;
	}
	image = image_ptr->image;

	return ipa_Utils::RET_OK;
}

/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
//void PeopleDetectionDisplayNode::inputCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_recognition_msg, const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::PointCloud2::ConstPtr& pointcloud_msg)
void PeopleDetectionDisplayNode::inputCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_recognition_msg,
		const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	// convert color image to cv::Mat
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

	//	// get color image from point cloud
	//	pcl::PointCloud<pcl::PointXYZRGB> point_cloud_src;
	//	pcl::fromROSMsg(*pointcloud_msg, point_cloud_src);
	//
	//	cv::Mat color_image = cv::Mat::zeros(point_cloud_src.height, point_cloud_src.width, CV_8UC3);
	//	for (unsigned int v=0; v<point_cloud_src.height; v++)
	//	{
	//		for (unsigned int u=0; u<point_cloud_src.width; u++)
	//		{
	//			pcl::PointXYZRGB point = point_cloud_src(u,v);
	//			if (isnan_(point.z) == false)
	//				color_image.at<cv::Point3_<unsigned char> >(v,u) = cv::Point3_<unsigned char>(point.b, point.g, point.r);
	//		}
	//	}

	// insert all detected heads and faces
	for (int i = 0; i < (int)face_detection_msg->head_detections.size(); i++)
	{
		// paint head
		const cob_perception_msgs::Rect& head_rect = face_detection_msg->head_detections[i].head_detection;
		cv::Rect head(head_rect.x, head_rect.y, head_rect.width, head_rect.height);
		cv::rectangle(color_image, cv::Point(head.x, head.y), cv::Point(head.x + head.width, head.y + head.height), CV_RGB(148, 219, 255), 2, 8, 0);

		// paint faces
		for (int j = 0; j < (int)face_detection_msg->head_detections[i].face_detections.size(); j++)
		{
			const cob_perception_msgs::Rect& face_rect = face_detection_msg->head_detections[i].face_detections[j];
			cv::Rect face(face_rect.x + head.x, face_rect.y + head.y, face_rect.width, face_rect.height);
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(191, 255, 148), 2, 8, 0);
		}
	}

	// insert recognized faces
	for (int i = 0; i < (int)face_recognition_msg->detections.size(); i++)
	{
		const cob_perception_msgs::Rect& face_rect = face_recognition_msg->detections[i].mask.roi;
		cv::Rect face(face_rect.x, face_rect.y, face_rect.width, face_rect.height);

		if (face_recognition_msg->detections[i].detector == "head")
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
		else
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

		if (face_recognition_msg->detections[i].label == "Unknown")
			// Distance to face class is too high
			cv::putText(color_image, "Unknown", cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 0), 2);
		else if (face_recognition_msg->detections[i].label == "No face")
			// Distance to face space is too high
			cv::putText(color_image, "No face", cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(255, 0, 0), 2);
		else
			// Face classified
			cv::putText(color_image, face_recognition_msg->detections[i].label.c_str(), cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB(0, 255, 0),
					2);
	}

	// display image
	if (display_ == true)
	{
		cv::imshow("Detections and Recognitions", color_image);
		cv::waitKey(10);
	}

	// publish image
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = color_image;
	cv_ptr.encoding = "bgr8";
	people_detection_image_pub_.publish(cv_ptr.toImageMsg());

	if (display_timing_ == true)
		ROS_INFO("%d Display: Time stamp of image message: %f. Delay: %f.", color_image_msg->header.seq, color_image_msg->header.stamp.toSec(),
				ros::Time::now().toSec() - color_image_msg->header.stamp.toSec());
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "people_detection_display");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh("~");

	// Create FaceRecognizerNode class instance
	PeopleDetectionDisplayNode people_detection_display_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
