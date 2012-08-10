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
* functions for recognizing a face within a color image (patch)
* current approach: eigenfaces on color image
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
	#include "cob_people_detection/face_recognizer_node.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#endif

// OpenCV
#include "opencv/cv.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/shared_ptr.hpp>


using namespace ipa_PeopleDetector;

FaceRecognizerNode::FaceRecognizerNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";

	// Parameters
	int eigenface_size;						// Desired width and height of the Eigenfaces (=eigenvectors).
	int eigenvectors_per_person;			// Number of eigenvectors per person to identify -> controls the total number of eigenvectors
	double threshold_facespace;				// Threshold to facespace
	double threshold_unknown;				// Threshold to detect unknown faces
	int metric; 							// metric for nearest neighbor search in face space: 0 = Euklidean, 1 = Mahalanobis, 2 = Mahalanobis Cosine
	bool debug;								// enables some debug outputs
	std::cout << "\n--------------------------\nFace Recognizer Parameters:\n--------------------------\n";
	node_handle_.param("data_directory", data_directory_, data_directory_);
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("eigenface_size", eigenface_size, 100);
	std::cout << "eigenface_size = " << eigenface_size << "\n";
	node_handle_.param("eigenvectors_per_person", eigenvectors_per_person, 1);
	std::cout << "eigenvectors_per_person = " << eigenvectors_per_person << "\n";
	node_handle_.param("threshold_facespace", threshold_facespace, 10000.0);
	std::cout << "threshold_facespace = " << threshold_facespace << "\n";
	node_handle_.param("threshold_unknown", threshold_unknown, 1000.0);
	std::cout << "threshold_unknown = " << threshold_unknown << "\n";
	node_handle_.param("metric", metric, 0);
	std::cout << "metric = " << metric << "\n";
	node_handle_.param("debug", debug, false);
	std::cout << "debug = " << debug << "\n";

	// initialize face recognizer
	face_recognizer_.init(data_directory_, eigenface_size, eigenvectors_per_person, threshold_facespace, threshold_unknown, metric, debug);

	// advertise topics
	// todo
	face_recognition_publisher_ = node_handle_.advertise<cob_people_detection_msgs::ColorDepthImageArray>("face_recognitions", 1);

	// subscribe to head detection topic
	face_position_subscriber_ = nh.subscribe("face_positions", 1, &FaceRecognizerNode::face_positions_callback, this);
}

FaceRecognizerNode::~FaceRecognizerNode(void)
{
}

void FaceRecognizerNode::face_positions_callback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_positions)
{
	// receive head and face positions and recognize faces in the face region, finally publish detected and recognized faces

	// convert color image patches of head regions and contained face bounding boxes
	cv_bridge::CvImageConstPtr cv_ptr;
	std::vector<cv::Mat> heads_color_images;
	heads_color_images.resize(face_positions->head_detections.size());
	std::vector< std::vector<cv::Rect> > face_bounding_boxes;
	face_bounding_boxes.resize(face_positions->head_detections.size());
	for (unsigned int i=0; i<face_positions->head_detections.size(); i++)
	{
		// color image
		sensor_msgs::Image msg = face_positions->head_detections[i].color_image;
		sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image>(&msg);
		try
		{
			cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		heads_color_images[i] = cv_ptr->image;

		// face bounding box
		face_bounding_boxes.resize(face_positions->head_detections[i].face_detections.size());
		for (uint j=0; j<face_bounding_boxes.size(); j++)
		{
			const cob_people_detection_msgs::Rect& source_rect = face_positions->head_detections[i].face_detections[j];
			cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
			face_bounding_boxes[i][j] = rect;
		}
	}

	// recognize faces
	std::vector< std::vector<std::string> > identification_labels;
	face_recognizer_.recognizeFaces(heads_color_images, face_bounding_boxes, identification_labels);

	// publish detection message


//	std::vector<std::vector<cv::Rect> > face_coordinates;
//	face_detector_.detectColorFaces(heads_color_images, heads_depth_images, face_coordinates);
//
//	cob_people_detection_msgs::ColorDepthImageArray image_array;
//	image_array = *face_positions;
//	for (unsigned int i=0; i<face_coordinates.size(); i++)
//	{
//		for (unsigned int j=0; j<face_coordinates[i].size(); j++)
//		{
//			cob_people_detection_msgs::Rect rect;
//			rect.x = face_coordinates[i][j].x;
//			rect.y = face_coordinates[i][j].y;
//			rect.width = face_coordinates[i][j].width;
//			rect.height = face_coordinates[i][j].height;
//			image_array.head_detections[i].face_detections.push_back(rect);
//		}
//	}
//
//	face_position_publisher_.publish(image_array);
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_recognizer");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceRecognizerNode class instance
	FaceRecognizerNode face_recognizer_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
