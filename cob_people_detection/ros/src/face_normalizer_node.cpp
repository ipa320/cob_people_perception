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
	#include "cob_people_detection/face_normalizer_node.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#endif

// OpenCV
#include "opencv/cv.h"
#include "opencv/highgui.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/shared_ptr.hpp>
	#include "cob_people_detection/face_normalizer_node.h"
using namespace ipa_PeopleDetector;

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(const sensor_msgs::Image* const) {}


FaceNormalizerNode::FaceNormalizerNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	//node_handle_.param("debug", debug, false);
	//std::cout << "debug = " << debug << "\n";


	// advertise topics
	norm_face_publisher_ = node_handle_.advertise<cob_people_detection_msgs::DetectionArray>("norm_faces", 1);

	// subscribe to head detection topic
	face_position_subscriber_ = nh.subscribe("face_positions", 1, &FaceNormalizerNode::facePositionsCallback, this);
}

FaceNormalizerNode::~FaceNormalizerNode(void)
{
}


void FaceNormalizerNode::facePositionsCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_positions)
{
// receive head and face positions and recognize faces in the face region, finally publish detected and recognized faces

	// --- convert color image patches of head regions and contained face bounding boxes ---
	cv_bridge::CvImageConstPtr cv_ptr;
	std::vector<cv::Mat> heads_color_images;
	heads_color_images.resize(face_positions->head_detections.size());
	std::vector<cv::Mat> heads_depth_images;
	heads_depth_images.resize(face_positions->head_detections.size());
	std::vector< std::vector<cv::Rect> > face_bounding_boxes;
	face_bounding_boxes.resize(face_positions->head_detections.size());
	std::vector<cv::Rect> head_bounding_boxes;
	head_bounding_boxes.resize(face_positions->head_detections.size());
	for (unsigned int i=0; i<face_positions->head_detections.size(); i++)
	{
		// color image
		{
			sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->head_detections[i].color_image), voidDeleter);
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
		}

		// depth image
		sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->head_detections[i].depth_image), voidDeleter);
		try
		{
			cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::TYPE_32FC3);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		heads_depth_images[i] = cv_ptr->image;

		// face bounding boxes
		face_bounding_boxes[i].resize(face_positions->head_detections[i].face_detections.size());
		for (uint j=0; j<face_bounding_boxes[i].size(); j++)
		{
			const cob_people_detection_msgs::Rect& source_rect = face_positions->head_detections[i].face_detections[j];
			cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
			face_bounding_boxes[i][j] = rect;
		}

		// head bounding box
		const cob_people_detection_msgs::Rect& source_rect = face_positions->head_detections[i].head_detection;
		cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
		head_bounding_boxes[i] = rect;
	}

  for (int i = 0; i <heads_depth_images.size(); i++) {
  //cv::namedWindow("win",CV_WINDOW_AUTOSIZE);
  //std::cout<<" # faces = "<<face_bounding_boxes[i].size()<<std::endl;
  for(int j=0;j<face_bounding_boxes[i].size();j++)
  {
    //cv::imshow("win",heads_color_images[i](face_bounding_boxes[i][j]));
    //cv::waitKey(100);
    int dim=160;
    cv::Mat bgr_crop=heads_color_images[i](face_bounding_boxes[i][j]);
    cv::Mat xyz_crop=heads_depth_images[i](face_bounding_boxes[i][j]);
    cv::Vec2f offset;
     offset[0]=head_bounding_boxes[i].x + face_bounding_boxes[i][j].x;
     offset[1]=head_bounding_boxes[i].y + face_bounding_boxes[i][j].y;
    bool is_norm=face_normalizer_.normalizeFace(bgr_crop,xyz_crop,dim,offset);
  }
 }
//
//	// --- face recognition ---
//	std::vector< std::vector<std::string> > identification_labels;
//	bool identification_failed = false;
//	if (enable_face_recognition_ == true)
//	{
//		// recognize faces
//		unsigned long result_state = face_recognizer_.recognizeFaces(heads_color_images, face_bounding_boxes, identification_labels);
//		if (result_state == ipa_Utils::RET_FAILED)
//		{
//			ROS_ERROR("FaceNormalizerNode::face_positions_callback: Please load a face recognition model at first.");
//			identification_failed = true;
//		}
//	}
//	if (enable_face_recognition_ == false || identification_failed == true)
//	{
//		// label all image unknown if face recognition disabled
//		identification_labels.resize(face_positions->head_detections.size());
//		for (uint i=0; i<identification_labels.size(); i++)
//		{
//			identification_labels[i].resize(face_positions->head_detections[i].face_detections.size());
//			for (uint j=0; j<identification_labels[i].size(); j++)
//				identification_labels[i][j] = "Unknown";
//		}
//	}
//
//	// --- publish detection message ---
//	cob_people_detection_msgs::DetectionArray detection_msg;
//	detection_msg.header = face_positions->header;
//
//	// prepare message
//	for (int head=0; head<(int)head_bounding_boxes.size(); head++)
//	{
//		if (face_bounding_boxes[head].size() == 0)
//		{
//			// no faces detected in head region -> publish head position
//			cob_people_detection_msgs::Detection det;
//			cv::Rect& head_bb = head_bounding_boxes[head];
//			// set 3d position of head's center
//			bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], 0.5*(float)head_bb.width, 0.5*(float)head_bb.height, det.pose.pose.position, 6);
//			if (valid_3d_position==false)
//				continue;
//			// write bounding box
//			det.mask.roi.x = head_bb.x;           det.mask.roi.y = head_bb.y;
//			det.mask.roi.width = head_bb.width;   det.mask.roi.height = head_bb.height;
//			// set label
//			det.label="UnknownHead";
//			// set origin of detection
//			det.detector = "head";
//			// header
//			det.header = face_positions->header;
//			// add to message
//			detection_msg.detections.push_back(det);
//		}
//		else
//		{
//			// process all faces in head region
//			for (int face=0; face<(int)face_bounding_boxes[head].size(); face++)
//			{
//				cob_people_detection_msgs::Detection det;
//				cv::Rect& head_bb = head_bounding_boxes[head];
//				cv::Rect& face_bb = face_bounding_boxes[head][face];
//				// set 3d position of head's center
//				bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], face_bb.x+0.5*(float)face_bb.width, face_bb.y+0.5*(float)face_bb.height, det.pose.pose.position, 6);
//				if (valid_3d_position==false)
//					continue;
//				// write bounding box
//				det.mask.roi.x = head_bb.x+face_bb.x; det.mask.roi.y = head_bb.y+face_bb.y;
//				det.mask.roi.width = face_bb.width;   det.mask.roi.height = face_bb.height;
//				// set label
//				det.label=identification_labels[head][face];
//				// set origin of detection
//				det.detector = "face";
//				// header
//				det.header = face_positions->header;
//				// add to message
//				detection_msg.detections.push_back(det);
//			}
//		}
//	}
//
//	// publish message
//	face_recognition_publisher_.publish(detection_msg);
//}
  }



//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_normalizer");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceNormalizerNode class instance
	FaceNormalizerNode face_recognizer_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
