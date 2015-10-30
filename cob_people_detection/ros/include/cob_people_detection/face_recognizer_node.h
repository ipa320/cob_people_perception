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

#ifndef __FACE_RECOGNIZER_NODE_H__
#define __FACE_RECOGNIZER_NODE_H__

#ifdef __LINUX__
#include "cob_people_detection/face_recognizer.h"
#else
#endif

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>		// use as: directory_ = ros::package::getPath("cob_people_detection") + "/common/files/windows/";
// ROS message includes
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>

// Actions
#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/loadModelAction.h>

//boost includes

#include<boost/filesystem.hpp>

namespace ipa_PeopleDetector
{

typedef actionlib::SimpleActionServer<cob_people_detection::loadModelAction> LoadModelServer;

class FaceRecognizerNode
{
public:

	/// Constructor
	/// @param nh ROS node handle
	FaceRecognizerNode(ros::NodeHandle nh);
	~FaceRecognizerNode(void); ///< Destructor


protected:

	/// Callback for incoming head detections
	void facePositionsCallback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_positions);
	//void facePositionsCallback(const cob_perception_msgs::ColorDepthImageCropArray::ConstPtr& face_positions);

	/// Computes the 3D coordinate of a detected face.
	/// @param depth_image Coordinate image in format CV32FC3
	/// @param center2Dx Image x-coordinate of the center of the detected face
	/// @param center2Dy Image y-coordinate of the center of the detected face
	/// @param center3D (x,y,z) coordinates of the face's center point
	/// @param search_radius Radius of pixel neighborhood which is searched for valid 3D coordinates.
	/// @return Indicates whether the found 3D coordinates are valid, i.e. if true, the 3D coordinates do not contain NaN values and are valid.
	bool determine3DFaceCoordinates(cv::Mat& depth_image, int center2Dx, int center2Dy, geometry_msgs::Point& center3D, int search_radius);

	/// Callback for load requests to load a new recognition model
	void loadModelServerCallback(const cob_people_detection::loadModelGoalConstPtr& goal);

	ros::NodeHandle node_handle_;

	ros::Subscriber face_position_subscriber_; ///< subscribes to the positions of detected face regions

	ros::Publisher face_recognition_publisher_; ///< publisher for the positions and labels of the detected faces

	LoadModelServer* load_model_server_; ///< Action server that handles load requests for a new recognition model

	FaceRecognizer face_recognizer_; ///< implementation of the face recognizer


	// parameters
	std::string data_directory_; ///< path to the classifier model
	std::string classifier_directory_; ///< path to the face feature haarcascades
	bool enable_face_recognition_; ///< this flag enables or disables the face recognition step
	bool display_timing_;

};

} // end namespace

#endif // __FACE_RECOGNIZER_NODE_H__
