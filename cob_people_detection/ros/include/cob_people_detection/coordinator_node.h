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
 * coordinates a detection pipeline
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

#ifndef _COORDINATOR_NODE_H_
#define _COORDINATOR_NODE_H_

// standard includes
#include <sstream>
#include <string>
#include <vector>

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <cob_perception_msgs/DetectionArray.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/getDetectionsAction.h>

// services
#include <std_srvs/Empty.h>
#include <cob_people_detection/recognitionTrigger.h>

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// external includes
#include "cob_vision_utils/GlobalDefines.h"

#include "cob_people_detection/face_recognizer.h"

typedef actionlib::SimpleActionServer<cob_people_detection::getDetectionsAction> GetDetectionsServer;

class CoordinatorNode
{
protected:

	ros::NodeHandle node_handle_; ///< ROS node handle

	boost::mutex active_action_mutex_; ///< mutex to avoid double call of an action
	boost::mutex last_detection_mutex_; ///< mutex to guard access to last_detection_message_
	cob_perception_msgs::DetectionArray last_detection_message_; ///> buffer for the last received detection

	bool sensor_message_gateway_open_; ///< tracks whether the gateway was opened or not

	// subscriber
	ros::Subscriber detection_sub_; ///< recieves messages of detections

	// actions
	GetDetectionsServer* get_detections_server_; ///< Action server that handles requests for people detections

	// services
	ros::ServiceServer service_server_start_recognition_; ///< Service server that starts the recognition pipeline
	ros::ServiceServer service_server_stop_recognition_; ///< Service server that stops the recognition pipeline

	// parameters
	std::string namespace_gateway_; ///< namespace of the pipeline's sensor message gateway

	void detectionsCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detection_array);

	void getDetectionsServerCallback(const cob_people_detection::getDetectionsGoalConstPtr& goal);

	bool startRecognitionCallback(cob_people_detection::recognitionTrigger::Request &req, cob_people_detection::recognitionTrigger::Response &res);

	bool stopRecognitionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

public:

	CoordinatorNode(ros::NodeHandle nh);
	~CoordinatorNode();
};

#endif // _COORDINATOR_NODE_H_
