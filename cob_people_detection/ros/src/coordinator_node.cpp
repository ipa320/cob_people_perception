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

#include <cob_people_detection/coordinator_node.h>

CoordinatorNode::CoordinatorNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	sensor_message_gateway_open_ = false;
	last_detection_message_.header.stamp = ros::Time::now();

	// parameters
	namespace_gateway_ = "";
	std::cout << "\n---------------------------\nCoordinator Node Parameters:\n---------------------------\n";
	node_handle_.param("namespace_gateway", namespace_gateway_, namespace_gateway_);
	std::cout << "namespace_gateway = " << namespace_gateway_ << "\n";

	detection_sub_ = node_handle_.subscribe("detection_array", 1, &CoordinatorNode::detectionsCallback, this);

	// actions
	get_detections_server_ = new GetDetectionsServer(node_handle_, "get_detections_server", boost::bind(&CoordinatorNode::getDetectionsServerCallback, this, _1), false);
	get_detections_server_->start();

	// activate service servers for gateway control
	service_server_start_recognition_ = node_handle_.advertiseService("start_recognition", &CoordinatorNode::startRecognitionCallback, this);
	service_server_stop_recognition_ = node_handle_.advertiseService("stop_recognition", &CoordinatorNode::stopRecognitionCallback, this);

	std::cout << "CoordinatorNode initialized." << std::endl;
}

CoordinatorNode::~CoordinatorNode()
{
	if (get_detections_server_ != 0)
		delete get_detections_server_;
}

void CoordinatorNode::detectionsCallback(const cob_perception_msgs::DetectionArray::ConstPtr& detection_array)
{
	// secure this function with a mutex
	boost::lock_guard < boost::mutex > lock(last_detection_mutex_);

	last_detection_message_ = *detection_array;
}

void CoordinatorNode::getDetectionsServerCallback(const cob_people_detection::getDetectionsGoalConstPtr& goal)
{
	// secure this function with a mutex
	boost::lock_guard < boost::mutex > lock(active_action_mutex_);

	cob_people_detection::getDetectionsResult result;
	result.detections.detections.clear();
	if (sensor_message_gateway_open_ == false)
	{
		// open gateway
		std::string cmd = "rosrun dynamic_reconfigure dynparam set " + namespace_gateway_ + " target_publishing_rate 5.0";
		int retval = system(cmd.c_str());
	}

	bool message_received = false;
	bool collect_messages = true;
	ros::Duration maximal_message_age(goal->maximum_message_age);
	ros::Duration timeout(goal->timeout);
	ros::Time start_time = ros::Time::now();
	while (collect_messages == true && (goal->timeout == 0 || (ros::Time::now() - start_time) < timeout))
	{
		// secure the access to last_detection_message_ with a mutex
		boost::lock_guard < boost::mutex > lock(last_detection_mutex_);

		std::cout << "timeout " << (ros::Time::now() - start_time).toSec() << "      maximal_age " << (ros::Time::now() - last_detection_message_.header.stamp).toSec()
				<< std::endl;
		if ((ros::Time::now() - last_detection_message_.header.stamp) < maximal_message_age || goal->maximum_message_age == 0)
		{
			// take the first message for the result or any other message with at least so many detections as before
			if (message_received == false || (result.detections.detections.size() <= last_detection_message_.detections.size()))
				result.detections = last_detection_message_;
			message_received = true;
			// answer directly when in continous mode
			if (sensor_message_gateway_open_ == true)
				collect_messages = false;
		}
		ros::spinOnce();
	}

	if (sensor_message_gateway_open_ == false)
	{
		// close gateway
		std::string cmd = "rosrun dynamic_reconfigure dynparam set " + namespace_gateway_ + " target_publishing_rate 0.0";
		int retval = system(cmd.c_str());
	}

	if (message_received == true)
		get_detections_server_->setSucceeded(result, "Found a valid detection message.");
	else
		get_detections_server_->setAborted(result, "No valid detection message available.");
}

bool CoordinatorNode::startRecognitionCallback(cob_people_detection::recognitionTrigger::Request &req, cob_people_detection::recognitionTrigger::Response &res)
{
	// secure this function with a mutex
	boost::lock_guard < boost::mutex > lock(active_action_mutex_);

	// set target frame rate
	std::stringstream ss;
	ss << "rosrun dynamic_reconfigure dynparam set " << namespace_gateway_ << " target_publishing_rate " << req.target_frame_rate;
	int retval = system(ss.str().c_str());

	sensor_message_gateway_open_ = true;

	return true;
}

bool CoordinatorNode::stopRecognitionCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	// secure this function with a mutex
	boost::lock_guard < boost::mutex > lock(active_action_mutex_);

	// set target frame rate
	std::string cmd = "rosrun dynamic_reconfigure dynparam set " + namespace_gateway_ + " target_publishing_rate 0.0";
	int retval = system(cmd.c_str());

	sensor_message_gateway_open_ = false;

	return true;
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "coordinator_node");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh("~");

	// Create FaceRecognizerNode class instance
	CoordinatorNode coordinator_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
