/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_people_detection
 * Description:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Richard Bormann, email:richard.bormann@ipa.fhg.de
 * Supervised by:
 *
 * Date of creation: 03/2011
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

//##################
//#### includes ####

// ROS includes
#include <ros/ros.h>

// Actions
#include <actionlib/client/simple_action_client.h>
#include <cob_people_detection/addDataAction.h>
#include <cob_people_detection/updateDataAction.h>
#include <cob_people_detection/deleteDataAction.h>
#include <cob_people_detection/loadModelAction.h>
#include <cob_people_detection/getDetectionsAction.h>

// services
#include <cob_people_detection/captureImage.h>
#include <cob_people_detection/finishRecording.h>
#include <cob_people_detection/recognitionTrigger.h>
#include <std_srvs/Empty.h>

// standard includes
#ifdef __LINUX__
#include <termios.h>

int getch();

int getch()
{
	static int ch = -1, fd = 0;
	struct termios neu, alt;
	fd = fileno(stdin);
	tcgetattr(fd, &alt);
	neu = alt;
	neu.c_lflag &= ~(ICANON|ECHO);
	tcsetattr(fd, TCSANOW, &neu);
	ch = getchar();
	tcsetattr(fd, TCSANOW, &alt);
	return ch;
}
#endif

//####################
//#### node class ####

typedef actionlib::SimpleActionClient<cob_people_detection::addDataAction> AddDataClient;
typedef actionlib::SimpleActionClient<cob_people_detection::updateDataAction> UpdateDataClient;
typedef actionlib::SimpleActionClient<cob_people_detection::deleteDataAction> DeleteDataClient;
typedef actionlib::SimpleActionClient<cob_people_detection::loadModelAction> LoadModelClient;
typedef actionlib::SimpleActionClient<cob_people_detection::getDetectionsAction> GetDetectionsClient;

void addData(AddDataClient& add_data_client, ros::ServiceClient& capture_image_client, ros::ServiceClient& finish_recording_client)
{
	cob_people_detection::addDataGoal goal;

	std::cout << "Input the label of the captured person: ";
	std::cin >> goal.label;

	std::cout << "Mode of data capture: 0=manual, 1=continuous: ";
	std::cin >> goal.capture_mode;

	if (goal.capture_mode == 1)
	{
		std::cout << "How many images shall be captured automatically? ";
		std::cin >> goal.continuous_mode_images_to_capture;

		std::cout << "What is the desired delay time in seconds between two recordings? ";
		std::cin >> goal.continuous_mode_delay;
	}

	// send goal to server
	add_data_client.sendGoal(goal);
	std::cout << "Recording job was sent to the server ..." << std::endl;

	// enable control in manual mode
	if (goal.capture_mode == 0)
	{
		// wait for server to provide the capture and finish service
		std::cout << "Waiting for the capture service to become available ..." << std::endl;
		capture_image_client.waitForExistence();
		finish_recording_client.waitForExistence();

		// capture
		std::cout << "Hit 'q' key to quit or 'c' key to capture an image.\n";
		cob_people_detection::captureImageRequest capture_image_request;
		cob_people_detection::captureImageResponse capture_image_response;
		char key;
		while ((key = getch()) != 'q')
		{
			if (key == 'c')
			{
				printf("Image capture initiated ... \n");
				if (capture_image_client.call(capture_image_request, capture_image_response) == true)
					printf("   image %d successfully captured.\n", capture_image_response.number_captured_images);
				else
					printf("   image capture not successful.\n");
			}
		}

		printf("Finishing recording ...\n");

		// tell server to finish recording
		cob_people_detection::finishRecording finish_recording;
		finish_recording_client.call(finish_recording);
	}

	add_data_client.waitForResult();

	if (add_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Data recording finished successfully.\n");
	else
		printf("Data recording did not finish successfully.\n");

	printf("Current State: %s   Message: %s\n", add_data_client.getState().toString().c_str(), add_data_client.getState().getText().c_str());
}

void updateData(UpdateDataClient& update_data_client)
{
	cob_people_detection::updateDataGoal goal;

	std::cout << "Choose a mode for data update: 1=by index (changes exactly one entry), 2=by label (changes all entries with that label): ";
	std::cin >> goal.update_mode;

	if (goal.update_mode == 1)
	{
		std::cout << "Enter the index of the database entry that shall be updated: ";
		std::cin >> goal.update_index;
	}
	else if (goal.update_mode == 2)
	{
		std::cout << "Enter the label of the database entries that shall be updated: ";
		std::cin >> goal.old_label;
	}
	else
	{
		std::cout << "Error: updateData: Wrong update_mode." << std::endl;
		return;
	}

	std::cout << "Enter the new label that shall be assigned to the chosen data: ";
	std::cin >> goal.new_label;

	// send goal to server
	update_data_client.sendGoal(goal);
	std::cout << "Update job was sent to the server ..." << std::endl;
	update_data_client.waitForResult();

	if (update_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Data update finished successfully.\n");
	else
		printf("Data update did not finish successfully.\n");

	printf("Current State: %s   Message: %s\n", update_data_client.getState().toString().c_str(), update_data_client.getState().getText().c_str());
}

void deleteData(DeleteDataClient& delete_data_client)
{
	cob_people_detection::deleteDataGoal goal;

	std::cout << "Choose a mode for data deletion: 1=by index (deletes exactly one entry), 2=by label (deletes all entries with that label): ";
	std::cin >> goal.delete_mode;

	if (goal.delete_mode == 1)
	{
		std::cout << "Enter the index of the database entry that shall be deleted: ";
		std::cin >> goal.delete_index;
	}
	else if (goal.delete_mode == 2)
	{
		std::cout << "Enter the label of the database entries that shall be deleted: ";
		std::cin >> goal.label;
	}
	else
	{
		std::cout << "Error: deleteData: Wrong delete_mode." << std::endl;
		return;
	}

	// send goal to server
	delete_data_client.sendGoal(goal);
	std::cout << "Delete job was sent to the server ..." << std::endl;
	delete_data_client.waitForResult();

	if (delete_data_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Data deletion finished successfully.\n");
	else
		printf("Data deletion did not finish successfully.\n");

	printf("Current State: %s   Message: %s\n", delete_data_client.getState().toString().c_str(), delete_data_client.getState().getText().c_str());
}

void loadRecognitionModel(LoadModelClient& load_model_client)
{
	cob_people_detection::loadModelGoal goal;

	std::cout << "Enter the labels that should occur in the recognition model. By sending an empty list, all available data will be used." << std::endl;
	while (true)
	{
		std::cout << "Enter label (finish by entering 'q'): ";
		std::string label;
		std::cin >> label;

		if (label.compare("q") == 0)
			break;
		else
			goal.labels.push_back(label);
	}

	// send goal to server
	load_model_client.sendGoal(goal);
	std::cout << "Recognition model is loaded by the server ..." << std::endl;
	std::cout << "\nA new recognition model is currently loaded or generated by the server. The following labels will be covered: " << std::endl;
	for (int i = 0; i < (int)goal.labels.size(); i++)
		std::cout << "   - " << goal.labels[i] << std::endl;

	load_model_client.waitForResult();

	if (load_model_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("The new recognition model has been successfully loaded.\n");
	else
		printf("Loading a new recognition model was not successful.\n");

	printf("Current State: %s   Message: %s\n", load_model_client.getState().toString().c_str(), load_model_client.getState().getText().c_str());
}

void activateSensorMessageGateway(ros::ServiceClient& sensor_message_gateway_open_client, ros::ServiceClient& sensor_message_gateway_close_client)
{
	int open_close = 0;
	std::cout << "Type 1 to activate or 2 to deactivate the sensor message gateway: ";
	std::cin >> open_close;

	if (open_close == 1)
	{
		// activate
		cob_people_detection::recognitionTriggerRequest req;
		cob_people_detection::recognitionTriggerResponse res;
		std::cout << "At which target frame rate (Hz) shall the sensor message gateway operate: ";
		std::cin >> req.target_frame_rate;

		if (sensor_message_gateway_open_client.call(req, res) == true)
			printf("Gateway successfully opened.\n");
		else
			printf("Opening gateway was not successful.\n");
	}
	else if (open_close == 2)
	{
		// deactivate
		std_srvs::Empty rec;
		if (sensor_message_gateway_close_client.call(rec) == true)
			printf("Gateway successfully closed.\n");
		else
			printf("Closing gateway was not successful.\n");
	}
}

void getDetections(GetDetectionsClient& get_detections_client)
{
	cob_people_detection::getDetectionsGoal goal;

	std::cout << "Enter a maximum age of the detection message (in seconds): ";
	std::cin >> goal.maximum_message_age;

	std::cout << "Enter the maximum waiting time to receive the message (in seconds): ";
	std::cin >> goal.timeout;

	// send goal to server
	get_detections_client.sendGoal(goal);
	std::cout << "Waiting for the server to send the detections ..." << std::endl;
	get_detections_client.waitForResult();

	if (get_detections_client.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		cob_people_detection::getDetectionsResultConstPtr result = get_detections_client.getResult();
		std::cout << "Received a detection message with " << result->detections.detections.size() << " detections.\nThe labels are" << std::endl;
		for (int i = 0; i < (int)result->detections.detections.size(); i++)
			std::cout << "   - " << result->detections.detections[i].label << std::endl;
	}
	else
		std::cout << "No detections received.\n";

	printf("Current State: %s   Message: %s\n", get_detections_client.getState().toString().c_str(), get_detections_client.getState().getText().c_str());
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cob_people_detection_client");

	ros::NodeHandle nh;

	AddDataClient add_data_client("/face_capture/add_data_server", true); // true -> don't need ros::spin()
	ros::ServiceClient capture_image_client = nh.serviceClient<cob_people_detection::captureImage>("/face_capture/capture_image");
	ros::ServiceClient finish_recording_client = nh.serviceClient<cob_people_detection::finishRecording>("/face_capture/finish_recording");
	UpdateDataClient update_data_client("/face_capture/update_data_server", true);
	DeleteDataClient delete_data_client("/face_capture/delete_data_server", true);
	LoadModelClient load_model_client("/face_recognizer/load_model_server", true);
	GetDetectionsClient get_detections_client("/coordinator/get_detections_server", true);
	ros::ServiceClient sensor_message_gateway_open_client = nh.serviceClient<cob_people_detection::recognitionTrigger>("/coordinator/start_recognition");
	ros::ServiceClient sensor_message_gateway_close_client = nh.serviceClient<std_srvs::Empty>("/coordinator/stop_recognition");

	if (!add_data_client.waitForServer(ros::Duration(2.0)))
	{
		std::cout << "No connection to server 'add_data_server'.\n";
		return 0;
	}
	if (!update_data_client.waitForServer(ros::Duration(2.0)))
	{
		std::cout << "No connection to server 'update_data_server'.\n";
		return 0;
	}
	if (!delete_data_client.waitForServer(ros::Duration(2.0)))
	{
		std::cout << "No connection to server 'delete_data_server'.\n";
		return 0;
	}
	if (!load_model_client.waitForServer(ros::Duration(2.0)))
	{
		std::cout << "No connection to server 'load_model_server'.\n";
		return 0;
	}
	if (!get_detections_client.waitForServer(ros::Duration(2.0)))
	{
		std::cout << "No connection to server 'get_detections_server'.\n";
		return 0;
	}

	std::cout << "Connected to servers.\n";

	char key = 'q';
	do
	{
		std::cout
				<< "\n\nChoose an option:\n1 - capture face images\n2 - update database labels\n3 - delete database entries\n4 - load recognition model (necessary if new images/persons were added to the database)\n5 - activate/deactivate sensor message gateway\n6 - get detections\nq - Quit\n\n";
		key = getch();
		if (key == '1')
			addData(add_data_client, capture_image_client, finish_recording_client);//train(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '2')
			updateData(update_data_client);//recognize(recognizeClient);
		else if (key == '3')
			deleteData(delete_data_client);//train_continuous(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '4')
			loadRecognitionModel(load_model_client);//show(showClient, 0);
		else if (key == '5')
			activateSensorMessageGateway(sensor_message_gateway_open_client, sensor_message_gateway_close_client);//show(showClient, 1);
		else if (key == '6')
			getDetections(get_detections_client);
	} while (key != 'q');

	return 0;
}

