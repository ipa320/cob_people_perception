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
//#include <cob_people_detection/ShowAction.h>

// services
#include <cob_people_detection/captureImage.h>
#include <cob_people_detection/finishRecording.h>

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
//typedef actionlib::SimpleActionClient<cob_people_detection::ShowAction> ShowClient;

/*
void train(TrainContinuousClient& trainContinuousClient, TrainCaptureSampleClient& trainCaptureSampleClient)
{
	std::string id;
	std::cout << "Input the ID of the captured person: ";
	std::cin >> id;

	cob_people_detection::TrainContinuousGoal goal;
	// Fill in goal here
	goal.running = true;
	goal.doPCA = true;
	goal.appendData = true;
	goal.numberOfImagesToCapture = 0;
	goal.trainingID = id;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration(3.0));
	if (trainContinuousClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Training on!\n");
	printf("Current State: %s\n", trainContinuousClient.getState().toString().c_str());

	std::cout << "Hit 'q' key to quit or 'c' key to capture an image.\n";
	char key;
	while ((key=getch()) != 'q')
	{
		if (key == 'c')
		{
			cob_people_detection::TrainCaptureSampleGoal captureGoal;
			trainCaptureSampleClient.sendGoal(captureGoal);
			trainContinuousClient.waitForResult(ros::Duration(3.0));
			if (trainContinuousClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				printf("Image capture initiated.\n");
		}
	}

	goal.running = false;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration(10.0));
	if (trainContinuousClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Training off!\n");
	printf("Current State: %s\n", trainContinuousClient.getState().toString().c_str());
}


void train_continuous(TrainContinuousClient& trainContinuousClient, TrainCaptureSampleClient& trainCaptureSampleClient)
{
	std::string id;
	std::cout << "Input the ID of the captured person: ";
	std::cin >> id;

	cob_people_detection::TrainContinuousGoal goal;
	// Fill in goal here
	goal.running = true;
	goal.doPCA = true;
	goal.appendData = true;
	goal.numberOfImagesToCapture = 30;
	goal.trainingID = id;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration(120.0));


	goal.running = false;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration(10.0));
	if (trainContinuousClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Training off!\n");
	printf("Current State: %s\n", trainContinuousClient.getState().toString().c_str());
}


void recognize(RecognizeClient& recognizeClient)
{
	cob_people_detection::RecognizeGoal goal;
	// Fill in goal here
	goal.running = true;
	goal.doRecognition = true;
	goal.display = true;
	recognizeClient.sendGoal(goal);
	recognizeClient.waitForResult(ros::Duration(3.0));
	if (recognizeClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Recognition on!\n");
	printf("Current State: %s\n", recognizeClient.getState().toString().c_str());

	std::cout << "hit any key to quit.\n";
	getch();

	goal.running = false;
	recognizeClient.sendGoal(goal);
	recognizeClient.waitForResult(ros::Duration(2.0));
	if (recognizeClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Recognition off!\n");
	printf("Current State: %s\n", recognizeClient.getState().toString().c_str());
}


void show(ShowClient& showClient, int mode)
{
	cob_people_detection::ShowGoal goal;
	// Fill in goal here
	goal.mode = mode;
	showClient.sendGoal(goal);
	showClient.waitForResult(ros::Duration(3.0));
	printf("Current State: %s\n", showClient.getState().toString().c_str());
}
*/

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
		while ((key=getch()) != 'q')
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, "cob_people_detection_client");

	ros::NodeHandle nh;

	AddDataClient add_data_client("/cob_people_detection/face_capture/add_data_server", true); // true -> don't need ros::spin()
	ros::ServiceClient capture_image_client = nh.serviceClient<cob_people_detection::captureImage>("/cob_people_detection/face_capture/capture_image");
	ros::ServiceClient finish_recording_client = nh.serviceClient<cob_people_detection::finishRecording>("/cob_people_detection/face_capture/finish_recording");
	UpdateDataClient update_data_client("/cob_people_detection/face_capture/update_data_server", true);
	DeleteDataClient delete_data_client("/cob_people_detection/face_capture/delete_data_server", true);
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

	std::cout << "Connected to servers.\n";


	char key = 'q';
	do
	{
		std::cout << "\n\nChoose an option:\n1 - capture face images\n2 - update database labels\n3 - delete database entries\n4 - Show average image\n5 - Show Eigenfaces\na - Train\nq - Quit\n\n";
		key = getch();
		if (key == '1') addData(add_data_client, capture_image_client, finish_recording_client);//train(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '2') updateData(update_data_client);//recognize(recognizeClient);
		else if (key == '3') deleteData(delete_data_client);//train_continuous(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '4') ;//show(showClient, 0);
		else if (key == '5') ;//show(showClient, 1);
	}while(key != 'q');


	return 0;
}


