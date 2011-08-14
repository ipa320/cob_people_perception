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
#include <cob_people_detection/RecognizeAction.h>
#include <cob_people_detection/TrainContinuousAction.h>
#include <cob_people_detection/TrainCaptureSampleAction.h>
#include <cob_people_detection/ShowAction.h>
#include <actionlib/client/simple_action_client.h>

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

typedef actionlib::SimpleActionClient<cob_people_detection::RecognizeAction> RecognizeClient;
typedef actionlib::SimpleActionClient<cob_people_detection::TrainContinuousAction> TrainContinuousClient;
typedef actionlib::SimpleActionClient<cob_people_detection::TrainCaptureSampleAction> TrainCaptureSampleClient;
typedef actionlib::SimpleActionClient<cob_people_detection::ShowAction> ShowClient;


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
	trainContinuousClient.waitForResult(ros::Duration::Duration(3.0));
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
			trainContinuousClient.waitForResult(ros::Duration::Duration(3.0));
			if (trainContinuousClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
				printf("Image capture initiated.\n");
		}
	}

	goal.running = false;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration::Duration(10.0));
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
	trainContinuousClient.waitForResult(ros::Duration::Duration(120.0));


	goal.running = false;
	trainContinuousClient.sendGoal(goal);
	trainContinuousClient.waitForResult(ros::Duration::Duration(10.0));
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
	recognizeClient.waitForResult(ros::Duration::Duration(3.0));
	if (recognizeClient.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
		printf("Recognition on!\n");
	printf("Current State: %s\n", recognizeClient.getState().toString().c_str());

	std::cout << "hit any key to quit.\n";
	getch();

	goal.running = false;
	recognizeClient.sendGoal(goal);
	recognizeClient.waitForResult(ros::Duration::Duration(2.0));
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
	showClient.waitForResult(ros::Duration::Duration(3.0));
	printf("Current State: %s\n", showClient.getState().toString().c_str());
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "cob_people_detection_client");
	RecognizeClient recognizeClient("cob_people_detection/face_detection/recognize_server", true); // true -> don't need ros::spin()
	TrainContinuousClient trainContinuousClient("cob_people_detection/face_detection/train_continuous_server", true);
	TrainCaptureSampleClient trainCaptureSampleClient("cob_people_detection/face_detection/train_capture_sample_server", true);
	ShowClient showClient("cob_people_detection/face_detection/show_server", true);
	if (!recognizeClient.waitForServer(ros::Duration::Duration(2.0)))
	{
		std::cout << "No connection to server 'recognize_server'.\n";
		return 0;
	}
	if (!trainContinuousClient.waitForServer(ros::Duration::Duration(2.0)))
	{
		std::cout << "No connection to server 'train_continuous_server'.\n";
		return 0;
	}
	if (!trainCaptureSampleClient.waitForServer(ros::Duration::Duration(2.0)))
	{
		std::cout << "No connection to server 'train_capture_sample_server'.\n";
		return 0;
	}
	if (!showClient.waitForServer(ros::Duration::Duration(2.0)))
	{
		std::cout << "No connection to server 'show_server'.\n";
		return 0;
	}

	std::cout << "Connected to servers.\n";


	char key = 'q';
	do
	{
		std::cout << "\n\nChoose an option:\n1 - train with manual capture\n2 - recognize\n3 - train continuously (30 images)\n4 - Show average image\n5 - Show Eigenfaces\nq - Quit\n\n";
		key = getch();
		if (key == '1') train(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '2') recognize(recognizeClient);
		else if (key == '3') train_continuous(trainContinuousClient, trainCaptureSampleClient);
		else if (key == '4') show(showClient, 0);
		else if (key == '5') show(showClient, 1);
	}while(key != 'q');


	return 0;
}


