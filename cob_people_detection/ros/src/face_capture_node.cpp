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
* functions for capturing face images and storing them to a common database
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

#include <cob_people_detection/face_capture_node.h>

using namespace ipa_PeopleDetector;

FaceCaptureNode::FaceCaptureNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	it_ = 0;
	sync_input_2_ = 0;
	capture_image_ = false;
	finish_image_capture_ = false;
	face_images_.clear();

	// parameters
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";
	int eigenface_size;						// Desired width and height of the Eigenfaces (=eigenvectors).
	bool debug;								// enables some debug outputs

	std::cout << "\n---------------------------\nFace Capture Node Parameters:\n---------------------------\n";
	node_handle_.param("data_directory", data_directory_, data_directory_);
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("eigenface_size", eigenface_size, 100);
	std::cout << "eigenface_size = " << eigenface_size << "\n";
	node_handle_.param("debug", debug, false);
	std::cout << "debug = " << debug << "\n";

	// face recognizer trainer
	face_recognizer_trainer_.initTraining(data_directory_, eigenface_size, debug, face_images_);

	// subscribers
	it_ = new image_transport::ImageTransport(node_handle_);
//	people_segmentation_image_sub_.subscribe(*it_, "people_segmentation_image", 1);
//	face_recognition_subscriber_.subscribe(node_handle_, "face_position_array", 1);
	face_detection_subscriber_.subscribe(node_handle_, "face_detections", 1);
	color_image_sub_.subscribe(*it_, "color_image", 1);

	// actions
	add_data_server_ = new AddDataServer(node_handle_, "add_data_server", boost::bind(&FaceCaptureNode::addDataServerCallback, this, _1), false);
	add_data_server_->start();
	update_data_server_ = new UpdateDataServer(node_handle_, "update_data_server", boost::bind(&FaceCaptureNode::updateDataServerCallback, this, _1), false);
	update_data_server_->start();
	delete_data_server_ = new DeleteDataServer(node_handle_, "delete_data_server", boost::bind(&FaceCaptureNode::deleteDataServerCallback, this, _1), false);
	delete_data_server_->start();

	// input synchronization
	sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, sensor_msgs::Image> >(30);
	sync_input_2_->connectInput(face_detection_subscriber_, color_image_sub_);

	std::cout << "FaceCaptureNode initialized.\n";
}
    
FaceCaptureNode::~FaceCaptureNode()
{
	if (it_ != 0) delete it_;
	if (sync_input_2_ != 0) delete sync_input_2_;
	if (add_data_server_ != 0) delete add_data_server_;
	if (update_data_server_ != 0) delete update_data_server_;
	if (delete_data_server_ != 0) delete delete_data_server_;
}

void FaceCaptureNode::addDataServerCallback(const cob_people_detection::addDataGoalConstPtr& goal)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(active_action_mutex_);

	// open the gateway for sensor messages
	// rosrun dynamic_reconfigure dynparam set /cob_people_detection/sensor_message_gateway/sensor_message_gateway target_publishing_rate 2.0

	// set the label for the images than will be captured
	current_label_ = goal->label;

	// subscribe to face image topics
	message_filters::Connection input_callback_connection = sync_input_2_->registerCallback(boost::bind(&FaceCaptureNode::inputCallback, this, _1, _2));

	if (goal->capture_mode == MANUAL)
	{
		// configure manual recording
		number_captured_images_ = 0;
		finish_image_capture_ = false;		// finishes the image recording
		capture_image_ = false;				// trigger for image capture -> set true by service message

		// activate service server for image capture trigger messages and finish
		service_server_capture_image_ = node_handle_.advertiseService("capture_image", &FaceCaptureNode::captureImageCallback, this);
		service_server_finish_recording_ = node_handle_.advertiseService("finish_recording", &FaceCaptureNode::finishRecordingCallback, this);

		// wait until finish manual capture service message arrives
		while (finish_image_capture_ == false)
			ros::spinOnce();

		// unadvertise the manual image capture trigger service and finish recording service
		service_server_capture_image_.shutdown();
		service_server_finish_recording_.shutdown();

		// save new database status
		face_recognizer_trainer_.saveTrainingData(face_images_);

		// close action
		cob_people_detection::addDataResult result;
		add_data_server_->setSucceeded(result, "Manual capture finished successfully.");
	}
	else if (goal->capture_mode == CONTINUOUS)
	{
		// configure continuous recording
		number_captured_images_ = 0;
		while (number_captured_images_ < goal->continuous_mode_images_to_capture)
		{
			capture_image_ = true;		// trigger for image recording
			ros::Duration(goal->continuous_mode_delay).sleep();		// wait for a given time until next image can be captured
		}

		// save new database status
		face_recognizer_trainer_.saveTrainingData(face_images_);

		// close action
		cob_people_detection::addDataResult result;
		std::stringstream ss;
		ss << "Continuous capture of " << goal->continuous_mode_images_to_capture << " images finished successfully.";
		add_data_server_->setSucceeded(result, ss.str());
	}
	else
	{
		ROS_ERROR("FaceCaptureNode::addDataServerCallback: Unknown capture mode: %d.", goal->capture_mode);
		cob_people_detection::addDataResult result;
		std::stringstream ss;
		ss << "Unknown capture mode: " << goal->capture_mode;
		add_data_server_->setAborted(result, ss.str());
	}

	// unsubscribe face image topics
	input_callback_connection.disconnect();
}

/// captures the images
void FaceCaptureNode::inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	ROS_INFO("inputCallback");

	// only capture images if a recording is triggered
	if (capture_image_ == true)
	{
		// check number of detected faces -> accept only exactly one
		if (face_detection_msg->head_detections.size() != 1)
		{
			ROS_WARN("Either no head or more than one head detected. Discarding image.");
			return;
		}
		if (face_detection_msg->head_detections[0].face_detections.size() != 1)
		{
			ROS_WARN("Either no face or more than one face detected. Discarding image.");
			return;
		}

		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// store image and label
		const cob_people_detection_msgs::Rect& face_rect = face_detection_msg->head_detections[0].face_detections[0];
		const cob_people_detection_msgs::Rect& head_rect = face_detection_msg->head_detections[0].head_detection;
		cv::Rect face_bounding_box(head_rect.x+face_rect.x, head_rect.y+face_rect.y, face_rect.width, face_rect.height);
		cv::Mat img = color_image.clone();
    // normalize face
		if (face_recognizer_trainer_.addFace(img, face_bounding_box, current_label_, face_images_)==ipa_Utils::RET_FAILED)
     {
      ROS_WARN("Normalizing failed");
     return;
     }

		// only after successful recording
		capture_image_ = false;			// reset trigger for recording
		number_captured_images_++;		// increase number of captured images

		ROS_INFO("Face number %d captured.", number_captured_images_);
	}
}

/// Converts a color image message to cv::Mat format.
unsigned long FaceCaptureNode::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
{
	try
	{
		image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return ipa_Utils::RET_FAILED;
	}
	image = image_ptr->image;

	return ipa_Utils::RET_OK;
}

bool FaceCaptureNode::captureImageCallback(cob_people_detection::captureImage::Request &req, cob_people_detection::captureImage::Response &res)
{
	capture_image_ = true;
	// block until captured
	while (capture_image_ == true)
		ros::spinOnce();
	res.number_captured_images = number_captured_images_;
	return true;
}

bool FaceCaptureNode::finishRecordingCallback(cob_people_detection::finishRecording::Request &req, cob_people_detection::finishRecording::Response &res)
{
	finish_image_capture_ = true;
	return true;
}

void FaceCaptureNode::updateDataServerCallback(const cob_people_detection::updateDataGoalConstPtr& goal)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(active_action_mutex_);

	cob_people_detection::updateDataResult result;

	// determine the update mode
	if (goal->update_mode == BY_INDEX)
	{
		// update only one entry identified by its index
		face_recognizer_trainer_.updateFaceLabel(goal->update_index, goal->new_label);
	}
	else if (goal->update_mode == BY_LABEL)
	{
		// update all entries identified by their old label
		face_recognizer_trainer_.updateFaceLabels(goal->old_label, goal->new_label);
	}
	else
	{
		ROS_ERROR("FaceCaptureNode::updateDataServerCallback: Unknown update_mode.");
		update_data_server_->setAborted(result, "Unknown update_mode. No entries have been updated.");
		return;
	}

	// save new database status
	face_recognizer_trainer_.saveTrainingData(face_images_);

	// close action
	update_data_server_->setSucceeded(result, "Database update finished successfully.");
}

void FaceCaptureNode::deleteDataServerCallback(const cob_people_detection::deleteDataGoalConstPtr& goal)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(active_action_mutex_);

	cob_people_detection::deleteDataResult result;

	// determine the delete mode
	if (goal->delete_mode == BY_INDEX)
	{
		// delete only one entry identified by its index
		face_recognizer_trainer_.deleteFace(goal->delete_index, face_images_);
	}
	else if (goal->delete_mode == BY_LABEL)
	{
		// delete all entries identified by their label
		face_recognizer_trainer_.deleteFaces(goal->label, face_images_);
	}
	else
	{
		ROS_ERROR("FaceCaptureNode::deleteDataServerCallback: Unknown delete_mode.");
		delete_data_server_->setAborted(result, "Unknown delete_mode. No entries have been deleted from the database.");
		return;
	}

	// save new database status
	face_recognizer_trainer_.saveTrainingData(face_images_);

	// close action
	delete_data_server_->setSucceeded(result, "Deleting entries from the database finished successfully.");
}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_capture");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceRecognizerNode class instance
	FaceCaptureNode face_capture_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
