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
//	node_handle_.param("display", display_, true);
//	std::cout << "display = " << display_ << "\n";
//	node_handle_.param("face_redetection_time", face_redetection_time_, 2.0);
//	std::cout << "face_redetection_time = " << face_redetection_time_ << "\n";
//	node_handle_.param("min_segmented_people_ratio_color", min_segmented_people_ratio_color_, 0.7);
//	std::cout << "min_segmented_people_ratio_color = " << min_segmented_people_ratio_color_ << "\n";
//	node_handle_.param("min_segmented_people_ratio_range", min_segmented_people_ratio_range_, 0.2);
//	std::cout << "min_segmented_people_ratio_range = " << min_segmented_people_ratio_range_ << "\n";
//	node_handle_.param("use_people_segmentation", use_people_segmentation_, true);
//	std::cout << "use_people_segmentation = " << use_people_segmentation_ << "\n";
//	node_handle_.param("tracking_range_m", tracking_range_m_, 0.3);
//	std::cout << "tracking_range_m = " << tracking_range_m_ << "\n";
//	node_handle_.param("face_identification_score_decay_rate", face_identification_score_decay_rate_, 0.9);
//	std::cout << "face_identification_score_decay_rate = " << face_identification_score_decay_rate_ << "\n";
//	node_handle_.param("min_face_identification_score_to_publish", min_face_identification_score_to_publish_, 0.9);
//	std::cout << "min_face_identification_score_to_publish = " << min_face_identification_score_to_publish_ << "\n";
//	node_handle_.param("fall_back_to_unknown_identification", fall_back_to_unknown_identification_, true);
//	std::cout << "fall_back_to_unknown_identification = " << fall_back_to_unknown_identification_ << "\n";

	// face recognizer trainer
	face_recognizer_trainer_.initTraining(data_directory_, eigenface_size, debug, face_images_);

	// subscribers
	it_ = new image_transport::ImageTransport(node_handle_);
//	people_segmentation_image_sub_.subscribe(*it_, "people_segmentation_image", 1);
//	face_recognition_subscriber_.subscribe(node_handle_, "face_position_array", 1);
	face_detection_subscriber_.subscribe(node_handle_, "face_detections", 1);
	color_image_sub_.subscribe(*it_, "colorimage", 1);

	// actions
	add_data_server_ = new AddDataServer(node_handle_, "add_data_server", boost::bind(&FaceCaptureNode::addDataServerCallback, this, _1), false);
	add_data_server_->start();

	// input synchronization
	sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_people_detection_msgs::ColorDepthImageArray, sensor_msgs::Image> >(10);

	std::cout << "FaceCaptureNode initialized.\n";
}
    
FaceCaptureNode::~FaceCaptureNode()
{
	if (it_ != 0) delete it_;
	if (sync_input_2_ != 0) delete sync_input_2_;
	if (add_data_server_ != 0) delete add_data_server_;
}

void FaceCaptureNode::addDataServerCallback(const cob_people_detection::addDataGoalConstPtr& goal)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(active_action_mutex_);

	// accept goal
	add_data_server_->acceptNewGoal();

	// set the label for the images than will be captured
	current_label_ = goal->label;

	// subscribe to face image topics
	sync_input_2_->connectInput(face_detection_subscriber_, color_image_sub_);
	sync_input_2_->registerCallback(boost::bind(&FaceCaptureNode::inputCallback, this, _1, _2));

	if (goal->capture_mode == MANUAL)
	{
		// configure manual recording
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
}

/// captures the images
void FaceCaptureNode::inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
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
		const cob_people_detection_msgs::Rect& rect = face_detection_msg->head_detections[0].face_detections[0];
		cv::Rect face_bounding_box(rect.x, rect.y, rect.width, rect.height);
		face_recognizer_trainer_.addFace(color_image, face_bounding_box, current_label_, face_images_);

		// only after successful recording
		capture_image_ = false;			// reset trigger for recording
		number_captured_images_++;		// increase number of captured images
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
	return true;
}

bool FaceCaptureNode::finishRecordingCallback(cob_people_detection::finishRecording::Request &req, cob_people_detection::finishRecording::Response &res)
{
	finish_image_capture_ = true;
	return true;
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
