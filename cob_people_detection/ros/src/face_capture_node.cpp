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
	image_capture_service_enabled_ = false;
	capture_image_manually_ = false;
	capture_image_continuously_ = false;
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

	if (goal->capture_mode == cob_people_detection::CaptureMode::MANUAL)
	{
		// configure manual recording
		finish_image_capture_ = false;		// finishes the image recording
		capture_image_manually_ = false;	// trigger for image capture -> set true by service message
		capture_image_continuously_ = false;	// images are only recorded on demand
		image_capture_service_enabled_ = true;	// allow image capture service messages to capture images

		// wait until finish manual capture service message arrives
		while (finish_image_capture_ == false)
			ros::spinOnce();

		// save new database status
		face_recognizer_trainer_.saveTrainingData(face_images_);

		// close action
		cob_people_detection::addDataResult result;
		add_data_server_->setSucceeded(result, "Manual capture finished successfully.");
	}
	else if (goal->capture_mode == cob_people_detection::CaptureMode::CONTINUOUS)
	{
		// configure continuous recording
		captured_images_ = 0;
		while (captured_images_ < goal->continuous_mode_images_to_capture)
		{
			capture_image_continuously_ = true;		// trigger for image recording
			ros::Duration(goal->continuous_mode_delay).sleep();		// wait for given time until next image can be captured
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
    
/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
void FaceCaptureNode::inputCallback(const cob_people_detection_msgs::ColorDepthImageArray::ConstPtr& face_detection_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	// only capture images if a recording is triggered
	if (capture_image_continuously_ == true || (image_capture_service_enabled_ == true && capture_image_manually_ == true))
	{
		// convert color image to cv::Mat
		cv_bridge::CvImageConstPtr color_image_ptr;
		cv::Mat color_image;
		convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);

		// check number of detected faces -> accept only exactly one

		// store image and label
//		face_recognizer_trainer_.addFace(color_image, face_detection_msg->)

		// only after successful recording
		capture_image_continuously_ = false;		// reset trigger for continuous recording
		capture_image_manually_ = false;	// reset trigger for manual recording
	}

/*

	// insert all detected heads and faces
	for (int i=0; i<(int)face_detection_msg->head_detections.size(); i++)
	{
		// paint head
		const cob_people_detection_msgs::Rect& head_rect = face_detection_msg->head_detections[i].head_detection;
		cv::Rect head(head_rect.x, head_rect.y, head_rect.width, head_rect.height);
		cv::rectangle(color_image, cv::Point(head.x, head.y), cv::Point(head.x + head.width, head.y + head.height), CV_RGB(148, 219, 255), 2, 8, 0);

		// paint faces
		for (int j=0; j<(int)face_detection_msg->head_detections[i].face_detections.size(); j++)
		{
			const cob_people_detection_msgs::Rect& face_rect = face_detection_msg->head_detections[i].face_detections[j];
			cv::Rect face(face_rect.x+head.x, face_rect.y+head.y, face_rect.width, face_rect.height);
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(191, 255, 148), 2, 8, 0);
		}
	}

	// insert recognized faces
	for(int i=0; i<(int)face_recognition_msg->detections.size(); i++)
	{
		const cob_people_detection_msgs::Rect& face_rect = face_recognition_msg->detections[i].mask.roi;
		cv::Rect face(face_rect.x, face_rect.y, face_rect.width, face_rect.height);

		if (face_recognition_msg->detections[i].detector == "head")
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
		else
			cv::rectangle(color_image, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

		if (face_recognition_msg->detections[i].label == "Unknown")
			// Distance to face class is too high
			cv::putText(color_image, "Unknown", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
		else if (face_recognition_msg->detections[i].label == "No face")
			// Distance to face space is too high
			cv::putText(color_image, "No face", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
		else
			// Face classified
			cv::putText(color_image, face_recognition_msg->detections[i].label.c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
	}

	// display image
	cv::imshow("Detections and Recognitions", color_image);
	cv::waitKey(10);

	// publish image
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = color_image;
	cv_ptr.encoding = "bgr8";
	people_detection_image_pub_.publish(cv_ptr.toImageMsg());
	*/
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
