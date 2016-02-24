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
 * Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
 * Author: Richard Bormann, email: richard.bormann@ipa.fhg.de
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

#include "cob_people_detection/face_detection.h"
#include <pluginlib/class_list_macros.h>

#ifdef __LINUX__
#include "cob_vision_utils/GlobalDefines.h"
#include "cob_vision_utils/VisionUtils.h"
#else
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/VisionUtils.h"
#endif

#include <opencv/highgui.h>

PLUGINLIB_DECLARE_CLASS(ipa_PeopleDetector, CobFaceDetectionNodelet, ipa_PeopleDetector::CobFaceDetectionNodelet, nodelet::Nodelet)
;

using namespace ipa_PeopleDetector;

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(sensor_msgs::PointCloud2* const )
{
}

//####################
//#### node class ####
CobFaceDetectionNodelet::CobFaceDetectionNodelet()
{
	it_ = 0;
	sync_pointcloud_ = 0;
	people_detector_ = 0;
	train_continuous_server_ = 0;
	train_capture_sample_server_ = 0;
	recognize_server_ = 0;
	load_server_ = 0;
	save_server_ = 0;
	show_server_ = 0;
	occupied_by_action_ = false;
	recognize_server_running_ = false;
	train_continuous_server_running_ = false;
	number_training_images_captured_ = 0;
	do_recognition_ = true;
	display_ = false;
	directory_ = ros::package::getPath("cob_people_detection") + "/common/files/windows/"; // can be changed by a parameter
}

CobFaceDetectionNodelet::~CobFaceDetectionNodelet()
{
	if (people_detector_ != 0)
		delete people_detector_;
	if (train_continuous_server_ != 0)
		delete train_continuous_server_;
	if (train_capture_sample_server_ != 0)
		delete train_capture_sample_server_;
	if (recognize_server_ != 0)
		delete recognize_server_;
	if (load_server_ != 0)
		delete load_server_;
	if (save_server_ != 0)
		delete save_server_;
	if (show_server_ != 0)
		delete show_server_;
	if (it_ != 0)
		delete it_;
	if (sync_pointcloud_ != 0)
		delete sync_pointcloud_;
}

void CobFaceDetectionNodelet::onInit()
{
	sync_pointcloud_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> >(2);
	node_handle_ = getNodeHandle();
	it_ = new image_transport::ImageTransport(node_handle_);
	face_position_publisher_ = node_handle_.advertise<cob_perception_msgs::DetectionArray>("face_position_array", 1);
	face_detection_image_pub_ = it_->advertise("face_detection_image", 1);

	recognize_server_ = new RecognizeServer(node_handle_, "recognize_server", boost::bind(&CobFaceDetectionNodelet::recognizeServerCallback, this, _1), false);
	recognize_server_->start();

	recognize_service_server_ = node_handle_.advertiseService("recognize_service_server", &CobFaceDetectionNodelet::recognizeServiceServerCallback, this);

	train_continuous_server_ = new TrainContinuousServer(node_handle_, "train_continuous_server", boost::bind(&CobFaceDetectionNodelet::trainContinuousServerCallback, this, _1),
			false);
	train_continuous_server_->start();

	train_capture_sample_server_ = new TrainCaptureSampleServer(node_handle_, "train_capture_sample_server",
			boost::bind(&CobFaceDetectionNodelet::trainCaptureSampleServerCallback, this, _1), false);
	train_capture_sample_server_->start();

	load_server_ = new LoadServer(node_handle_, "load_server", boost::bind(&CobFaceDetectionNodelet::loadServerCallback, this, _1), false);
	load_server_->start();

	save_server_ = new SaveServer(node_handle_, "save_server", boost::bind(&CobFaceDetectionNodelet::saveServerCallback, this, _1), false);
	save_server_->start();

	show_server_ = new ShowServer(node_handle_, "show_server", boost::bind(&CobFaceDetectionNodelet::showServerCallback, this, _1), false);
	show_server_->start();

	// Parameters
	std::cout << "\n--------------------------\nFace Detection Parameters:\n--------------------------\n";
	node_handle_.param("face_size_min_m", face_size_min_m_, FACE_SIZE_MIN_M);
	std::cout << "face_size_min_m = " << face_size_min_m_ << "\n";
	node_handle_.param("face_size_max_m", face_size_max_m_, FACE_SIZE_MAX_M);
	std::cout << "face_size_max_m = " << face_size_max_m_ << "\n";
	node_handle_.param("max_face_z_m", max_face_z_m_, MAX_FACE_Z_M);
	std::cout << "max_face_z_m = " << max_face_z_m_ << "\n";
	node_handle_.param("data_directory", directory_, directory_);
	std::cout << "data_directory = " << directory_ << "\n";
	node_handle_.param("fill_unassigned_depth_values", fill_unassigned_depth_values_, true);
	std::cout << "fill_unassigned_depth_values = " << fill_unassigned_depth_values_ << "\n";
	node_handle_.param("reason_about_3dface_size", reason_about_3dface_size_, true);
	std::cout << "reason_about_3dface_size = " << reason_about_3dface_size_ << "\n";

	// initializations
	init();

	//ros::spin(); not necessary with nodelets
}

unsigned long CobFaceDetectionNodelet::init()
{
	std::cout << "CobFaceDetectionNodelet::init()..." << std::endl;

	shared_image_sub_.subscribe(node_handle_, "pointcloud", 1);
	color_camera_image_sub_.subscribe(*it_, "colorimage", 1);
	//	sync_pointcloud_->connectInput(shared_image_sub_, color_camera_image_sub_);
	//	sync_pointcloud_->registerCallback(boost::bind(&CobFaceDetectionNodelet::recognizeCallback, this, _1, _2));

	sync_pointcloud_->connectInput(shared_image_sub_, color_camera_image_sub_);
	sync_pointcloud_callback_connection_ = sync_pointcloud_->registerCallback(boost::bind(&CobFaceDetectionNodelet::recognizeCallback, this, _1, _2));

	std::cout << "   inputs connected." << std::endl;

#ifdef __LINUX__
#else
	colored_pc_ = ipa_SensorFusion::CreateColoredPointCloud();
#endif

	if (people_detector_ != 0)
		delete people_detector_;
	people_detector_ = new ipa_PeopleDetector::PeopleDetector();

	std::cout << "   PeopleDetector created." << std::endl;

	filename_ = 0;

	// load data for face recognition
	loadRecognizerData();
	std::cout << "   recognizer data loaded." << std::endl;

	// use this instead if the rdata.xml file is corrupted
	// todo:
	//	loadTrainingData(false);
	//	//loadRecognizerData();
	//	//loadTrainingData(false);
	//	run_pca_ = true;
	//	PCA();
	//	saveRecognizerData();

	std::string iniFileNameAndPath = directory_ + "peopleDetectorIni.xml";

	//if (CameraSensorsControlFlow::Init(directory, "peopleDetectorIni.xml", colorCamera0, colorCamera1, rangeImagingSensor) & ipa_Utils::RET_FAILED)
	//{
	//	std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
	//	std::cerr << "\t ... Could not initialize 'CameraSensorsControlFlow'" << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}

	if (people_detector_->Init(directory_) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::Init:" << std::endl;
		std::cerr << "\t ... Could not initialize people detector library.\n";
		return ipa_Utils::RET_FAILED;
	}
	std::cout << "   PeopleDetector initilized." << std::endl;

	if (loadParameters(iniFileNameAndPath.c_str()) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::Init:" << std::endl;
		std::cerr << "\t ... Error while loading configuration file '" << std::endl;
		std::cerr << "\t ... " << iniFileNameAndPath << "'.\n";
		return ipa_Utils::RET_FAILED;
	}
	std::cout << "   ini-file loaded." << std::endl;

	run_pca_ = false;

	return ipa_Utils::RET_OK;
}

bool CobFaceDetectionNodelet::recognizeServiceServerCallback(cob_people_detection::Recognition::Request &req, cob_people_detection::Recognition::Response &res)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	if (lock.owns_lock() == false || (occupied_by_action_ == true && recognize_server_running_ == false))
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::recognizeServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		res.success = ipa_Utils::RET_FAILED;
		return false;
	}

	// read out req message
	// set up the recognition callback linkage
	if (req.running == true)
	{
		// enable recognition
		occupied_by_action_ = true;
		recognize_server_running_ = true;
		do_recognition_ = req.doRecognition;
		display_ = req.display;
		//cv::namedWindow("Face Detector");
		//cv::waitKey(1);
		//sync_pointcloud_->connectInput(shared_image_sub_, color_camera_image_sub_);
		//sync_pointcloud_callback_connection_ = sync_pointcloud_->registerCallback(boost::bind(&CobFaceDetectionNodelet::recognizeCallback, this, _1, _2, req->doRecognition, req->display));
	}
	else
	{
		// disable recognition
		occupied_by_action_ = false;
		recognize_server_running_ = false;
	}

	res.success = ipa_Utils::RET_OK;
	return true;
}

void CobFaceDetectionNodelet::recognizeServerCallback(const cob_people_detection::RecognizeGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	cob_people_detection::RecognizeResult result;
	if (lock.owns_lock() == false || (occupied_by_action_ == true && recognize_server_running_ == false))
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::recognizeServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		recognize_server_->setSucceeded(result, "Some other action is handled at the moment.");
		return;
	}

	// read out goal message
	// set up the recognition callback linkage
	if (goal->running == true)
	{
		// enable recognition
		occupied_by_action_ = true;
		recognize_server_running_ = true;
		do_recognition_ = goal->doRecognition;
		display_ = goal->display;
		//cv::namedWindow("Face Detector");
		//cv::waitKey(1);
		//sync_pointcloud_->connectInput(shared_image_sub_, color_camera_image_sub_);
		//sync_pointcloud_callback_connection_ = sync_pointcloud_->registerCallback(boost::bind(&CobFaceDetectionNodelet::recognizeCallback, this, _1, _2, goal->doRecognition, goal->display));
	}
	else
	{
		// disable recognition
		occupied_by_action_ = false;
		recognize_server_running_ = false;
	}

	result.success = ipa_Utils::RET_OK;
	recognize_server_->setSucceeded(result);
}

void CobFaceDetectionNodelet::trainContinuousServerCallback(const cob_people_detection::TrainContinuousGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	cob_people_detection::TrainContinuousResult result;
	if (lock.owns_lock() == false || (occupied_by_action_ == true && train_continuous_server_running_ == false))
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::trainContinuousServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		train_continuous_server_->setSucceeded(result, "Some other action is handled at the moment.");
		return;
	}

	// read out goal message
	// set up the recognition callback linkage
	if (goal->running == true)
	{
		// enable training
		capture_training_face_ = false;
		current_training_id_ = goal->trainingID;

		if (goal->appendData == true)
			loadTrainingData(false);
		else
		{
			filename_ = 0;
			std::string path = directory_ + "TrainingData/";
			try
			{
				boost::filesystem::remove_all(path.c_str());
				boost::filesystem::create_directory(path.c_str());
			} catch (const std::exception &ex)
			{
				std::cerr << "ERROR - PeopleDetector::trainContinuousServerCallback():" << std::endl;
				std::cerr << "\t ... Exception catch of '" << ex.what() << "'" << std::endl;
			}
		}
		occupied_by_action_ = true;
		train_continuous_server_running_ = true;
		//cv::namedWindow("Face Recognizer Training");
		//cv::waitKey(10);
		//sync_pointcloud_->connectInput(shared_image_sub_, color_camera_image_sub_);
		//sync_pointcloud_callback_connection_ = sync_pointcloud_->registerCallback(boost::bind(&CobFaceDetectionNodelet::trainContinuousCallback, this, _1, _2));
		std::cout << "train run...\n";

		// check whether the continuous mode was enabled or if the image capture is manually triggered
		if (goal->numberOfImagesToCapture > 0)
		{
			lock.unlock();
			number_training_images_captured_ = 0;
			bool capture = true;
			while (capture)
			{
				boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(10));
				if (lock.owns_lock() && capture_training_face_ == false)
					capture_training_face_ = true;
				capture = (number_training_images_captured_ < goal->numberOfImagesToCapture);
			}

			// turn off training mode
			std::cout << "continuous train off...\n";

			// disable training
			occupied_by_action_ = false;
			train_continuous_server_running_ = false;

			// save images
			saveTrainingData();

			// do data analysis if enabled
			if (goal->doPCA)
			{
				PCA();
				saveRecognizerData();
			}
		}
	}
	else
	{
		std::cout << "train off...\n";
		// disable training
		//sync_pointcloud_callback_connection_.disconnect();
		occupied_by_action_ = false;
		train_continuous_server_running_ = false;
		//cv::destroyWindow("Face Recognizer Training");
		//cv::waitKey(10);

		// save images
		saveTrainingData();

		// do data analysis if enabled
		if (goal->doPCA)
		{
			PCA();
			saveRecognizerData();
		}
	}

	result.success = ipa_Utils::RET_OK;
	train_continuous_server_->setSucceeded(result);
}

void CobFaceDetectionNodelet::trainCaptureSampleServerCallback(const cob_people_detection::TrainCaptureSampleGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(10));

	cob_people_detection::TrainCaptureSampleResult result;
	if (lock.owns_lock() == false || (occupied_by_action_ == true && train_continuous_server_running_ == false) || occupied_by_action_ == false)
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::trainCaptureSampleServerCallback:" << std::endl;
		std::cerr
				<< "\t ... Either the training mode is not started or another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		train_capture_sample_server_->setSucceeded(result, "Some other action is handled at the moment or training mode not started.");
		return;
	}

	capture_training_face_ = true;
	result.success = ipa_Utils::RET_OK;
	train_capture_sample_server_->setSucceeded(result);
}

void CobFaceDetectionNodelet::loadServerCallback(const cob_people_detection::LoadGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	cob_people_detection::LoadResult result;
	if (lock.owns_lock() == false || occupied_by_action_ == true)
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::loadServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		load_server_->setSucceeded(result, "Some other action is handled at the moment or training mode not started.");
		return;
	}

	// load data
	result.success = ipa_Utils::RET_OK;
	if (goal->loadMode == 0)
		result.success = loadAllData();
	else if (goal->loadMode == 1)
		result.success = loadTrainingData(goal->doPCA);
	else if (goal->loadMode == 2)
		result.success = loadRecognizerData();

	load_server_->setSucceeded(result);
}

void CobFaceDetectionNodelet::saveServerCallback(const cob_people_detection::SaveGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	cob_people_detection::SaveResult result;
	if (lock.owns_lock() == false || occupied_by_action_ == true)
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::saveServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		save_server_->setSucceeded(result, "Some other action is handled at the moment or training mode not started.");
		return;
	}

	// save data
	result.success = ipa_Utils::RET_OK;
	if (goal->saveMode == 0)
		result.success = saveAllData();
	else if (goal->saveMode == 1)
		result.success = saveTrainingData();
	else if (goal->saveMode == 2)
		result.success = saveRecognizerData();

	save_server_->setSucceeded(result);
}

void CobFaceDetectionNodelet::showServerCallback(const cob_people_detection::ShowGoalConstPtr& goal)
{
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(2000));

	cob_people_detection::ShowResult result;
	if (lock.owns_lock() == false || occupied_by_action_ == true)
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::showServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		show_server_->setSucceeded(result, "Some other action is handled at the moment or training mode not started.");
		return;
	}

	// display
	loadTrainingData(false);
	if (goal->mode == 0)
	{
		// show average image
		cv::Mat avgImage(100, 100, CV_8UC1);
		PCA();
		showAVGImage(avgImage);

		cv::namedWindow("AVGImage");
		cv::imshow("AVGImage", avgImage);
		cv::waitKey(3000);

		cv::destroyWindow("AVGImage");
	}
	else if (goal->mode == 1)
	{
		// show eigenfaces
		cv::Mat eigenface;
		cv::namedWindow("Eigenfaces");
		PCA();
		for (int i = 0; i < (int)20/*face_images_.size()-1*/; i++)
		{
			getEigenface(eigenface, i);
			cv::imshow("Eigenfaces", eigenface);
			cv::waitKey();
		}
		cv::destroyWindow("Eigenfaces");
	}

	result.success = ipa_Utils::RET_OK;
	show_server_->setSucceeded(result);
}

unsigned long CobFaceDetectionNodelet::detectFaces(cv::Mat& xyz_image, cv::Mat& color_image)
{
	cv::Mat xyz_image_8U3;
	ipa_Utils::ConvertToShowImage(xyz_image, xyz_image_8U3, 3);
	if (people_detector_->DetectFaces(color_image, xyz_image_8U3, color_faces_, range_faces_, range_face_indices_with_color_face_detection_, fill_unassigned_depth_values_)
			& ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetection::detectFaces" << std::endl;
		std::cerr << "\t ... Could not detect faces.\n";
		return ipa_Utils::RET_FAILED;
	}

	// check whether the color faces have a reasonable 3D size
	std::vector < cv::Rect > tempFaces = color_faces_; // todo: why copy to tempFaces?
	color_faces_.clear();
	// For each face...
	for (uint iface = 0; iface < tempFaces.size(); iface++)
	{
		cv::Rect& face = tempFaces[iface];

		//		one_face.box2d = faces_vec[iface];
		//		one_face.id = i; // The cascade that computed this face.

		// Get the median disparity in the middle half of the bounding box.
		int uStart = floor(0.25 * face.width);
		int uEnd = floor(0.75 * face.width) + 1;
		int vStart = floor(0.25 * face.height);
		int vEnd = floor(0.75 * face.height) + 1;
		int du = abs(uEnd - uStart);

		cv::Mat faceRegion = xyz_image(face);
		cv::Mat tmat(1, du * abs(vEnd - vStart), CV_32FC1); // vector of all depth values in the face region
		float* tmatPtr = (float*)tmat.data;
		for (int v = vStart; v < vEnd; v++)
		{
			float* zPtr = (float*)faceRegion.row(v).data;
			zPtr += 2 + 3 * uStart;
			for (int u = uStart; u < uEnd; u++)
			{
				float depthval = *zPtr;
				if (!isnan(depthval))
					*tmatPtr = depthval;
				else
					*tmatPtr = 1e20;
				tmatPtr++;
				zPtr += 3;
			}
		}
		//
		//
		//		int vector_position = 0;
		//		for (int v=vStart; v<vEnd; v++)
		//		  for (int u=uStart; u<uEnd; u++, vector_position++)
		//		  {
		//			float depthval = xyz_image.at<cv::Vec3f>(v,u).z;
		//			if (!isnan(depthval)) tmat.at<float>(0,vector_position) = depthval;
		//			else tmat.at<float>(0,vector_position) = 1e20;
		//		  }

		cv::Mat tmat_sorted;
		cv::sort(tmat, tmat_sorted, CV_SORT_EVERY_COLUMN + CV_SORT_DESCENDING);
		double avg_depth = tmat_sorted.at<float>(floor(cv::countNonZero(tmat_sorted >= 0.0) * 0.5)); // Get the middle valid disparity (-1 disparities are invalid)

		// Fill in the rest of the face data structure.
		//		one_face.center2d = cv::Point2d(one_face.box2d.x+one_face.box2d.width*0.5,
		//										one_face.box2d.y+one_face.box2d.height*0.5);
		//		one_face.radius2d = one_face.box2d.width*0.5;

		// If the median disparity was valid and the face is a reasonable size, the face status is "good".
		// If the median disparity was valid but the face isn't a reasonable size, the face status is "bad".
		// Otherwise, the face status is "unknown".
		// Only bad faces are removed
		if (avg_depth > 0)
		{
			double radiusX, radiusY, radius3d = 1e20;
			cv::Vec3f a, b;
			// vertical line regularly lies completely on the head whereas this does not hold very often for the horizontal line crossing the bounding box of the face
			// rectangle in the middle
			a = xyz_image.at<cv::Vec3f>((int)(face.y + face.height * 0.25), (int)(face.x + 0.5 * face.width));
			b = xyz_image.at<cv::Vec3f>((int)(face.y + face.height * 0.75), (int)(face.x + 0.5 * face.width));
			if (display_)
				std::cout << "a: " << a.val[0] << " " << a.val[1] << " " << a.val[2] << "   b: " << " " << b.val[0] << " " << b.val[1] << " " << b.val[2] << "\n";
			if (isnan(a.val[0]) || isnan(b.val[0]))
				radiusY = 0.0;
			else
				radiusY = cv::norm(b - a);
			radius3d = radiusY;

			// for radius estimation with the horizontal line through the face rectangle use points which typically still lie on the face and not in the background
			a = xyz_image.at<cv::Vec3f>((int)(face.y + face.height * 0.5), (int)(face.x + face.width * 0.25));
			b = xyz_image.at<cv::Vec3f>((int)(face.y + face.height * 0.5), (int)(face.x + face.width * 0.75));
			if (display_)
				std::cout << "a: " << a.val[0] << " " << a.val[1] << " " << a.val[2] << "   b: " << " " << b.val[0] << " " << b.val[1] << " " << b.val[2] << "\n";
			if (isnan(a.val[0]) || isnan(b.val[0]))
				radiusX = 0.0;
			else
			{
				radiusX = cv::norm(b - a);
				if (radiusY != 0.0)
					radius3d = (radiusX + radiusY) * 0.5;
				else
					radius3d = radiusX;
			}

			//			cv::Point pup(face.x+0.5*face.width, face.y+face.height*0.25);
			//			cv::Point plo(face.x+0.5*face.width, face.y+face.height*0.75);
			//			cv::Point ple(face.x+face.width*0.25, face.y+face.height*0.5);
			//			cv::Point pri(face.x+face.width*0.75, face.y+face.height*0.5);
			//			cv::line(xyz_image_8U3, pup, plo, CV_RGB(255, 255, 255), 2);
			//			cv::line(xyz_image_8U3, ple, pri, CV_RGB(255, 255, 255), 2);

			if (display_)
			{
				std::cout << "radiusX: " << radiusX << "  radiusY: " << radiusY << "\n";
				std::cout << "avg_depth: " << avg_depth << " > max_face_z_m_: " << max_face_z_m_ << " ?  2*radius3d: " << 2.0 * radius3d << " < face_size_min_m_: "
						<< face_size_min_m_ << " ?  2radius3d: " << 2.0 * radius3d << " > face_size_max_m_:" << face_size_max_m_ << "?\n";
			}
			if (reason_about_3dface_size_ == true && radius3d > 0.0 && (avg_depth > max_face_z_m_ || 2.0 * radius3d < face_size_min_m_ || 2.0 * radius3d > face_size_max_m_))
				continue; // face does not match normal human appearance
		}
		color_faces_.push_back(face);
	}
	//			imshow("xyz image", xyz_image_8U3);
	//			cv::waitKey(10);

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::recognizeFace(cv::Mat& color_image, std::vector<int>& index)
{
	if (people_detector_->RecognizeFace(color_image, color_faces_, &n_eigens_, eigen_vectors_, avg_image_, face_class_avg_projections_, index, &threshold_, &threshold_fs_,
			eigen_val_mat_, /*&person_classifier_*/0) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::recognizeFace:" << std::endl;
		std::cerr << "\t ... Error while recognizing faces.\n";
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::addFace(cv::Mat& image, std::string id)
{
	// addFace should only be called if there is exactly one face found --> so we access it with color_faces_[0]
	if (people_detector_->AddFace(image, color_faces_[0], id, face_images_, id_) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::AddFace:" << std::endl;
		std::cerr << "\t ... Could not save face.\n";
		return ipa_Utils::RET_FAILED;
	}
	run_pca_ = true;

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::PCA()
{
	// only run PCA if new data has arrived after last computation
	if (!run_pca_)
	{
		//std::cout << "INFO - PeopleDetector::PCA:" << std::endl;
		//std::cout << "\t ... PCA algorithm skipped.\n";
		return ipa_Utils::RET_OK;
	}

	if (face_images_.size() < 2)
	{
		std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Less than two images are trained.\n";
		return ipa_Utils::RET_OK;
	}

	// Delete memory
	eigen_vectors_.clear();

	// mirror images
	//todo:
	//	std::vector<cv::Mat> face_images_doubled;
	//	std::vector<std::string> id_doubled;
	//	for (int i=0; i<(int)face_images_.size(); i++)
	//	{
	//		face_images_doubled.push_back(face_images_[i]);
	//		id_doubled.push_back(id_[i]);
	//		cv::Mat temp;
	//		cv::flip(face_images_[i], temp, 1);
	//		face_images_doubled.push_back(temp);
	//		id_doubled.push_back(id_[i]);
	//	}

	// Do PCA
	if (people_detector_->PCA(&n_eigens_, eigen_vectors_, eigen_val_mat_, avg_image_, face_images_/*doubled*/, projected_train_face_mat_) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::PCA:" << std::endl;
		std::cerr << "\t ... Error while PCA.\n";
		return ipa_Utils::RET_FAILED;
	}

	// Calculate FaceClasses
	std::cout << "Debug: n_eigens: " << n_eigens_ << " id: " << id_.size() << "\n";
	if (people_detector_->CalculateFaceClasses(projected_train_face_mat_, id_/*doubled*/, &n_eigens_, face_class_avg_projections_, id_unique_, /*&person_classifier_*/0)
			& ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::PCA:" << std::endl;
		std::cerr << "\t ... Error while calculating FaceClasses.\n";
		return ipa_Utils::RET_FAILED;
	}

	run_pca_ = false;

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::saveAllData()
{
	//	try
	//	{
	//		fs::remove_all(path.c_str());
	//		fs::create_directory(path.c_str());
	//	}
	//	catch (const std::exception &ex)
	//	{
	//		std::cerr << "ERROR - PeopleDetector::SaveTrainingData():" << std::endl;
	//		std::cerr << "\t ... Exception catch of '" << ex.what() << "'" << std::endl;
	//	}

	if (saveTrainingData() != ipa_Utils::RET_OK)
		return ipa_Utils::RET_FAILED;
	if (saveRecognizerData() != ipa_Utils::RET_OK)
		return ipa_Utils::RET_FAILED;

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::saveTrainingData()
{
	std::string path = directory_ + "TrainingData/";
	std::string filename = "tdata.xml";
	std::string img_ext = ".bmp";

	std::ostringstream complete;
	complete << path << filename;

	cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
	if (!fileStorage.isOpened())
	{
		std::cout << "WARNING - PeopleDetector::SaveTrainingData:" << std::endl;
		std::cout << "\t ... Can't save training data.\n";
		return ipa_Utils::RET_OK;
	}

	// Ids
	fileStorage << "id_num" << (int)id_.size();
	for (int i = 0; i < (int)id_.size(); i++)
	{
		std::ostringstream tag;
		tag << "id_" << i;
		fileStorage << tag.str().c_str() << id_[i].c_str();
	}

	// Face images
	fileStorage << "faces_num" << (int)face_images_.size();
	for (int i = 0; i < (int)face_images_.size(); i++)
	{
		std::ostringstream img, shortname;
		img << path << i << img_ext;
		shortname << "TrainingData/" << i << img_ext;
		std::ostringstream tag;
		tag << "img_" << i;
		fileStorage << tag.str().c_str() << shortname.str().c_str();
		cv::imwrite(img.str().c_str(), face_images_[i]);
	}

	fileStorage.release();

	std::cout << "INFO - PeopleDetector::SaveTrainingData:" << std::endl;
	std::cout << "\t ... " << face_images_.size() << " images saved.\n";
	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::saveRecognizerData()
{
	std::string path = directory_ + "TrainingData/";
	std::string filename = "rdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
	if (!fileStorage.isOpened())
	{
		std::cout << "WARNING - PeopleDetector::saveRecognizerData:" << std::endl;
		std::cout << "\t ... Can't save training data.\n";
		return ipa_Utils::RET_OK;
	}

	// Number eigenvalues/eigenvectors
	fileStorage << "eigens_num" << n_eigens_;

	// Eigenvectors
	for (int i = 0; i < n_eigens_; i++)
	{
		std::ostringstream tag;
		tag << "ev_" << i;
		fileStorage << tag.str().c_str() << eigen_vectors_[i];
	}

	// Eigenvalue matrix
	fileStorage << "eigen_val_mat" << eigen_val_mat_;

	// Average image
	fileStorage << "avg_image" << avg_image_;

	// Projection coefficients of the training faces
	fileStorage << "projected_train_face_mat" << projected_train_face_mat_;

	// Unique Ids (id_unique_[i] stores the corresponding id to the average face coordinates in the face subspace in face_class_avg_projections_.row(i))
	fileStorage << "id_unique_num" << (int)id_unique_.size();
	for (int i = 0; i < (int)id_unique_.size(); i++)
	{
		std::ostringstream tag;
		tag << "id_unique_" << i;
		fileStorage << tag.str().c_str() << id_unique_[i].c_str();
	}

	// The average factors of the eigenvector decomposition from each face class
	fileStorage << "face_class_avg_projections" << face_class_avg_projections_;

	fileStorage.release();

	// save classifier
	std::string classifierFile = path + "svm.dat";
	//person_classifier_.save(classifierFile.c_str());

	std::cout << "INFO - PeopleDetector::saveRecognizerData:" << std::endl;
	std::cout << "\t ... recognizer data saved.\n";
	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::loadAllData()
{
	if (loadTrainingData(true) != ipa_Utils::RET_OK)
		return ipa_Utils::RET_FAILED;
	if (loadRecognizerData() != ipa_Utils::RET_OK)
		return ipa_Utils::RET_FAILED;

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::loadTrainingData(bool runPCA)
{
	std::string path = directory_ + "TrainingData/";
	std::string filename = "tdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	if (fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "WARNING - PeopleDetector::loadTrainingData:" << std::endl;
			std::cout << "\t ... Cant open " << complete.str() << ".\n";
			return ipa_Utils::RET_OK;
		}

		// Ids
		id_.clear();
		int id_num = (int)fileStorage["id_num"];
		for (int i = 0; i < id_num; i++)
		{
			std::ostringstream tag;
			tag << "id_" << i;
			id_.push_back((std::string)fileStorage[tag.str().c_str()]);
		}

		// Images
		face_images_.clear();
		int faces_num = (int)fileStorage["faces_num"];
		for (int i = 0; i < faces_num; i++)
		{
			std::ostringstream tag;
			tag << "img_" << i;
			std::string path = directory_ + (std::string)fileStorage[tag.str().c_str()];
			cv::Mat temp = cv::imread(path.c_str(), 0);
			face_images_.push_back(temp);
		}

		// set next free filename
		filename_ = faces_num;

		fileStorage.release();

		// Run PCA - now or later
		run_pca_ = true;
		if (runPCA)
			PCA();

		std::cout << "INFO - PeopleDetector::loadTrainingData:" << std::endl;
		std::cout << "\t ... " << faces_num << " images loaded.\n";
	}
	else
	{
		std::cerr << "ERROR - PeopleDetector::loadTrainingData:" << std::endl;
		std::cerr << "\t .... Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::loadRecognizerData()
{
	std::string path = directory_ + "TrainingData/";
	std::string filename = "rdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	if (fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "WARNING - PeopleDetector::loadRecognizerData:" << std::endl;
			std::cout << "\t ... Cant open " << complete.str() << ".\n";
			return ipa_Utils::RET_OK;
		}
		std::cout << "   loading recognizer data..." << std::endl;

		// Number eigenvalues/eigenvectors
		n_eigens_ = (int)fileStorage["eigens_num"];

		// Eigenvectors
		eigen_vectors_.clear();
		eigen_vectors_.resize(n_eigens_, cv::Mat());
		for (int i = 0; i < n_eigens_; i++)
		{
			std::ostringstream tag;
			tag << "ev_" << i;
			fileStorage[tag.str().c_str()] >> eigen_vectors_[i];
		}

		// Eigenvalue matrix
		eigen_val_mat_ = cv::Mat();
		fileStorage["eigen_val_mat"] >> eigen_val_mat_;

		// Average image
		avg_image_ = cv::Mat();
		fileStorage["avg_image"] >> avg_image_;

		// Projections of the training faces
		projected_train_face_mat_ = cv::Mat();
		fileStorage["projected_train_face_mat"] >> projected_train_face_mat_;

		// Unique Ids (id_unique_[i] stores the corresponding id to the average face coordinates in the face subspace in face_class_avg_projections_.row(i))
		id_unique_.clear();
		int idUniqueNum = (int)fileStorage["id_unique_num"];
		for (int i = 0; i < idUniqueNum; i++)
		{
			std::ostringstream tag;
			tag << "id_unique_" << i;
			id_unique_.push_back((std::string)fileStorage[tag.str().c_str()]);
		}

		// The average factors of the eigenvector decomposition from each face class
		face_class_avg_projections_ = cv::Mat();
		fileStorage["face_class_avg_projections"] >> face_class_avg_projections_;

		fileStorage.release();

		// load classifier
		std::string classifierFile = path + "svm.dat";
		//person_classifier_.load(classifierFile.c_str());

		// do not run PCA
		run_pca_ = false;

		std::cout << "INFO - PeopleDetector::loadRecognizerData:" << std::endl;
		std::cout << "\t ... recognizer data loaded.\n";
	}
	else
	{
		std::cerr << "ERROR - PeopleDetector::loadRecognizerData():" << std::endl;
		std::cerr << "\t .... Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::getEigenface(cv::Mat& eigenface, int index)
{
	cv::normalize(eigen_vectors_[index], eigenface, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::showAVGImage(cv::Mat& avgImage)
{
	if (!run_pca_ && face_images_.size() < 2)
	{
		std::cerr << "PeopleDetector::showAvgImage()" << std::endl;
		std::cerr << "No AVG image calculated" << std::endl;
		return 0;
	}

	cv::normalize(avg_image_, avgImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::saveRangeTrainImages(cv::Mat& xyz_image)
{
	std::string path = directory_ + "haarcascades/";
	std::string img_ext = ".jpg";
	cv::Mat xyzImage_8U3(xyz_image.size(), CV_8UC3); //IplImage* xyzImage_8U3 = cvCreateImage(cvGetSize(pc->GetXYZImage()), 8, 3);
	ipa_Utils::ConvertToShowImage(xyz_image, xyzImage_8U3, 3);

	for (int i = 0; i < (int)color_faces_.size(); i++)
	{
		cv::Mat xyzImage_8U3_resized(100, 100, CV_8UC3); //cvCreateImage(cvSize(100,100), 8, 3); 8=IPL_DEPTH_8U

		double scale = 1.6;
		cv::Rect rangeFace;
		rangeFace.height = (int)(color_faces_[i].height * scale);
		rangeFace.width = (int)(color_faces_[i].width * scale);
		rangeFace.x = color_faces_[i].x - ((rangeFace.width - color_faces_[i].width) / 2);
		rangeFace.y = color_faces_[i].y - ((rangeFace.height - color_faces_[i].height) / 2) - 10;

		cv::Mat xyzImage_8U3_roi = xyzImage_8U3(rangeFace);
		//cvSetImageROI(xyzImage_8U3, rangeFace);
		cv::resize(xyzImage_8U3_roi, xyzImage_8U3_resized, xyzImage_8U3_resized.size());

		std::ostringstream file;
		file << path << filename_ << img_ext;

		cv::imwrite(file.str().c_str(), xyzImage_8U3_resized);//cvSaveImage(file.str().c_str(), xyzImage_8U3_resized);
		filename_++;
	}

	return ipa_Utils::RET_OK;
}

//unsigned long CobFaceDetectionNodelet::getMeasurement(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
//{
//	cv::Mat color_image_8U3(shared_image_msg->height, shared_image_msg->width, CV_8UC3);
//	cv::Mat xyz_image_32F3(shared_image_msg->height, shared_image_msg->width, CV_32FC3);
//	float* f_ptr = 0;
//	const uint8_t* data_ptr = 0;
//	unsigned char* uc_ptr = 0;
//	unsigned int xyz_offset = shared_image_msg->fields[0].offset;
//	unsigned int rgb_offset = shared_image_msg->fields[3].offset;
//	size_t b_offset = 2*sizeof(unsigned char);
//	size_t g_offset = sizeof(unsigned char);
//	size_t r_offset = 0;
//	unsigned int col_times_3 = 0;
//	for (unsigned int row = 0; row < shared_image_msg->height; row++)
//	{
//		uc_ptr = color_image_8U3.ptr<unsigned char>(row);
//		f_ptr = xyz_image_32F3.ptr<float>(row);
//
//		data_ptr = &shared_image_msg->data[row * shared_image_msg->width * shared_image_msg->point_step];
//
//		for (unsigned int col = 0; col < shared_image_msg->width; col++)
//		{
//			col_times_3 = 3*col;
//			// Reorder incoming image channels
//			memcpy(&uc_ptr[col_times_3], &data_ptr[col * shared_image_msg->point_step + rgb_offset + b_offset], sizeof(unsigned char));
//			memcpy(&uc_ptr[col_times_3 + 1], &data_ptr[col * shared_image_msg->point_step + rgb_offset + g_offset], sizeof(unsigned char));
//			memcpy(&uc_ptr[col_times_3 + 2], &data_ptr[col * shared_image_msg->point_step + rgb_offset + r_offset], sizeof(unsigned char));
//
//			memcpy(&f_ptr[col_times_3], &data_ptr[col * shared_image_msg->point_step + xyz_offset], 3*sizeof(float));
//		}
//	}
//
//#ifdef __LINUX__
//	color_image_ = color_image_8U3;
//	range_image_ = xyz_image_32F3;
//#else
//	// Images are cloned within setter functions
//	colored_pc_->SetColorImage(color_image_8U3);
//	colored_pc_->SetXYZImage(xyz_image_32F3);
//#endif
//
////    		cv::Mat xyz_image_8U3;
////			ipa_Utils::ConvertToShowImage(colored_pc_->GetXYZImage(), xyz_image_8U3, 3);
////	    	cv::imshow("xyz data", xyz_image_8U3);
////	    	cv::imshow("color data", colored_pc_->GetColorImage());
////	    	cv::waitKey();
//
//	return ipa_Utils::RET_OK;
//}


unsigned long CobFaceDetectionNodelet::convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr,
		cv::Mat& color_image)
{
	try
	{
		color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return ipa_Utils::RET_FAILED;
	}
	color_image = color_image_ptr->image;

	return ipa_Utils::RET_OK;
}

unsigned long CobFaceDetectionNodelet::convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, cv::Mat& depth_image)
{
	pcl::PointCloud < pcl::PointXYZ > depth_cloud; // point cloud
	pcl::fromROSMsg(*shared_image_msg, depth_cloud);
	depth_image.create(depth_cloud.height, depth_cloud.width, CV_32FC3);
	uchar* depth_image_ptr = (uchar*)depth_image.data;
	for (int v = 0; v < (int)depth_cloud.height; v++)
	{
		int baseIndex = depth_image.step * v;
		for (int u = 0; u < (int)depth_cloud.width; u++)
		{
			int index = baseIndex + 3 * u * sizeof(float);
			float* data_ptr = (float*)(depth_image_ptr + index);
			pcl::PointXYZ point_xyz = depth_cloud(u, v);
			data_ptr[0] = point_xyz.x;
			data_ptr[1] = point_xyz.y;
			data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
			//if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
		}
	}
	return ipa_Utils::RET_OK;
}

inline std::string CobFaceDetectionNodelet::getLabel(int index)
{
	switch (index)
	{
	case -1:
		// Distance to face class is too high
		return "Unknown";
		break;
	case -2:
		// Distance to face space is too high
		return "No face";
		break;
	default:
		// Face classified
		return id_unique_[index];
	}
}

inline int abs(int x)
{
	return ((x < 0) ? -x : x);
}

void CobFaceDetectionNodelet::recognizeCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	// check if this is a training call
	if (train_continuous_server_running_ == true)
	{
		trainContinuousCallback(shared_image_msg, color_image_msg);
		return;
	}

	// check for disabled recognition
	if (recognize_server_running_ == false)
		return;

	// convert input to cv::Mat images
	// color
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);//color_image_ptr->image;

	// point cloud
	cv::Mat depth_image;
	convertPclMessageToMat(shared_image_msg, depth_image);

#ifdef __LINUX__
	color_image_ = color_image;
	range_image_ = depth_image;
#else
	// convert point cloud and color image to colored point cloud
	colored_pc_->SetColorImage(color_image);
	colored_pc_->SetXYZImage(depth_image);
#endif
	//getMeasurement(shared_image_msg, color_image_msg);

	// todo: should not be necessary here (e.g. set a trained flag)
	PCA();

	if (eigen_vectors_.size() < 1)
	{
		std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Less than two images are trained.\n";
		return;
	}

	//DWORD start = timeGetTime();
	cv::Mat colorImage_8U3;
#ifdef __LINUX__
	detectFaces(range_image_, color_image_);
	colorImage_8U3 = color_image_.clone();
#else
	detectFaces(colored_pc_->GetXYZImage(), colored_pc_->GetColorImage());
	colored_pc_->GetColorImage().copyTo(colorImage_8U3);
#endif

	std::vector<int> index;
	if (do_recognition_ == true)
	{
		recognizeFace(color_image, index);
		//std::cout << "INFO - PeopleDetector::Recognize:" << std::endl;
	}
	else
		for (int i = 0; i < (int)color_faces_.size(); i++)
			index.push_back(-1);
	//std::cout << "\t ... Recognize Time: " << (timeGetTime() - start) << std::endl;

	// publish face positions
	cob_perception_msgs::DetectionArray facePositionMsg;
	// image dimensions
	// time stamp
	facePositionMsg.header.stamp = shared_image_msg->header.stamp;
	//facePositionMsg.detections.reserve(color_faces_.size());
	// add all range faces that do not contain a face detection in the color image
	for (int i = 0; i < (int)range_faces_.size(); i++)
	{
		// if no color face was detected in that region, add it to the published list
		if (range_face_indices_with_color_face_detection_.find(i) == range_face_indices_with_color_face_detection_.end())
		{
			cv::Rect face = range_faces_[i];

			// 2D image coordinates
			cob_perception_msgs::Detection det;
			det.header = shared_image_msg->header;
			det.mask.roi.x = face.x;
			det.mask.roi.y = face.y;
			det.mask.roi.width = face.width;
			det.mask.roi.height = face.height;
			float center2Dx = face.x + face.width * 0.5f;
			float center2Dy = face.y + face.height * 0.5f;

			// 3D world coordinates (and verify that the read pixel contained valid coordinates, otherwise search for valid pixel in neighborhood)
			geometry_msgs::Point* point = &(det.pose.pose.position);
			cv::Point3f p;
			bool validCoordinates = false;
			for (int d = 0; (d < 6 && !validCoordinates); d++)
			{
				for (int v = -d; (v <= d && !validCoordinates); v++)
				{
					for (int u = -d; (u <= d && !validCoordinates); u++)
					{
						if (abs(v) != d && abs(u) != d)
							continue;

						p = depth_image.at<cv::Point3f>(center2Dy + v, center2Dx + u);
						point->x = p.x;
						point->y = p.y;
						point->z = p.z;

						if (!isnan(point->x) && !isnan(point->y) && point->z != 0.f)
							validCoordinates = true;
					}
				}
			}
			if (validCoordinates == false)
				continue;

			// person ID
			det.label = "UnknownRange";

			// origin of detection
			det.detector = "range";

			facePositionMsg.detections.push_back(det);
		}
	}

	// add all color face detections
	for (int i = 0; i < (int)color_faces_.size(); i++)
	{
		cv::Rect face = color_faces_[i];

		// 2D image coordinates
		cob_perception_msgs::Detection det;
		det.mask.roi.x = face.x;
		det.mask.roi.y = face.y;
		det.mask.roi.width = face.width;
		det.mask.roi.height = face.height;
		float center2Dx = face.x + face.width * 0.5f;
		float center2Dy = face.y + face.height * 0.5f;

		// 3D world coordinates
		//		cv::Point3f p = depth_image.at<cv::Point3f>(center2Dy, center2Dx);
		//		geometry_msgs::Point* point = &(det.pose.pose.position);
		//		point->x = p.x; point->y=p.y; point->z=p.z;

		// 3D world coordinates (and verify that the read pixel contained valid coordinates, otherwise search for valid pixel in neighborhood)
		geometry_msgs::Point* point = &(det.pose.pose.position);
		cv::Point3f p;
		bool validCoordinates = false;
		for (int d = 0; (d < 6 && !validCoordinates); d++)
		{
			for (int v = -d; (v <= d && !validCoordinates); v++)
			{
				for (int u = -d; (u <= d && !validCoordinates); u++)
				{
					if (abs(v) != d && abs(u) != d)
						continue;

					p = depth_image.at<cv::Point3f>(center2Dy + v, center2Dx + u);
					point->x = p.x;
					point->y = p.y;
					point->z = p.z;

					if (!isnan(point->x) && !isnan(point->y) && point->z != 0.f)
						validCoordinates = true;
				}
			}
		}
		if (validCoordinates == false)
			continue;

		// person ID
		det.label = getLabel(index[i]);

		// origin of detection
		det.detector = "color";

		facePositionMsg.detections.push_back(det);
	}
	face_position_publisher_.publish(facePositionMsg);

	//	std_msgs::Float32MultiArrayPtr facePositionMsg(new std_msgs::Float32MultiArray);
	//	const int dataBlockSize = 8;
	//	std::vector<float> data(dataBlockSize*color_faces_.size());
	//	for(int i=0; i<(int)color_faces_.size(); i++)
	//	{
	//		cv::Rect face = color_faces_[i];
	//		int baseIndex = i*dataBlockSize;
	//
	//		// 2D image coordinates
	//		data[baseIndex]=face.x; data[baseIndex+1]=face.y; data[baseIndex+2]=face.width; data[baseIndex+3]=face.height;
	//		float center2Dx = face.x + face.width*0.5f;
	//		float center2Dy = face.y + face.height*0.5f;
	//
	//		// 3D world coordinates
	//		cv::Point3f p = depth_image.at<cv::Point3f>(center2Dy, center2Dx);
	//		data[baseIndex+4]=p.x; data[baseIndex+5]=p.y; data[baseIndex+6]=p.z;
	//
	//		// person ID
	//		data[baseIndex+7]=index[i];
	//	}
	//	facePositionMsg->data = data;
	//	std::vector<std_msgs::MultiArrayDimension> dim(2, std_msgs::MultiArrayDimension());
	//	dim[0].label = "faces";       dim[0].size = color_faces_.size();  	dim[0].stride = data.size();
	//	dim[1].label = "coordinates"; dim[1].size = dataBlockSize;          dim[1].stride = dataBlockSize;
	//	std_msgs::MultiArrayLayout layout;
	//	layout.dim = dim;
	//	layout.data_offset = 0;
	//	facePositionMsg->layout = layout;
	//	face_position_publisher_->publish(facePositionMsg);

	// display results
	if (display_ == true)
	{
		for (int i = 0; i < (int)range_faces_.size(); i++)
		{
			cv::Rect face = range_faces_[i];
			cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
		}

		for (int i = 0; i < (int)color_faces_.size(); i++)
		{
			cv::Rect face = color_faces_[i];

			cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

			std::stringstream tmp;
			switch (index[i])
			{
			case -1:
				// Distance to face class is too high
				tmp << "Unknown";
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				break;
			case -2:
				// Distance to face space is too high
				tmp << "No face";
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				break;
			default:
				// Face classified
				tmp << id_unique_[index[i]].c_str();
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x, face.y + face.height + 25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
			}
		}
		//cv::imshow("Face Detector", colorImage_8U3);
		//cv::waitKey(10);
		// publish topic
		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = colorImage_8U3;
		cv_ptr.encoding = "bgr8";
		face_detection_image_pub_.publish(cv_ptr.toImageMsg());
	}
}

void CobFaceDetectionNodelet::trainContinuousCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	if (train_continuous_server_running_ == false)
		return;

	// convert input to cv::Mat images
	// color
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;
	convertColorImageMessageToMat(color_image_msg, color_image_ptr, color_image);//color_image_ptr->image;

	// point cloud
	cv::Mat depth_image;
	convertPclMessageToMat(shared_image_msg, depth_image);

#ifdef __LINUX__
	color_image_ = color_image;
	range_image_ = depth_image;
#else
	// convert point cloud and color image to colored point cloud
	colored_pc_->SetColorImage(color_image);
	colored_pc_->SetXYZImage(depth_image);
#endif
	//getMeasurement(shared_image_msg, color_image_msg);

	cv::Mat colorImage_8U3;
#ifdef __LINUX__
	detectFaces(range_image_, color_image_);
	colorImage_8U3 = color_image_.clone();
#else
	detectFaces(colored_pc_->GetXYZImage(), colored_pc_->GetColorImage());
	colored_pc_->GetColorImage().copyTo(colorImage_8U3);
#endif

	for (int i = 0; i < (int)color_faces_.size(); i++)
	{
		cv::Rect face = color_faces_[i];
		cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB( 0, 255, 0 ), 2, 8, 0);
	}

	//cv::imshow("Face Recognizer Training", colorImage_8U3);
	//cv::waitKey(10);
	// publish topic
	cv_bridge::CvImage cv_ptr;
	cv_ptr.image = colorImage_8U3;
	cv_ptr.encoding = "bgr8";
	face_detection_image_pub_.publish(cv_ptr.toImageMsg());

	// capture image if triggered by an action
	// secure this section with a mutex
	boost::timed_mutex::scoped_timed_lock lock(action_mutex_, boost::posix_time::milliseconds(10));
	if (lock.owns_lock() && capture_training_face_ == true)
	{
		capture_training_face_ = false;

		// Check if there is more than one face detected
		if (color_faces_.size() > 1)
		{
			std::cout << "WARNING - CobFaceDetectionNodelet::trainContinuousCallback:" << std::endl;
			std::cout << "\t ... More than one faces are detected in image. Please try again." << std::endl;
			return;
		}

		// Check if there is less than one face detected
		if (color_faces_.size() < 1)
		{
			std::cout << "WARNING - CobFaceDetectionNodelet::trainContinuousCallback:" << std::endl;
			std::cout << "\t ... Less than one faces are detected in image. Please try again." << std::endl;
			return;
		}

#ifdef __LINUX__
		addFace(color_image_, current_training_id_);
#else
		addFace(colored_pc_->GetColorImage(), current_training_id_);
#endif
		number_training_images_captured_++;
		std::cout << "INFO - CuiPeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Face captured (" << face_images_.size() << ")." << std::endl;
	}
}

unsigned long CobFaceDetectionNodelet::loadParameters(const char* iniFileName)
{
	/// Load parameters from file
	TiXmlDocument* p_configXmlDocument = new TiXmlDocument(iniFileName);
	if (!p_configXmlDocument->LoadFile())
	{
		std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
		std::cerr << "\t ... Error while loading xml configuration file" << std::endl;
		std::cerr << "\t ... (Check filename and syntax of the file):" << std::endl;
		std::cerr << "\t ... '" << iniFileName << "'" << std::endl;
		return ipa_Utils::RET_FAILED;
	}
	std::cout << "INFO - PeopleDetector::LoadParameters:" << std::endl;
	std::cout << "\t ... Parsing xml configuration file:" << std::endl;
	std::cout << "\t ... '" << iniFileName << "'" << std::endl;

	if (p_configXmlDocument)
	{

		//************************************************************************************
		//	BEGIN PeopleDetector
		//************************************************************************************
		// Tag element "PeopleDetector" of Xml Inifile
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement("PeopleDetector");

		if (p_xmlElement_Root)
		{

			//************************************************************************************
			//	BEGIN PeopleDetector->adaBoost
			//************************************************************************************
			// Tag element "adaBoost" of Xml Inifile
			TiXmlElement *p_xmlElement_Root_OD = NULL;
			p_xmlElement_Root_OD = p_xmlElement_Root->FirstChildElement("adaBoost");

			if (p_xmlElement_Root_OD)
			{

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Faces_increase_search_scale
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Faces_increase_search_scale");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_faces_increase_search_scale) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Faces_increase_search_scale'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Faces_increase_search_scale'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Faces_drop_groups
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Faces_drop_groups");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_faces_drop_groups) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Faces_drop_groups'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Faces_drop_groups'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Faces_min_search_scale_x
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Faces_min_search_scale_x");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_faces_min_search_scale_x) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Faces_min_search_scale_x'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Faces_min_search_scale_x'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Faces_min_search_scale_y
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Faces_min_search_scale_y");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_faces_min_search_scale_y) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Faces_min_search_scale_y'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Faces_min_search_scale_y'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Range_increase_search_scale
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Range_increase_search_scale");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_range_increase_search_scale) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Range_increase_search_scale'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Range_increase_search_scale'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Range_drop_groups
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Range_drop_groups");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_range_drop_groups) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Range_drop_groups'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Range_drop_groups'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Range_min_search_scale_x
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Range_min_search_scale_x");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_range_min_search_scale_x) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Range_min_search_scale_x'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Range_min_search_scale_x'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}

				//************************************************************************************
				//	BEGIN PeopleDetector->adaBoost->Range_min_search_scale_y
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Range_min_search_scale_y");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &people_detector_->m_range_min_search_scale_y) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Range_min_search_scale_y'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Range_min_search_scale_y'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}
			}
			//************************************************************************************
			//	END CameraDataViewerControlFlow->adaBoost
			//************************************************************************************

			//************************************************************************************
			//	BEGIN PeopleDetector->eigenfaces
			//************************************************************************************
			// Tag element "adaBoost" of Xml Inifile
			p_xmlElement_Root_OD = NULL;
			p_xmlElement_Root_OD = p_xmlElement_Root->FirstChildElement("eigenfaces");

			if (p_xmlElement_Root_OD)
			{

				//************************************************************************************
				//	BEGIN PeopleDetector->eigenfaces->Threshold_Face_Class
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Threshold_Face_Class");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &threshold_) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Threshold_Face_Class'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Threshold_Face_Class'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}
				//************************************************************************************
				//	BEGIN PeopleDetector->eigenfaces->Threshold_Facespace
				//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement("Threshold_Facespace");

				if (p_xmlElement_Child)
				{
					// read and save value of attribute
					if (p_xmlElement_Child->QueryValueAttribute("value", &threshold_fs_) != TIXML_SUCCESS)
					{
						std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
						std::cerr << "\t ... Can't find attribute 'value' of tag 'Threshold_Facespace'" << std::endl;
						return ipa_Utils::RET_FAILED;
					}
				}
				else
				{
					std::cerr << "ERROR - PeopleDetector::LoadParameters:" << std::endl;
					std::cerr << "\t ... Can't find tag 'Threshold_Facespace'" << std::endl;
					return ipa_Utils::RET_FAILED;
				}
				//************************************************************************************
				//	END CameraDataViewerControlFlow->eigenfaces
				//************************************************************************************

			}
			else
			{
				std::cerr << "ERROR - CameraDataViewerControlFlow::LoadParameters:" << std::endl;
				std::cerr << "\t ... Can't find tag 'ObjectDetectorParameters'" << std::endl;
				return ipa_Utils::RET_FAILED;
			}

		}

		//************************************************************************************
		//	END ObjectDetector
		//************************************************************************************
		else
		{
			std::cerr << "ERROR - CameraDataViewerControlFlow::LoadParameters:" << std::endl;
			std::cerr << "\t ... Can't find tag 'ObjectDetector'" << std::endl;
			return ipa_Utils::RET_FAILED;
		}
	}
	return ipa_Utils::RET_OK;
}

//#######################
//#### main programm ####
//int main(int argc, char** argv)
//{
//	// Initialize ROS, specify name of node
//	ros::init(argc, argv, "people_detection");
//
//	// Create a handle for this node, initialize node
//	ros::NodeHandle nh;
//
//	// Create people detection node class instance
//	CobFaceDetectionNodelet people_detection_node(nh);
//
//	// Initialize people detection node
//	if (people_detection_node.init() != ipa_Utils::RET_OK)
//		return 0;
//
//	// Create action nodes
//	//DetectObjectsAction detect_action_node(object_detection_node, nh);
//	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
//	//TrainObjectAction train_object_node(object_detection_node, nh);
//
//	ros::spin();
//
//	return 0;
//}

