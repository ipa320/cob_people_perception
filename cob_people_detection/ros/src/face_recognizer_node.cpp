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

#ifdef __LINUX__
#include "cob_people_detection/face_recognizer_node.h"
#include "cob_vision_utils/GlobalDefines.h"
#include "cob_people_detection/face_detection_message_helper.h"
#else
#endif

// OpenCV
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// Boost
#include <boost/shared_ptr.hpp>

#include <sys/time.h>

using namespace ipa_PeopleDetector;

FaceRecognizerNode::FaceRecognizerNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	//	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";
	//	classifier_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";
	// Parameters
	bool norm_illumination;
	bool norm_align;
	bool norm_extreme_illumination;
	int norm_size; // Desired width and height of the Eigenfaces (=eigenvectors).
	int feature_dimension; // Number of eigenvectors per person to identify -> controls the total number of eigenvectors
	double threshold_facespace; // Threshold to facespace
	double threshold_unknown; // Threshold to detect unknown faces
	int metric; // metric for nearest neighbor search in face space: 0 = Euklidean, 1 = Mahalanobis, 2 = Mahalanobis Cosine
	bool debug; // enables some debug outputs
	int recognition_method; // choose subspace method
	bool use_unknown_thresh; // use threshold for unknown faces
	bool use_depth; // use depth for recognition
	std::vector < std::string > identification_labels_to_recognize; // a list of labels of persons that shall be recognized
	std::cout << "\n--------------------------\nFace Recognizer Parameters:\n--------------------------\n";
	//if(!node_handle_.getParam("~data_directory", data_directory_)) std::cout<<"PARAM NOT AVAILABLE"<<std::endl;
	if (!node_handle_.getParam("data_storage_directory", data_directory_))
		std::cout << "PARAM NOT AVAILABLE" << std::endl;
	std::cout << "data_directory = " << data_directory_ << "\n";
	node_handle_.param("enable_face_recognition", enable_face_recognition_, true);
	std::cout << "enable_face_recognition = " << enable_face_recognition_ << "\n";
	node_handle_.param("feature_dimension", feature_dimension, 10);
	std::cout << "feature dimension = " << feature_dimension << "\n";
	node_handle_.param("threshold_facespace", threshold_facespace, 10000.0);
	std::cout << "threshold_facespace = " << threshold_facespace << "\n";
	node_handle_.param("threshold_unknown", threshold_unknown, 1000.0);
	std::cout << "threshold_unknown = " << threshold_unknown << "\n";
	node_handle_.param("metric", metric, 0);
	std::cout << "metric = " << metric << "\n";
	node_handle_.param("debug", debug, false);
	std::cout << "debug = " << debug << "\n";
	node_handle_.param("recognition_method", recognition_method, 3);
	std::cout << "recognition method: " << recognition_method << "\n";
	node_handle_.param("use_unknown_thresh", use_unknown_thresh, true);
	std::cout << " use use unknown thresh: " << use_unknown_thresh << "\n";
	node_handle_.param("use_depth", use_depth, true);
	std::cout << " use depth: " << use_depth << "\n";
	node_handle_.param("display_timing", display_timing_, false);
	std::cout << "display_timing = " << display_timing_ << "\n";
	node_handle_.param("norm_size", norm_size, 100);
	std::cout << "norm_size = " << norm_size << "\n";
	node_handle_.param("norm_illumination", norm_illumination, true);
	std::cout << "norm_illumination = " << norm_illumination << "\n";
	node_handle_.param("norm_align", norm_align, false);
	std::cout << "norm_align = " << norm_align << "\n";
	node_handle_.param("norm_extreme_illumination", norm_extreme_illumination, false);
	std::cout << "norm_extreme_illumination = " << norm_extreme_illumination << "\n";
	node_handle_.param("debug", debug, false);
	std::cout << "debug = " << debug << "\n";
	node_handle_.param("use_depth", use_depth, false);
	std::cout << "use depth: " << use_depth << "\n";
	// todo: make parameters for illumination and alignment normalization on/off

	std::cout << "identification_labels_to_recognize: \n";
	XmlRpc::XmlRpcValue identification_labels_to_recognize_list;
	node_handle_.getParam("identification_labels_to_recognize", identification_labels_to_recognize_list);
	if (identification_labels_to_recognize_list.getType() == XmlRpc::XmlRpcValue::TypeArray)
	{
		identification_labels_to_recognize.resize(identification_labels_to_recognize_list.size());
		for (int i = 0; i < identification_labels_to_recognize_list.size(); i++)
		{
			ROS_ASSERT(identification_labels_to_recognize_list[i].getType() == XmlRpc::XmlRpcValue::TypeString);
			identification_labels_to_recognize[i] = static_cast<std::string>(identification_labels_to_recognize_list[i]);
		}
	}

	// initialize face recognizer
	unsigned long return_value = face_recognizer_.init(data_directory_, norm_size, norm_illumination, norm_align, norm_extreme_illumination, metric, debug,
			identification_labels_to_recognize, recognition_method, feature_dimension, use_unknown_thresh, use_depth);
	if (return_value == ipa_Utils::RET_FAILED)
	{
		ROS_ERROR("Recognition model not trained.");
	}
	else if (return_value == ipa_Utils::RET_OK)
	{
		std::cout << "Recognition model trained or loaded for:\n";
		for (unsigned int i = 0; i < identification_labels_to_recognize.size(); i++)
			std::cout << "   - " << identification_labels_to_recognize[i] << std::endl;
	}
	// launch LoadModel server
	load_model_server_ = new LoadModelServer(node_handle_, "load_model_server", boost::bind(&FaceRecognizerNode::loadModelServerCallback, this, _1), false);
	load_model_server_->start();
	ROS_INFO("FaceRecognizerNode initialized.");


	// advertise topics
	face_recognition_publisher_ = node_handle_.advertise<cob_perception_msgs::DetectionArray>("face_recognitions", 1);

	// subscribe to head detection topic
	face_position_subscriber_ = nh.subscribe("face_positions", 1, &FaceRecognizerNode::facePositionsCallback, this);
}

FaceRecognizerNode::~FaceRecognizerNode(void)
{
	if (load_model_server_ != 0)
		delete load_model_server_;
}

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(const sensor_msgs::Image* const )
{
}

//void FaceRecognizerNode::facePositionsCallback(const cob_perception_msgs::ColorDepthImageCropArray::ConstPtr& face_positions)
//{
//	// receive head and face positions and recognize faces in the face region, finally publish detected and recognized faces
//
//	// --- convert color image patches of head regions and contained face bounding boxes ---
//	cv_bridge::CvImageConstPtr cv_ptr;
//	std::vector<cv::Mat> heads_color_images;
//	heads_color_images.resize(face_positions->cdia.head_detections.size());
//	std::vector<cv::Mat> heads_depth_images;
//	heads_depth_images.resize(face_positions->cdia.head_detections.size());
//
//	std::vector< std::vector<cv::Rect> > face_bounding_boxes;
//	std::vector< std::vector<cv::Rect> > crop_bounding_boxes;
//	face_bounding_boxes.resize(face_positions->cdia.head_detections.size());
//	crop_bounding_boxes.resize(face_positions->cdia.head_detections.size());
//  cv::Rect bb=cv::Rect(0,0,160,160);
//
//	std::vector<cv::Rect> head_bounding_boxes;
//	head_bounding_boxes.resize(face_positions->cdia.head_detections.size());
//
//	for (unsigned int i=0; i<face_positions->cdia.head_detections.size(); i++)
//	{
//		// color image
//		if (enable_face_recognition_ == true)
//		{
//			sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->cdia.head_detections[i].color_image), voidDeleter);
//			try
//			{
//				cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::BGR8);
//			}
//			catch (cv_bridge::Exception& e)
//			{
//				ROS_ERROR("cv_bridge exception: %s", e.what());
//				return;
//			}
//			heads_color_images[i] = cv_ptr->image;
//		}
//
//		// depth image
//		sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->cdia.head_detections[i].depth_image), voidDeleter);
//		try
//		{
//			cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::TYPE_32FC3);
//		}
//		catch (cv_bridge::Exception& e)
//		{
//			ROS_ERROR("cv_bridge exception: %s", e.what());
//			return;
//		}
//		heads_depth_images[i] = cv_ptr->image;
//
//		// face bounding boxes
//		face_bounding_boxes[i].resize(face_positions->cdia.head_detections[i].face_detections.size());
//		crop_bounding_boxes[i].resize(face_positions->cdia.head_detections[i].face_detections.size());
//		for (uint j=0; j<face_bounding_boxes[i].size(); j++)
//		{
//			const cob_perception_msgs::Rect& source_rect = face_positions->cdia.head_detections[i].face_detections[j];
//			cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
//			face_bounding_boxes[i][j] = rect;
//			crop_bounding_boxes[i][j] = bb;
//		}
//
//		// head bounding box
//		const cob_perception_msgs::Rect& source_rect = face_positions->cdia.head_detections[i].head_detection;
//		cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
//		head_bounding_boxes[i] = rect;
//	}
//
////--------------------------------------------------------------------
////--------------------------------------------------------------------
//	std::vector<cv::Mat> crops;
//	//std::vector<std::vector<cv::Rect> > crop_bounding_boxes;
//	crops.resize(face_positions->crops.size());
//    for(int k=0;k<face_positions->crops.size();k++)
//    {
//      //tenpora TODO:
//			sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->crops[k]), voidDeleter);
//			try
//			{
//				cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::BGR8);
//			}
//			catch (cv_bridge::Exception& e)
//			{
//				ROS_ERROR("cv_bridge exception: %s", e.what());
//				return;
//			}
//			crops[k]= cv_ptr->image;
//      //temporary TODO
//      //crop_bounding_boxes[k][0]=bb;
//
//    }
////--------------------------------------------------------------------
////--------------------------------------------------------------------
//
//
//	// --- face recognition ---
//	std::vector< std::vector<std::string> > identification_labels;
//	bool identification_failed = false;
//	if (enable_face_recognition_ == true)
//	{
//		// recognize faces
//		//unsigned long result_state = face_recognizer_.recognizeFaces(heads_color_images, face_bounding_boxes, identification_labels);
//		unsigned long result_state = face_recognizer_.recognizeFaces(crops,crop_bounding_boxes, identification_labels);
//		if (result_state == ipa_Utils::RET_FAILED)
//		{
//			ROS_ERROR("FaceRecognizerNode::face_positions_callback: Please load a face recognition model at first.");
//			identification_failed = true;
//		}
//	}
//	if (enable_face_recognition_ == false || identification_failed == true)
//	{
//		// label all image unknown if face recognition disabled
//		identification_labels.resize(face_positions->cdia.head_detections.size());
//		for (uint i=0; i<identification_labels.size(); i++)
//		{
//			identification_labels[i].resize(face_positions->cdia.head_detections[i].face_detections.size());
//			for (uint j=0; j<identification_labels[i].size(); j++)
//				identification_labels[i][j] = "Unknown";
//		}
//	}
//
//	// --- publish detection message ---
//	cob_perception_msgs::DetectionArray detection_msg;
//	detection_msg.header = face_positions->header;
//
//	// prepare message
//	for (int head=0; head<(int)head_bounding_boxes.size(); head++)
//	{
//		if (face_bounding_boxes[head].size() == 0)
//		{
//			// no faces detected in head region -> publish head position
//			cob_perception_msgs::Detection det;
//			cv::Rect& head_bb = head_bounding_boxes[head];
//			// set 3d position of head's center
//			bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], 0.5*(float)head_bb.width, 0.5*(float)head_bb.height, det.pose.pose.position, 6);
//			if (valid_3d_position==false)
//				continue;
//			// write bounding box
//			det.mask.roi.x = head_bb.x;           det.mask.roi.y = head_bb.y;
//			det.mask.roi.width = head_bb.width;   det.mask.roi.height = head_bb.height;
//			// set label
//			det.label="UnknownHead";
//			// set origin of detection
//			det.detector = "head";
//			// header
//			det.header = face_positions->header;
//			// add to message
//			detection_msg.detections.push_back(det);
//		}
//		else
//		{
//			// process all faces in head region
//			for (int face=0; face<(int)face_bounding_boxes[head].size(); face++)
//			{
//				cob_perception_msgs::Detection det;
//				cv::Rect& head_bb = head_bounding_boxes[head];
//				cv::Rect& face_bb = face_bounding_boxes[head][face];
//				// set 3d position of head's center
//				bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], face_bb.x+0.5*(float)face_bb.width, face_bb.y+0.5*(float)face_bb.height, det.pose.pose.position, 6);
//				if (valid_3d_position==false)
//					continue;
//				// write bounding box
//				det.mask.roi.x = head_bb.x+face_bb.x; det.mask.roi.y = head_bb.y+face_bb.y;
//				det.mask.roi.width = face_bb.width;   det.mask.roi.height = face_bb.height;
//				// set label
//				det.label=identification_labels[head][face];
//				// set origin of detection
//				det.detector = "face";
//				// header
//				det.header = face_positions->header;
//				// add to message
//				detection_msg.detections.push_back(det);
//			}
//		}
//	}
//
//	// publish message
//	face_recognition_publisher_.publish(detection_msg);
//}
void FaceRecognizerNode::facePositionsCallback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& face_positions)
{
	//	Timer tim;
	//	tim.start();

	// receive head and face positions and recognize faces in the face region, finally publish detected and recognized faces

	// --- convert color image patches of head regions and contained face bounding boxes ---
	cv_bridge::CvImageConstPtr cv_ptr;
	std::vector<cv::Mat> heads_color_images(face_positions->head_detections.size());
	std::vector<cv::Mat> heads_depth_images(face_positions->head_detections.size());
	std::vector<std::vector<cv::Rect> > face_bounding_boxes(face_positions->head_detections.size());
	std::vector<cv::Rect> head_bounding_boxes(face_positions->head_detections.size());
	for (unsigned int i = 0; i < face_positions->head_detections.size(); i++)
	{
		// color image
		if (enable_face_recognition_ == true)
		{
			sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->head_detections[i].color_image), voidDeleter);
			try
			{
				cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::BGR8);
			} catch (cv_bridge::Exception& e)
			{
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
			}
			heads_color_images[i] = cv_ptr->image;
		}

		// depth image
		sensor_msgs::ImageConstPtr msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(face_positions->head_detections[i].depth_image), voidDeleter);
		try
		{
			cv_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::TYPE_32FC3);
		} catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		heads_depth_images[i] = cv_ptr->image;

		// face bounding boxes
		face_bounding_boxes[i].resize(face_positions->head_detections[i].face_detections.size());
		for (uint j = 0; j < face_bounding_boxes[i].size(); j++)
		{
			const cob_perception_msgs::Rect& source_rect = face_positions->head_detections[i].face_detections[j];
			cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
			face_bounding_boxes[i][j] = rect;
		}

		// head bounding box
		const cob_perception_msgs::Rect& source_rect = face_positions->head_detections[i].head_detection;
		cv::Rect rect(source_rect.x, source_rect.y, source_rect.width, source_rect.height);
		head_bounding_boxes[i] = rect;
	}

	// --- face recognition ---
	std::vector<std::vector<std::string> > identification_labels;
	bool identification_failed = false;
	if (enable_face_recognition_ == true)
	{
		// recognize faces
		//unsigned long result_state = face_recognizer_.recognizeFaces(heads_color_images, face_bounding_boxes, identification_labels);

		//timeval t1,t2;
		//gettimeofday(&t1,NULL);
		unsigned long result_state = face_recognizer_.recognizeFaces(heads_color_images, heads_depth_images, face_bounding_boxes, identification_labels);
		//gettimeofday(&t2,NULL);
		//std::cout<<(t2.tv_sec - t1.tv_sec) * 1000.0<<std::endl;
		if (result_state == ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("FaceRecognizerNode::face_positions_callback: Please load a face recognition model at first.");
			identification_failed = true;
		}
	}
	if (enable_face_recognition_ == false || identification_failed == true)
	{
		// label all image unknown if face recognition disabled
		identification_labels.resize(face_positions->head_detections.size());
		for (uint i = 0; i < identification_labels.size(); i++)
		{
			identification_labels[i].resize(face_positions->head_detections[i].face_detections.size());
			for (uint j = 0; j < identification_labels[i].size(); j++)
				identification_labels[i][j] = "Unknown";
		}
	}

	// --- publish detection message ---
	FaceDetectionMessageHelper face_detection_message_helper;
	cob_perception_msgs::DetectionArray detection_msg;
	face_detection_message_helper.prepareCartesionDetectionMessage(detection_msg, face_positions->header, heads_depth_images, head_bounding_boxes,
			face_bounding_boxes, &identification_labels);
	// publish message
	face_recognition_publisher_.publish(detection_msg);

	if (display_timing_ == true)
		ROS_INFO("%d FaceRecognition: Time stamp of pointcloud message: %f. Delay: %f.", face_positions->header.seq, face_positions->header.stamp.toSec(),
				ros::Time::now().toSec() - face_positions->header.stamp.toSec());
	//	ROS_INFO("Face recognition took %f ms", tim.getElapsedTimeInMilliSec());
}

bool FaceRecognizerNode::determine3DFaceCoordinates(cv::Mat& depth_image, int center2Dx, int center2Dy, geometry_msgs::Point& center3D, int search_radius)
{
	// 3D world coordinates (and verify that the read pixel contained valid coordinates, otherwise search for valid pixel in neighborhood)
	cv::Point3f p;
	bool valid_coordinates = false;
	for (int d = 0; (d < search_radius && !valid_coordinates); d++)
	{
		for (int v = -d; (v <= d && !valid_coordinates); v++)
		{
			for (int u = -d; (u <= d && !valid_coordinates); u++)
			{
				if ((abs(v) != d && abs(u) != d) || center2Dx + u < 0 || center2Dx + u >= depth_image.cols || center2Dy + v < 0 || center2Dy + v >= depth_image.rows)
					continue;

				p = depth_image.at<cv::Point3f>(center2Dy + v, center2Dx + u);
				if (!isnan(p.x) && !isnan(p.y) && p.z != 0.f)
				{
					valid_coordinates = true;
					center3D.x = p.x;
					center3D.y = p.y;
					center3D.z = p.z;
				}
			}
		}
	}

	return valid_coordinates;
}

void FaceRecognizerNode::loadModelServerCallback(const cob_people_detection::loadModelGoalConstPtr& goal)
{
	// read list of labels of persons that shall be recognized from goal message
	std::vector < std::string > identification_labels_to_recognize(goal->labels.size());
	for (int i = 0; i < (int)goal->labels.size(); i++)
		identification_labels_to_recognize[i] = goal->labels[i];

	// load the corresponding recognition model
	unsigned long result_state = face_recognizer_.loadRecognitionModel(identification_labels_to_recognize);

	cob_people_detection::loadModelResult result;
	if (result_state == ipa_Utils::RET_OK)
		load_model_server_->setSucceeded(result, "Model successfully loaded.");
	else
		load_model_server_->setAborted(result, "Loading new model failed.");
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "face_recognizer");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh("~");

	// Create FaceRecognizerNode class instance
	FaceRecognizerNode face_recognizer_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
