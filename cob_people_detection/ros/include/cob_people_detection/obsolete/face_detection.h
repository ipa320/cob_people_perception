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

#ifndef _FACE_DETECTION_
#define _FACE_DETECTION_

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

#include <actionlib/server/simple_action_server.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <std_msgs/Float32MultiArray.h>
#include <cob_perception_msgs/DetectionArray.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// services
#include <cob_people_detection/Recognition.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/TrainContinuousAction.h>
#include <cob_people_detection/TrainCaptureSampleAction.h>
#include <cob_people_detection/RecognizeAction.h>
#include <cob_people_detection/LoadAction.h>
#include <cob_people_detection/SaveAction.h>
#include <cob_people_detection/ShowAction.h>

// opencv
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// tiny xml
#include "tinyxml.h"

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
namespace fs = boost::filesystem;

// external includes
#ifdef __LINUX__
#else
#include "cob_vision_ipa_utils/MathUtils.h"
#include "cob_sensor_fusion/ColoredPointCloudSequence.h"
#endif

#include "cob_people_detection/people_detector.h"

#include <sstream>
#include <string>
#include <vector>
#include <set>

namespace ipa_PeopleDetector
{

typedef actionlib::SimpleActionServer<cob_people_detection::TrainContinuousAction> TrainContinuousServer;
typedef actionlib::SimpleActionServer<cob_people_detection::TrainCaptureSampleAction> TrainCaptureSampleServer;
typedef actionlib::SimpleActionServer<cob_people_detection::RecognizeAction> RecognizeServer;
typedef actionlib::SimpleActionServer<cob_people_detection::LoadAction> LoadServer;
typedef actionlib::SimpleActionServer<cob_people_detection::SaveAction> SaveServer;
typedef actionlib::SimpleActionServer<cob_people_detection::ShowAction> ShowServer;

//####################
//#### node class ####
class CobFaceDetectionNodelet: public nodelet::Nodelet
{
protected:
	message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_; ///< Shared xyz image and color image topic
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_camera_image_sub_; ///< Color camera image topic
	image_transport::Publisher face_detection_image_pub_; ///< topic for publishing the image containing the faces
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> >* sync_pointcloud_;
	message_filters::Connection sync_pointcloud_callback_connection_;
	ros::Publisher face_position_publisher_; ///< publisher for the positions of the detected faces

	ros::NodeHandle node_handle_; ///< ROS node handle

#ifdef __LINUX__
	cv::Mat color_image_; ///< Storage for acquired color
	cv::Mat range_image_; /// and depth image
#else
	ipa_SensorFusion::ColoredPointCloudPtr colored_pc_; ///< Storage for acquired colored point cloud
#endif

	std::string directory_; ///< directory for the data files
	bool run_pca_; ///< has to run a PCA when the data was modified
	int filename_; ///< increasing number for files to save

	std::vector<cv::Mat> face_images_; ///< Trained face images
	std::vector<std::string> id_; ///< Id of learned faces
	std::vector<std::string> id_unique_; ///< A vector containing all different Ids from the training session exactly once (id_unique_[i] stores the corresponding id to the average face coordinates in the face subspace in face_class_avg_projections_.row(i))

	int n_eigens_; ///< Number of eigenvalues
	std::vector<cv::Mat> eigen_vectors_; ///< Eigenvectors (spanning the face space)
	cv::Mat eigen_val_mat_; ///< Eigenvalues
	cv::Mat avg_image_; ///< Trained average Image
	cv::Mat projected_train_face_mat_; ///< Projected training faces (coefficients for the eigenvectors of the face subspace)
	cv::Mat face_class_avg_projections_; ///< The average factors of the eigenvector decomposition from each face class
#if CV_MAJOR_VERSION == 2
	cv::SVM person_classifier_; ///< classifier for the identity of a person
#else
// OpenCV 3
	cv::ml::SVM person_classifier_; ///< classifier for the identity of a person
#endif

	PeopleDetector* people_detector_; ///< People detector core code
	int threshold_; ///< Threshold to detect unknown faces
	int threshold_fs_; ///< Threshold to facespace
	std::vector<cv::Rect> color_faces_; ///< Vector with detected faces
	std::vector<cv::Rect> range_faces_; ///< Vector with detected rangeFaces
	std::set<size_t> range_face_indices_with_color_face_detection_; ///< this set stores which range faces also had a face detection in the color image

	// Services
	ros::ServiceServer recognize_service_server_; ///< Service server to switch recognition on or off

	// Actions
	TrainContinuousServer* train_continuous_server_;
	TrainCaptureSampleServer* train_capture_sample_server_;
	RecognizeServer* recognize_server_;
	LoadServer* load_server_;
	SaveServer* save_server_;
	ShowServer* show_server_;
	bool occupied_by_action_; ///< must be set true as long as an action callback is running or while the continuous recognition or training mode is running
	bool recognize_server_running_; ///< is true while the recognition module is running
	bool train_continuous_server_running_; ///< is true while the continuous training display is running
	bool capture_training_face_; ///< can be set true by an action while in training mode. then an image is captured.
	int number_training_images_captured_; ///< if the training is in continuous mode, this variable counts how many training images already have been collected for the current training job
	//	bool turn_off_recognition_;					///< is set true on quit request during recognition mode
	bool do_recognition_; ///< does people identification if true, else it's just people detection
	bool display_; ///< enables debug output

	std::string current_training_id_; ///< the ID of the current person who is trained
	boost::timed_mutex action_mutex_; ///< secures write and read operations to varibales occupied_by_action_, etc.

	// constants
	static const double FACE_SIZE_MIN_M = 0.12;
	static const double FACE_SIZE_MAX_M = 0.35;
	static const double MAX_FACE_Z_M = 8.0;

	// parameters
	double face_size_min_m_; ///< in meters
	double face_size_max_m_; ///< in meters
	double max_face_z_m_; ///< in meters
	bool fill_unassigned_depth_values_; ///< fills the unassigned depth values in the depth image, must be true for a kinect sensor
	bool reason_about_3dface_size_; ///< if true, the 3d face size is determined and only faces with reasonable size are accepted

public:

	CobFaceDetectionNodelet();

	~CobFaceDetectionNodelet();

	/// Nodelet init function
	void onInit();

	/// Initialization function
	/// @return Return code
	unsigned long init();

	/// Function to detect and verify faces
	/// @param xyz_image Coordinate image with x,y,z coordinates in meters
	/// @param color_image Color image
	/// @return Return code
	unsigned long detectFaces(cv::Mat& xyz_image, cv::Mat& color_image);

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param color_image Color image
	/// @param index Index of classified facespace in vector
	/// @return Return code
	unsigned long recognizeFace(cv::Mat& color_image, std::vector<int>& index);

	/// Function to add a new face
	/// @param image Color image
	/// @param id Id of the new face
	/// @return Return code
	unsigned long addFace(cv::Mat& image, std::string id);

	/// Function to Run the PCA algorithm and project the training images to the PCA subspace
	/// @return Return code
	unsigned long PCA();

	unsigned long saveAllData();

	/// Function to save the training data
	/// @return Return code.
	unsigned long saveTrainingData();

	unsigned long saveRecognizerData();

	/// Loads the training data as well as the recognizer data.
	/// @return Return code.
	unsigned long loadAllData();

	/// Function to load the training data (images, ids)
	/// @param runPCA Do a PCA after loading the files. Necessary if the eigenvector matrices etc. are not loaded as well.
	/// @return Return code.
	unsigned long loadTrainingData(bool runPCA);

	/// Function to load the recognizer data (eigenvectors, -values, average image, etc.)
	/// @return Return code.
	unsigned long loadRecognizerData();

	/// Function to show the eigenfaces. Only for debugging and development
	/// @param eigenface Eigenface
	/// @param index Index of the eigenface
	/// @return Return code.
	unsigned long getEigenface(cv::Mat& eigenface, int index);

	/// Function to show the average image of all trained images
	/// @param avgImage The average image
	/// @return Return code.
	unsigned long showAVGImage(cv::Mat& avgImage);

	/// Function to extract images for training range classifier
	/// @param pc ColoredPointCloud with images
	/// @return Return code.
	unsigned long saveRangeTrainImages(cv::Mat& xyz_image);

	//unsigned long getMeasurement(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg);

	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);

	unsigned long convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, cv::Mat& depth_image);

	/// returns the ID string corresponding to index index
	std::string getLabel(int index);

	/// Topic callback managing the treatment of incoming data.
	void recognizeCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg);

	bool recognizeServiceServerCallback(cob_people_detection::Recognition::Request &req, cob_people_detection::Recognition::Response &res);

	/// Action server callback which manages the execution of the recognition functionality
	void recognizeServerCallback(const cob_people_detection::RecognizeGoalConstPtr& goal);

	void trainContinuousCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg);

	void trainContinuousServerCallback(const cob_people_detection::TrainContinuousGoalConstPtr& goal);

	void trainCaptureSampleServerCallback(const cob_people_detection::TrainCaptureSampleGoalConstPtr& goal);

	void loadServerCallback(const cob_people_detection::LoadGoalConstPtr& goal);

	void saveServerCallback(const cob_people_detection::SaveGoalConstPtr& goal);

	void showServerCallback(const cob_people_detection::ShowGoalConstPtr& goal);

	unsigned long loadParameters(const char* iniFileName);
};

}
;

#endif // _FACE_DETECTION_
