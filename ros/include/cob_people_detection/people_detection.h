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

#ifndef _PEOPLE_DETECTION_
#define _PEOPLE_DETECTION_


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

//#include <cob_object_detection/DetectObjectsAction.h>	//wo, wozu
//#include <cob_object_detection/AcquireObjectImageAction.h>
//#include <cob_object_detection/TrainObjectAction.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// actions
#include <actionlib/server/simple_action_server.h>
#include <cob_people_detection/TrainContinuousAction.h>
#include <cob_people_detection/TrainCaptureSampleAction.h>
#include <cob_people_detection/RecognizeAction.h>
#include <cob_people_detection/LoadAction.h>
#include <cob_people_detection/SaveAction.h>
#include <cob_people_detection/ShowAction.h>

// opencv
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// tiny xml
#include "tinyxml/tinyxml.h"

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
namespace fs = boost::filesystem;

// external includes
#include "cob_vision_ipa_utils/MathUtils.h"

#include "cob_sensor_fusion/ColoredPointCloudSequence.h"

#include "cob_people_detection/PeopleDetector.h"

#include <sstream>
#include <string>
#include <vector>

namespace ipa_PeopleDetector {

typedef actionlib::SimpleActionServer<cob_people_detection::TrainContinuousAction> TrainContinuousServer;
typedef actionlib::SimpleActionServer<cob_people_detection::TrainCaptureSampleAction> TrainCaptureSampleServer;
typedef actionlib::SimpleActionServer<cob_people_detection::RecognizeAction> RecognizeServer;
typedef actionlib::SimpleActionServer<cob_people_detection::LoadAction> LoadServer;
typedef actionlib::SimpleActionServer<cob_people_detection::SaveAction> SaveServer;
typedef actionlib::SimpleActionServer<cob_people_detection::ShowAction> ShowServer;


//####################
//#### node class ####
class cobPeopleDetectionNodelet : public nodelet::Nodelet
{
protected:
	message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_;	///< Shared xyz image and color image topic
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter color_camera_image_sub_;	///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> >* sync_pointcloud; /**< Pointcloud synchronizer without disparity display. */
	message_filters::Connection m_syncPointcloudCallbackConnection;

	ros::NodeHandle node_handle_;				 ///< ROS node handle

	ipa_SensorFusion::ColoredPointCloudPtr colored_pc_; ///< Storage for acquired colored point cloud

	std::string m_directory;					///< directory for the data files
	bool m_runPCA;								///< has to run a PCA when the data was modified
	int m_filename;								///< increasing number for files to save

	std::vector<cv::Mat> m_faceImages;			///< Trained face images
	std::vector<std::string> m_id;				///< Id of learned faces
	std::vector<std::string> m_idUnique;		///< A vector containing all different Ids from the training session exactly once (m_idUnique[i] stores the corresponding id to the average face coordinates in the face subspace in m_faceClassAvgProjections.row(i))

	int m_nEigens;								///< Number of eigenvalues
	std::vector<cv::Mat> m_eigenVectors;		///< Eigenvectors (spanning the face space)
	cv::Mat m_eigenValMat;						///< Eigenvalues
	cv::Mat m_avgImage;							///< Trained average Image
	cv::Mat m_projectedTrainFaceMat;			///< Projected training faces (coefficients for the eigenvectors of the face subspace)
	cv::Mat m_faceClassAvgProjections;			///< The average factors of the eigenvector decomposition from each face class

	PeopleDetector* m_peopleDetector;			///< People detector core code
	int m_threshold;							///< Threshold to detect unknown faces
	int m_threshold_FS;							///< Threshold to facespace
	std::vector<cv::Rect> m_colorFaces;			///< Vector with detected faces
	std::vector<cv::Rect> m_rangeFaces;			///< Vector with detected rangeFaces

	// Actions
	TrainContinuousServer* m_trainContinuousServer;
	TrainCaptureSampleServer* m_trainCaptureSampleServer;
	RecognizeServer* m_recognizeServer;
	LoadServer* m_loadServer;
	SaveServer* m_saveServer;
	ShowServer* m_showServer;
	bool m_occupiedByAction;					///< must be set true as long as an action callback is running or while the continuous recognition or training mode is running
	bool m_recognizeServerRunning;				///< is true while the recognition module is running
	bool m_trainContinuousServerRunning;		///< is true while the continuous training display is running
	bool m_captureTrainingFace;					///< can be set true by an action while in training mode. then an image is captured.

	std::string m_currentTrainingID;				///< the ID of the current person who is trained
	boost::timed_mutex m_actionMutex;			///< secures write and read operations to varibales m_occupiedByAction, etc.

public:

	cobPeopleDetectionNodelet();

	~cobPeopleDetectionNodelet();

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
	unsigned long saveRangeTrainImages(ipa_SensorFusion::ColoredPointCloudPtr pc);

	unsigned long getMeasurement(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg);

	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg, cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);

	unsigned long convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, cv::Mat& depth_image);

	/// Topic callback managing the treatment of incoming data.
	void recognizeCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg, bool doRecognition, bool display);

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

};

#endif // _PEOPLE_DETECTION_
