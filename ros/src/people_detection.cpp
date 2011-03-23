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


#include "cob_people_detection/people_detection.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_DECLARE_CLASS(ipa_PeopleDetector, cobPeopleDetectionNodelet, ipa_PeopleDetector::cobPeopleDetectionNodelet, nodelet::Nodelet);


using namespace ipa_PeopleDetector;

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(sensor_msgs::PointCloud2* const) {}

//####################
//#### node class ####
cobPeopleDetectionNodelet::cobPeopleDetectionNodelet()
		//: it_(node_handle_)		// node_handle_ not initialized yet at this place, but already needed. Fixed in onInit().
		//sync_pointcloud(2)
{
	it_ = 0;
	sync_pointcloud = 0;
	m_peopleDetector = 0;
	m_trainContinuousServer = 0;
	m_trainCaptureSampleServer = 0;
	m_recognizeServer = 0;
	m_loadServer = 0;
	m_saveServer = 0;
	m_occupiedByAction = false;
	m_directory = ros::package::getPath("cob_people_detection") + "/common/files/windows/";	// todo: make it a parameter
}

cobPeopleDetectionNodelet::~cobPeopleDetectionNodelet()
{
	cvDestroyAllWindows();
	delete m_peopleDetector;
	m_peopleDetector = 0;
	if (m_trainContinuousServer != 0) delete m_trainContinuousServer;
	if (m_trainCaptureSampleServer != 0) delete m_trainCaptureSampleServer;
	if (m_recognizeServer != 0) delete m_recognizeServer;
	if (m_loadServer != 0) delete m_loadServer;
	if (m_saveServer != 0) delete m_saveServer;
	if (it_ != 0) delete it_;
	if (sync_pointcloud != 0) delete sync_pointcloud;
}

void cobPeopleDetectionNodelet::onInit()
{
	sync_pointcloud = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::Image> >(2);
	node_handle_ = getNodeHandle();
	it_ = new image_transport::ImageTransport(node_handle_);

	m_recognizeServer = new RecognizeServer(node_handle_, "recognize_server", boost::bind(&cobPeopleDetectionNodelet::recognizeServerCallback, this, _1), false);

	// initializations
	init();

    ros::spin();
}

unsigned long cobPeopleDetectionNodelet::init()
{
	shared_image_sub_.subscribe(node_handle_, "/camera/depth/points", 1);
	color_camera_image_sub_.subscribe(*it_, "/camera/rgb/image_color", 1);
//	sync_pointcloud->connectInput(shared_image_sub_, color_camera_image_sub_);
//	sync_pointcloud->registerCallback(boost::bind(&cobPeopleDetectionNodelet::recognizeCallback, this, _1, _2));

	colored_pc_ = ipa_SensorFusion::CreateColoredPointCloud();

	if (m_peopleDetector != 0) delete m_peopleDetector;
	m_peopleDetector = new ipa_PeopleDetector::PeopleDetector();

	m_filename = 0;

	// load data for face recognition
	loadRecognizerData();

	cv::namedWindow("Face Detector");

	std::string iniFileNameAndPath = m_directory + "peopleDetectorIni.xml";

	//if (CameraSensorsControlFlow::Init(directory, "peopleDetectorIni.xml", colorCamera0, colorCamera1, rangeImagingSensor) & ipa_Utils::RET_FAILED)
	//{
	//	std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
	//	std::cerr << "\t ... Could not initialize 'CameraSensorsControlFlow'" << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}

	if (m_peopleDetector->Init(m_directory) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::Init:" << std::endl;
		std::cerr << "\t ... Could not initialize people detector library.\n";
		return ipa_Utils::RET_FAILED;
	}

	if(loadParameters(iniFileNameAndPath.c_str()) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::Init:" << std::endl;
		std::cerr << "\t ... Error while loading configuration file '" << std::endl;
		std::cerr << "\t ... " << iniFileNameAndPath << "'.\n";
		return ipa_Utils::RET_FAILED;
	}

	m_runPCA = false;

	return ipa_Utils::RET_OK;
}

void cobPeopleDetectionNodelet::recognizeServerCallback(const cob_people_detection::RecognizeGoalConstPtr& goal)
{
	cob_people_detection::RecognizeResult result;
	if (m_occupiedByAction == true && m_recognizeServerRunning == false)
	{
		// another action is running at the moment, first the other action has to finish or to be stopped before this action can run
		std::cerr << "ERROR - PeopleDetector::recognizeServerCallback:" << std::endl;
		std::cerr << "\t ... Another action is running at the moment. The other action has to finish or to be stopped before this action can run.\n";
		result.success = ipa_Utils::RET_FAILED;
		m_recognizeServer->setSucceeded(result, "Some other action is handled at the moment.");
		return;
	}

	m_occupiedByAction = true;		// todo: guard it with a mutex
	m_recognizeServerRunning = true;
	// read out goal message
	// set up the recognition callback linkage
	if (goal->running == true)
	{
		// enable recognition
		sync_pointcloud->connectInput(shared_image_sub_, color_camera_image_sub_);
		m_syncPointcloudCallbackConnection = sync_pointcloud->registerCallback(boost::bind(&cobPeopleDetectionNodelet::recognizeCallback, this, _1, _2, goal->doRecognition, goal->display));
	}
	else
	{
		// disable recognition
		m_syncPointcloudCallbackConnection.disconnect();
	}

	result.success = ipa_Utils::RET_OK;
	m_recognizeServer->setSucceeded(result);

	if (goal->running == false)
	{
		m_occupiedByAction = false;   // todo: guard it with a mutex
		m_recognizeServerRunning = false;
	}
}


unsigned long cobPeopleDetectionNodelet::detectFaces(cv::Mat& xyz_image, cv::Mat& color_image)
{
	cv::Mat xyz_image_8U3;
	ipa_Utils::ConvertToShowImage(xyz_image, xyz_image_8U3, 3);
//todo: read parameter whether a kinect sensor is used
	if (m_peopleDetector->DetectFaces(color_image, xyz_image_8U3, m_colorFaces, m_rangeFaces, true/*(m_RangeImagingCameraType==ipa_CameraSensors::CAM_KINECT)*/) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetection::detectFaces" << std::endl;
		std::cerr << "\t ... Could not detect faces.\n";
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::recognizeFace(cv::Mat& color_image, std::vector<int>& index)
{
	if (m_peopleDetector->RecognizeFace(color_image, m_colorFaces, &m_nEigens, m_eigenVectors, m_avgImage, m_faceClassAvgProjections, index, &m_threshold, &m_threshold_FS, m_eigenValMat) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetector::recognizeFace:" << std::endl;
		std::cerr << "\t ... Error while recognizing faces.\n";
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::addFace(cv::Mat& image, std::string id)
{
	// addFace should only be called if there is exactly one face found --> so we access it with m_colorFaces[0]
	if (m_peopleDetector->AddFace(image, m_colorFaces[0], id, m_faceImages, m_id) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::AddFace:" << std::endl;
		std::cerr << "\t ... Could not save face.\n";
		return ipa_Utils::RET_FAILED;
	}
	m_runPCA = true;

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::PCA()
{
	// only run PCA if new data has arrived after last computation
	if(!m_runPCA)
	{
		std::cout << "INFO - PeopleDetector::PCA:" << std::endl;
		std::cout << "\t ... PCA algorithm skipped.\n";
		return ipa_Utils::RET_OK;
	}

	if(m_faceImages.size() < 2)
	{
		std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Less than two images are trained.\n";
		return ipa_Utils::RET_OK;
	}

	// Delete memory
	m_eigenVectors.clear();

	// Do PCA
	if (m_peopleDetector->PCA(&m_nEigens, m_eigenVectors, m_eigenValMat, m_avgImage, m_faceImages, m_projectedTrainFaceMat) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::PCA:" << std::endl;
		std::cerr << "\t ... Error while PCA.\n";
		return ipa_Utils::RET_FAILED;
	}

	// Calculate FaceClasses
	if (m_peopleDetector->CalculateFaceClasses(m_projectedTrainFaceMat, m_id, &m_nEigens, m_faceClassAvgProjections, m_idUnique) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetectorControlFlow::PCA:" << std::endl;
		std::cerr << "\t ... Error while calculating FaceClasses.\n";
		return ipa_Utils::RET_FAILED;
	}

	m_runPCA = false;

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::saveTrainingData()
{
	std::string path = m_directory + "TrainingData/";
	std::string filename = "data.xml";
	std::string img_ext = ".bmp";

	std::ostringstream complete;
	complete << path << filename;
	path = "TrainingData/";

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

	cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
	if(!fileStorage.isOpened())
	{
		std::cout << "WARNING - PeopleDetector::SaveTrainingData:" << std::endl;
		std::cout << "\t ... Can't save training data.\n";
		return ipa_Utils::RET_OK;
	}

	// Ids
	fileStorage << "id_num" << (int)m_id.size();
	for(int i=0; i<(int)m_id.size(); i++)
	{
		std::ostringstream tag;
		tag << "id_" << i;
		fileStorage << tag.str().c_str() << m_id[i].c_str();
	}

	// Face images
	fileStorage << "faces_num" << (int)m_faceImages.size();
	for(int i=0; i<(int)m_faceImages.size(); i++)
	{
		std::ostringstream img;
		img << path << i << img_ext;
		std::ostringstream tag;
		tag << "img_" << i;
		fileStorage << tag.str().c_str() << img.str().c_str();
		//cvSaveImage(img.str().c_str(), &m_faceImages[i]);
		cv::imwrite(img.str().c_str(), m_faceImages[i]);
	}

	// Number eigenvalues/eigenvectors
	fileStorage << "eigens_num" << m_nEigens;

	// Eigenvectors
	for (int i=0; i<m_nEigens; i++)
	{
		std::ostringstream tag;
		tag << "ev_" << i;
		fileStorage << tag.str().c_str() << m_eigenVectors[i];
	}

	// Eigenvalue matrix
	fileStorage << "eigen_val_mat" << m_eigenValMat;

	// Average image
	fileStorage << "avg_image" << m_avgImage;

	// Projection coefficients of the training faces
	fileStorage << "projected_train_face_mat" << m_projectedTrainFaceMat;

	// Unique Ids (m_idUnique[i] stores the corresponding id to the average face coordinates in the face subspace in m_faceClassAvgProjections.row(i))
	fileStorage << "id_unique_num" << (int)m_idUnique.size();
	for(int i=0; i<(int)m_idUnique.size(); i++)
	{
		std::ostringstream tag;
		tag << "id_unique_" << i;
		fileStorage << tag.str().c_str() << m_idUnique[i].c_str();
	}

	// The average factors of the eigenvector decomposition from each face class
	fileStorage << "face_class_avg_projections" << m_faceClassAvgProjections;

	fileStorage.release();

	std::cout << "INFO - PeopleDetector::SaveTrainingData:" << std::endl;
	std::cout << "\t ... " << m_faceImages.size() << " images saved.\n";
	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::loadAllData()
{
	if (loadTrainingData(false) != ipa_Utils::RET_OK) return ipa_Utils::RET_FAILED;
	if (loadRecognizerData() != ipa_Utils::RET_OK) return ipa_Utils::RET_FAILED;

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::loadTrainingData(bool runPCA)
{
	std::string path = m_directory + "TrainingData/";
	std::string filename = "data.xml";

	std::ostringstream complete;
	complete << path << filename;

	if(fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if(!fileStorage.isOpened())
		{
			std::cout << "WARNING - PeopleDetector::loadTrainingData:" << std::endl;
			std::cout << "\t ... Cant open " << complete.str() << ".\n";
			return ipa_Utils::RET_OK;
		}

		// Ids
		m_id.clear();
		int id_num = (int)fileStorage["id_num"];
		for(int i=0; i<id_num; i++)
		{
			std::ostringstream tag;
			tag << "id_" << i;
			m_id.push_back((std::string)fileStorage[tag.str().c_str()]);
		}

		// Images
		m_faceImages.clear();
		int faces_num = (int)fileStorage["faces_num"];
		for(int i=0; i<faces_num; i++)
		{
			std::ostringstream tag;
			tag << "img_" << i;
			std::string path = m_directory + (std::string)fileStorage[tag.str().c_str()];
			cv::Mat temp = cv::imread(path.c_str(),0);
			m_faceImages.push_back(temp);
		}

		fileStorage.release();

		// Run PCA - now or later
		m_runPCA = true;
		if (runPCA) PCA();

		std::cout << "INFO - PeopleDetector::loadRecognizerData:" << std::endl;
		std::cout << "\t ... " << faces_num << " images loaded.\n";
	}
	else
	{
		std::cerr << "ERROR - PeopleDetector::loadTrainingData():" << std::endl;
		std::cerr << "\t .... Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}
}

unsigned long cobPeopleDetectionNodelet::loadRecognizerData()
{
	std::string path = m_directory + "TrainingData/";
	std::string filename = "data.xml";

	std::ostringstream complete;
	complete << path << filename;

	if(fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if(!fileStorage.isOpened())
		{
			std::cout << "WARNING - PeopleDetector::loadRecognizerData:" << std::endl;
			std::cout << "\t ... Cant open " << complete.str() << ".\n";
			return ipa_Utils::RET_OK;
		}

		// Number eigenvalues/eigenvectors
		m_nEigens = (int)fileStorage["eigens_num"];

		// Eigenvectors
		m_eigenVectors.clear();
		m_eigenVectors.resize(m_nEigens, cv::Mat());
		for (int i=0; i<m_nEigens; i++)
		{
			std::ostringstream tag;
			tag << "ev_" << i;
			fileStorage[tag.str().c_str()] >> m_eigenVectors[i];
		}

		// Eigenvalue matrix
		m_eigenValMat = cv::Mat();
		fileStorage["eigen_val_mat"] >> m_eigenValMat;

		// Average image
		m_avgImage = cv::Mat();
		fileStorage["avg_image"] >> m_avgImage;

		// Projections of the training faces
		m_projectedTrainFaceMat = cv::Mat();
		fileStorage["projected_train_face_mat"] >> m_projectedTrainFaceMat;

		// Unique Ids (m_idUnique[i] stores the corresponding id to the average face coordinates in the face subspace in m_faceClassAvgProjections.row(i))
		m_idUnique.clear();
		int idUniqueNum = (int)fileStorage["id_unique_num"];
		for(int i=0; i<idUniqueNum; i++)
		{
			std::ostringstream tag;
			tag << "id_unique_" << i;
			m_idUnique.push_back((std::string)fileStorage[tag.str().c_str()]);
		}

		// The average factors of the eigenvector decomposition from each face class
		m_faceClassAvgProjections = cv::Mat();
		fileStorage["face_class_avg_projections"] >> m_faceClassAvgProjections;

		fileStorage.release();

		// do not run PCA
		m_runPCA = false;

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

unsigned long cobPeopleDetectionNodelet::getEigenface(cv::Mat& eigenface, int index)
{
	cv::normalize(m_eigenVectors[index], eigenface, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::showAVGImage(cv::Mat& avgImage)
{
	if(!m_runPCA && m_faceImages.size()<2)
	{
		std::cerr << "PeopleDetector::showAvgImage()" << std::endl;
		std::cerr << "No AVG image calculated" << std::endl;
		return 0;
	}

	cv::normalize(m_avgImage, avgImage, 0, 255, cv::NORM_MINMAX, CV_8UC1);

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::saveRangeTrainImages(ipa_SensorFusion::ColoredPointCloudPtr pc)
{
	std::string path = m_directory + "haarcascades/";
	std::string img_ext = ".jpg";
	cv::Mat xyzImage_8U3(pc->GetXYZImage().size(), CV_8UC3);	//IplImage* xyzImage_8U3 = cvCreateImage(cvGetSize(pc->GetXYZImage()), 8, 3);
	ipa_Utils::ConvertToShowImage(pc->GetXYZImage(), xyzImage_8U3, 3);

	for(int i=0; i<(int)m_colorFaces.size(); i++)
	{
		cv::Mat xyzImage_8U3_resized(100, 100, CV_8UC3);    //cvCreateImage(cvSize(100,100), 8, 3); 8=IPL_DEPTH_8U

		double scale = 1.6;
		cv::Rect rangeFace;
		rangeFace.height = (int)(m_colorFaces[i].height*scale);
		rangeFace.width = (int)(m_colorFaces[i].width*scale);
		rangeFace.x = m_colorFaces[i].x - ((rangeFace.width - m_colorFaces[i].width)/2);
		rangeFace.y = m_colorFaces[i].y - ((rangeFace.height - m_colorFaces[i].height)/2)-10;

		cv::Mat xyzImage_8U3_roi = xyzImage_8U3(rangeFace);
		//cvSetImageROI(xyzImage_8U3, rangeFace);
		cv::resize(xyzImage_8U3_roi, xyzImage_8U3_resized, xyzImage_8U3_resized.size());

		std::ostringstream file;
		file << path << m_filename << img_ext;

		cv::imwrite(file.str().c_str(), xyzImage_8U3_resized);//cvSaveImage(file.str().c_str(), xyzImage_8U3_resized);
		m_filename++;
	}

	return ipa_Utils::RET_OK;
}

unsigned long cobPeopleDetectionNodelet::getMeasurement(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	cv::Mat color_image_8U3(shared_image_msg->height, shared_image_msg->width, CV_8UC3);
	cv::Mat xyz_image_32F3(shared_image_msg->height, shared_image_msg->width, CV_32FC3);
	float* f_ptr = 0;
	const uint8_t* data_ptr = 0;
	unsigned char* uc_ptr = 0;
	unsigned int xyz_offset = shared_image_msg->fields[0].offset;
	unsigned int rgb_offset = shared_image_msg->fields[3].offset;
	size_t b_offset = 2*sizeof(unsigned char);
	size_t g_offset = sizeof(unsigned char);
	size_t r_offset = 0;
	unsigned int col_times_3 = 0;
	for (unsigned int row = 0; row < shared_image_msg->height; row++)
	{
		uc_ptr = color_image_8U3.ptr<unsigned char>(row);
		f_ptr = xyz_image_32F3.ptr<float>(row);

		data_ptr = &shared_image_msg->data[row * shared_image_msg->width * shared_image_msg->point_step];

		for (unsigned int col = 0; col < shared_image_msg->width; col++)
		{
			col_times_3 = 3*col;
			// Reorder incoming image channels
			memcpy(&uc_ptr[col_times_3], &data_ptr[col * shared_image_msg->point_step + rgb_offset + b_offset], sizeof(unsigned char));
			memcpy(&uc_ptr[col_times_3 + 1], &data_ptr[col * shared_image_msg->point_step + rgb_offset + g_offset], sizeof(unsigned char));
			memcpy(&uc_ptr[col_times_3 + 2], &data_ptr[col * shared_image_msg->point_step + rgb_offset + r_offset], sizeof(unsigned char));

			memcpy(&f_ptr[col_times_3], &data_ptr[col * shared_image_msg->point_step + xyz_offset], 3*sizeof(float));
		}
	}

	// Images are cloned within setter functions
	colored_pc_->SetColorImage(color_image_8U3);
	colored_pc_->SetXYZImage(xyz_image_32F3);

//    		cv::Mat xyz_image_8U3;
//			ipa_Utils::ConvertToShowImage(colored_pc_->GetXYZImage(), xyz_image_8U3, 3);
//	    	cv::imshow("xyz data", xyz_image_8U3);
//	    	cv::imshow("color data", colored_pc_->GetColorImage());
//	    	cv::waitKey();

	return ipa_Utils::RET_OK;
}


void cobPeopleDetectionNodelet::recognizeCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg, bool doRecognition, bool display)
{
	// convert input to cv::Mat images
	// color
	cv_bridge::CvImageConstPtr color_image_ptr;
	try
	{
	  color_image_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
	  ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
	  return;
	}
	cv::Mat color_image = color_image_ptr->image;

	// point cloud
	pcl::PointCloud<pcl::PointXYZ> depth_cloud; // point cloud
	pcl::fromROSMsg(*shared_image_msg, depth_cloud);
	cv::Mat depth_image(depth_cloud.height, depth_cloud.width, CV_32FC3);
	uchar* depth_image_ptr = (uchar*) depth_image.data;
	for (int v=0; v<(int)depth_cloud.height; v++)
	{
		int baseIndex = depth_image.step*v;
		for (int u=0; u<(int)depth_cloud.width; u++)
		{
			int index = baseIndex + 3*u*sizeof(float);
			float* data_ptr = (float*)(depth_image_ptr+index);
			pcl::PointXYZ point_xyz = depth_cloud(u,v);
			data_ptr[0] = point_xyz.x;
			data_ptr[1] = point_xyz.y;
			data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
			//if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
		}
	}

	// convert point cloud and color image to colored point cloud
	colored_pc_->SetColorImage(color_image);
	colored_pc_->SetXYZImage(depth_image);
	//getMeasurement(shared_image_msg, color_image_msg);


	PCA();

	if(m_eigenVectors.size() < 1)
	{
		std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
		std::cout << "\t ... Less than two images are trained.\n";
		return;
	}

	//DWORD start = timeGetTime();
	detectFaces(colored_pc_->GetXYZImage(), colored_pc_->GetColorImage());

	cv::Mat colorImage_8U3;
	colored_pc_->GetColorImage().copyTo(colorImage_8U3);

	std::vector<int> index;
	if (doRecognition==true)
	{
		recognizeFace(color_image, index);
		std::cout << "INFO - PeopleDetector::Recognize:" << std::endl;
	}
	else for (int i=0; i<(int)m_colorFaces.size(); i++) index.push_back(-1);
	//std::cout << "\t ... Recognize Time: " << (timeGetTime() - start) << std::endl;

	// display results
	if (display==true)
	{
		for(int i=0; i<(int)m_rangeFaces.size(); i++)
		{
			cv::Rect face = m_rangeFaces[i];
			cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
		}

		for(int i=0; i<(int)m_colorFaces.size(); i++)
		{
			cv::Rect face = m_colorFaces[i];

			cv::rectangle(colorImage_8U3, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

			std::stringstream tmp;
			switch(index[i])
			{
			case -1:
				// Distance to face class is too high
				tmp << "Unknown";
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				break;
			case -2:
				// Distance to face space is too high
				tmp << "No face";
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				break;
			default:
				// Face classified
				tmp << m_idUnique[index[i]].c_str();
				cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
			}
		}
		cv::imshow("Face Detector", colorImage_8U3);
	}
}


unsigned long cobPeopleDetectionNodelet::loadParameters(const char* iniFileName)
{
	/// Load parameters from file
	TiXmlDocument* p_configXmlDocument = new TiXmlDocument( iniFileName );
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

	if ( p_configXmlDocument )
	{

//************************************************************************************
//	BEGIN PeopleDetector
//************************************************************************************
		// Tag element "PeopleDetector" of Xml Inifile
		TiXmlElement *p_xmlElement_Root = NULL;
		p_xmlElement_Root = p_configXmlDocument->FirstChildElement( "PeopleDetector" );

		if ( p_xmlElement_Root )
		{

//************************************************************************************
//	BEGIN PeopleDetector->adaBoost
//************************************************************************************
			// Tag element "adaBoost" of Xml Inifile
			TiXmlElement *p_xmlElement_Root_OD = NULL;
			p_xmlElement_Root_OD = p_xmlElement_Root->FirstChildElement( "adaBoost" );

			if ( p_xmlElement_Root_OD )
			{

//************************************************************************************
//	BEGIN PeopleDetector->adaBoost->Faces_increase_search_scale
//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Faces_increase_search_scale" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_faces_increase_search_scale) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Faces_drop_groups" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_faces_drop_groups) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Faces_min_search_scale_x" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_faces_min_search_scale_x) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Faces_min_search_scale_y" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_faces_min_search_scale_y) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Range_increase_search_scale" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_range_increase_search_scale) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Range_drop_groups" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_range_drop_groups) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Range_min_search_scale_x" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_range_min_search_scale_x) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Range_min_search_scale_y" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_peopleDetector->m_range_min_search_scale_y) != TIXML_SUCCESS)
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
			p_xmlElement_Root_OD = p_xmlElement_Root->FirstChildElement( "eigenfaces" );

			if ( p_xmlElement_Root_OD )
			{

//************************************************************************************
//	BEGIN PeopleDetector->eigenfaces->Threshold_Face_Class
//************************************************************************************
				// Subtag element "adaBoost" of Xml Inifile
				TiXmlElement *p_xmlElement_Child = NULL;
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Threshold_Face_Class" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_threshold) != TIXML_SUCCESS)
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
				p_xmlElement_Child = p_xmlElement_Root_OD->FirstChildElement( "Threshold_Facespace" );

				if ( p_xmlElement_Child )
				{
					// read and save value of attribute
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_threshold_FS) != TIXML_SUCCESS)
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
//	// Initialize ROS, spezify name of node
//	ros::init(argc, argv, "people_detection");
//
//	// Create a handle for this node, initialize node
//	ros::NodeHandle nh;
//
//	// Create people detection node class instance
//	cobPeopleDetectionNodelet people_detection_node(nh);
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
