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

using namespace ipa_PeopleDetector;

// Prevent deleting memory twice, when using smart pointer
void voidDeleter(sensor_msgs::PointCloud2* const) {}

//####################
//#### node class ####
cobPeopleDetectionNode::cobPeopleDetectionNode(const ros::NodeHandle& node_handle)
		: node_handle_(node_handle),
		it_(node_handle),
		sync_pointcloud(2)
{
	m_PeopleDetector = 0;
}

cobPeopleDetectionNode::~cobPeopleDetectionNode()
{
	cvDestroyAllWindows();
	delete m_PeopleDetector;
	m_PeopleDetector = 0;
}

unsigned long cobPeopleDetectionNode::init()
{
	shared_image_sub_.subscribe(node_handle_, "/camera/depth/points", 1);
	color_camera_image_sub_.subscribe(it_, "/camera/rgb/image_color", 1);
	sync_pointcloud.connectInput(shared_image_sub_, color_camera_image_sub_);
	sync_pointcloud.registerCallback(boost::bind(&cobPeopleDetectionNode::recognizeCallback, this, _1, _2));

	colored_pc_ = ipa_SensorFusion::CreateColoredPointCloud();

	cv::namedWindow("Face Detector");

	std::string directory = "ConfigurationFiles/";
	std::string iniFileNameAndPath = directory + "peopleDetectorIni.xml";

	//if (CameraSensorsControlFlow::Init(directory, "peopleDetectorIni.xml", colorCamera0, colorCamera1, rangeImagingSensor) & ipa_Utils::RET_FAILED)
	//{
	//	std::cerr << "ERROR - CameraDataViewerControlFlow::Init:" << std::endl;
	//	std::cerr << "\t ... Could not initialize 'CameraSensorsControlFlow'" << std::endl;
	//	return ipa_Utils::RET_FAILED;
	//}

	m_PeopleDetector = new ipa_PeopleDetector::PeopleDetector();

	if (m_PeopleDetector->Init() & ipa_Utils::RET_FAILED)
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


unsigned long cobPeopleDetectionNode::detectFaces(cv::Mat& xyz_image, cv::Mat& color_image)
{
	cv::Mat xyz_image_8U3;
	ipa_Utils::ConvertToShowImage(xyz_image, xyz_image_8U3, 3);
//todo: read parameter whether a kinect sensor is used
	if (m_PeopleDetector->DetectFaces(color_image, xyz_image_8U3, m_colorFaces, m_rangeFaces, true/*(m_RangeImagingCameraType==ipa_CameraSensors::CAM_KINECT)*/) & ipa_Utils::RET_FAILED)
	{
		std::cerr << "ERROR - PeopleDetection::detectFaces" << std::endl;
		std::cerr << "\t ... Could not detect faces.\n";
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

/*	unsigned long cobPeopleDetectionNode::recognizeFace(cv::Mat& color_image, std::vector<int>& index)
	{
		if (m_PeopleDetector->RecognizeFace(color_image, m_colorFaces, &m_nEigens, m_eigenVectArr, m_avgImage, m_projectedTrainFaceMat, index, &m_threshold, &m_threshold_FS, m_eigenValMat) & ipa_Utils::RET_FAILED)
		{
			std::cerr << "ERROR - PeopleDetector::recognizeFace:" << std::endl;
			std::cerr << "\t ... Error while recognizing faces.\n";
			return ipa_Utils::RET_FAILED;
		}

		return ipa_Utils::RET_OK;
	}*/

unsigned long cobPeopleDetectionNode::getMeasurement(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
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


void cobPeopleDetectionNode::recognizeCallback(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
{
	// convert input to cv::Mat images
	// color
	cv_bridge::CvImagePtr color_image_ptr;
	try
	{
	  color_image_ptr = cv_bridge::toCvCopy(color_image_msg);
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
			data_ptr[0] = depth_cloud(u,v).x;
			data_ptr[1] = depth_cloud(u,v).y;
			data_ptr[2] = (isnan(depth_cloud(u,v).z)) ? 0.f : depth_cloud(u,v).z;
			if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
		}
	}
	colored_pc_->SetColorImage(color_image);
	colored_pc_->SetXYZImage(depth_image);

	// convert point cloud and color image to colored point cloud
	//getMeasurement(shared_image_msg, color_image_msg);

	//m_DetectorControlFlow->PCA();

	//if(m_DetectorControlFlow->m_faceImages.size() < 2)
	//{
	//	std::cout << "WARNING - PeopleDetector::ConsoleGUI:" << std::endl;
	//	std::cout << "\t ... Less than two images are trained.\n";
	//	return ipa_Utils::RET_OK;
	//}

	//ipa_SensorFusion::ColoredPointCloudPtr pc;

	//DWORD start = timeGetTime();

	detectFaces(colored_pc_->GetXYZImage(), colored_pc_->GetColorImage());

	cv::Mat colorImage_8U3;
	colored_pc_->GetColorImage().copyTo(colorImage_8U3);

	std::vector<int> index;
	//recognizeFace(color_image, index);
	std::cout << "INFO - PeopleDetector::Recognize:" << std::endl;
	for (int i=0; i<(int)m_colorFaces.size(); i++) index.push_back(-1);
	//std::cout << "\t ... Recognize Time: " << (timeGetTime() - start) << std::endl;

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
			tmp << m_id[index[i]].c_str();
			cv::putText(colorImage_8U3, tmp.str().c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
		}
	}

	cv::imshow("Face Detector", colorImage_8U3);
}


unsigned long cobPeopleDetectionNode::loadParameters(const char* iniFileName)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_faces_increase_search_scale) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_faces_drop_groups) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_faces_min_search_scale_x) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_faces_min_search_scale_y) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_range_increase_search_scale) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_range_drop_groups) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_range_min_search_scale_x) != TIXML_SUCCESS)
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
					if ( p_xmlElement_Child->QueryValueAttribute( "value", &m_PeopleDetector->m_range_min_search_scale_y) != TIXML_SUCCESS)
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
int main(int argc, char** argv)
{
	// Initialize ROS, spezify name of node
	ros::init(argc, argv, "people_detection");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;
	
	// Create people detection node class instance   
	cobPeopleDetectionNode people_detection_node(nh);

	// Initialize people detection node
	if (people_detection_node.init() != ipa_Utils::RET_OK)
		return 0;

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);
	
	ros::spin();
    
	return 0;
}















/*
class CobObjectDetectionNode
{
public:
	bool visualization_; ///< Use 3-D OpenGL based visualization
	ipa_ObjectModel::AbstractMultiObjectModelPtr object_model_; ///< Map of object models 

protected:
	cob_srvs::GetPointCloud2 service_colored_pc_;
	
	ros::Subscriber reprojection_sub_; ///< Listens for the reprojection matrix
	ros::ServiceServer service_server_detect_object_; ///< Service server to request object detection
	ros::ServiceServer service_server_train_object_; ///< Service server to train object from previously acquired images
	ros::ServiceServer service_server_acquire_object_image_; ///< Service server to acquire object image
	ros::ServiceClient service_client_colored_pc_; ///< Service client to get colored point cloud
	ros::NodeHandle node_handle_; ///< ROS node handle

	cv::Scalar xyzr_learning_coordinates_; ///< xyz [m] learning center and learning radius 
	ipa_ObjectModel::t_ModelType object_model_type_; ///< Object model type
	
	std::string directory_; ///< Working directory, from which models are loaded and saved
	std::string images_directory_;
	std::string segmented_images_directory_;
	std::string features_directory_;
	std::string model_directory_;
	std::string model_name_suffix_;
	ipa_ObjectModel::t_DetectionResultSequence detection_results_; ///< Variable to hold the detected objects and their pose
	cv::Mat output_image_8U3_; ///< Visualizes detection results

	unsigned int image_counter_;	///< 3D image counter used for saving training images
	ipa_SensorFusion::ColoredPointCloudPtr colored_pc_; ///< Storage for acquired colored point cloud
	ipa_SensorFusion::ColoredPointCloudToolboxPtr colored_pc_toolbox_; ///< Toolbox for creating a colored point cloud
	ColoredPointCloudToolbox::t_PointCloudMode pc_mode_; 	///< Point cloud creation mode (MODE_SHARED, MODE_STEREO or MODE_AUTO)

	cv::Mat reprojection_matrix_; ///< 3x3 reprojection matrix to convert 3-D points to 2-D image coordinates
	tf::TransformBroadcaster tf_broadcaster; ///< Broadcast transforms of detected objects

public:
		
	/// Constructor
	CobObjectDetectionNode();
	
	/// Destructor
	~CobObjectDetectionNode()
	{
		// Void
	}

	CobObjectDetectionNode(const ros::NodeHandle& node_handle)
	: node_handle_(node_handle),
	  image_counter_(0)
	{
	}

	bool init()
	{
		if (loadParameters() == false) 
			return false;

		reprojection_sub_ = node_handle_.subscribe("reprojection_matrix", 1, &CobObjectDetectionNode::reprojectionCallback, this);

		service_client_colored_pc_ = node_handle_.serviceClient<cob_srvs::GetPointCloud2>("get_colored_pc");
		service_server_detect_object_ = node_handle_.advertiseService("detect_object", &CobObjectDetectionNode::detectObjectCallback, this);
		service_server_acquire_object_image_ = node_handle_.advertiseService("acquire_object_image", &CobObjectDetectionNode::acquireImageCallback, this);
		service_server_train_object_ = node_handle_.advertiseService("train_object", &CobObjectDetectionNode::trainObjectCallback, this);
		images_directory_ = directory_ + "TrainingImages/";
		segmented_images_directory_ = directory_ + "SegmentedTrainingImages/";
		features_directory_ = directory_ + "BlobFeatures/";
		model_directory_ = directory_ + "ObjectModels/";
		model_name_suffix_ = "model";
		
		colored_pc_toolbox_ = ipa_SensorFusion::CreateColoredPointCloudToolbox();
		colored_pc_ = ipa_SensorFusion::CreateColoredPointCloud();
		pc_mode_ = ColoredPointCloudToolbox::MODE_STEREO;
		switch (object_model_type_)
		{
#ifdef __BUILD_WITH_MODEL_BAYES__
		case ipa_ObjectModel::MODEL_BAYES:
			object_model_ = ipa_ObjectModel::CreateMultiObjectModel_Bayes(TYPE_SURF);
			break;
#endif
#ifdef __BUILD_WITH_MODEL_SLAM_6D__
		case ipa_ObjectModel::MODEL_SLAM_6D:
			object_model_ = ipa_ObjectModel::CreateMultiObjectModel_SLAM6D(TYPE_SURF);
			break;
#endif
#ifdef __BUILD_WITH_MODEL_SIFT__
		case ipa_ObjectModel::MODEL_SIFT:
			object_model_ = ipa_ObjectModel::CreateMultiObjectModel_Sift(TYPE_SURF);
			break;
#endif
		default:
			ROS_ERROR("[object_detection] Init [FAILED]");
			ROS_ERROR("[object_detection] ... 1. Check launch file, if you selected the correct object model");
			ROS_ERROR("[object_detection] ... 2. Check if you compiled the corresponding 'ObjectModel*.cpp' file");
			ROS_ERROR("[object_detection] ... 3. Check if you defined the corresponding '__BUILD_WITH_MODEL_*__'");
			ROS_ERROR("[object_detection] ...    preprocessor directive in the CMake file");
			return false;
		}

		ROS_INFO("[object_detection] Loading object models [IN PROCESS]");
		if (object_model_->Load(model_directory_) & ipa_Utils::RET_FAILED)
		{
			ROS_WARN("[object_detection] Loading of object models [FAILED]");
		}

		ROS_INFO("[object_detection] Loading object models [DONE]");
		return true;	
	}

	void reprojectionCallback(const cob_msgs::ReprojectionMatrixPtr &data)
	{
		if (reprojection_matrix_.empty())
		{
			ROS_INFO("[object_detection] Received reprojection matrix");
			reprojection_matrix_.create( 3, 3, CV_64FC1);
			double* f_ptr = reprojection_matrix_.ptr<double>(0);
			for (int i = 0; i < 9; i++)
				f_ptr[i] = data->reprojection_matrix[i];

			std::cout << "\t... / " << std::setw(8) <<  reprojection_matrix_.at<double>(0, 0) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(0, 1) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(0, 2) << " \\ " << std::endl;
		        std::cout << "\t... | " << std::setw(8) << reprojection_matrix_.at<double>(1, 0) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(1, 1) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(1, 2) << " | "<< std::endl;;
		        std::cout << "\t... \\ " << std::setw(8) << reprojection_matrix_.at<double>(2, 0) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(2, 1) << " ";
		        std::cout << std::setw(8) << reprojection_matrix_.at<double>(2, 2) << " / "<< std::endl << std::endl;
		}
	}

	bool detectObjectCallback(cob_srvs::DetectObjects::Request &req, cob_srvs::DetectObjects::Response &res)
	{
		std::string object_name = req.object_name.data;
		if (object_name == "")
			object_name = "ALL_OBJECTS";
	
		// Request colored point cloud
		if (getMeasurement() == false)
		{
			ROS_ERROR("[object_detection] Colored point cloud service call [FAILED].");				
			return false;
		}

		// Detect objects
		if (detectObjects(object_name) == false)
		{
			ROS_ERROR("[object_detection] Object detection [FAILED].");				
			return false;
		}

		// Return detection results
		assignDetectionResults(res.object_list);

		// Render detection results
		if (visualization_) 
			reprojectTransformationResults();
		
		return true;
	}

	bool assignDetectionResults(cob_msgs::DetectionArray& res)
	{
		for (unsigned int i = 0; i < detection_results_.size(); i++)
 		{
			cob_msgs::Detection object_instance;
			object_instance.label = detection_results_[i].m_ObjectName;
			object_instance.detector = "IPA_3D_object_recognition";
			object_instance.score = detection_results_[i].m_DetectionQuality;

			// TODO: Set Mask

			object_instance.pose.pose.position.x =  detection_results_[i].m_tx;
			object_instance.pose.pose.position.y =  detection_results_[i].m_ty;
			object_instance.pose.pose.position.z =  detection_results_[i].m_tz;

			double w = detection_results_[i].m_q1;
			double q1 = detection_results_[i].m_q2;
			double q2 = detection_results_[i].m_q3;
			double q3 = detection_results_[i].m_q4;
			
			object_instance.pose.pose.orientation.x =  q1;
			object_instance.pose.pose.orientation.y =  q2;
			object_instance.pose.pose.orientation.z =  q3;
			object_instance.pose.pose.orientation.w =  w;
			
			object_instance.pose.header.stamp = ros::Time::now();
			object_instance.pose.header.frame_id = "/head_color_camera_l_link";

			// Rotation about x-axis
//			object_instance.roll = Wm4::Mathd::ATan2(2*q0*q1+2*q2*q3, 1-2*q1*q1-2*q2*q2);
			// Rotation about y-axis
//			object_instance.pitch = Wm4::Mathd::ASin(2*q0*q2-2*q3*q1);
			// Rotation about z-axis
//			object_instance.yaw = Wm4::Mathd::ATan2(2*q0*q3+2*q1*q2, 1-2*q2*q2*-2*q3*q3);  
			res.detections.push_back(object_instance);
			ROS_INFO("Detected object '%s' at ( %f, %f, %f, %f, %f, %f, %f ) ",
				object_instance.label.c_str(), object_instance.pose.pose.position.x, 
				object_instance.pose.pose.position.y, object_instance.pose.pose.position.z,
				w, q1, q2, q3);

			// Broadcast transform of object
			tf::Transform transform;
			std::stringstream tf_name;
			tf_name << "object_" << i; 
			transform.setOrigin(tf::Vector3(detection_results_[i].m_tx, detection_results_[i].m_ty, detection_results_[i].m_tz));
			transform.setRotation(tf::Quaternion(q1, q2, q3, w));
			tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/head_color_camera_l_link", tf_name.str()));
		}
		
		return true;
	}
		
	bool getMeasurement()
	{
		ROS_INFO("[object_detection] Colored point cloud service call.");
	
		if(service_client_colored_pc_.call(service_colored_pc_))
		{
			ROS_INFO("[object_detection] Colored point cloud service call [OK].");
		}
		else
		{
			ROS_ERROR("[object_detection] Colored point cloud service call [FAILED].");
			return false;
		}

		sensor_msgs::PointCloud2ConstPtr pc(&(service_colored_pc_.response.pointCloud2), voidDeleter);

		cv::Mat color_image_8U3(pc->height, pc->width, CV_8UC3);
		cv::Mat xyz_image_32F3(pc->height, pc->width, CV_32FC3);
		float* f_ptr = 0;
		const uint8_t* data_ptr = 0;
		unsigned char* uc_ptr = 0;
		unsigned int xyz_offset = pc->fields[0].offset;
		unsigned int rgb_offset = pc->fields[3].offset;
		size_t b_offset = 2*sizeof(unsigned char);
		size_t g_offset = sizeof(unsigned char);
		size_t r_offset = 0;
		unsigned int col_times_3 = 0;
		for (unsigned int row = 0; row < pc->height; row++)	
		{
			uc_ptr = color_image_8U3.ptr<unsigned char>(row);
			f_ptr = xyz_image_32F3.ptr<float>(row);

			data_ptr = &pc->data[row * pc->width * pc->point_step];
			
			for (unsigned int col = 0; col < pc->width; col++)	
			{
				col_times_3 = 3*col;
				// Reorder incoming image channels
				memcpy(&uc_ptr[col_times_3], &data_ptr[col * pc->point_step + rgb_offset + b_offset], sizeof(unsigned char));
				memcpy(&uc_ptr[col_times_3 + 1], &data_ptr[col * pc->point_step + rgb_offset + g_offset], sizeof(unsigned char));
				memcpy(&uc_ptr[col_times_3 + 2], &data_ptr[col * pc->point_step + rgb_offset + r_offset], sizeof(unsigned char));

				memcpy(&f_ptr[col_times_3], &data_ptr[col * pc->point_step + xyz_offset], 3*sizeof(float));
			}
		}

		// Images are cloned within setter functions
		colored_pc_->SetColorImage(color_image_8U3);
		colored_pc_->SetXYZImage(xyz_image_32F3);

//    		cv::Mat xyz_image_8U3;
//		ipa_Utils::ConvertToShowImage(colored_pc_->GetXYZImage(), xyz_image_8U3, 3);
//	    	cv::imshow("xyz data", xyz_image_8U3);
//	    	cv::imshow("color data", colored_pc_->GetColorImage());
//	    	cv::waitKey();

		return true;
	}

	bool detectObjects(std::string object_name)
	{
		// Check if any object models have been loaded
		if(object_model_->IsEmpty())
		{
			ROS_ERROR("[object_detection] No object models available for detection");
			ROS_ERROR("[object_detection] Model map has not been initialized");
			return false;
		}

		
		// Call the detection function
		if(object_model_->DetectModel(object_name, *colored_pc_) & ipa_Utils::RET_FAILED)
		{
			return false;
		}
		
		// Retrieve the detection results
		detection_results_ = object_model_->GetDetectionResults();
		return true;
	}

	bool reprojectTransformationResults()
	{
		cv::Mat image_8U3 = colored_pc_->GetColorImage();
		
		if (output_image_8U3_.empty())
		{
			cvNamedWindow("XYZRGB image");
			cvStartWindowThread();
		}

		// Check if user manually closed the window and abort
		// drawing if so
		if (!cvGetWindowHandle("XYZRGB image"))
			return true;

		std::vector<int> u_vec;
		std::vector<int> v_vec;
		std::vector<ipa_Utils::Vector3d> pVec;
	
		ipa_Utils::Vector3d p;
		int u=0;
		int v=0;
		
		ipa_Utils::Vector3d translation;
		ipa_Utils::Quaterniond q;

		// Render 2-D detection results
		for(unsigned int j=0; j<detection_results_.size(); j++)
		{
			/// Render detection results if object has been detected
			if (detection_results_[j].m_DetectionQuality > 1.8)
			{
				cv::Point pUp = cv::Point(detection_results_[j].m_u, detection_results_[j].m_v-20);
				cv::Point pDown = cv::Point(detection_results_[j].m_u, detection_results_[j].m_v+20);
				cv::Point pLeft = cv::Point(detection_results_[j].m_u-20, detection_results_[j].m_v);
				cv::Point pRight = cv::Point(detection_results_[j].m_u+20, detection_results_[j].m_v);
	
				cv::line(image_8U3, pLeft, pRight, cv::Scalar(255, 255, 255), 6);
				cv::line(image_8U3, pUp, pDown, cv::Scalar(255, 255, 255), 6);
				cv::circle(image_8U3, cv::Point(detection_results_[j].m_u, detection_results_[j].m_v), 6, cv::Scalar(255, 255, 255), 1);
	
				cv::line(image_8U3, pLeft, pRight, cv::Scalar(0, 255, 0), 1);
				cv::line(image_8U3, pUp, pDown, cv::Scalar(0, 255, 0), 1);
				cv::circle(image_8U3, cv::Point(detection_results_[j].m_u, detection_results_[j].m_v), 3, cv::Scalar(0, 255, 0), 1);
			}
		}
	
		// Render 3-D detection results
		// Reproject bounding box of the object on the image
		if (pc_mode_ == ColoredPointCloudToolbox::MODE_STEREO)
		{
			for (unsigned int i=0; i<detection_results_.size(); i++)
			{
				pVec.clear();
				u_vec.clear();
				v_vec.clear();
		
				/// Get transformation
				translation[0] = detection_results_[i].m_tx;
				translation[1] = detection_results_[i].m_ty;
				translation[2] = detection_results_[i].m_tz;
				q[0] = detection_results_[i].m_q1;
				q[1] = detection_results_[i].m_q2;
				q[2] = detection_results_[i].m_q3;
				q[3] = detection_results_[i].m_q4;
		
				/// Express bounding box in xyz coordinates
				double y = detection_results_[i].m_MinY;
				for (int j=0; j<2; j++)
				{
					p[0] = detection_results_[i].m_MaxX;
					p[1] = y;
					p[2] = detection_results_[i].m_MinZ;
					pVec.push_back(p);
	
					p[0] = detection_results_[i].m_MinX;
					p[1] = y;
					p[2] = detection_results_[i].m_MinZ;
					pVec.push_back(p);
		
					p[0] = detection_results_[i].m_MinX;
					p[1] = y;
					p[2] = detection_results_[i].m_MaxZ;
					pVec.push_back(p);
		
					p[0] = detection_results_[i].m_MaxX;
					p[1] = y;
					p[2] = detection_results_[i].m_MaxZ;
					pVec.push_back(p);
		
					y = detection_results_[i].m_MaxY;
				}
		
				/// Reproject xyz coordinates to image coordinates
				for (unsigned int j=0; j<pVec.size(); j++)
				{
					ipa_Utils::RotateTranslate(pVec[j], translation, q);
					colored_pc_toolbox_->ReprojectXYZ(pVec[j][0], pVec[j][1], pVec[j][2], u , v, &reprojection_matrix_);
					u_vec.push_back(u);
					v_vec.push_back(v);
				}
		
				/// Draw lower rectangle of bounding box
				for (unsigned int j=0; j<3; j++)
				{
					cv::line(image_8U3, cv::Point(u_vec[j], v_vec[j]), cv::Point(u_vec[j+1], v_vec[j+1]), detection_results_[i].m_rgb, 3);
				}
				cv::line(image_8U3, cv::Point(u_vec[3], v_vec[3]), cv::Point(u_vec[0], v_vec[0]), detection_results_[i].m_rgb);
		
				/// Draw upper rectangle of bounding box
				for (unsigned int j=4; j<pVec.size()-1; j++)
				{
					cv::line(image_8U3, cv::Point(u_vec[j], v_vec[j]), cv::Point(u_vec[j+1], v_vec[j+1]), detection_results_[i].m_rgb, 3);
				}
				cv::line(image_8U3, cv::Point(u_vec[7], v_vec[7]), cv::Point(u_vec[4], v_vec[4]), detection_results_[i].m_rgb);
		
				/// Draw side lines of bounding box
				for (unsigned int j=0; j<4; j++)
				{
					cv::line(image_8U3, cv::Point(u_vec[j], v_vec[j]), cv::Point(u_vec[j+4], v_vec[j+4]), detection_results_[i].m_rgb, 3);
				}
			}
		}
			
		cv::resize(image_8U3, output_image_8U3_, cv::Size(), 0.5, 0.5);
		cv::imshow("XYZRGB image", output_image_8U3_);
			
		return true;
	}

	bool resetImageCounter()
	{
		image_counter_ = 0;
		return true;
	}
		
	bool acquireImageCallback(cob_srvs::AcquireObjectImage::Request &req, cob_srvs::AcquireObjectImage::Response &res)
	{
		ROS_INFO("[object_detection] AcquireObjectImage service call");				

		// Request colored point cloud
		if (getMeasurement() == false)
		{
			ROS_ERROR("[object_detection] Colored point cloud service call [FAILED].");				
			return false;
		}
		
		segmentAndSaveTrainingImage(req.object_name, req.reset_image_counter);
			
		return true;
	}

	bool segmentAndSaveTrainingImage(std::string object_name, bool reset_image_counter)
	{
			
		// Segment colored point cloud
		cv::Scalar uv_boundaries(0, 424242, 0, 424242);
		// Reproject bounding box of the object on the image and calculate extremal
		// image coordinates of segmentation area
		if (pc_mode_ == ColoredPointCloudToolbox::MODE_STEREO)
		{
			double z0 = xyzr_learning_coordinates_.val[2] - xyzr_learning_coordinates_.val[3];
			double z1 = xyzr_learning_coordinates_.val[2] + xyzr_learning_coordinates_.val[3];
			double minZ = std::min(z0, z1);
	
			double x0 = xyzr_learning_coordinates_.val[0] - xyzr_learning_coordinates_.val[3];
			double x1 = xyzr_learning_coordinates_.val[0] + xyzr_learning_coordinates_.val[3];
			double maxX = std::max(x0, x1);
			double minX = std::min(x0, x1);
	
			double y0 = xyzr_learning_coordinates_.val[1] - xyzr_learning_coordinates_.val[3];
			double y1 = xyzr_learning_coordinates_.val[1] + xyzr_learning_coordinates_.val[3];
			double maxY = std::max(y0, y1);
			double minY = std::min(y0, y1);
	
			int minU = 0;
			int maxU = 0;
			int minV = 0;
			int maxV = 0;

			std::cout << minX << " " << maxX << " " << minY << " " << maxY << " " << minZ << " " << std::endl; 

			colored_pc_toolbox_->ReprojectXYZ(minX, minY, minZ, minU, minV, &reprojection_matrix_);
			colored_pc_toolbox_->ReprojectXYZ(maxX, maxY, minZ, maxU, maxV, &reprojection_matrix_);
	
			uv_boundaries = cv::Scalar(minU, maxU, minV, maxV);
		}
	
		ROS_INFO("[object detection] Segmentation boundaries");
		ROS_INFO("[object detection] %f <= u <= %f, %f <= v <= %f", 
			uv_boundaries.val[0], uv_boundaries.val[1], uv_boundaries.val[2], uv_boundaries.val[3]);
		
		colored_pc_toolbox_->DoRangeSegmentation(colored_pc_, &xyzr_learning_coordinates_, &uv_boundaries, pc_mode_);
		colored_pc_toolbox_->CutImageBorder(colored_pc_, 200);

		if (reset_image_counter) 
			image_counter_ = 0;
				
		saveTrainingImage(object_name, image_counter_);

		if (visualization_)
		{
			if (output_image_8U3_.empty()) 
			{
				cvNamedWindow("XYZRGB image");
				cvStartWindowThread();
			}

			if (cvGetWindowHandle("XYZRGB image"))
			{
				cv::resize(colored_pc_->GetColorImage(), output_image_8U3_, cv::Size(), 0.5, 0.5);
				cv::imshow("XYZRGB image", output_image_8U3_);
			}
		}

		image_counter_++;
		return true;
	}
	
	bool saveTrainingImage(std::string objectName, int index)
	{
		std::stringstream file_name_stream;
		file_name_stream << segmented_images_directory_ << objectName << "_info.txt";
		/// Delete old info header
		remove((file_name_stream.str()).c_str());
	
		/// Delete old images with same filename
		std::stringstream file_name_stream2;
		file_name_stream2 << segmented_images_directory_ << objectName << "_" << index;
		colored_pc_->DeleteColoredPointCloud(file_name_stream2.str());
		
		/// Create and save new info header
		std::ofstream f((file_name_stream.str()).c_str());
		if(!f.is_open()) 
		{
			ROS_ERROR("[object_detection] Could not open file '%s'", file_name_stream.str().c_str());				
			return false;
		}
		f << index+1 << std::endl;
		
		/// Save image
		colored_pc_->SaveColoredPointCloud(file_name_stream2.str());
		
		return true;
	}
		
		
	bool trainObjectCallback(cob_srvs::TrainObject::Request &req, cob_srvs::TrainObject::Response &res)
	{
		if (!trainObjectModel(req.object_name))
		{
			ROS_ERROR("[object_detection] object model training failed");				
			return false;
		}
		return true;
	}

	bool trainObjectModel(std::string object_name)
	{

		ColoredPointCloudSequencePtr pc_seq = ipa_SensorFusion::CreateColoredPointCloudSequence();
		
		if(pc_seq->LoadColoredPointCloudSequence(segmented_images_directory_+object_name, 500) & ipa_Utils::RET_FAILED)
		{	
			ROS_ERROR("[object_detection] Loading shared image sequence failed");
			return false;
		}
		
		/// Train object
		ROS_INFO("[object_detection] Training object '%s'", object_name.c_str());
		object_model_->SetLearningCenter(xyzr_learning_coordinates_);

		ipa_ObjectModel::AbstractObjectModelParametersPtr params = ipa_ObjectModel::AbstractObjectModelParametersPtr(new ipa_ObjectModel::AbstractObjectModelParameters());
		params->objectName = object_name;
		if(object_model_->BuildAndAddModel(params, (*pc_seq)) & ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("[object_detection] Model creation failed");
			return false;
		}
		
		if(object_model_->Save(model_directory_) & ipa_Utils::RET_FAILED)
		{
			ROS_ERROR("[object_detection] Saving object model failed");
			return false;
		}
		
		return true;
	}

	bool loadParameters()
	{
		std::string tmp_string;
		double learning_pos_x; ///< x-val of object position for learning
		double learning_pos_y; ///< y-val of object position for learning
		double learning_pos_z; ///< z-val of object position for learning
		double learning_radius; ///< radius of object model for learning
	
		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/model_directory", directory_) == false)
		{
			ROS_ERROR("[object_detection] Path to model files not specified");
			return false;
		}

		ROS_INFO("Working directory: %s", directory_.c_str());

		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/object_model_type", tmp_string) == false)
		{
			ROS_ERROR("[object_detection] object model type");
			return false;
		}
		if (tmp_string == "MODEL_BAYES")
		{	
			object_model_type_ = ipa_ObjectModel::MODEL_BAYES;
		}
		else if (tmp_string == "MODEL_SLAM_6D")
		{	
			object_model_type_ = ipa_ObjectModel::MODEL_SLAM_6D;
		}
		else if (tmp_string == "MODEL_SIFT")
		{	
			object_model_type_ = ipa_ObjectModel::MODEL_SIFT;
		}
		else 
		{
			std::string str = "[object_detection] Object model type '" + tmp_string + "' unknown, try 'MODEL_6D_CLASSIFIER' or 'MODEL_SIFT'";	
			ROS_ERROR("%s", str.c_str());
			return false;
		}

		ROS_INFO("Object model type: %s", tmp_string.c_str());

		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/visualization", visualization_) == false)
		{
			ROS_ERROR("[object_detection] shared working size factor not specified");
			return false;
		}

		if (visualization_) ROS_INFO("Visualization of detection results: TRUE");
		else ROS_INFO("Visualization of detection results: FALSE");
	
		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/learning_pos_x", learning_pos_x) == false)
		{
			ROS_ERROR("[object_detection] X value of learning position not specified");
			return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/learning_pos_y", learning_pos_y) == false)
		{
			ROS_ERROR("[object_detection] Y value of learning position not specified");
			return false;
		}

		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/learning_pos_z", learning_pos_z) == false)
		{
			ROS_ERROR("[object_detection] Z value of learning position not specified");
			return false;
		}

		ROS_INFO("Learning coordinates (x - y - z): %f - %f - %f", learning_pos_x, learning_pos_y, learning_pos_z);

		/// Parameters are set within the launch file
		if (node_handle_.getParam("object_detection/learning_radius", learning_radius) == false)
		{
			ROS_ERROR("[object_detection] Radius of learning position not specified");
			return false;
		}

		ROS_INFO("Learning radius: %f", learning_radius);
		xyzr_learning_coordinates_ = cvScalar(learning_pos_x, learning_pos_y, learning_pos_z, learning_radius);

		return true;
	}
};


// Action class for object detection
class DetectObjectsAction
{
protected:
	actionlib::SimpleActionServer<cob_object_detection::DetectObjectsAction> as_detect_;	///< The action server for object detection
	cob_object_detection::DetectObjectsFeedback as_detect_feedback_;	///< The feedback message
	cob_object_detection::DetectObjectsResult as_detect_result_;	///< the result message

	CobObjectDetectionNode& object_detection_node_;
	ros::NodeHandle node_handle_; ///< ROS node handle
public:
	DetectObjectsAction(CobObjectDetectionNode& object_detection_node, const ros::NodeHandle& node_handle)
	: as_detect_(node_handle, "detect_object", boost::bind(&DetectObjectsAction::detectObjectActionCallback, this, _1), false),
	object_detection_node_(object_detection_node),
	node_handle_(node_handle)
	{
		as_detect_.start();
	}
	
	void detectObjectActionCallback(const cob_object_detection::DetectObjectsGoalConstPtr &goal)
	{
		std::string object_name = goal->object_name.data;
		if (object_name == "")
			object_name = "ALL_OBJECTS";
	
		// Request colored point cloud
		if (object_detection_node_.getMeasurement() == false)
		{
			ROS_ERROR("[object_detection] Colored point cloud service call [FAILED].");				
			return;
		}

		as_detect_feedback_.percent_complete = 0.2;
		as_detect_.publishFeedback(as_detect_feedback_);
		if (as_detect_.isPreemptRequested() || !ros::ok())
		{		
			ROS_INFO("[object_detection] object detection preempted");
			// set the action state to preempted
			as_detect_.setPreempted();
			return;
		}


		// Detect objects
		if (object_detection_node_.detectObjects(object_name) == false)
		{
			ROS_ERROR("[object_detection] Object detection [FAILED].");				
			return;
		}

		as_detect_feedback_.percent_complete = 0.9;
		as_detect_.publishFeedback(as_detect_feedback_);
		if (as_detect_.isPreemptRequested() || !ros::ok())
		{		
			ROS_INFO("[object_detection] object detection preempted");
			// set the action state to preempted
			as_detect_.setPreempted();
			return;
		}

		// Assign detection results
		object_detection_node_.assignDetectionResults(as_detect_result_.object_list);

		as_detect_feedback_.percent_complete = 0.95;
		as_detect_.publishFeedback(as_detect_feedback_);
		if (as_detect_.isPreemptRequested() || !ros::ok())
		{		
			ROS_INFO("[object_detection] object detection preempted");
			// set the action state to preempted
			as_detect_.setPreempted();
			return;
		}

		// Render detection results
		if (object_detection_node_.visualization_) 
			object_detection_node_.reprojectTransformationResults();

		as_detect_feedback_.percent_complete = 1.0;
		as_detect_.publishFeedback(as_detect_feedback_);
		as_detect_.setSucceeded();
	}
};

// Action class for image acquisition
class AcquireObjectImageAction
{
protected:
	actionlib::SimpleActionServer<cob_object_detection::AcquireObjectImageAction> as_acquire_;	///< The action server for object detection
	cob_object_detection::AcquireObjectImageFeedback as_acquire_feedback_;	///< The feedback message
	cob_object_detection::AcquireObjectImageResult as_acquire_result_;	///< the result message

	CobObjectDetectionNode& object_detection_node_;
	ros::NodeHandle node_handle_; ///< ROS node handle
public:
	AcquireObjectImageAction(CobObjectDetectionNode& object_detection_node, const ros::NodeHandle& node_handle)
	: as_acquire_(node_handle, "acquire_object_image", boost::bind(&AcquireObjectImageAction::acquireImageActionCallback, this, _1), false),
	object_detection_node_(object_detection_node),
	node_handle_(node_handle)
	{
		as_acquire_.start();
	}

	void acquireImageActionCallback(const cob_object_detection::AcquireObjectImageGoalConstPtr &goal)
	{
		ROS_INFO("[object_detection] AcquireObjectActionImage action call");				

		// Request colored point cloud
		if (object_detection_node_.getMeasurement() == false)
		{
			ROS_ERROR("[object_detection] Colored point cloud service call [FAILED].");				
			return;
		}
		
		as_acquire_feedback_.percent_complete = 0.5;
		as_acquire_.publishFeedback(as_acquire_feedback_);
		if (as_acquire_.isPreemptRequested() || !ros::ok())
		{		
			ROS_INFO("[object_detection] image acquisition preempted");
			// set the action state to preempted
			as_acquire_.setPreempted();
			return;
		}

		object_detection_node_.segmentAndSaveTrainingImage(goal->object_name, goal->reset_image_counter);
	
		as_acquire_feedback_.percent_complete = 1.0;
		as_acquire_.publishFeedback(as_acquire_feedback_);
		as_acquire_.setSucceeded();

		return;
	}
};

// Action class for object training
class TrainObjectAction
{
protected:
	actionlib::SimpleActionServer<cob_object_detection::TrainObjectAction> as_train_;	///< The action server for object training
	cob_object_detection::TrainObjectFeedback as_train_feedback_;	///< The feedback message
	cob_object_detection::TrainObjectResult as_train_result_;	///< the result message

	CobObjectDetectionNode& object_detection_node_;
	ros::NodeHandle node_handle_; ///< ROS node handle
public:
	TrainObjectAction(CobObjectDetectionNode& object_detection_node, const ros::NodeHandle& node_handle)
	: as_train_(node_handle, "train_object", boost::bind(&TrainObjectAction::trainObjectActionCallback, this, _1), false),
	object_detection_node_(object_detection_node),
	node_handle_(node_handle)
	{
		as_train_.start();
	}

	void trainObjectActionCallback(const cob_object_detection::TrainObjectGoalConstPtr &goal)
	{
		if (!object_detection_node_.trainObjectModel(goal->object_name))
		{
			ROS_ERROR("[object_detection] object model training failed");				
			return;
		}
	
		as_train_feedback_.percent_complete = 1.0;
		as_train_.publishFeedback(as_train_feedback_);
		as_train_.setSucceeded();
		
		return;
	}
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, spezify name of node
	ros::init(argc, argv, "object_detection");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;
	
	// Create object detection  node class instance   
	CobObjectDetectionNode object_detection_node(nh);

	// Initialize object detection node
	if (!object_detection_node.init()) 
		return 0;

	// Create action nodes
	DetectObjectsAction detect_action_node(object_detection_node, nh);
	AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	TrainObjectAction train_object_node(object_detection_node, nh);
	
	ros::spin();
    
	return 0;
}
*/
