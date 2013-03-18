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
	#include "cob_people_detection/face_recognizer.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

// stream
#include <fstream>

// opencv
#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

// boost
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/convenience.hpp"
#include "boost/filesystem/path.hpp"
namespace fs = boost::filesystem;


using namespace ipa_PeopleDetector;

ipa_PeopleDetector::FaceRecognizer::FaceRecognizer(void)
{
	m_eigenvectors_ipl = 0;
}

ipa_PeopleDetector::FaceRecognizer::~FaceRecognizer(void)
{
	if (m_eigenvectors_ipl != 0)
	{
		for (uint i=0; i<m_eigenvectors.size(); i++)
			cvReleaseImage(&(m_eigenvectors_ipl[i]));
		cvFree(&m_eigenvectors_ipl);
	}
}

unsigned long ipa_PeopleDetector::FaceRecognizer::init(std::string data_directory, int eigenface_size, int metric, bool debug, std::vector<std::string>& identification_labels_to_recognize,int subs_meth,int class_meth,bool use_unknown_thresh,bool use_depth)
{
	// parameters
	m_data_directory = data_directory;
	m_eigenface_size = eigenface_size;
	//	m_eigenvectors_per_person = eigenvectors_per_person;
	//	m_threshold_facespace = threshold_facespace;
	//	m_threshold_unknown = threshold_unknown;
	m_metric = metric;
	m_debug = debug;
	m_depth_mode = use_depth;
	m_use_unknown_thresh = use_unknown_thresh;

	switch (subs_meth)
	{
	case 0:
	{
		m_subs_meth = SubspaceAnalysis::METH_FISHER;
		break;
	}
	case 1:
	{
		m_subs_meth = SubspaceAnalysis::METH_EIGEN;
		break;
	}
	default:
	{
		m_subs_meth = SubspaceAnalysis::METH_FISHER;
		break;
	}
	};

	switch (class_meth)
	{
	case 0:
	{
		m_class_meth = SubspaceAnalysis::CLASS_DIFS;
		break;
	}
	case 1:
	{
		m_class_meth = SubspaceAnalysis::CLASS_KNN;
		break;
	}
	case 2:
	{
		m_class_meth = SubspaceAnalysis::CLASS_SVM;
		break;
	}
	default:
	{
		m_class_meth = SubspaceAnalysis::CLASS_DIFS;
		break;
	}
	};

	// load model
	loadRecognitionModel(identification_labels_to_recognize);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::initTraining(std::string data_directory, int eigenface_size, bool debug, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps,bool use_depth)
{
	// parameters
	m_data_directory = data_directory;
	m_eigenface_size = eigenface_size;
	m_debug = debug;
  m_depth_mode=use_depth;

	// load model
	m_current_label_set.clear();	 // keep empty to load all available data
	loadTrainingData(face_images,face_depthmaps, m_current_label_set);

	return ipa_Utils::RET_OK;
}


unsigned long ipa_PeopleDetector::FaceRecognizer::addFace(cv::Mat& color_image, cv::Mat& depth_image,cv::Rect& face_bounding_box,cv::Rect& head_bounding_box,std::string label, std::vector<cv::Mat>& face_images,std::vector<cv::Mat>& face_depthmaps)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

//	// store in appropriate format for this method
//	cv::Mat resized_8U1;
//	cv::Size new_size(m_eigenface_size, m_eigenface_size);
//	convertAndResize(color_image, resized_8U1, face_bounding_box, new_size);


  cv::Rect combined_face_bounding_box=cv::Rect(face_bounding_box.x+head_bounding_box.x,face_bounding_box.y+head_bounding_box.y,face_bounding_box.width,face_bounding_box.height);

	cv::Mat roi_color = color_image(combined_face_bounding_box);
	cv::Mat roi_depth_xyz = depth_image(face_bounding_box).clone();
  cv::Size norm_size=cv::Size(m_eigenface_size,m_eigenface_size);
  cv::Mat roi_depth;
  //TODO MAKE TEMPORARY SWITCH OFF
  //if(!face_normalizer_.normalizeFace(roi_color,roi_depth_xyz,norm_size)) ;
  if(!face_normalizer_.normalizeFace(roi_color,roi_depth_xyz,norm_size)) return ipa_Utils::RET_FAILED;



	// Save image
	face_images.push_back(roi_color);
  face_depthmaps.push_back(roi_depth_xyz);
	m_face_labels.push_back(label);
  dm_exist.push_back(true);

	return ipa_Utils::RET_OK;
}
unsigned long ipa_PeopleDetector::FaceRecognizer::addFace(cv::Mat& color_image, cv::Mat& depth_image,cv::Rect& face_bounding_box,cv::Rect& head_bounding_box,std::string label, std::vector<cv::Mat>& face_images)
{

	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

//	// store in appropriate format for this method
//	cv::Mat resized_8U1;
//	cv::Size new_size(m_eigenface_size, m_eigenface_size);
//	convertAndResize(color_image, resized_8U1, face_bounding_box, new_size);


  cv::Rect combined_face_bounding_box=cv::Rect(face_bounding_box.x+head_bounding_box.x,face_bounding_box.y+head_bounding_box.y,face_bounding_box.width,face_bounding_box.height);

	cv::Mat roi_color = color_image(combined_face_bounding_box);
	cv::Mat roi_depth_xyz= depth_image(face_bounding_box);
  cv::Size norm_size=cv::Size(m_eigenface_size,m_eigenface_size);
  cv::Mat roi_depth;
  if(!face_normalizer_.normalizeFace(roi_color,roi_depth_xyz,norm_size)) return ipa_Utils::RET_FAILED;
  //if(!face_normalizer_.normalizeFace(roi_color,norm_size)) return ipa_Utils::RET_FAILED;
  cv::imshow("FACE TO ADD",roi_color);
  cv::waitKey(50);



	// Save image
	face_images.push_back(roi_color);
	m_face_labels.push_back(label);

	return ipa_Utils::RET_OK;
}
unsigned long ipa_PeopleDetector::FaceRecognizer::addFace(cv::Mat& color_image, cv::Rect& face_bounding_box, std::string label, std::vector<cv::Mat>& face_images)
{

	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

//	// store in appropriate format for this method
//	cv::Mat resized_8U1;
//	cv::Size new_size(m_eigenface_size, m_eigenface_size);
//	convertAndResize(color_image, resized_8U1, face_bounding_box, new_size);

	// keep image in original format --> more flexibility later
	cv::Mat roi = color_image(face_bounding_box);

	// Save image
	face_images.push_back(roi);
	m_face_labels.push_back(label);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::updateFaceLabels(std::string old_label, std::string new_label)
{
	for (int i=0; i<(int)m_face_labels.size(); i++)
	{
		if (m_face_labels[i].compare(old_label) == 0)
			m_face_labels[i] = new_label;
	}
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::updateFaceLabel(int index, std::string new_label)
{
	m_face_labels[index] = new_label;
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::deleteFaces(std::string label, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps)
{
	for (int i=0; i<(int)m_face_labels.size(); i++)
	{
		if (m_face_labels[i].compare(label) == 0)
		{
			m_face_labels.erase(m_face_labels.begin()+i);
			face_images.erase(face_images.begin()+i);
      //TODO delete depthmaps
			//face_depthmaps.erase(face_depthmaps.begin()+i);
			i--;
		}
	}
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::deleteFace(int index, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps)
{
	m_face_labels.erase(m_face_labels.begin()+index);
	face_images.erase(face_images.begin()+index);
      //TODO delete depthmaps
	//face_depthmaps.erase(face_depthmaps.begin()+index);
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::initModel(SubspaceAnalysis::FishEigFaces& eff, std::vector<cv::Mat>& data, std::vector<int>& labels)
{
	//TODO set ss_dim dynamically
	int ss_dim = 1;

	std::vector<cv::Mat> in_vec;
	for (unsigned int i = 0; i < data.size(); i++)
	{

		cv::Mat temp = data[i];
		temp.convertTo(temp, CV_64FC1);
		in_vec.push_back(temp);
	}

	m_rec_method = 1;
	if (!eff.trainModel(in_vec, labels, ss_dim, m_subs_meth, true, m_use_unknown_thresh))
	{
		std::cout << "[FACEREC] Reognition module could not be initialized ....!" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::trainRecognitionModel(std::vector<std::string>& identification_labels_to_train)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	// load necessary data

	// If depth mode is enabled load available depthmaps
	std::vector<cv::Mat> face_images, face_depthmaps;
	if (m_depth_mode)
	{
		loadTrainingData(face_images, face_depthmaps, identification_labels_to_train);

		//make label vec for depth
		depth_str_labels.clear();
		depth_str_labels_unique.clear();
		depth_num_labels.clear();

		//int lbl = 0;
		for (unsigned int i = 0; i < dm_exist.size(); i++)
		{
			if (dm_exist[i])
			{
				depth_str_labels.push_back(m_face_labels[i]);
				bool unique = true;
				for (unsigned int k = 0; k < depth_str_labels_unique.size(); k++)
				{
					if (depth_str_labels_unique[k].compare(m_face_labels[i]) == 0)
						unique = false;
				}
				if (unique == true)
					depth_str_labels_unique.push_back(m_face_labels[i]);
			}
		}

		for (unsigned int li = 0; li < depth_str_labels.size(); li++)
		{
			for (unsigned int lj = 0; lj < depth_str_labels_unique.size(); lj++)
			{
				if (depth_str_labels_unique[lj].compare(depth_str_labels[li]) == 0)
					depth_num_labels.push_back(lj);
			}
		}

	}
	else if (!m_depth_mode)
	{
		loadTrainingData(face_images, identification_labels_to_train);
	}

	m_current_label_set = identification_labels_to_train;

	// convert face_images to right format if necessary
	//cv::Size new_size(m_eigenface_size, m_eigenface_size);
	//for (int i=0; i<face_images.size(); i++)
	//{
	//	// convert to grayscale if necessary
	//	if (face_images[i].type() == CV_8UC3)
	//	{
	//		cv::Mat temp = face_images[i];
	//		cv::cvtColor(temp, face_images[i], CV_BGR2GRAY);
	//	}
	//	// rescale if necessary to m_eigenface_size
	//	if (face_images[i].cols != m_eigenface_size || face_images[i].rows != m_eigenface_size)
	//	{
	//		cv::Mat temp = face_images[i];
	//    //cv::imshow("k",temp);
	//    //cv::waitKey(0);
	//    std::cout<<"i="<<i<<std::endl;
	//    std::cout<<"new_size"<<new_size.width<<" "<<new_size.height<<std::endl;
	//    std::cout<<"face_i"<<temp.rows<<" "<<temp.cols<<std::endl;
	//		cv::resize(temp, face_images[i], new_size);
	//	}
	//}

	//	// PCA
	//	int number_eigenvectors = std::min(m_eigenvectors_per_person * identification_labels_to_train.size(), face_images.size()-1);
	//	bool return_value = PCA(number_eigenvectors, face_images);
	//	if (return_value == ipa_Utils::RET_FAILED)
	//		return ipa_Utils::RET_FAILED;

	//	// compute average face projections per class
	//	computeAverageFaceProjections();

	//	// save new model
	//	saveRecognitionModel();

	//--------------------------------------------
	//--------------------------------------------
	//--------------------------------------------

	m_label_num.clear();
	for (unsigned int li = 0; li < m_face_labels.size(); li++)
	{
		for (unsigned int lj = 0; lj < identification_labels_to_train.size(); lj++)
		{
			if (identification_labels_to_train[lj].compare(m_face_labels[li]) == 0)
				m_label_num.push_back(lj);
		}
	}

	//alocate memory for eigenvectors
	m_eigenvectors.clear();

	// reset effs
	eff_depth.releaseModel();
	eff_color.releaseModel();

	std::string path = m_data_directory + "training_data/";
	std::string path_color = m_data_directory + "training_data/" + "rdata_color.xml";
	std::string path_depth = m_data_directory + "training_data/" + "rdata_depth.xml";

	//--> INIT MODEL
	if (m_depth_mode)
	{
		if (face_depthmaps.size() > 0)
			initModel(eff_depth, face_depthmaps, depth_num_labels);
		eff_depth.saveModel(path_depth);
	}

	if (face_images.size() > 0)
		initModel(eff_color, face_images, m_label_num);
	eff_color.saveModel(path_color);

	//TODO ALWAYS TRAINING NECESSARY - NO INTERFACE FOR SSA CLASS FOR MODEL ASSOCIATION
	//saveRecognitionModel();


	// output
	if (m_debug)
	{
		std::cout << "FaceRecognizer::trainRecognitionModel: New model trained with labels: " << std::endl;
		for (int i = 0; i < (int)m_current_label_set.size(); i++)
			std::cout << "   - " << m_current_label_set[i] << std::endl;
		std::cout << std::endl;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::saveRecognitionModel()
{
	std::string path = m_data_directory + "training_data/";
	std::string filename = "rdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	if(fs::is_directory(path.c_str()))
	{
		if (fs::is_regular_file(complete.str().c_str()))
		{
			if (fs::remove(complete.str().c_str()) == false)
			{
				std::cout << "Error: FaceRecognizer::saveRecognitionModel: Cannot remove old recognizer data.\n" << std::endl;
				return ipa_Utils::RET_FAILED;
			}
		}

		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
		if(!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::saveRecognitionModel: Can't save training data.\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// Number eigenvalues/eigenvectors
		int number_eigenfaces = m_eigenvectors.size();
		fileStorage << "number_eigenfaces" << number_eigenfaces;

		// Eigenvectors
		for (int i=0; i<number_eigenfaces; i++)
		{
			std::ostringstream tag;
			tag << "ev_" << i;
			fileStorage << tag.str().c_str() << m_eigenvectors[i];
		}

		// Eigenvalue matrix
		fileStorage << "eigenvalues" << m_eigenvalues;

		// Average image
		fileStorage << "average_image" << m_average_image;

		// Projection coefficients of the training faces
		fileStorage << "projected_training_faces" << m_projected_training_faces;

		// corresponding labels to each face image projection in m_projected_training_faces
		fileStorage << "number_face_labels" << (int)m_face_labels.size();
		for (uint i=0; i<m_face_labels.size(); i++)
		{
			std::ostringstream tag;
			tag << "face_label_" << i;
			fileStorage << tag.str().c_str() << m_face_labels[i];
		}

		// The average factors of the eigenvector decomposition from each face class
		fileStorage << "face_class_average_projections" << m_face_class_average_projections;

		// A vector containing all different labels from the training session exactly once, order of appearance matters! (m_current_label_set[i] stores the corresponding name to the average face coordinates in the face subspace in m_face_class_average_projections.rows(i))
		fileStorage << "number_current_labels" << (int)m_current_label_set.size();
		for(int i=0; i<(int)m_current_label_set.size(); i++)
		{
			std::ostringstream tag;
			tag << "current_label_" << i;
			fileStorage << tag.str().c_str() << m_current_label_set[i].c_str();
		}

		fileStorage.release();

		// save classifier
		std::string classifier_file = path + "svm.dat";
		//m_face_classifier.save(classifier_file.c_str());	// todo

		std::cout << "INFO: FaceRecognizer::saveRecognitionModel: recognizer data saved.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::saveRecognitionModel: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::loadRecognitionModel(std::vector<std::string>& identification_labels_to_recognize)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex>* lock = new boost::lock_guard<boost::mutex>(m_data_mutex);

	// check whether currently trained data set corresponds with intentification_labels_to_recognize
	std::string path = m_data_directory + "training_data/";
	std::string filename = "rdata.xml";

	bool training_necessary = false;
	if(fs::is_directory(path.c_str()))
	{
		std::ostringstream complete;
		complete << path << filename;
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if(!fileStorage.isOpened())
		{
			std::cout << "Info: FaceRecognizer::loadRecognitionModel: Can't open " << complete.str() << ".\n" << std::endl;
			training_necessary = true;
		}
		else
		{
			// A vector containing all different labels from the training session exactly once, order of appearance matters! (m_current_label_set[i] stores the corresponding name to the average face coordinates in the face subspace in m_face_class_average_projections.rows(i))
			m_current_label_set.clear();
			int number_current_labels = (int)fileStorage["number_current_labels"];
			m_current_label_set.resize(number_current_labels);
			for(int i=0; i<number_current_labels; i++)
			{
				std::ostringstream tag;
				tag << "current_label_" << i;
				m_current_label_set[i] = (std::string)fileStorage[tag.str().c_str()];
			}

			// compare m_current_label_set with identification_labels_to_recognize, only load data if both vectors contain same elements in same order
			bool same_data_set = true;
			if (identification_labels_to_recognize.size()==0 || m_current_label_set.size()!=identification_labels_to_recognize.size())
				same_data_set = false;
			else
			{
				for (uint i=0; i<identification_labels_to_recognize.size(); i++)
				{
					if(identification_labels_to_recognize[i].compare(m_current_label_set[i]) != 0)
					{
						same_data_set = false;
						break;
					}
				}
			}

			if (same_data_set == true)
			{
				// stored set is equal to requested set -> just load the data

				// Number eigenvalues/eigenvectors
				int number_eigenfaces = (int)fileStorage["number_eigenfaces"];

				// Eigenvectors
				m_eigenvectors.clear();
				m_eigenvectors.resize(number_eigenfaces, cv::Mat());
				for (int i=0; i<number_eigenfaces; i++)
				{
					std::ostringstream tag;
					tag << "ev_" << i;
					fileStorage[tag.str().c_str()] >> m_eigenvectors[i];
				}

				// Eigenvalue matrix
				m_eigenvalues = cv::Mat();
				fileStorage["eigenvalues"] >> m_eigenvalues;

				// Average image
				m_average_image = cv::Mat();
				fileStorage["average_image"] >> m_average_image;

				// Projections of the training faces
				m_projected_training_faces = cv::Mat();
				fileStorage["projected_training_faces"] >> m_projected_training_faces;

				// corresponding labels to each face image projection in m_projected_training_faces
				int number_face_labels = (int)fileStorage["number_face_labels"];
				m_face_labels.clear();
				m_face_labels.resize(number_face_labels);
				for (int i=0; i<number_face_labels; i++)
				{
					std::ostringstream tag;
					tag << "face_label_" << i;
					fileStorage[tag.str().c_str()] >> m_face_labels[i];
				}

				// The average factors of the eigenvector decomposition from each face class
				m_face_class_average_projections = cv::Mat();
				fileStorage["face_class_average_projections"] >> m_face_class_average_projections;

				// load classifier
				std::string classifier_file = path + "svm.dat";
				//m_face_classifier_.load(classifier_file.c_str());  // todo

				std::cout << "INFO: FaceRecognizer::loadRecognitionModel: recognizer data loaded.\n" << std::endl;
			}
			else
			{
				training_necessary = true;
			}

			fileStorage.release();
			delete lock;
			lock = 0;
		}
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::loadRecognizerData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	if (training_necessary == true)
	{
		// stored set differs from requested set -> recompute the model from training data

		// release lock, trainRecognitionModel requests its own lock
		if (lock != 0)
			delete lock;

		bool return_value = trainRecognitionModel(identification_labels_to_recognize);
		if (return_value == ipa_Utils::RET_FAILED)
			return ipa_Utils::RET_FAILED;
	}

	if (m_debug == true)
	{
		std::cout << "Current model set:" << std::endl;
		for (int i=0; i<(int)m_current_label_set.size(); i++)
			std::cout << "   - " << m_current_label_set[i] << std::endl;
		std::cout << std::endl;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::recognizeFace(cv::Mat& color_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	if (eff_color.trained==false )
	{
		std::cout << "Error: FaceRecognizer::recognizeFace: Load or train some identification model, first.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	identification_labels.clear();

	cv::Mat resized_8U1;
	cv::Size resized_size(m_eigenvectors[0].size());
	for(int i=0; i<(int)face_coordinates.size(); i++)
	{
		cv::Rect face = face_coordinates[i];
    cv::Size norm_size=cv::Size(m_eigenface_size,m_eigenface_size);
		convertAndResize(color_image, resized_8U1, face, norm_size);



     double DFFS;
     resized_8U1.convertTo(resized_8U1,CV_64FC1);

      cv::Mat coeff_arr;
        eff_color.projectToSubspace(resized_8U1,coeff_arr,DFFS);

		if (m_debug) std::cout << "distance to face space: " << DFFS << std::endl;
    //TODO temporary turned off
		if(0==1)
		{
			// no face
			identification_labels.push_back("No face");
		}
		else
		{

      int res_label;
      eff_color.classify(coeff_arr,m_class_meth,res_label);
      if(res_label==-1)
      {
        identification_labels.push_back("Unknown Face");
      }
      else
      {
        identification_labels.push_back(m_current_label_set[res_label]);
      }
		}
	}

	return ipa_Utils::RET_OK;
}


unsigned long ipa_PeopleDetector::FaceRecognizer::recognizeFace(cv::Mat& color_image,cv::Mat& depth_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

//	int number_eigenvectors = m_eigenvectors.size();
	if (eff_depth.trained == false && eff_color.trained == false)
	{
		std::cout << "Error: FaceRecognizer::recognizeFace: Load or train some identification model, first.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	identification_labels.clear();

	cv::Size resized_size(m_eigenvectors[0].size());
	for (int i = 0; i < (int)face_coordinates.size(); i++)
	{
		cv::Rect face = face_coordinates[i];
		//convertAndResize(depth_image, resized_8U1, face, resized_size);
		cv::Mat color_crop = color_image(face);
		cv::Mat depth_crop_xyz = depth_image(face);

		cv::Mat DM_crop = cv::Mat::zeros(m_eigenface_size, m_eigenface_size, CV_8UC1);
		cv::Size norm_size = cv::Size(m_eigenface_size, m_eigenface_size);
		if (!face_normalizer_.normalizeFace(color_crop, depth_crop_xyz, norm_size, DM_crop))
			return ipa_Utils::RET_FAILED;

		double DFFS;
		cv::Mat temp, temp2;
		DM_crop.convertTo(DM_crop, CV_64FC1);
		color_crop.convertTo(color_crop, CV_64FC1);
		//depth_crop.convertTo(temp2,CV_8UC1,255);
		//cv::equalizeHist(temp2,temp2);
		//cv::imshow("temp",temp2);
		//cv::waitKey(10);

		cv::Mat coeff_arr_color, coeff_arr_depth;
		int res_label_color, res_label_depth;
		std::string class_depth, class_color;

		if ((eff_depth.trained) && (m_depth_mode == true))
		{
			std::cout << "classification with depth" << std::endl;
			eff_depth.projectToSubspace(DM_crop, coeff_arr_depth, DFFS);
			eff_depth.classify(coeff_arr_depth, m_class_meth, res_label_depth);
			if (res_label_depth == -1)
			{
				class_depth = "Unknown";
			}
			else
			{
				class_depth = depth_str_labels[res_label_depth];
			}
		}

		if (eff_color.trained)
		{
			std::cout << "classification with color" << std::endl;
			eff_color.projectToSubspace(color_crop, coeff_arr_color, DFFS);
			eff_color.classify(coeff_arr_color, m_class_meth, res_label_color);
			if (res_label_color == -1)
			{
				class_color = "Unknown";
			}
			else
			{
				class_color = m_current_label_set[res_label_color];
			}
		}

		//compare and combine classification resutls
		//
		// case 1 - A  A        --> A
		// case 2 - C  Unknown --> A
		// case 3 - Unknown A   --> A
		std::string classification_combined;
		if (class_color.compare(class_depth) == 0)
			classification_combined = class_color; //case 1
		else
		{
			bool found = false;
			for (unsigned int l = 0; l < depth_str_labels.size(); l++)
			{
				if (class_color.compare(depth_str_labels[l]) == 0)
					found = true;
			}
			if (!found && class_depth.compare("Unknown") == 0)
				classification_combined = class_color;
			else if (!found && class_color.compare("Unknown") == 0)
				classification_combined = class_depth;
			else
			{
				// this is the case when the algorithm yields different labels
				// insert weighting mechanism --> disadvantage of winner takes it all
				classification_combined = class_color;
			}
		}

		//	if (m_debug) std::cout << "distance to face space: " << DFFS << std::endl;
		//  //TODO temporary
		//  DFFS=0;
		//	if(DFFS > m_threshold_facespace)
		//	{
		//		// no face
		//		identification_labels.push_back("No face");
		//	}
		//	else
		//	{

		std::cout << "CLASS COLOR= " << class_color << std::endl;
		std::cout << "CLASS DEPTH= " << class_depth << std::endl;
		identification_labels.push_back(class_color);
		//}
	}

	return ipa_Utils::RET_OK;
}


unsigned long ipa_PeopleDetector::FaceRecognizer::convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size)
{

  resized=img(face);
  cv::resize(resized,resized,new_size);

	//cv::cvtColor(resized, resized, CV_BGR2GRAY);

	return ipa_Utils::RET_OK;
}


unsigned long ipa_PeopleDetector::FaceRecognizer::saveTrainingData(std::vector<cv::Mat>& face_images)
{
	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";
	std::string img_ext = ".bmp";

	std::ostringstream complete;
	complete << path << filename;

	if(fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
		if(!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::saveTrainingData: Can't save training data.\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// store data
		fileStorage << "number_entries" << (int)m_face_labels.size();
		for(int i=0; i<(int)m_face_labels.size(); i++)
		{
			// labels
			std::ostringstream tag;
			tag << "label_" << i;
			fileStorage << tag.str().c_str() << m_face_labels[i].c_str();

			// face images
			std::ostringstream img, shortname;
			img << path << i << img_ext;
			shortname << "training_data/" << i << img_ext;
			std::ostringstream tag2;
			tag2 << "image_" << i;
			fileStorage << tag2.str().c_str() << shortname.str().c_str();
			cv::imwrite(img.str().c_str(), face_images[i]);
		}

		fileStorage.release();

		std::cout << "INFO: FaceRecognizer::saveTrainingData: " << face_images.size() << " images saved.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::saveTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}


unsigned long ipa_PeopleDetector::FaceRecognizer::saveTrainingData(std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps)
{
	std::cout << "Size Vec " << face_images.size() << " " << face_depthmaps.size() << std::endl;
	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";
	std::string img_ext = ".bmp";
	std::string dm_ext = ".xml";

	std::ostringstream complete;
	complete << path << filename;

	if (fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
		if (!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::saveTrainingData: Can't save training data.\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// store data
		fileStorage << "number_entries" << (int)m_face_labels.size();
		for (int i = 0; i < (int)m_face_labels.size(); i++)
		{
			// labels
			std::ostringstream tag;
			tag << "label_" << i;
			fileStorage << tag.str().c_str() << m_face_labels[i].c_str();

			// face images
			std::ostringstream img, shortname_img, shortname_depth;
			img << path << i << img_ext;
			shortname_img << "training_data/" << i << img_ext;
			std::ostringstream tag2, tag3;
			tag2 << "image_" << i;
			fileStorage << tag2.str().c_str() << shortname_img.str().c_str();

			if (dm_exist[i])
			{
				shortname_depth << "training_data/" << i << dm_ext;
				tag3 << "depthmap_" << i;
				fileStorage << tag3.str().c_str() << shortname_depth.str().c_str();
			}
			cv::imwrite(img.str().c_str(), face_images[i]);
		}

		fileStorage.release();

		int j=0;
		for (unsigned int i = 0; i < dm_exist.size(); i++)
		{
			if (dm_exist[i])
			{
				// depth maps
				std::ostringstream dm;
				dm << path << i << dm_ext;
				cv::FileStorage fs(dm.str().c_str(), FileStorage::WRITE);
				fs << "depthmap" << face_depthmaps[j];
				fs.release();
				j++;
			}
		}
		std::cout << "INFO: FaceRecognizer::saveTrainingData: " << face_images.size() << " images saved.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::saveTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::loadTrainingData(std::vector<cv::Mat>& face_images, std::vector<std::string>& identification_labels_to_train)
{
	bool use_all_data = false;
	if (identification_labels_to_train.size() == 0)
		use_all_data = true;

	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	if(fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if(!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::loadTrainingData: Can't open " << complete.str() << ".\n" << std::endl;
			return ipa_Utils::RET_OK;
		}

		// labels
		m_face_labels.clear();
		face_images.clear();
    cv::Size norm_size=cv::Size(m_eigenface_size,m_eigenface_size);
		int number_entries = (int)fileStorage["number_entries"];
		for(int i=0; i<number_entries; i++)
		{
			// labels
			std::ostringstream tag_label;
			tag_label << "label_" << i;
			std::string label = (std::string)fileStorage[tag_label.str().c_str()];
			// look up this label in the list of unique labels identification_labels_to_train
			bool class_exists = false;
			for(int j=0; j<(int)identification_labels_to_train.size(); j++)
			{
				if(!identification_labels_to_train[j].compare(label))
					class_exists = true;
			}
			// if it does not appear in the list either append it (use all data option) or neglect this piece of data
			if (class_exists == false)
			{
				if (use_all_data == true)
				{
					// append this label to the list of labels
					identification_labels_to_train.push_back(label);
				}
				else
				{
					// skip this data because it does not contain one of the desired labels
					continue;
				}
			}
			m_face_labels.push_back(label);

			// face images
			std::ostringstream tag_image;
			tag_image << "image_" << i;
			std::string path = m_data_directory + (std::string)fileStorage[tag_image.str().c_str()];
			cv::Mat temp = cv::imread(path.c_str(),-1);
      face_normalizer_.normalizeFace(temp,norm_size);
			face_images.push_back(temp);
		}

		// clean identification_labels_to_train -> only keep those labels that appear in the training data
		for(int j=0; j<(int)identification_labels_to_train.size(); j++)
		{
			bool class_exists = false;
			for (int k=0; k<(int)m_face_labels.size(); k++)
			{
				if(identification_labels_to_train[j].compare(m_face_labels[k]) == 0)
					class_exists = true;
			}
			if (class_exists == false)
			{
				identification_labels_to_train.erase(identification_labels_to_train.begin()+j);
				j--;
			}
		}

		// set next free filename
		// filename_ = number_face_images;

		fileStorage.release();

		std::cout << "INFO: FaceRecognizer::loadTrainingData: " << number_entries << " images loaded.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::loadTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}
unsigned long ipa_PeopleDetector::FaceRecognizer::loadTrainingData(std::vector<cv::Mat>& face_images,std::vector<cv::Mat>& face_depthmaps, std::vector<std::string>& identification_labels_to_train)
{
	bool use_all_data = false;
	dm_exist.clear();
	if (identification_labels_to_train.size() == 0)
		use_all_data = true;

	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";

	std::ostringstream complete;
	complete << path << filename;

	if (fs::is_directory(path.c_str()))
	{
		cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::loadTrainingData: Can't open " << complete.str() << ".\n" << std::endl;
			return ipa_Utils::RET_OK;
		}

		// labels
		m_face_labels.clear();
		face_images.clear();
		cv::Size norm_size = cv::Size(m_eigenface_size, m_eigenface_size);
		int number_entries = (int)fileStorage["number_entries"];
		for (int i = 0; i < number_entries; i++)
		{
			// labels
			std::ostringstream tag_label;
			tag_label << "label_" << i;
			std::string label = (std::string)fileStorage[tag_label.str().c_str()];
			// look up this label in the list of unique labels identification_labels_to_train
			bool class_exists = false;
			for (int j = 0; j < (int)identification_labels_to_train.size(); j++)
			{
				if (!identification_labels_to_train[j].compare(label))
					class_exists = true;
			}
			// if it does not appear in the list either append it (use all data option) or neglect this piece of data
			if (class_exists == false)
			{
				if (use_all_data == true)
				{
					// append this label to the list of labels
					identification_labels_to_train.push_back(label);
				}
				else
				{
					// skip this data because it does not contain one of the desired labels
					continue;
				}
			}
			m_face_labels.push_back(label);

			// face images
			std::ostringstream tag_image, tag_dm;
			tag_image << "image_" << i;
			tag_dm << "depthmap_" << i;
			std::string img_path = m_data_directory + (std::string)fileStorage[tag_image.str().c_str()];
			std::string dm_path = m_data_directory + (std::string)fileStorage[tag_dm.str().c_str()];
			cv::Mat temp = cv::imread(img_path.c_str(), -1);
			//cv::imshow("T",temp);
			//cv::waitKey(0);

			if (m_depth_mode)
			{
				if (dm_path.compare(m_data_directory))
				{
					cv::Mat xyz_temp, dm_temp;
					cv::FileStorage fs(dm_path, FileStorage::READ);
					fs["depthmap"] >> xyz_temp;

					face_normalizer_.normalizeFace(temp, xyz_temp, norm_size, dm_temp);

					face_depthmaps.push_back(dm_temp);
					dm_exist.push_back(true);
					fs.release();
				}
				else
					dm_exist.push_back(false);
				{
					face_normalizer_.normalizeFace(temp, norm_size);
					face_images.push_back(temp);
				}
			}
			else
			{
				face_normalizer_.normalizeFace(temp, norm_size);
			}
		}

		// clean identification_labels_to_train -> only keep those labels that appear in the training data
		for (int j = 0; j < (int)identification_labels_to_train.size(); j++)
		{
			bool class_exists = false;
			for (int k = 0; k < (int)m_face_labels.size(); k++)
			{
				if (identification_labels_to_train[j].compare(m_face_labels[k]) == 0)
					class_exists = true;
			}
			if (class_exists == false)
			{
				identification_labels_to_train.erase(identification_labels_to_train.begin() + j);
				j--;
			}
		}

		// set next free filename
		// filename_ = number_face_images;

		fileStorage.release();

		std::cout << "INFO: FaceRecognizer::loadTrainingData: " << number_entries << " images loaded.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::loadTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}
