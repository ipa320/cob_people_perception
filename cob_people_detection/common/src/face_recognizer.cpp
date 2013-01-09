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



//INCLUDE  subspace analysis
#include "cob_people_detection/subspace_analysis.h"

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

unsigned long ipa_PeopleDetector::FaceRecognizer::init(std::string data_directory, int eigenface_size, int eigenvectors_per_person, double threshold_facespace, double threshold_unknown, int metric, bool debug, std::vector<std::string>& identification_labels_to_recognize)
{
	// parameters
	m_data_directory = data_directory;
	m_eigenface_size = eigenface_size;
	m_eigenvectors_per_person = eigenvectors_per_person;
	m_threshold_facespace = threshold_facespace;
	m_threshold_unknown = threshold_unknown;
	m_metric = metric;
	m_debug = debug;

	// load model
	loadRecognitionModel(identification_labels_to_recognize);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::initTraining(std::string data_directory, int eigenface_size, bool debug, std::vector<cv::Mat>& face_images)
{
	// parameters
	m_data_directory = data_directory;
	m_eigenface_size = eigenface_size;
	m_debug = debug;

	// load model
	m_current_label_set.clear();	 // keep empty to load all available data
	loadTrainingData(face_images, m_current_label_set);

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
	cv::Mat roi_depth = depth_image(face_bounding_box);
  cv::Vec2f offset = cv::Vec2f(face_bounding_box.x,face_bounding_box.y);
  cv::Size norm_size=cv::Size(m_eigenface_size,m_eigenface_size);
  if(!face_normalizer_.normalizeFace(roi_color,roi_depth,norm_size,offset)) return ipa_Utils::RET_FAILED;
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

unsigned long ipa_PeopleDetector::FaceRecognizer::deleteFaces(std::string label, std::vector<cv::Mat>& face_images)
{
	for (int i=0; i<(int)m_face_labels.size(); i++)
	{
		if (m_face_labels[i].compare(label) == 0)
		{
			m_face_labels.erase(m_face_labels.begin()+i);
			face_images.erase(face_images.begin()+i);
			i--;
		}
	}
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::deleteFace(int index, std::vector<cv::Mat>& face_images)
{
	m_face_labels.erase(m_face_labels.begin()+index);
	face_images.erase(face_images.begin()+index);
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::trainRecognitionModel(std::vector<std::string>& identification_labels_to_train)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	// load necessary data
	std::vector<cv::Mat> face_images;
	loadTrainingData(face_images, identification_labels_to_train);
	m_current_label_set = identification_labels_to_train;

	// convert face_images to right format if necessary
	cv::Size new_size(m_eigenface_size, m_eigenface_size);
	for (uint i=0; i<face_images.size(); i++)
	{
		// convert to grayscale if necessary
		if (face_images[i].type() == CV_8UC3)
		{
			cv::Mat temp = face_images[i];
			cv::cvtColor(temp, face_images[i], CV_BGR2GRAY);
		}
		// rescale if necessary to m_eigenface_size
		if (face_images[i].cols != m_eigenface_size || face_images[i].rows != m_eigenface_size)
		{
			cv::Mat temp = face_images[i];
			cv::resize(temp, face_images[i], new_size);
		}
	}

	// PCA
	int number_eigenvectors = std::min(m_eigenvectors_per_person * identification_labels_to_train.size(), face_images.size()-1);
	bool return_value = PCA(number_eigenvectors, face_images);

//--------------------------------------------
//--------------------------------------------
//--------------------------------------------
  std::vector<int>label_vec;
  int ss_dim = 10;

  for(int li=0;li<m_face_labels.size();li++)
  {
    for(int lj=0;lj<identification_labels_to_train.size();lj++)
    {
      if(identification_labels_to_train[lj].compare(m_face_labels[li])==0) label_vec.push_back(lj);
    }
  }


  std::vector<cv::Mat> in_vec;
  for(int i=0;i<face_images.size();i++)
  {
    cv::Mat temp=cv::Mat(face_images[i].rows,face_images[i].cols,CV_8UC1);
    temp=face_images[i];
    temp.convertTo(temp,CV_64FC1);
    in_vec.push_back(temp);
  }
  SubspaceAnalysis::Eigenfaces EF(in_vec,label_vec,ss_dim);
//--------------------------------------------
//--------------------------------------------

	if (return_value == ipa_Utils::RET_FAILED)
		return ipa_Utils::RET_FAILED;

	// compute average face projections per class
	computeAverageFaceProjections();

	// save new model
	saveRecognitionModel();

	// output
	if (m_debug)
	{
		std::cout << "FaceRecognizer::trainRecognitionModel: New model trained with labels: " << std::endl;
		for (int i=0; i<(int)m_current_label_set.size(); i++)
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

	int number_eigenvectors = m_eigenvectors.size();
	if (number_eigenvectors == 0)
	{
		std::cout << "Error: FaceRecognizer::recognizeFace: Load or train some identification model, first.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	identification_labels.clear();

	float* eigen_vector_weights = 0;
	eigen_vector_weights = (float *)cvAlloc(number_eigenvectors*sizeof(float));

	// Convert vector to array
//	IplImage** m_eigenvectors_ipl = (IplImage**)cvAlloc(number_eigenvectors*sizeof(IplImage*));
//	for(int j=0; j<number_eigenvectors; j++)
//	{
//		IplImage temp = (IplImage)m_eigenvectors[j];
//		m_eigenvectors_ipl[j] = cvCloneImage(&temp);
//	}

	cv::Mat resized_8U1;
	cv::Size resized_size(m_eigenvectors[0].size());
	for(int i=0; i<(int)face_coordinates.size(); i++)
	{
		cv::Rect face = face_coordinates[i];
		convertAndResize(color_image, resized_8U1, face, resized_size);


		// todo: preprocess
		//cv::Mat preprocessedImage = preprocessImage(resized_8U1);

		IplImage avg_image_ipl = (IplImage)m_average_image;

		// Project the test image onto the PCA subspace
		IplImage resized_8U1Ipl = (IplImage)resized_8U1;
		cvEigenDecomposite(&resized_8U1Ipl, number_eigenvectors, m_eigenvectors_ipl, 0, 0, &avg_image_ipl, eigen_vector_weights);

		// Calculate FaceSpace Distance
		cv::Mat src_reconstruction = cv::Mat::zeros(resized_size, m_eigenvectors[0].type());
		for(int i=0; i<number_eigenvectors; i++)
			src_reconstruction += eigen_vector_weights[i]*m_eigenvectors[i];

		// todo:
//		cv::Mat reconstrTemp = src_reconstruction + m_average_image;
//		cv::Mat reconstr(m_eigenvectors[0].size(), CV_8UC1);
//		reconstrTemp.convertTo(reconstr, CV_8UC1, 1);
//		cv::imshow("reconstruction", reconstr);
//		cv::waitKey();

		cv::Mat temp;
		resized_8U1.convertTo(temp, CV_32FC1, 1.0/255.0);
		double distance = cv::norm((temp-m_average_image), src_reconstruction, cv::NORM_L2);

		//std::cout.precision( 10 );
		if (m_debug) std::cout << "distance to face space: " << distance << std::endl;

		// -2=distance to face space is too high
		// -1=distance to face classes is too high
		if(distance > m_threshold_facespace)
		{
			// no face
			identification_labels.push_back("No face");
		}
		else
		{
			std::string face_label;
			classifyFace(eigen_vector_weights, face_label, number_eigenvectors);
			identification_labels.push_back(face_label);
		}
	}

	// clear
	//for (int i=0; i<number_eigenvectors; i++) cvReleaseImage(&(m_eigenvectors_ipl[i]));
	cvFree(&eigen_vector_weights);
	//cvFree(&m_eigenvectors_ipl);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::classifyFace(float *eigen_vector_weights, std::string& face_label, int number_eigenvectors)
{
	double least_dist_sqared = DBL_MAX;

	// todo: compare against single examples from training data, i.e. KNN
	// comparing against average is arbitrarily bad
	cv::Mat& model_data = m_projected_training_faces; 		//m_face_class_average_projections
	std::vector<std::string>& label_data = m_face_labels;	//m_current_label_set
	for(int i=0; i<model_data.rows; i++)
	{
		double distance=0;
		double cos=0;
		double length_sample=0;
		double length_projection=0;
		for(int e=0; e<number_eigenvectors; e++)
		{
			if (m_metric < 2)
			{
				float d = eigen_vector_weights[e] - ((float*)(model_data.data))[i * number_eigenvectors + e];
				if (m_metric==0)
					distance += d*d;							//Euklid
				else
					distance += d*d / ((float*)(m_eigenvalues.data))[e];	//Mahalanobis
			}
			else
			{
				cos += eigen_vector_weights[e] * ((float*)(model_data.data))[i * number_eigenvectors + e] / ((float*)(m_eigenvalues.data))[e];
				length_projection += ((float*)(model_data.data))[i * number_eigenvectors + e] * ((float*)(model_data.data))[i * number_eigenvectors + e] / ((float*)(m_eigenvalues.data))[e];
				length_sample += eigen_vector_weights[e]*eigen_vector_weights[e] / ((float*)(m_eigenvalues.data))[e];
			}
		}
		if (m_metric < 2)
			distance = sqrt(distance);
		else
		{
			length_sample = sqrt(length_sample);
			length_projection = sqrt(length_projection);
			cos /= (length_projection * length_sample);
			distance = std::abs(cos); //-cos;		// todo why not abs?
		}

		if (m_debug) std::cout << "distance to face class: " << distance << std::endl;

		if(distance < least_dist_sqared)
		{
			least_dist_sqared = distance;
			if(least_dist_sqared > m_threshold_unknown)
				face_label = "Unknown";
			else
				face_label = label_data[i];
		}
	}
	if (m_debug) std::cout << "least distance to face class: " << least_dist_sqared << std::endl;

	// todo:
//	if (personClassifier != 0 && *nearest != -1)
//	{
//		cv::Mat temp(1, *nEigens, CV_32FC1, eigenVectorWeights);
//		std::cout << "class. output: " << (int)personClassifier->predict(temp) << "\n";
//		*nearest = (int)personClassifier->predict(temp);
//	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::PCA(int number_eigenvectors, std::vector<cv::Mat>& face_images)
{
	if(face_images.size() < 2)
	{
		std::cout << "Error: FaceRecognizer::PCA: Less than two images available for training.\n";
		return ipa_Utils::RET_FAILED;
	}

	// Allocate memory
	cv::Size face_image_size(face_images[0].cols, face_images[0].rows);
	int old_number_eigenvectors = m_eigenvectors.size();
	m_eigenvectors.clear();
	m_eigenvectors.resize(number_eigenvectors, cv::Mat(face_image_size, CV_32FC1));
	m_eigenvalues.create(1, number_eigenvectors, CV_32FC1);
	m_average_image.create(face_image_size, CV_32FC1);

	// Set the PCA termination criterion
	CvTermCriteria calcLimit;
	calcLimit = cvTermCriteria(CV_TERMCRIT_ITER, number_eigenvectors, 1);

	// Convert face image vector to array
	IplImage** face_images_ipl = (IplImage**)cvAlloc((int)face_images.size()*sizeof(IplImage*));
	for(int j=0; j<(int)face_images.size(); j++)
	{
		// todo: preprocess
		cv::Mat preprocessed_image = preprocessImage(face_images[j]);
		IplImage temp = (IplImage)preprocessed_image;
		face_images_ipl[j] = cvCloneImage(&temp);
	}

	// Convert eigenvector vector to array and delete old data if available
	convertEigenvectorsToIpl(old_number_eigenvectors);

	// Compute average image, eigenvalues, and eigenvectors
	IplImage average_image_ipl = (IplImage)m_average_image;

	float eigenvalues[number_eigenvectors*number_eigenvectors*number_eigenvectors];		// hack: if strange crashes occur, the array size should be increased
	cvCalcEigenObjects((int)face_images.size(), (void*)face_images_ipl, (void*)m_eigenvectors_ipl, CV_EIGOBJ_NO_CALLBACK, 0, 0, &calcLimit, &average_image_ipl, eigenvalues);
	for (int i=0; i<m_eigenvalues.cols; i++)
		m_eigenvalues.at<float>(i) = eigenvalues[i];

	cv::normalize(m_eigenvalues, m_eigenvalues, 1, 0, /*CV_L1*/CV_L2);	//, 0);		0=bug?

	// Project the training images onto the PCA subspace
	m_projected_training_faces.create(face_images.size(), number_eigenvectors, CV_32FC1);
	for(int i=0; i<(int)face_images.size(); i++)
	{
		IplImage temp = (IplImage)face_images[i];
		cvEigenDecomposite(&temp, number_eigenvectors, m_eigenvectors_ipl, 0, 0, &average_image_ipl, (float*)m_projected_training_faces.data + i * number_eigenvectors);	//attention: if image step of m_projected_training_faces is not number_eigenvectors * sizeof(float) then reading functions which access with (x,y) coordinates might fail
	};

	// Copy back
	//int eigenVectorsCount = (int)m_eigenvectors.size();
	m_eigenvectors.clear();
	for (int i=0; i<number_eigenvectors; i++)
		m_eigenvectors.push_back(cv::Mat(m_eigenvectors_ipl[i], true));

	// Clean
	for (int i=0; i<(int)face_images.size(); i++) cvReleaseImage(&(face_images_ipl[i]));
	cvFree(&face_images_ipl);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::computeAverageFaceProjections()
{
	int number_eigenvectors = m_eigenvectors.size();

	// Calculate per class average projection coefficients
	m_face_class_average_projections = cv::Mat::zeros((int)m_current_label_set.size(), number_eigenvectors, m_projected_training_faces.type());
	for(int i=0; i<(int)m_current_label_set.size(); i++)
	{
		std::string face_class = m_current_label_set[i];

		int count=0;
		for(int j=0;j<(int)m_face_labels.size(); j++)
		{
			if(!(m_face_labels[j].compare(face_class)))
			{
				for(int e=0; e<number_eigenvectors; e++)
					((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e] += ((float*)(m_projected_training_faces.data))[j * number_eigenvectors + e];
				count++;
			}
		}
		for(int e=0; e<number_eigenvectors; e++)
			((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e] /= (float)count;
	}

	// todo: machine learning technique for person identification
	if (false)
	{
		// prepare ground truth
		cv::Mat data(m_face_labels.size(), number_eigenvectors, CV_32FC1);
		cv::Mat labels(m_face_labels.size(), 1, CV_32SC1);
		std::ofstream fout("svm.dat", std::ios::out);
		for(int sample=0; sample<(int)m_face_labels.size(); sample++)
		{
			// copy data
			for (int e=0; e<number_eigenvectors; e++)
			{
				data.at<float>(sample, e) = ((float*)m_projected_training_faces.data)[sample * number_eigenvectors + e];
				fout << data.at<float>(sample, e) << "\t";
			}
			// find corresponding label
			for(int i=0; i<(int)m_current_label_set.size(); i++)	// for each person
				if(!(m_face_labels[sample].compare(m_current_label_set[i])))		// compare the labels
					labels.at<int>(sample) = i;					// and assign the corresponding label's index from the m_current_label_set list
			fout << labels.at<int>(sample) << "\n";
		}
		fout.close();

		// train the classifier
		cv::SVMParams svmParams(CvSVM::NU_SVC, CvSVM::RBF, 0.0, 0.001953125, 0.0, 0.0, 0.8, 0.0, 0, cv::TermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, FLT_EPSILON));
		//m_face_classifier.train_auto(data, labels, cv::Mat(), cv::Mat(), svmParams, 10, cv::SVM::get_default_grid(CvSVM::C), CvParamGrid(0.001953125, 2.01, 2.0), cv::SVM::get_default_grid(CvSVM::P), CvParamGrid(0.0125, 1.0, 2.0));
		m_face_classifier.train(data, labels, cv::Mat(), cv::Mat(), svmParams);
		cv::SVMParams svmParamsOptimal = m_face_classifier.get_params();
		if (m_debug) std::cout << "\nOptimal SVM params: gamma=" << svmParamsOptimal.gamma << "  nu=" << svmParamsOptimal.nu << "\n";
	}

	if (m_debug) std::cout << "done\n";

	return ipa_Utils::RET_OK;
}

cv::Mat ipa_PeopleDetector::FaceRecognizer::preprocessImage(cv::Mat& input_image)
{

  //TODO:
  //  -- Histogramm equalization
  //  -- Image gradient for linear illumination changes 


	// todo:
	return input_image;

	// do a modified census transform
	cv::Mat output(input_image.cols, input_image.rows, input_image.type());
	//cv::Mat smoothedImage = input_image.clone();
	//cv::GaussianBlur(smoothedImage, smoothedImage, cv::Size(3,3), 0, 0, cv::BORDER_REPLICATE);

	for (int v=0; v<input_image.rows; v++)
	{
		uchar* srcPtr = input_image.ptr(v);
		//uchar* smoothPtr = smoothedImage.ptr(v);
		uchar* outPtr = output.ptr(v);
		for (int u=0; u<input_image.cols; u++)
		{
			int ctOutcome = 0;
			int offset = -1;
			for (int dv=-1; dv<=1; dv++)
			{
				for (int du=-1; du<=1; du++)
				{
					if (dv==0 && du==0) continue;
					offset++;
					if (v+dv<0 || v+dv>=input_image.rows || u+du<0 || u+du>=input_image.cols) continue;
					//if (*smoothPtr < *(srcPtr+dv*input_image.step+du)) ctOutcome += 1<<offset;
					if (*srcPtr < *(srcPtr+dv*input_image.step+du)) ctOutcome += 1<<offset;
				}
			}
			*outPtr = ctOutcome;

			srcPtr++;
			outPtr++;
		}
	}

//	cv::imshow("census transform", output);
//	cv::waitKey();

	return output;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size)
{

  resized=img(face);
  cv::resize(resized,resized,new_size);

	cv::cvtColor(resized, resized, CV_BGR2GRAY);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::convertEigenvectorsToIpl(int old_number_eigenvectors)
{
	int new_number_eigenvectors = m_eigenvectors.size();

	// clear
	if (m_eigenvectors_ipl != 0)
	{
		for (int i=0; i<old_number_eigenvectors; i++)
			cvReleaseImage(&(m_eigenvectors_ipl[i]));
		cvFree(&m_eigenvectors_ipl);
	}

	// Convert vector to array
	m_eigenvectors_ipl = (IplImage**)cvAlloc(new_number_eigenvectors*sizeof(IplImage*));
	for(int j=0; j<new_number_eigenvectors; j++)
	{
		IplImage temp = (IplImage)m_eigenvectors[j];
		m_eigenvectors_ipl[j] = cvCloneImage(&temp);
	}

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
