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

#include <sys/time.h>

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

unsigned long ipa_PeopleDetector::FaceRecognizer::init(std::string data_directory, int norm_size, bool norm_illumination, bool norm_align, bool norm_extreme_illumination,
		int metric, bool debug, std::vector<std::string>& identification_labels_to_recognize, int subs_meth, int feature_dim, bool use_unknown_thresh, bool use_depth)
{
	// parameters
	m_data_directory = boost::filesystem::path(data_directory);
	m_data_directory /= "training_data";
	assertDirectories(m_data_directory);

	m_norm_size = norm_size;
	m_metric = metric;
	m_debug = debug;
	m_depth_mode = use_depth;
	m_use_unknown_thresh = use_unknown_thresh;

	m_feature_dim = feature_dim;

	switch(subs_meth)
	{
	case 0:
	{
		m_subs_meth = ipa_PeopleDetector::METH_FISHER;
		eff_color = new ipa_PeopleDetector::FaceRecognizer_Fisherfaces();
		break;
	}
	case 1:
	{
		m_subs_meth = ipa_PeopleDetector::METH_EIGEN;
		eff_color = new ipa_PeopleDetector::FaceRecognizer_Eigenfaces();
		break;
	}
	case 2:
	{
		m_subs_meth = ipa_PeopleDetector::METH_LDA2D;
		eff_color = new ipa_PeopleDetector::FaceRecognizer_LDA2D();
		break;
	}
	case 3:
	{
		m_subs_meth = ipa_PeopleDetector::METH_PCA2D;
		eff_color = new ipa_PeopleDetector::FaceRecognizer_PCA2D();
		break;
	}
	default:
	{
		m_subs_meth = ipa_PeopleDetector::METH_FISHER;
		eff_color = new ipa_PeopleDetector::FaceRecognizer_Fisherfaces();
		break;
	}
	};

	FaceNormalizer::FNConfig fn_cfg;
	fn_cfg.eq_ill = norm_illumination;
	fn_cfg.align = norm_align;
	fn_cfg.resize = true;
	fn_cfg.cvt2gray = true;
	fn_cfg.extreme_illumination_condtions = norm_extreme_illumination;

	std::string classifier_directory = data_directory + "haarcascades/";
	//std::string storage_directory="/share/goa-tz/people_detection/eval/KinectIPA/";
	std::string storage_directory = "/share/goa-tz/people_detection/eval/KinectIPA/";
	face_normalizer_.init(classifier_directory, storage_directory, fn_cfg, 0, false, false);

	// load model
	return loadRecognitionModel(identification_labels_to_recognize);
}

unsigned long ipa_PeopleDetector::FaceRecognizer::initTraining(std::string data_directory, int norm_size, bool norm_illumination, bool norm_align, bool norm_extreme_illumination,
		bool debug, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps, bool use_depth)
{
	// parameters
	m_data_directory = boost::filesystem::path(data_directory);
	m_data_directory /= "training_data";
	assertDirectories(m_data_directory);
	m_norm_size = norm_size;
	m_debug = debug;
	m_depth_mode = use_depth;

	FaceNormalizer::FNConfig fn_cfg;
	fn_cfg.eq_ill = norm_illumination;
	fn_cfg.align = norm_align;
	fn_cfg.resize = true;
	fn_cfg.cvt2gray = true;
	fn_cfg.extreme_illumination_condtions = norm_extreme_illumination;

	std::string classifier_directory = data_directory + "haarcascades/";
	//std::string storage_directory="/share/goa-tz/people_detection/eval/KinectIPA/";
	std::string storage_directory = "/share/goa-tz/people_detection/eval/KinectIPA/";
	face_normalizer_.init(classifier_directory, storage_directory, fn_cfg, 0, false, false);
	// load model
	m_current_label_set.clear(); // keep empty to load all available data
	return loadTrainingData(face_images, m_current_label_set);
}

unsigned long ipa_PeopleDetector::FaceRecognizer::addFace(cv::Mat& color_image, cv::Mat& depth_image, cv::Rect& face_bounding_box, cv::Rect& head_bounding_box, std::string label,
		std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	//cv::Rect combined_face_bounding_box=cv::Rect(face_bounding_box.x+head_bounding_box.x,face_bounding_box.y+head_bounding_box.y,face_bounding_box.width,face_bounding_box.height);
	//cv::Mat roi_color = color_image(combined_face_bounding_box);
	cv::Mat roi_color = color_image(face_bounding_box);	// color image has size of head area, not only face area
	cv::Mat roi_depth_xyz = depth_image(face_bounding_box).clone();
	cv::Size norm_size = cv::Size(m_norm_size, m_norm_size);
	cv::Mat roi_depth;
	//if(!face_normalizer_.normalizeFace(roi_color,roi_depth_xyz,norm_size)) ;
	// this is probably obsolete:  face_normalizer_.recordFace(roi_color, roi_depth_xyz);

	if (!face_normalizer_.normalizeFace(roi_color, roi_depth_xyz, norm_size))
		return ipa_Utils::RET_FAILED;

	// Save image
	face_images.push_back(roi_color);
	face_depthmaps.push_back(roi_depth_xyz);
	m_face_labels.push_back(label);
	dm_exist.push_back(true);

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
			m_face_labels.erase(m_face_labels.begin() + i);
			face_images.erase(face_images.begin() + i);
			i--;
		}
	}
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::deleteFace(int index, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps)
{
	m_face_labels.erase(m_face_labels.begin() + index);
	face_images.erase(face_images.begin() + index);
	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::trainFaceRecognition(ipa_PeopleDetector::FaceRecognizerBaseClass* eff, std::vector<cv::Mat>& data, std::vector<int>& labels)
{

	std::vector<cv::Mat> in_vec;
	for (unsigned int i = 0; i < data.size(); i++)
	{
		cv::Mat temp = data[i];
		temp.convertTo(temp, CV_64FC1);
		in_vec.push_back(temp);
	}

	if (!eff->trainModel(in_vec, labels, m_feature_dim))
	{
		std::cout << "[FACEREC] Reognition module could not be initialized !" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::trainRecognitionModel(std::vector<std::string>& identification_labels_to_train)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	// load necessary data

	std::vector<cv::Mat> face_images;
	unsigned long return_value = loadTrainingData(face_images, identification_labels_to_train);
	if (return_value == ipa_Utils::RET_FAILED)
			return ipa_Utils::RET_FAILED;

	m_current_label_set = identification_labels_to_train;

	m_label_num.clear();
	for (unsigned int li = 0; li < m_face_labels.size(); li++)
	{
		for (unsigned int lj = 0; lj < identification_labels_to_train.size(); lj++)
		{
			if (identification_labels_to_train[lj].compare(m_face_labels[li]) == 0)
				m_label_num.push_back(lj);
		}
	}

	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path path_color = path / "rdata_color.xml";

	unsigned long trained = ipa_Utils::RET_FAILED;
	if (face_images.size() > 0)
	{
		trained = trainFaceRecognition(eff_color, face_images, m_label_num);
	}

	saveRecognitionModel();
	return trained;

}

unsigned long ipa_PeopleDetector::FaceRecognizer::saveRecognitionModel()
{
	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "rdata_color.xml";

	if (fs::is_directory(path.string()))
	{
		if (fs::is_regular_file(complete.string()))
		{
			if (fs::remove(complete.string()) == false)
			{
				std::cout << "Error: FaceRecognizer::saveRecognitionModel: Cannot remove old recognizer data.\n" << std::endl;
				return ipa_Utils::RET_FAILED;
			}
		}
		eff_color->saveModel(complete);

		std::cout << "OPENING at " << complete.string() << std::endl;
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::APPEND);
		if (!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::saveRecognitionModel: Can't save training data.\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		fileStorage << "string_labels" << "[";
		for (int i = 0; i < m_face_labels.size(); i++)
		{
			fileStorage << m_face_labels[i];
		}
		fileStorage << "]";

		fileStorage.release();

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
	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "rdata_color.xml";

	bool training_necessary = false;
	std::vector<string> temp_face_labels;
	if (fs::is_directory(path.string()))
	{
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "Info: FaceRecognizer::loadRecognitionModel: Can't open " << complete.string() << ".\n" << std::endl;
			training_necessary = true;
		}
		else
		{
			// load model labels
			cv::FileNode fn = fileStorage["string_labels"];
			cv::FileNodeIterator it = fn.begin(), it_end = fn.end();
			int idx = 0;
			m_current_label_set.clear();
			for (; it != it_end; ++it, idx++)
			{
				temp_face_labels.push_back(*it);
				bool new_label = true;
				if (idx > 0)
				{
					for (int i = 0; i < m_current_label_set.size(); i++)
					{
						if (m_current_label_set[i].compare(*it) == 0)
							new_label = false;
					}
				}
				if (new_label)
					m_current_label_set.push_back(*it);
			}

			// A vector containing all different labels from the training session exactly once, order of appearance matters! (m_current_label_set[i] stores the corresponding name to the average face coordinates in the face subspace in m_face_class_average_projections.rows(i))
			bool same_data_set = true;
			if (identification_labels_to_recognize.size() == 0 || m_current_label_set.size() != identification_labels_to_recognize.size())
			{
				same_data_set = false;
			}
			else
			{
				for (uint i = 0; i < identification_labels_to_recognize.size(); i++)
				{
					if (identification_labels_to_recognize[i].compare(m_current_label_set[i]) != 0)
					{
						same_data_set = false;
						break;
					}
				}
			}

			if (same_data_set == true)
			{
				eff_color->loadModel(complete);
				m_face_labels = temp_face_labels;
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
		std::cerr << "Error: FaceRecognizer::loadRecognizerData: Path '" << path.string() << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	if (training_necessary == true)
	{
		// stored set differs from requested set -> recompute the model from training data
		// release lock, trainRecognitionModel requests its own lock
		if (lock != 0)
			delete lock;

		unsigned long return_value = trainRecognitionModel(identification_labels_to_recognize);
		if (return_value == ipa_Utils::RET_FAILED)
			return ipa_Utils::RET_FAILED;

	}

	if (m_debug == true)
	{
		std::cout << "Current model set:" << std::endl;
		for (int i = 0; i < (int)m_current_label_set.size(); i++)
			std::cout << "   - " << m_current_label_set[i] << std::endl;
		std::cout << std::endl;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::recognizeFace(cv::Mat& color_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels)
{
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	if (eff_color->trained_ == false)
	{
		std::cout << "Error: FaceRecognizer::recognizeFace: Load or train some identification model, first.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	identification_labels.clear();

	cv::Mat resized_8U1;
	cv::Size resized_size(m_eigenvectors[0].size());
	for (int i = 0; i < (int)face_coordinates.size(); i++)
	{
		cv::Rect face = face_coordinates[i];
		cv::Size norm_size = cv::Size(m_norm_size, m_norm_size);
		convertAndResize(color_image, resized_8U1, face, norm_size);

		double DFFS;
		resized_8U1.convertTo(resized_8U1, CV_64FC1);

		cv::Mat coeff_arr;
		//eff_color.projectToSubspace(resized_8U1,coeff_arr,DFFS);

		if (m_debug)
			std::cout << "distance to face space: " << DFFS << std::endl;
		//TODO temporary turned off
		if (0 == 1)
		{
			// no face
			identification_labels.push_back("No face");
		}
		else
		{

			int res_label;
			eff_color->classifyImage(resized_8U1, res_label);
			if (res_label == -1)
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

unsigned long ipa_PeopleDetector::FaceRecognizer::recognizeFace(cv::Mat& color_image, cv::Mat& depth_image, std::vector<cv::Rect>& face_coordinates,
		std::vector<std::string>& identification_labels)
{
	timeval t1, t2;
	gettimeofday(&t1, NULL);
	// secure this function with a mutex
	boost::lock_guard<boost::mutex> lock(m_data_mutex);

	//int number_eigenvectors = m_eigenvectors.size();
	if (eff_color->trained_ == false)
	{
		std::cout << "Error: FaceRecognizer::recognizeFace: Load or train some identification model, first.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	identification_labels.clear();

	//cv::Size resized_size(m_eigenvectors[0].size());
	for (int i = 0; i < (int)face_coordinates.size(); i++)
	{
		cv::Rect face = face_coordinates[i];
		//convertAndResize(depth_image, resized_8U1, face, resized_size);
		cv::Mat color_crop = color_image(face);
		cv::Mat depth_crop_xyz = depth_image(face);

		cv::Mat DM_crop = cv::Mat::zeros(m_norm_size, m_norm_size, CV_8UC1);
		cv::Size norm_size = cv::Size(m_norm_size, m_norm_size);
		if (face_normalizer_.normalizeFace(color_crop, depth_crop_xyz, norm_size, DM_crop))
			;

		double DFFS;
		cv::Mat temp;
		color_crop.convertTo(color_crop, CV_64FC1);

		int res_label_color, res_label_depth;
		std::string class_color;

		if (eff_color->trained_)
		{
			eff_color->classifyImage(color_crop, res_label_color);
			if (res_label_color == -1)
			{
				class_color = "Unknown";
			}
			else
			{
				class_color = m_current_label_set[res_label_color];
			}
		}

		identification_labels.push_back(class_color);
	}

	gettimeofday(&t2, NULL);
	if (m_debug)
		std::cout << "time =" << (t2.tv_usec - t1.tv_usec) / 1000.0 << std::endl;

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size)
{
	resized = img(face);
	cv::resize(resized, resized, new_size);

	//cv::cvtColor(resized, resized, CV_BGR2GRAY);

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::saveTrainingData(std::vector<cv::Mat>& face_images)
{
	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "tdata.xml";
	std::string img_ext = ".bmp";

	if (fs::is_directory(complete))
	{
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::WRITE);
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
			boost::filesystem::path img_path = path / "img" / (boost::lexical_cast<string>(i) + img_ext);
			std::ostringstream img, shortname;
			shortname << "img/" << i << img_ext;
			std::ostringstream tag2;
			tag2 << "image_" << i;
			fileStorage << tag2.str().c_str() << shortname.str().c_str();
			cv::imwrite(img_path.string(), face_images[i]);
		}

		fileStorage.release();

		std::cout << "INFO: FaceRecognizer::saveTrainingData: " << face_images.size() << " color images saved.\n" << std::endl;
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
	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "tdata.xml";
	std::string img_ext = ".bmp";
	std::string dm_ext = ".xml";

	if (fs::is_directory(path.string()))
	{
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::WRITE);
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
			boost::filesystem::path img_path = path / "img" / (boost::lexical_cast<string>(i) + img_ext);
			std::ostringstream img, shortname_img, shortname_depth;
			shortname_img << "img/" << i << img_ext;
			std::ostringstream tag2, tag3;
			tag2 << "image_" << i;
			fileStorage << tag2.str().c_str() << shortname_img.str().c_str();

			if (dm_exist[i])
			{
				shortname_depth << "depth/" << i << dm_ext;
				tag3 << "depthmap_" << i;
				fileStorage << tag3.str().c_str() << shortname_depth.str().c_str();
			}
			cv::imwrite(img_path.string(), face_images[i]);
		}

		fileStorage.release();

		int j = 0;
		for (unsigned int i = 0; i < dm_exist.size(); i++)
		{
			if (dm_exist[i])
			{
				// depth maps
				boost::filesystem::path dm_path = path / "depth" / (boost::lexical_cast<string>(i) + ".xml");
				cv::FileStorage fs(dm_path.string(), FileStorage::WRITE);
				fs << "depthmap" << face_depthmaps[j];
				fs.release();
				j++;
			}
		}
		std::cout << "INFO: FaceRecognizer::saveTrainingData: " << face_images.size() << " color images and " << face_depthmaps.size() << " depth images saved.\n" << std::endl;
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

	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "tdata.xml";

	if (fs::is_directory(path.string()))
	{
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::loadTrainingData: Can't open " << complete.string() << ".\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// labels
		m_face_labels.clear();
		face_images.clear();
		cv::Size norm_size = cv::Size(m_norm_size, m_norm_size);
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
			std::ostringstream tag_image;
			tag_image << "image_" << i;
			boost::filesystem::path path = m_data_directory / (std::string)fileStorage[tag_image.str().c_str()];
			cv::Mat temp = cv::imread(path.string(), -1);
			cv::resize(temp, temp, cv::Size(m_norm_size, m_norm_size));
			//face_normalizer_.normalizeFace(temp,norm_size);
			face_images.push_back(temp);
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

		std::cout << "INFO: FaceRecognizer::loadTrainingData: " << number_entries << " color images loaded.\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::loadTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}

unsigned long ipa_PeopleDetector::FaceRecognizer::loadTrainingData(std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps,
		std::vector<std::string>& identification_labels_to_train)
{
	bool use_all_data = false;
	dm_exist.clear();
	if (identification_labels_to_train.size() == 0)
		use_all_data = true;

	boost::filesystem::path path = m_data_directory;
	boost::filesystem::path complete = path / "tdata.xml";

	if (fs::is_directory(path.string()))
	{
		cv::FileStorage fileStorage(complete.string(), cv::FileStorage::READ);
		if (!fileStorage.isOpened())
		{
			std::cout << "Error: FaceRecognizer::loadTrainingData: Can't open " << complete.string() << ".\n" << std::endl;
			return ipa_Utils::RET_FAILED;
		}

		// labels
		m_face_labels.clear();
		face_images.clear();
		cv::Size norm_size = cv::Size(m_norm_size, m_norm_size);
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
			boost::filesystem::path img_path = m_data_directory / (std::string)fileStorage[tag_image.str().c_str()];
			boost::filesystem::path dm_path = m_data_directory / (std::string)fileStorage[tag_dm.str().c_str()];
			cv::Mat temp = cv::imread(img_path.string(), -1);

			if (dm_path.string().compare(m_data_directory.string()))
			{
				cv::Mat xyz_temp, dm_temp;
				cv::FileStorage fs(dm_path.string(), FileStorage::READ);
				fs["depthmap"] >> xyz_temp;
				face_normalizer_.normalizeFace(temp, xyz_temp, norm_size, dm_temp);
				face_depthmaps.push_back(dm_temp);
				dm_exist.push_back(true);
				fs.release();
			}
			else
			{
				dm_exist.push_back(false);
			}

			cv::resize(temp, temp, cv::Size(m_norm_size, m_norm_size));
			face_images.push_back(temp);
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

		std::cout << "INFO: FaceRecognizer::loadTrainingData: " << number_entries << " images loaded (" << face_images.size() << " color images and " << face_depthmaps.size() << " depth images).\n" << std::endl;
	}
	else
	{
		std::cerr << "Error: FaceRecognizer::loadTrainingData: Path '" << path << "' is not a directory." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	return ipa_Utils::RET_OK;
}
void ipa_PeopleDetector::FaceRecognizer::assertDirectories(boost::filesystem::path& data_directory)
{

	if (!boost::filesystem::exists(data_directory))
		boost::filesystem::create_directories(data_directory);
	if (!boost::filesystem::exists(data_directory / "depth"))
		boost::filesystem::create_directories(data_directory / "depth");
	if (!boost::filesystem::exists(data_directory / "img"))
		boost::filesystem::create_directories(data_directory / "img");
}
