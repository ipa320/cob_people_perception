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


using namespace ipa_PeopleDetector;

FaceRecognizer::FaceRecognizer(void)
{
	m_eigen_vectors_ipl = 0;
}

unsigned long FaceRecognizer::init(std::string data_directory, int eigenface_size, int eigenvectors_per_person, double threshold_facespace, double threshold_unknown, int metric, bool debug)
{
	// parameters
	m_data_directory = data_directory;
	m_eigenface_size = eigenface_size;
	m_eigenvectors_per_person = eigenvectors_per_person;
	m_threshold_facespace = threshold_facespace;
	m_threshold_unknown = threshold_unknown;
	m_metric = metric;
	m_debug = debug;

	return ipa_Utils::RET_OK;
}

FaceRecognizer::~FaceRecognizer(void)
{
	if (m_eigen_vectors_ipl != 0)
	{
		for (int i=0; i<number_eigenvectors; i++)
			cvReleaseImage(&(m_eigen_vectors_ipl[i]));
		cvFree(&m_eigen_vectors_ipl);
	}
}

//unsigned long FaceRecognizer::AddFace(cv::Mat& img, cv::Rect& face, std::string id, std::vector<cv::Mat>& images, std::vector<std::string>& ids)
//{
//	//IplImage *resized_8U1 = cvCreateImage(cvSize(100, 100), 8, 1);
//	cv::Mat resized_8U1(100, 100, CV_8UC1);
//	ConvertAndResize(img, resized_8U1, face);
//
//	// Save image
//	images.push_back(resized_8U1);
//	ids.push_back(id);
//
//	return ipa_Utils::RET_OK;
//}

unsigned long FaceRecognizer::trainRecognitionModel(std::string directory, std::vector<std::string>& identification_labels_to_train)
{
	// load necessary data
	std::vector<cv::Mat> face_images;
	loadTrainingData(face_images, identification_labels_to_train);
	m_current_label_set = identification_labels_to_train;

	// rescale if necessary to m_eigenface_size
	if (face_images.size() > 0 && (face_images[0].cols != m_eigenface_size || face_images[0].rows != m_eigenface_size))
	{
		cv::Size new_size(m_eigenface_size, m_eigenface_size);
		for (uint i=0; i<face_images.size(); i++)
		{
			cv::Mat temp = face_images[i];
			cv::resize(temp, face_images[i], new_size);
		}
	}

	// PCA
	int number_eigenvectors = std::min(m_eigenvectors_per_person * identification_labels_to_train.size(), face_images.size());
	PCA(number_eigenvectors, face_images);

	// compute average face projections per class
	computeAverageFaceProjections();

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::PCA(int number_eigenvectors, std::vector<cv::Mat>& face_images)
{
	if(face_images.size() < 2)
	{
		std::cout << "Error: FaceRecognizer::PCA: Less than two images available for training.\n";
		return ipa_Utils::RET_FAILED;
	}

	// Allocate memory
	cv::Size face_image_size(face_images[0].cols, face_images[0].rows);
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

	// Convert eigenvector vector to array
	convertEigenvectorsToIpl();

	// Compute average image, eigenvalues, and eigenvectors
	IplImage average_image_ipl = (IplImage)m_average_image;
	cvCalcEigenObjects((int)face_images.size(), (void*)face_images_ipl, (void*)m_eigen_vectors_ipl, CV_EIGOBJ_NO_CALLBACK, 0, 0, &calcLimit, &average_image_ipl, (float*)(m_eigenvalues.data));

	cv::normalize(m_eigenvalues,m_eigenvalues, 1, 0, /*CV_L1*/CV_L2);	//, 0);		0=bug?

	// Project the training images onto the PCA subspace
	m_projected_training_faces.create(face_images.size(), number_eigenvectors, CV_32FC1);
	for(int i=0; i<(int)face_images.size(); i++)
	{
		IplImage temp = (IplImage)face_images[i];
		cvEigenDecomposite(&temp, number_eigenvectors, m_eigen_vectors_ipl, 0, 0, &average_image_ipl, (float*)m_projected_training_faces.data + i * number_eigenvectors);	//attention: if image step of m_projected_training_faces is not number_eigenvectors * sizeof(float) then reading functions which access with (x,y) coordinates might fail
	};

	// Copy back
	//int eigenVectorsCount = (int)m_eigenvectors.size();
	m_eigenvectors.clear();
	for (int i=0; i<number_eigenvectors; i++) m_eigenvectors.push_back(cv::Mat(m_eigen_vectors_ipl[i], true));

	// Clean
	for (int i=0; i<(int)face_images.size(); i++) cvReleaseImage(&(face_images_ipl[i]));
	cvFree(&face_images_ipl);
	//for (int i=0; i<(int)m_eigenvectors.size(); i++) cvReleaseImage(&(m_eigen_vectors_ipl[i]));
	//cvFree(&m_eigen_vectors_ipl);

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::computeAverageFaceProjections()
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
		//personClassifier->train_auto(data, labels, cv::Mat(), cv::Mat(), svmParams, 10, cv::SVM::get_default_grid(CvSVM::C), CvParamGrid(0.001953125, 2.01, 2.0), cv::SVM::get_default_grid(CvSVM::P), CvParamGrid(0.0125, 1.0, 2.0));
		personClassifier->train(data, labels, cv::Mat(), cv::Mat(), svmParams);
		cv::SVMParams svmParamsOptimal = personClassifier->get_params();
		if (m_debug) std::cout << "\nOptimal SVM params: gamma=" << svmParamsOptimal.gamma << "  nu=" << svmParamsOptimal.nu << "\n";
	}

	if (m_debug) std::cout << "done\n";

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::saveRecognitionModel(std::string directory)
{
	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::loadRecognitionModel(std::string directory, std::vector<std::string>& identification_labels_to_recognize)
{
	// check whether currently trained data set corresponds with intentification_labels_to_recognize


	if (false)
	{
		// just load the current model
		// todo
	}
	else
	{
		// else recompute the model from training data
		trainRecognitionModel(directory, identification_labels_to_recognize);
	}

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::convertEigenvectorsToIpl()
{
	int number_eigenvectors = m_eigenvectors.size();

	// clear
	if (m_eigen_vectors_ipl != 0)
	{
		for (int i=0; i<number_eigenvectors; i++)
			cvReleaseImage(&(m_eigen_vectors_ipl[i]));
		cvFree(&m_eigen_vectors_ipl);
	}

	// Convert vector to array
	m_eigen_vectors_ipl = (IplImage**)cvAlloc(number_eigenvectors*sizeof(IplImage*));
	for(int j=0; j<number_eigenvectors; j++)
	{
		IplImage temp = (IplImage)m_eigenvectors[j];
		m_eigen_vectors_ipl[j] = cvCloneImage(&temp);
	}

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::recognizeFace(cv::Mat& color_image, std::vector<cv::Rect>& face_coordinates, std::vector<int>& identification_index)
{
	int number_eigenvectors = m_eigenvectors.size();
	float* eigen_vector_weights = 0;
	eigen_vector_weights = (float *)cvAlloc(number_eigenvectors*sizeof(float));

	// Convert vector to array
//	IplImage** m_eigen_vectors_ipl = (IplImage**)cvAlloc(number_eigenvectors*sizeof(IplImage*));
//	for(int j=0; j<number_eigenvectors; j++)
//	{
//		IplImage temp = (IplImage)m_eigenvectors[j];
//		m_eigen_vectors_ipl[j] = cvCloneImage(&temp);
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
		cvEigenDecomposite(&resized_8U1Ipl, number_eigenvectors, m_eigen_vectors_ipl, 0, 0, &avg_image_ipl, eigen_vector_weights);

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
		if (m_debug) std::cout << "face space distance: " << distance << std::endl;

		// -2=distance to face space is too high
		// -1=distance to face classes is too high
		if(distance > m_threshold_facespace)
		{
			// no face
			identification_index.push_back(-2);
		}
		else
		{
			int face_index;
			classifyFace(eigen_vector_weights, &face_index, number_eigenvectors);
			if(face_index < 0)
				identification_index.push_back(-1);	// face unknown
			else
				identification_index.push_back(face_index);	// face known, it's number nearest
		}
	}

	// clear
	//for (int i=0; i<number_eigenvectors; i++) cvReleaseImage(&(m_eigen_vectors_ipl[i]));
	cvFree(&eigen_vector_weights);
	//cvFree(&m_eigen_vectors_ipl);

	return ipa_Utils::RET_OK;
}

cv::Mat FaceRecognizer::preprocessImage(cv::Mat& input_image)
{
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

unsigned long FaceRecognizer::convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size)
{
	cv::Mat temp;
	cv::cvtColor(img, temp, CV_BGR2GRAY);
	cv::Mat roi = temp(face);
	cv::resize(roi, resized, new_size);

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::classifyFace(float *eigen_vector_weights, int *face_index, int number_eigenvectors)
{
	double least_dist_sqared = DBL_MAX;

	for(int i=0; i<m_face_class_average_projections.rows; i++)
	{
		double distance=0;
		double cos=0;
		double length_sample=0;
		double length_projection=0;
		for(int e=0; e<number_eigenvectors; e++)
		{
			if (m_metric < 2)
			{
				float d = eigen_vector_weights[e] - ((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e];
				if (m_metric==0)
					distance += d*d;							//Euklid
				else
					distance += d*d / ((float*)(m_eigenvalues.data))[e];	//Mahalanobis
			}
			else
			{
				cos += eigen_vector_weights[e] * ((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e] / ((float*)(m_eigenvalues.data))[e];
				length_projection += ((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e] * ((float*)(m_face_class_average_projections.data))[i * number_eigenvectors + e] / ((float*)(m_eigenvalues.data))[e];
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
			distance = -cos;
		}

		if (m_debug) std::cout << "Distance_FC: " << distance << std::endl;

		if(distance < least_dist_sqared)
		{
			least_dist_sqared = distance;
			if(least_dist_sqared > m_threshold_unknown)
				*face_index = -1;
			else
				*face_index = i;
		}
	}

	// todo:
//	if (personClassifier != 0 && *nearest != -1)
//	{
//		cv::Mat temp(1, *nEigens, CV_32FC1, eigenVectorWeights);
//		std::cout << "class. output: " << (int)personClassifier->predict(temp) << "\n";
//		*nearest = (int)personClassifier->predict(temp);
//	}

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::saveTrainingData(std::vector<cv::Mat>& face_images)
{
	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";
	std::string img_ext = ".bmp";

	std::ostringstream complete;
	complete << path << filename;

	cv::FileStorage fileStorage(complete.str().c_str(), cv::FileStorage::WRITE);
	if(!fileStorage.isOpened())
	{
		std::cout << "Error: FaceRecognizer::SaveTrainingData: Can't save training data.\n" << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	// labels
	fileStorage << "number_labels" << (int)m_face_labels.size();
	for(int i=0; i<(int)m_face_labels.size(); i++)
	{
		std::ostringstream tag;
		tag << "label_" << i;
		fileStorage << tag.str().c_str() << m_face_labels[i].c_str();
	}

	// face images
	fileStorage << "number_face_images" << (int)face_images.size();
	for(int i=0; i<(int)face_images.size(); i++)
	{
		std::ostringstream img, shortname;
		img << path << i << img_ext;
		shortname << "training_data/" << i << img_ext;
		std::ostringstream tag;
		tag << "image_" << i;
		fileStorage << tag.str().c_str() << shortname.str().c_str();
		cv::imwrite(img.str().c_str(), face_images_[i]);
	}

	fileStorage.release();

	std::cout << "INFO: FaceRecognizer::SaveTrainingData: " << face_images_.size() << " images saved.\n" << std::endl;

	return ipa_Utils::RET_OK;
}

unsigned long FaceRecognizer::loadTrainingData(std::vector<cv::Mat>& face_images, std::vector<std::string>& identification_labels_to_train)
{
	bool use_all_data = false;
	if (identification_labels_to_train.size() == 0)
		use_all_data = true;

	std::string path = m_data_directory + "training_data/";
	std::string filename = "tdata.xml";

	std::ostringstream complete;
	complete << path << filename;

//	if(fs::is_directory(path.c_str()))
//	{
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
			std::string path = directory_ + (std::string)fileStorage[tag_image.str().c_str()];
			cv::Mat temp = cv::imread(path.c_str(),0);
			face_images.push_back(temp);
		}

		// set next free filename
		// filename_ = number_face_images;

		fileStorage.release();

		std::cout << "INFO: FaceRecognizer::loadTrainingData: " << number_entries << " images loaded.\n" << std::endl;
//	}
//	else
//	{
//		std::cerr << "ERROR - FaceRecognizer::loadTrainingData:" << std::endl;
//		std::cerr << "\t .... Path '" << path << "' is not a directory." << std::endl;
//		return ipa_Utils::RET_FAILED;
//	}

	return ipa_Utils::RET_OK;
}
