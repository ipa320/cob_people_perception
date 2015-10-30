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

#ifndef __FACE_RECOGNIZER_H__
#define __FACE_RECOGNIZER_H__

#ifdef __LINUX__
#include <cob_people_detection/abstract_face_recognizer.h>
#include <cob_people_detection/face_normalizer.h>
#include <cob_people_detection/face_recognizer_algorithms.h>
#else
#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif

// opencv
#include <opencv/cv.h>
#include <opencv/ml.h>

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include "boost/filesystem/path.hpp"
#include "boost/lexical_cast.hpp"

#include <algorithm>
namespace ipa_PeopleDetector
{

class FaceRecognizer: public AbstractFaceRecognizer
{
public:

	/// Constructor.
	FaceRecognizer(void); ///< Constructor
	~FaceRecognizer(void); ///< Destructor

	/// Initialization function.
	/// Parameters: see class member explanations.
	/// @param data_directory The directory for data files
	/// @param identification_labels_to_recognize A list of labels of persons that shall be recognized
	/// @return Return code
	virtual unsigned long init(std::string data_directory, int norm_size, bool norm_illumination, bool norm_align, bool norm_extreme_illumination, int metric, bool debug,
			std::vector<std::string>& identification_labels_to_recognize, int subs_meth, int feature_dim, bool use_unknown_thresh, bool use_depth);

	/// Initialization function for training purposes (only for capturing images, not the training of recognition models).
	/// Parameters: see class member explanations.
	/// @param data_directory The directory for data files
	/// @param face_images A list of images of persons that shall be recognized which will be loaded by the function.
	/// @return Return code
	virtual unsigned long initTraining(std::string data_directory, int norm_size, bool norm_illumination, bool norm_align, bool norm_extreme_illumination, bool debug,
			std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps, bool use_depth);

	/// Function to add a new face
	/// The function adds a new face to the trained images. The labels are stored internally in m_face_labels. The face_images are stored externally to avoid waste of memory.
	/// @param color_image Color image containing the face
	/// @param face_bounding_box Rectangular bounding box of the detected face
	/// @param label Label of the new face
	/// @param face_images Vector containing all trained images
	/// @return Return code
	virtual unsigned long addFace(cv::Mat& color_image, cv::Mat& depth_image, cv::Rect& face_bounding_box, cv::Rect& head_bounding_box, std::string label,
			std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps);

	/// Updates the labels of a stored person.
	/// @param old_label The label in the database which shall be replaced by the new label
	/// @param new_label The new label
	/// @return Return code
	virtual unsigned long updateFaceLabels(std::string old_label, std::string new_label);

	/// Updates the label of a single face in the database.
	/// @param index The index of the face in the database whose label shall be replaced by the new label
	/// @param new_label The new label
	/// @return Return code
	virtual unsigned long updateFaceLabel(int index, std::string new_label);

	/// Deletes all database entries with the provided label.
	/// @param label The label of the database entries which shall be deleted
	/// @param face_images Vector containing all trained images
	/// @return Return code
	virtual unsigned long deleteFaces(std::string label, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps);

	/// Deletes the database entry with the provided index.
	/// @param index The index of the database entry which shall be deleted
	/// @param face_images Vector containing all trained images
	/// @return Return code
	virtual unsigned long deleteFace(int index, std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps);

	/// Saves the training data
	/// @param face_images A vector containing all training images
	/// @return Return code
	virtual unsigned long saveTrainingData(std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps);
	virtual unsigned long saveTrainingData(std::vector<cv::Mat>& face_images);

	/// Trains a model for the recognition of a given set of faces.
	/// @param identification_indices_to_train List of labels whose corresponding faces shall be trained. If empty, all available data is used and this list is filled with the labels.
	/// @return Return code
	virtual unsigned long trainRecognitionModel(std::vector<std::string>& identification_labels_to_train);

	/// Saves the currently trained model for the recognition of a given set of faces.
	/// @return Return code
	virtual unsigned long saveRecognitionModel();

	/// Loads a model for the recognition of a given set of faces.
	/// @param identification_labels_to_recognize List of labels whose corresponding faces shall be available for recognition
	/// @return Return code
	virtual unsigned long loadRecognitionModel(std::vector<std::string>& identification_labels_to_recognize);

	enum Metrics
	{
		EUCLIDEAN, MAHALANOBIS, MAHALANOBISCOSINE
	};

protected:

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param color_image source color image
	/// @param face_coordinates Bounding boxes of detected faces (input parameter)
	/// @param identification_labels Vector of labels of classified faces, vector indices correspond with bounding boxes in face_coordinates
	/// @return Return code
	virtual unsigned long recognizeFace(cv::Mat& color_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels);
	virtual unsigned long recognizeFace(cv::Mat& color_image, cv::Mat& depth_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels);

	/// Function to find the closest face class
	/// The function calculates the distance of each sample image to the trained face class
	/// @param eigen_vector_weights The weights of corresponding eigenvectors of projected test face
	/// @param face_label Label of closest face, or 'Unknown' if the face is unknown
	/// @param number_eigenvectors Number of eigenvalues
	/// @return Return code
	virtual unsigned long convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size);

	/// Loads the training data for the persons specified in identification_labels_to_train
	/// @param face_images A vector containing all training images
	/// @param identification_indices_to_train List of labels whose corresponding faces shall be trained. If empty, all available data is used and this list is filled with the labels.
	/// @return Return code
	virtual unsigned long loadTrainingData(std::vector<cv::Mat>& face_images, std::vector<std::string>& identification_labels_to_train);
	virtual unsigned long loadTrainingData(std::vector<cv::Mat>& face_images, std::vector<cv::Mat>& face_depthmaps, std::vector<std::string>& identification_labels_to_train);

	/// Function can be used to verify the existence of the data directory and created if it does not exist.
	/// @bief Assertion of the data directory
	/// @param[in] data_directory Path to top level directory with training data
	void assertDirectories(boost::filesystem::path& data_directory);

	// DEPTH
	std::vector<std::string> depth_str_labels; ///< Vector for class label strings for depth training data
	std::vector<std::string> depth_str_labels_unique; ///< Vector for class unique label strings for depth training data
	std::vector<int> depth_num_labels; ///< Number of classes for depth training data
	//
	FaceNormalizer face_normalizer_; ///< Face normalizer object

	ipa_PeopleDetector::FaceRecognizerBaseClass* eff_depth; ///< FaceRecognizer for depth maps
	ipa_PeopleDetector::FaceRecognizerBaseClass* eff_color; ///< FaceRecognizer for color images
	std::vector<int> m_label_num;
	int m_rec_method; ///< flag for recognition method
	std::vector<bool> dm_exist; ///< vector indicating if depth map exists for corresponding color image
	bool m_depth_mode; ///< flag indicates if depth maps are ignored or used for classification
	ipa_PeopleDetector::Method m_subs_meth; ///< recognition method
	bool m_use_unknown_thresh; ///< flag indicates if unknown threshold is used
	unsigned long trainFaceRecognition(ipa_PeopleDetector::FaceRecognizerBaseClass* eff, std::vector<cv::Mat>& data, std::vector<int>& labels);
	//----------------------------------------------------
	//----------------------------------------------------

	// data
	std::vector<cv::Mat> m_eigenvectors; ///< Eigenvectors (spanning the face space)
	IplImage** m_eigenvectors_ipl; ///< Eigenvalues stored in IplImage format (to avoid conversion each time the function is called)
	cv::Mat m_eigenvalues; ///< Eigenvalues
	cv::Mat m_average_image; ///< Trained average Image
	cv::Mat m_projected_training_faces; ///< Projected training faces (coefficients for the eigenvectors of the face subspace)
	std::vector<std::string> m_face_labels; ///< A vector containing the corresponding labels to each face image projection in m_projected_training_faces (m_face_labels[i] stores the corresponding name to the face representation in the face subspace in m_projected_training_faces.rows(i))
	cv::Mat m_face_class_average_projections; ///< The average factors of the eigenvector decomposition from each face class; The average factors from each face class originating from the eigenvector decomposition.
	std::vector<std::string> m_current_label_set; ///< A vector containing all different labels from the training session exactly once, order of appearance matters! (m_current_label_set[i] stores the corresponding name to the average face coordinates in the face subspace in m_face_class_average_projections.rows(i))
	cv::SVM m_face_classifier; ///< classifier for the identity of a person
	boost::filesystem::path m_data_directory; ///< folder that contains the training data

	// mutex
	boost::mutex m_data_mutex; ///< secures the internal data variables against usage during training

	// parameters
	int m_norm_size; ///< Desired width and height of the Eigenfaces (=eigenvectors).
	int m_eigenvectors_per_person; ///< Number of eigenvectors per person to identify -> controls the total number of eigenvectors
	double m_threshold_facespace; ///< Threshold to facespace
	double m_threshold_unknown; ///< Threshold to detect unknown faces
	int m_metric; ///< metric for nearest neighbor search in face space: 0 = Euklidean, 1 = Mahalanobis, 2 = Mahalanobis Cosine
	bool m_debug; ///< enables some debug outputs

	int m_feature_dim; ///< Dimension of features that is computed when using Eigenfaces,2D-PCA, 2D-LDA
};

} // end namespace

#endif // __FACE_RECOGNIZER_H__
