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
#else
	#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
	#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif

// opencv
#include <opencv/ml.h>
#include <opencv/cv.h>

namespace ipa_PeopleDetector {

/// Interface to Calibrate Head of Care-O-bot 3.
/// Long description
class FaceRecognizer
{
public:

	/// Constructor.
	FaceRecognizer(void); ///< Constructor
	~FaceRecognizer(void); ///< Destructor

	/// Initialization function.
	/// Creates an instance of a range imaging sensor (i.e. SwissRanger SR-3000) and an instance of
	/// @param directory The directory for data files
	/// @return Return code
	virtual unsigned long init();

	/// Function to add a new face
	/// The function adds a new face to the trained images
	/// @param img Image
	/// @param face The face
	/// @param id Id of the new face
	/// @param images Vector with trained images
	/// @param ids Vector with trained ids
	/// @return Return code
	virtual unsigned long AddFace(cv::Mat& img, cv::Rect& face, std::string id, std::vector<cv::Mat>& images, std::vector<std::string>& ids);

	/// Function to Convert and Resizes a given image
	/// The function converts a 8U3 image from camera to an 8U1 image and resizes the face to new_size.
	/// @param img Image from camera
	/// @param resized Resized image patch from bounding box face
	/// @param face The face bounding box
	/// @param new_size The target size of the resized image
	/// @return Return code
	virtual unsigned long convertAndResize(cv::Mat& img, cv::Mat& resized, cv::Rect& face, cv::Size new_size);

	/// Applies some preprocessing to the grayscale face images to obtain a more robust identification.
	/// @param input_image Grayscale face image.
	/// @return Preprocessed image.
	virtual cv::Mat preprocessImage(cv::Mat& input_image);

	/// Function to Run the PCA algorithm
	/// @param nEigens Number of eigenvalues
	/// @param eigenVectors Eigenvectors
	/// @param eigenValMat Eigenvalues
	/// @param avgImage Average image
	/// @param images Trained faces
	/// @param projectedTrainFaceMat Projected training faces (coefficients for the eigenvectors of the face subspace)
	/// @return Return code
	virtual unsigned long PCA(int* nEigens, std::vector<cv::Mat>& eigenVectors, cv::Mat& eigenValMat, cv::Mat& avgImage, std::vector<cv::Mat>& images, cv::Mat& projectedTrainFaceMat);

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param colorImage source color image
	/// @param faceCoordinates Detected faces
	/// @param nEigens Number of eigenvalues
	/// @param eigenVectArr Eigenvectors
	/// @param avgImage Average image
	/// @param projectedTrainFaceMat Projected training faces
	/// @param index Index of classified face in vector
	/// @param threshold The threshold to recognize unkown faces
	/// @param threshold_FS The threshold to the face space
	/// @param eigenValMat Eigenvalues
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long RecognizeFace(cv::Mat& colorImage, std::vector<cv::Rect>& colorFaces, std::vector<int>& index, cv::SVM* personClassifier = 0);

	/// Function to find the closest face class
	/// The function calculates the distance of each sample image to the trained face class
	/// @param projectedTestFace The projected test face
	/// @param nearest Index of nearest face, or -1 if the face is unknown
	/// @param nEigens Number of eigenvalues
	/// @param projectedTrainFaceMat The average factors from each face class originating from the eigenvector decomposition
	/// @param threshold The threshold to recognize unkown faces
	/// @param eigenValMat Eigenvalues
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long ClassifyFace(float *projectedTestFace, int *nearest, int *nEigens, cv::Mat& projectedTrainFaceMat, int *threshold, cv::Mat& eigenValMat, cv::SVM* personClassifier = 0);

	/// Function to calculate the FaceClasses
	/// The function calculates the average eigenvector decomposition factors for each face classes.
	/// @param projectedTrainFaceMat The projected training faces
	/// @param id The ids of the training faces
	/// @param nEigens Number of eigenvalues
	/// @param faceClassAvgProjections The average factors of the eigenvector decomposition from each face class
	/// @param idUnique A vector containing all different Ids from the training session exactly once (idUnique[i] stores the corresponding id to the average face coordinates in the face subspace in faceClassAvgProjections.row(i))
	/// @param personClassifier A classifier for person identification. It is trained in this function. Can be left out if a simpler identification method is used.
	/// @return Return code
	virtual unsigned long CalculateFaceClasses(cv::Mat& projectedTrainFaceMat, std::vector<std::string>& id, int *nEigens, cv::Mat& faceClassAvgProjections, std::vector<std::string>& idUnique, cv::SVM* personClassifier = 0);


private:
	/// interpolates unassigned pixels in the depth image when using the kinect
	/// @param img depth image
	/// @return Return code
	unsigned long InterpolateUnassignedPixels(cv::Mat& img);

//	int m_n_eigens;								///< Number of eigenvalues
	std::vector<cv::Mat> m_eigen_vectors;		///< Eigenvectors (spanning the face space)
	cv::Mat m_eigen_val_mat;					///< Eigenvalues
	cv::Mat m_avg_image;						///< Trained average Image
	cv::Mat m_projected_train_face_mat;			///< Projected training faces (coefficients for the eigenvectors of the face subspace)
	cv::Mat m_face_class_avg_projections;		///< The average factors of the eigenvector decomposition from each face class
	cv::SVM m_person_classifier;				///< classifier for the identity of a person

	// parameters
	int m_threshold_unknown;					///< Threshold to detect unknown faces
	int m_threshold_facespace;					///< Threshold to facespace

};

} // end namespace

#endif // __FACE_RECOGNIZER_H__
