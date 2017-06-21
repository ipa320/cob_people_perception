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
 * abstract class with common functions for recognizing a face within a color image (patch)
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

#ifndef __ABSTRACT_FACE_RECOGNIZER_H__
#define __ABSTRACT_FACE_RECOGNIZER_H__

#ifdef __LINUX__
#else
#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif

// opencv
#include <opencv2/opencv.hpp>

namespace ipa_PeopleDetector
{

class AbstractFaceRecognizer
{
public:

	/// Constructor.
	AbstractFaceRecognizer(void); ///< Constructor
	~AbstractFaceRecognizer(void); ///< Destructor

	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param color_images Source color images
	/// @param face_coordinates Bounding boxes of detected faces (input parameter, local coordinates wrt. to respective image patch), outer index corresponds to color_image index
	/// @param identification_labels Vector of labels of classified faces, both indices correspond with face_coordinates
	/// @return Return code
	virtual unsigned long recognizeFaces(std::vector<cv::Mat>& color_images, std::vector<std::vector<cv::Rect> >& face_coordinates,
			std::vector<std::vector<std::string> >& identification_labels);
	virtual unsigned long recognizeFaces(std::vector<cv::Mat>& color_images, std::vector<cv::Mat>& depth_images, std::vector<std::vector<cv::Rect> >& face_coordinates,
			std::vector<std::vector<std::string> >& identification_labels);

	/// Trains a model for the recognition of a given set of faces.
	/// @param identification_labels_to_train List of labels whose corresponding faces shall be trained.
	/// @return Return code
	virtual unsigned long trainRecognitionModel(std::vector<std::string>& identification_labels_to_train) = 0;

	/// Saves the currently trained model for the recognition of a given set of faces.
	/// @return Return code
	virtual unsigned long saveRecognitionModel() = 0;

	/// Loads a model for the recognition of a given set of faces.
	/// @param identification_labels_to_recognize List of labels whose corresponding faces shall be available for recognition
	/// @return Return code
	virtual unsigned long loadRecognitionModel(std::vector<std::string>& identification_labels_to_recognize) = 0;

protected:
	/// Function to Recognize faces
	/// The function recognize the faces
	/// @param color_image source color image
	/// @param face_coordinates Bounding boxes of detected faces (input parameter)
	/// @param identification_labels Vector of labels of classified faces, indices correspond with bounding boxes in color_face_coordinates
	/// @return Return code
	virtual unsigned long recognizeFace(cv::Mat& color_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels) = 0;
	virtual unsigned long recognizeFace(cv::Mat& color_image, cv::Mat& depth_image, std::vector<cv::Rect>& face_coordinates, std::vector<std::string>& identification_labels) = 0;
};

} // end namespace

#endif // __ABSTRACT_FACE_RECOGNIZER_H__
