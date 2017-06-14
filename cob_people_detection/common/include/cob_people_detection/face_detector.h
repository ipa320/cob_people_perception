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
 * functions for detecting a face within a color image (patch)
 * current approach: haar detector on color image
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

#ifndef __FACE_DETECTOR_H__
#define __FACE_DETECTOR_H__

#ifdef __LINUX__
#else
#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif

#include <opencv2/opencv.hpp>

namespace ipa_PeopleDetector
{

class FaceDetector
{
public:

	/// Constructor.
	FaceDetector(void); ///< Constructor
	~FaceDetector(void); ///< Destructor

	/// Initialization function.
	/// @param directory The directory for data files
	/// @param faces_increase_search_scale The factor by which the search window is scaled between the subsequent scans
	/// @param faces_drop_groups Minimum number (minus 1) of neighbor rectangles that makes up an object.
	/// @param faces_min_search_scale_x Minimum search scale x
	/// @param faces_min_search_scale_y Minimum search scale y
	/// @param reason_about_3dface_size if true, the 3d face size is determined and only faces with reasonable size are accepted
	/// @param face_size_max_m the maximum feasible face diameter [m] if reason_about_3dface_size is enabled
	/// @param face_size_min_m the minimum feasible face diameter [m] if reason_about_3dface_size is enabled
	/// @param max_face_z_m maximum distance [m] of detected faces to the sensor
	/// @param debug enables some debug outputs
	/// @return Return code
	virtual unsigned long init(std::string directory, double faces_increase_search_scale, int faces_drop_groups, int faces_min_search_scale_x, int faces_min_search_scale_y,
			bool reason_about_3dface_size, double face_size_max_m, double face_size_min_m, double max_face_z_m, bool debug);

	/// Function to detect the faces on color image
	/// The function detects the faces in an given image
	/// @param heads_color_images Color images of the regions that supposedly contain a head
	/// @param heads_depth_images Depth images of the regions that supposedly contain a head
	/// @param face_coordinates Vector of same size as heads_color_images, each entry becomes filled with another vector with the coordinates of detected faces in color image, i.e. outer index corresponds with index of heads_color_images
	/// @return Return code
	virtual unsigned long detectColorFaces(std::vector<cv::Mat>& heads_color_images, const std::vector<cv::Mat>& heads_depth_images,
			std::vector<std::vector<cv::Rect> >& face_coordinates);

protected:

	// parameters
	double m_faces_increase_search_scale; ///< The factor by which the search window is scaled between the subsequent scans
	int m_faces_drop_groups; ///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_faces_min_search_scale_x; ///< Minimum search scale x
	int m_faces_min_search_scale_y; ///< Minimum search scale y
	bool m_reason_about_3dface_size; ///< if true, the 3d face size is determined and only faces with reasonable size are accepted
	double m_face_size_max_m; ///< the maximum feasible face diameter [m] if reason_about_3dface_size is enabled
	double m_face_size_min_m; ///< the minimum feasible face diameter [m] if reason_about_3dface_size is enabled
	double m_max_face_z_m; ///< maximum distance [m] of detected faces to the sensor
	bool m_debug; ///< enables some debug outputs

	CvMemStorage* m_storage; ///< Storage for face and eye detection
	CvHaarClassifierCascade* m_face_cascade; ///< Haar-Classifier for face-detection

	bool m_initialized; ///< indicates whether the class was already initialized
};

} // end namespace

#endif // __FACE_DETECTOR_H__
