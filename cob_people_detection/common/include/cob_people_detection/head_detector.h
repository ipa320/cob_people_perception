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
 * functions for detecting a head within a point cloud/depth image
 * current approach: haar detector on depth image
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

#ifndef __HEAD_DETECTOR_H__
#define __HEAD_DETECTOR_H__

#ifdef __LINUX__
#else
#include "cob_vision/cob_vision_ipa_utils/common/include/cob_vision_ipa_utils/MathUtils.h"
#include "cob_vision/cob_sensor_fusion/common/include/cob_sensor_fusion/ColoredPointCloud.h"	// todo: necessary?
#endif
#include <opencv2/opencv.hpp>

namespace ipa_PeopleDetector
{

class HeadDetector
{
public:

	/// Constructor.
	HeadDetector(void); ///< Constructor
	~HeadDetector(void); ///< Destructor

	/// Initialization function.
	/// @param model_directory The directory for data files
	/// @param depth_increase_search_scale The factor by which the search window is scaled between the subsequent scans
	/// @param depth_drop_groups Minimum number (minus 1) of neighbor rectangles that makes up an object.
	/// @param depth_min_search_scale_x Minimum search scale x
	/// @param depth_min_search_scale_y Minimum search scale y
	/// @return Return code
	virtual unsigned long init(std::string model_directory, double depth_increase_search_scale, int depth_drop_groups, int depth_min_search_scale_x, int depth_min_search_scale_y);

	/// Function to detect the face on range image
	/// The function detects the face in a given range image
	/// @param img Depth image of the depth camera (in format CV_32FC3 - one channel for x, y and z)
	/// @param rangeFaceCoordinates Vector with the bounding boxes (image coordinates) of detected heads in range image
	/// @param fillUnassignedDepthValues this parameter should be true if the kinect sensor is used (activates a filling method for black pixels)
	/// @return Return code
	virtual unsigned long detectRangeFace(cv::Mat& depth_image, std::vector<cv::Rect>& rangeFaceCoordinates, bool fillUnassignedDepthValues = false);

protected:
	/// interpolates unassigned pixels in the depth image when using the kinect
	/// @param img depth image
	/// @return Return code
	unsigned long interpolateUnassignedPixels(cv::Mat& img);

	double m_depth_increase_search_scale; ///< The factor by which the search window is scaled between the subsequent scans
	int m_depth_drop_groups; ///< Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int m_depth_min_search_scale_x; ///< Minimum search scale x
	int m_depth_min_search_scale_y; ///< Minimum search scale y

	CvMemStorage* m_storage; ///< Storage for face and eye detection
	CvHaarClassifierCascade* m_range_cascade; ///< Haar-Classifier for range-detection

	bool m_initialized; ///< indicates whether the class was already initialized
};

} // end namespace

#endif // __HEAD_DETECTOR_H__
