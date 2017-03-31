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

/// @file HeadDetector.cpp

#ifdef __LINUX__
#include "cob_people_detection/head_detector.h"
#include "cob_vision_utils/GlobalDefines.h"
#include "cob_vision_utils/VisionUtils.h"
#else
#endif

#include <opencv/cv.h>
#include <opencv/cvaux.h>
#include <opencv/highgui.h>

using namespace ipa_PeopleDetector;

HeadDetector::HeadDetector(void)
{
	m_range_cascade = 0;
	m_initialized = false;
}

unsigned long HeadDetector::init(std::string model_directory, double depth_increase_search_scale, int depth_drop_groups, int depth_min_search_scale_x, int depth_min_search_scale_y)
{
	// parameters
	m_depth_increase_search_scale = depth_increase_search_scale;
	m_depth_drop_groups = depth_drop_groups;
	m_depth_min_search_scale_x = depth_min_search_scale_x;
	m_depth_min_search_scale_y = depth_min_search_scale_y;

	// Load Haar-Classifier for head detection in depth images
	std::string rangeCascadePath = model_directory + "haarcascades/haarcascade_range_multiview_5p_bg.xml";
	//std::string rangeCascadePath = model_directory + "haarcascades/haarcascade_range_multiview_5p_bg+.xml";	// + "haarcascades/haarcascade_range.xml";
	m_range_cascade = (CvHaarClassifierCascade*)cvLoad(rangeCascadePath.c_str(), 0, 0, 0);

	// Create Memory
	m_storage = cvCreateMemStorage(0);

	m_initialized = true;

	return ipa_Utils::RET_OK;
}

HeadDetector::~HeadDetector(void)
{
	// Release Classifiers and memory
	cvReleaseHaarClassifierCascade(&m_range_cascade);
	cvReleaseMemStorage(&m_storage);
}

unsigned long HeadDetector::interpolateUnassignedPixels(cv::Mat& img)
{
	CV_Assert(img.type() == CV_8UC3);

	cv::Mat temp = img.clone();

	uchar* data = img.data;
	uchar* data2 = temp.data;
	int stride = img.step;
	for (int repetitions = 0; repetitions < 10; repetitions++)
	{
		// each pixel with a value can propagate its value to black pixels in the 4 pixel neighborhood
		for (int v = 1; v < img.rows - 1; v++)
		{
			for (int u = 1; u < img.cols - 1; u++)
			{
				// only pixels with a value can propagate their value
				int index = v * stride + 3 * u;
				if (data[index] != 0)
				{
					uchar val = data[index];
					if (data2[index - 3] == 0)
						for (int i = -3; i < 0; i++)
							data2[index + i] = val; // left
					if (data2[index + 3] == 0)
						for (int i = 3; i < 6; i++)
							data2[index + i] = val; // right
					if (data2[index - stride] == 0)
						for (int i = -stride; i < -stride + 3; i++)
							data2[index + i] = val; // up
					if (data2[index + stride] == 0)
						for (int i = stride; i < stride + 3; i++)
							data2[index + i] = val; // down
				}
			}
		}
		// copy back new data
		for (int i = 0; i < img.rows * stride; i++)
			data[i] = data2[i];
	}
	return ipa_Utils::RET_OK;
}

unsigned long HeadDetector::detectRangeFace(cv::Mat& depth_image, std::vector<cv::Rect>& rangeFaceCoordinates, bool fillUnassignedDepthValues)
{
	if (m_initialized == false)
	{
		std::cout << "Error: HeadDetector::detectRangeFace: init() must be called first." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	rangeFaceCoordinates.clear();

	cv::Mat depth_image_8U3;
	ipa_Utils::ConvertToShowImage(depth_image, depth_image_8U3, 3);
	if (fillUnassignedDepthValues)
		interpolateUnassignedPixels(depth_image_8U3);
//	cv::namedWindow("depth image filled");
//	cv::imshow("depth image filled", depth_image_8U3);
//	cv::waitKey(10);
	IplImage imgPtr = (IplImage)depth_image_8U3;
	CvSeq* rangeFaces = cvHaarDetectObjects(&imgPtr, m_range_cascade, m_storage, m_depth_increase_search_scale, m_depth_drop_groups, CV_HAAR_DO_CANNY_PRUNING,
			cvSize(m_depth_min_search_scale_x, m_depth_min_search_scale_y));

	for (int i = 0; i < rangeFaces->total; i++)
	{
		cv::Rect *rangeFace = (cv::Rect*)cvGetSeqElem(rangeFaces, i);
		rangeFaceCoordinates.push_back(*rangeFace);
	}

	return ipa_Utils::RET_OK;
}
