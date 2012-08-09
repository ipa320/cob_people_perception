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

#ifdef __LINUX__
	#include "cob_people_detection/face_detector.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#include "cob_vision/cob_people_detection/common/include/cob_people_detection/PeopleDetector.h"
#include "cob_common/cob_vision_utils/common/include/cob_vision_utils/GlobalDefines.h"
#endif

#include <opencv/cv.h>
#include <opencv/cvaux.h>

using namespace ipa_PeopleDetector;

FaceDetector::FaceDetector(void)
{	
	m_face_cascade = 0;
	m_initialized = false;
}

unsigned long FaceDetector::init(std::string directory, double faces_increase_search_scale, int faces_drop_groups, int faces_min_search_scale_x, int faces_min_search_scale_y)
{
	// parameters
	m_faces_increase_search_scale = faces_increase_search_scale;
	m_faces_drop_groups = faces_drop_groups;
	m_faces_min_search_scale_x = faces_min_search_scale_x;
	m_faces_min_search_scale_y = faces_min_search_scale_y;

	// load Haar-Classifier for frontal face detection
	std::string faceCascadePath = directory + "haarcascades/haarcascade_frontalface_alt2.xml";
	m_face_cascade = (CvHaarClassifierCascade*)cvLoad(faceCascadePath.c_str(), 0, 0, 0 );	//"ConfigurationFiles/haarcascades/haarcascade_frontalface_alt2.xml", 0, 0, 0 );

	// Create Memory
	m_storage = cvCreateMemStorage(0);

	m_initialized = true;

	return ipa_Utils::RET_OK;
}

FaceDetector::~FaceDetector(void)
{
	// Release Classifiers and memory
	cvReleaseHaarClassifierCascade(&m_face_cascade);
	cvReleaseMemStorage(&m_storage);
}

unsigned long FaceDetector::detectColorFaces(std::vector<cv::Mat>& heads_color_images, std::vector<std::vector<cv::Rect> >& face_coordinates)
{
	if (m_initialized == false)
	{
		std::cout << "Error: FaceDetector::DetectColorFaces: init() must be called first." << std::endl;
		return ipa_Utils::RET_FAILED;
	}

	face_coordinates.clear();
	face_coordinates.resize(heads_color_images.size());

	// iterate over head regions
	for(unsigned int head=0; head<heads_color_images.size(); head++)
	{
		// detect faces in color image in proposed region
		IplImage imgPtr = (IplImage)heads_color_images[head];
		CvSeq* faces = cvHaarDetectObjects(&imgPtr,	m_face_cascade,	m_storage, m_faces_increase_search_scale, m_faces_drop_groups, CV_HAAR_DO_CANNY_PRUNING, cvSize(m_faces_min_search_scale_x, m_faces_min_search_scale_y));

		cv::Size parentSize;
		cv::Point roiOffset;
		for(int i=0; i<faces->total; i++)
		{
			cv::Rect* face = (cv::Rect*)cvGetSeqElem(faces, i);
//			heads_color_images[head].locateROI(parentSize, roiOffset);
//			face->x += roiOffset.x;		// todo: check what happens if the original matrix is used without roi
//			face->y += roiOffset.y;
			face_coordinates[head].push_back(*face);
		}
	}
	
	return ipa_Utils::RET_OK;
}
