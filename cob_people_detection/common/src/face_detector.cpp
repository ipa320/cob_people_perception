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

unsigned long FaceDetector::init(std::string directory, double faces_increase_search_scale, int faces_drop_groups, int faces_min_search_scale_x, int faces_min_search_scale_y,
		bool reason_about_3dface_size, double face_size_max_m, double face_size_min_m, double max_face_z_m, bool debug)
{
	// parameters
	m_faces_increase_search_scale = faces_increase_search_scale;
	m_faces_drop_groups = faces_drop_groups;
	m_faces_min_search_scale_x = faces_min_search_scale_x;
	m_faces_min_search_scale_y = faces_min_search_scale_y;
	m_reason_about_3dface_size = reason_about_3dface_size;
	m_face_size_max_m = face_size_max_m;
	m_face_size_min_m = face_size_min_m;
	m_max_face_z_m = max_face_z_m;
	m_debug = debug;

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

unsigned long FaceDetector::detectColorFaces(std::vector<cv::Mat>& heads_color_images, std::vector<cv::Mat>& heads_depth_images, std::vector<std::vector<cv::Rect> >& face_coordinates)
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
	if (m_reason_about_3dface_size==true)
	{
		// check whether the color faces have a reasonable 3D size
		for (uint head_index=0; head_index<face_coordinates.size(); head_index++)
		{
			for (uint face_index = 0; face_index < face_coordinates[head_index].size(); face_index++)
			{
				cv::Rect& face = face_coordinates[head_index][face_index];

				// Get the median disparity in the middle half of the bounding box.
				int uStart = floor(0.25*face.width);
				int uEnd = floor(0.75*face.width) + 1;
				int vStart = floor(0.25*face.height);
				int vEnd = floor(0.75*face.height) + 1;
				int du = abs(uEnd-uStart);

				// generate vector of all depth values in the face region, NaNs are converted to -1
				cv::Mat face_region = heads_depth_images[head_index];
        // EDIT
				cv::Mat tmat(1, du*abs(vEnd-vStart), CV_32FC1);
				float* tmatPtr = (float*)tmat.data;
				for (int v=vStart; v<vEnd; v++)
				{
					float* zPtr = (float*)face_region.row(v).data;
					zPtr += 2+3*uStart;
					for (int u=uStart; u<uEnd; u++)
					{
						float depthval = *zPtr;
						if (!isnan(depthval)) *tmatPtr = depthval;
						else *tmatPtr = -1.0;
						tmatPtr++;
						zPtr += 3;
					}
				}

				// median
				cv::Mat tmat_sorted;
				cv::sort(tmat, tmat_sorted, CV_SORT_EVERY_COLUMN+CV_SORT_DESCENDING);
				double avg_depth = tmat_sorted.at<float>(floor(cv::countNonZero(tmat_sorted>=0.0)*0.5)); // Get the middle valid disparity (-1 disparities are invalid)

				// If the median disparity was valid and the face is a reasonable size, the face status is "good".
				// If the median disparity was valid but the face isn't a reasonable size, the face status is "bad".
				// Otherwise, the face status is "unknown".
				// Only bad faces are removed
				if (avg_depth > 0)
				{
					double radiusX, radiusY, radius3d=1e20;
					cv::Vec3f a, b;
					// vertical line regularly lies completely on the head whereas this does not hold very often for the horizontal line crossing the bounding box of the face
					// rectangle in the middle
					a = heads_depth_images[head_index].at<cv::Vec3f>((int)(face.y+face.height*0.25), (int)(face.x+0.5*face.width));
					b = heads_depth_images[head_index].at<cv::Vec3f>((int)(face.y+face.height*0.75), (int)(face.x+0.5*face.width));
					if (m_debug) std::cout << "a: " << a.val[0] << " " << a.val[1] << " " << a.val[2] << "   b: " << " " << b.val[0] << " " << b.val[1] << " " << b.val[2] << "\n";
					if (isnan(a.val[0]) || isnan(b.val[0])) radiusY = 0.0;
					else radiusY = cv::norm(b-a);
					radius3d = radiusY;

					// for radius estimation with the horizontal line through the face rectangle use points which typically still lie on the face and not in the background
					a = heads_depth_images[head_index].at<cv::Vec3f>((int)(face.y+face.height*0.5), (int)(face.x+face.width*0.25));
					b = heads_depth_images[head_index].at<cv::Vec3f>((int)(face.y+face.height*0.5), (int)(face.x+face.width*0.75));
					if (m_debug) std::cout << "a: " << a.val[0] << " " << a.val[1] << " " << a.val[2] << "   b: " << " " << b.val[0] << " " << b.val[1] << " " << b.val[2] << "\n";
					if (isnan(a.val[0]) || isnan(b.val[0])) radiusX = 0.0;
					else
					{
						radiusX = cv::norm(b-a);
						if (radiusY != 0.0) radius3d = (radiusX+radiusY)*0.5;
						else radius3d = radiusX;
					}

//					cv::Point pup(face.x+0.5*face.width, face.y+face.height*0.25);
//					cv::Point plo(face.x+0.5*face.width, face.y+face.height*0.75);
//					cv::Point ple(face.x+face.width*0.25, face.y+face.height*0.5);
//					cv::Point pri(face.x+face.width*0.75, face.y+face.height*0.5);
//					cv::line(xyz_image_8U3, pup, plo, CV_RGB(255, 255, 255), 2);
//					cv::line(xyz_image_8U3, ple, pri, CV_RGB(255, 255, 255), 2);

					if (m_debug)
					{
						std::cout << "radiusX: " << radiusX << "  radiusY: " << radiusY << "\n";
						std::cout << "avg_depth: " << avg_depth << " > max_face_z_m: " << m_max_face_z_m << " ?  2*radius3d: " << 2.0*radius3d << " < face_size_min_m: " << m_face_size_min_m << " ?  2radius3d: " << 2.0*radius3d << " > face_size_max_m:" << m_face_size_max_m << "?\n";
					}
					if (radius3d > 0.0 && (avg_depth > m_max_face_z_m || 2.0*radius3d < m_face_size_min_m || 2.0*radius3d > m_face_size_max_m))
					{
						// face does not match normal human appearance -> remove from list
						face_coordinates[head_index].erase(face_coordinates[head_index].begin()+face_index);
						face_index--;
					}
				}
			}
//			imshow("xyz image", xyz_image_8U3);
//			cv::waitKey(10);
		}
	}

	return ipa_Utils::RET_OK;
}
