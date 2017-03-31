/*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2017 \n
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
 * \date Date of creation: 31.03.2017
 *
 * \brief
 * some helper functions for publishing face detection messages in cartesian coordinates
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

#pragma once

// ROS includes
#include <ros/ros.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <geometry_msgs/Point.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>

// opencv
#include <opencv2/opencv.hpp>

#include <vector>


class FaceDetectionMessageHelper
{
public:

	void prepareCartesionDetectionMessage(cob_perception_msgs::DetectionArray& detection_msg, const std_msgs::Header& msg_header,
			const std::vector<cv::Mat>& heads_depth_images, const std::vector<cv::Rect>& head_bounding_boxes,
			const std::vector<std::vector<cv::Rect> >& face_bounding_boxes, const std::vector<std::vector<std::string> >* identification_labels=0)
	{
		detection_msg.header = msg_header;

		// prepare message
		for (int head = 0; head < (int)head_bounding_boxes.size(); head++)
		{
			if (face_bounding_boxes[head].size() == 0)
			{
				// no faces detected in head region -> publish head position
				cob_perception_msgs::Detection det;
				const cv::Rect& head_bb = head_bounding_boxes[head];
				// set 3d position of head's center
				bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], 0.5 * (float)head_bb.width, 0.5 * (float)head_bb.height, det.pose.pose.position, 6);
				if (valid_3d_position == false)
					continue;
				det.pose.header = msg_header;
				det.pose.pose.orientation.x = 0.;
				det.pose.pose.orientation.y = 0.;
				det.pose.pose.orientation.z = 0.;
				det.pose.pose.orientation.w = 1.;
				// write bounding box
				det.mask.roi.x = head_bb.x;
				det.mask.roi.y = head_bb.y;
				det.mask.roi.width = head_bb.width;
				det.mask.roi.height = head_bb.height;
				// set label
				det.label = "UnknownHead";
				// set origin of detection
				det.detector = "head";
				// header
				det.header = msg_header;
				// add to message
				detection_msg.detections.push_back(det);
			}
			else
			{
				// process all faces in head region
				for (int face = 0; face < (int)face_bounding_boxes[head].size(); face++)
				{
					cob_perception_msgs::Detection det;
					const cv::Rect& head_bb = head_bounding_boxes[head];
					const cv::Rect& face_bb = face_bounding_boxes[head][face];
					// set 3d position of head's center
					bool valid_3d_position = determine3DFaceCoordinates(heads_depth_images[head], face_bb.x + 0.5 * (float)face_bb.width, face_bb.y + 0.5 * (float)face_bb.height,
							det.pose.pose.position, 6);
					if (valid_3d_position == false)
						continue;
					det.pose.header = msg_header;
					det.pose.pose.orientation.x = 0.;
					det.pose.pose.orientation.y = 0.;
					det.pose.pose.orientation.z = 0.;
					det.pose.pose.orientation.w = 1.;
					// write bounding box
					det.mask.roi.x = head_bb.x + face_bb.x;
					det.mask.roi.y = head_bb.y + face_bb.y;
					det.mask.roi.width = face_bb.width;
					det.mask.roi.height = face_bb.height;
					// set label
					if (identification_labels!=0 && identification_labels->size()>head)
						det.label = (*identification_labels)[head][face];
					else
						det.label = "Unknown";
					// set origin of detection
					det.detector = "face";
					// header
					det.header = msg_header;
					// add to message
					detection_msg.detections.push_back(det);
				}
			}
		}
	}


	bool determine3DFaceCoordinates(const cv::Mat& depth_image, int center2Dx, int center2Dy, geometry_msgs::Point& center3D, int search_radius)
	{
		// 3D world coordinates (and verify that the read pixel contained valid coordinates, otherwise search for valid pixel in neighborhood)
		bool valid_coordinates = false;
		for (int d = 0; (d < search_radius && !valid_coordinates); d++)
		{
			for (int v = -d; (v <= d && !valid_coordinates); v++)
			{
				for (int u = -d; (u <= d && !valid_coordinates); u++)
				{
					if ((abs(v) != d && abs(u) != d) || center2Dx + u < 0 || center2Dx + u >= depth_image.cols || center2Dy + v < 0 || center2Dy + v >= depth_image.rows)
						continue;

					const cv::Point3f& p = depth_image.at<cv::Point3f>(center2Dy + v, center2Dx + u);
					if (!isnan(p.x) && !isnan(p.y) && p.z != 0.f)
					{
						valid_coordinates = true;
						center3D.x = p.x;
						center3D.y = p.y;
						center3D.z = p.z;
					}
				}
			}
		}
		return valid_coordinates;
	}
};
