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



#ifdef __LINUX__
	#include "cob_people_detection/head_detector_node.h"
	#include "cob_vision_utils/GlobalDefines.h"
#else
#endif

//#include

using namespace ipa_PeopleDetector;

HeadDetectorNode::HeadDetectorNode(ros::NodeHandle nh)
: node_handle_(nh)
{
	// todo read from parameter
	std::string model_directory = "";
	head_detector_.init(model_directory);
}

HeadDetectorNode::~HeadDetectorNode(void)
{
}

//HeadDetectorNode::cc()
//{
//	convertPclMessageToMat(shared_image_msg, depth_image);
//}

//unsigned long HeadDetectorNode::convertPclMessageToMat(const sensor_msgs::PointCloud2::ConstPtr& shared_image_msg, cv::Mat& depth_image)
//{
//	pcl::PointCloud<pcl::PointXYZ> depth_cloud; // point cloud
//	pcl::fromROSMsg(*shared_image_msg, depth_cloud);
//	depth_image.create(depth_cloud.height, depth_cloud.width, CV_32FC3);
//	uchar* depth_image_ptr = (uchar*) depth_image.data;
//	for (int v=0; v<(int)depth_cloud.height; v++)
//	{
//		int baseIndex = depth_image.step*v;
//		for (int u=0; u<(int)depth_cloud.width; u++)
//		{
//			int index = baseIndex + 3*u*sizeof(float);
//			float* data_ptr = (float*)(depth_image_ptr+index);
//			pcl::PointXYZ point_xyz = depth_cloud(u,v);
//			data_ptr[0] = point_xyz.x;
//			data_ptr[1] = point_xyz.y;
//			data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
//			//if (u%100 == 0) std::cout << "u" << u << " v" << v << " z" << data_ptr[2] << "\n";
//		}
//	}
//	return ipa_Utils::RET_OK;
//}


//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "head_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create HeadDetectorNode class instance
	HeadDetectorNode head_detector_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
