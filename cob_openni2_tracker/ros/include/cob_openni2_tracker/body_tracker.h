/*
 *****************************************************************
 * Copyright (c) 2015 \n
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
 * ROS package name: cob_openni2_tracker
 *
 * \author
 * Author: Olha Meyer
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 01.11.2014
 *
 * \brief
 * functions for detecting people within a depth image
 * current approach: read the current video flow and detect people using OpenNI and NiTE2
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

 /*
  * Other Copyrights :
  *
 ********************************************************
 *                                                        *
 *   PrimeSense NiTE 2.0 - User Viewer Sample             *
 *   Copyright (C) 2012 PrimeSense Ltd.                   *
 *   (For a license distribution please see               *
 *    the package information):                           *
 **********************************************************/
#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

//ROS includes
#include <libnite2/NiTE.h>
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
//point cloud includes
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
//Messages
#include <visualization_msgs/Marker.h>
#include <cob_perception_msgs/Skeleton.h>
#include <cob_perception_msgs/People.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
//image and opencv
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
//boost
#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>
//mathes
#include <vector>

#define MAX_DEPTH 10000
typedef std::map<std::string, nite::SkeletonJoint>JointMap;

class BodyTracker
{

public:

	BodyTracker(ros::NodeHandle nh);
	virtual ~BodyTracker();
	//PARAMS
	std::string tf_prefix_, rel_frame_, depth_optical_frame_;
	openni::Device	device_;
	openni::VideoStream depthSensor_;
	void finalize();
	void runTracker();

	bool shutdown_;

private:

	//ROS ELEMENTS
	ros::NodeHandle nh_;

	//TF ELEMENTS
	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster transform_broadcaster_;
	ros::Publisher vis_pub_, pcl_pub_, skeleton_pub_, people_pub_;
	ros::Subscriber pcl_sub_;
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter image_sub_;
	//image_transport::SubscriberFilter pcl_sub_;
	image_transport::Publisher image_pub_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
	void imgConnectCB(const image_transport::SingleSubscriberPublisher& pub);
	void imgDisconnectCB(const image_transport::SingleSubscriberPublisher& pub);

	std::list<nite::UserData>* tracked_users_;

	float m_pDepthHist[MAX_DEPTH];
	char m_strSampleName[ONI_MAX_STR];

	unsigned int m_nTexMapX;
	unsigned int m_nTexMapY;

	nite::UserTracker* m_pUserTracker;
	nite::UserId m_poseUser;

	openni::RGB888Pixel* m_pTexMap_;

	uint64_t m_poseTime_;
	unsigned int marker_id_;
	int poseTimeoutToExit_;

	bool drawSkeleton_;
	bool drawCenterOfMass_;
	bool drawUserName_;
	bool drawBoundingBox_;
	bool drawBackground_;
	bool drawDepth_;
	bool drawFrames_;

	int init_counter_color_image_;
	int init_counter_point_cloud_;

	BodyTracker(const BodyTracker&);
	BodyTracker& operator=(BodyTracker&);

	void init();
	void imageCallback(const sensor_msgs::ImageConstPtr& rgb_image_msg);
	void pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud);
	void updateUserState(const nite::UserData& user, uint64_t ts);
	void publishJoints(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string joint_name,
			nite::SkeletonJoint joint, std::string tf_prefix, std::string rel_frame, int id);
	void publishTrackedUserMsg();
	void drawLine(const double r, const double g, const double b,
			const nite::Point3f& pose_start, const nite::Point3f& pose_end );
	void drawCircle(const double r, const double g, const double b,
			const nite::Point3f& pose);
	void drawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData);
	void drawUserName(const nite::UserData& user, cv::Mat& color_image, cv::Point& tag_coords);
	void drawFrames(const nite::UserData& user);
	void drawPointCloud();
	void drawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color);
	void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame);
	void convertColorImageMsgToMat(const sensor_msgs::Image::ConstPtr& color_image_msg,
			cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& color_image_msg,
			cv_bridge::CvImageConstPtr& color_image_ptr, cv::Mat& color_image);
	geometry_msgs::Pose convertNiteJointToMsgs(nite::SkeletonJoint joint);

};

#endif // _NITE_USER_VIEWER_H_
