/****************************************************************
 *
 * Copyright (c) 2010
 *
 * Fraunhofer Institute for Manufacturing Engineering
 * and Automation (IPA)
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Project name: care-o-bot
 * ROS stack name: cob_vision
 * ROS package name: cob_people_detection
 * Description: Takes the positions of detected or recognized faces and
 * a people segmentation of the environment as input and outputs people
 * positions.
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Author: Jan Fischer, email:jan.fischer@ipa.fhg.de
 * Author: Richard Bormann, email: richard.bormann@ipa.fhg.de
 * Supervised by:
 *
 * Date of creation: 03/2011
 * ToDo:
 *
 * +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Fraunhofer Institute for Manufacturing
 *       Engineering and Automation (IPA) nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ****************************************************************/

#ifndef _PEOPLE_DETECTION_
#define _PEOPLE_DETECTION_


//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
//#include <std_msgs/Float32MultiArray.h>
#include <cob_msgs/DetectionArray.h>

// services
#include <cob_people_detection/DetectPeople.h>

// topics
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

// opencv
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// point cloud
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
//#include "boost/filesystem/operations.hpp"
//#include "boost/filesystem/convenience.hpp"
//#include "boost/filesystem/path.hpp"
//namespace fs = boost::filesystem;

// external includes
#include "cob_vision_utils/GlobalDefines.h"


#include <sstream>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>

namespace ipa_PeopleDetector {


//####################
//#### node class ####
class cobPeopleDetectionNodelet : public nodelet::Nodelet
{
protected:
	//message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_;	///< Shared xyz image and color image topic
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter m_peopleSegmentationImageSub;	///< Color camera image topic
	image_transport::SubscriberFilter m_colorImageSub;	///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >* m_syncInput2;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image, sensor_msgs::Image> >* m_syncInput3;
	message_filters::Subscriber<cob_msgs::DetectionArray> m_facePositionSubscriber;		///< receives the face messages from the face detector
	ros::Publisher m_facePositionPublisher;		///< publisher for the positions of the detected faces
	image_transport::Publisher people_detection_image_pub_;	///< topic for publishing the image containing the people positions
	ros::ServiceServer m_serviceServerDetectPeople; ///< Service server to request people detection


	ros::NodeHandle m_nodeHandle;				///< ROS node handle

	std::vector<cob_msgs::Detection> m_facePositionAccumulator;	///< accumulates face positions over time
	boost::timed_mutex m_facePositionAccumulatorMutex;			///< secures write and read operations to m_facePositionAccumulator

	// parameters
	bool m_display;								///< if on, several debug outputs are activated
	bool m_usePeopleSegmentation;				///< enables the combination of face detections with the openni people segmentation
	double m_faceRedetectionTime;				///< timespan during which a face is preserved in the list of tracked faces although it is currently not visible
	double m_minSegmentedPeopleRatioColor;		///< the minimum area inside the face rectangle found in the color image that has to be covered with positive people segmentation results (from openni_tracker)
	double m_minSegmentedPeopleRatioRange;		///< the minimum area inside the face rectangle found in the range image that has to be covered with positive people segmentation results (from openni_tracker)
	double m_trackingRangeM;					///< maximum tracking manhattan distance for a face (in meters), i.e. a face can move this distance between two images and can still be tracked

public:

	cobPeopleDetectionNodelet()
	{
		it_ = 0;
		m_syncInput2 = 0;
		m_syncInput3 = 0;
	}

	~cobPeopleDetectionNodelet()
	{
		if (it_ != 0) delete it_;
		if (m_syncInput2 != 0) delete m_syncInput2;
		if (m_syncInput3 != 0) delete m_syncInput3;
	}

	/// Nodelet init function
	void onInit()
	{
		m_nodeHandle = getNodeHandle();
		ros::NodeHandle local_nh = getPrivateNodeHandle();

		// parameters
		local_nh.param("display", m_display, true);
		local_nh.param("face_redetection_time", m_faceRedetectionTime, 2.0);
		local_nh.param("min_segmented_people_ratio_color", m_minSegmentedPeopleRatioColor, 0.7);
		local_nh.param("min_segmented_people_ratio_range", m_minSegmentedPeopleRatioRange, 0.2);
		local_nh.param("use_people_segmentation", m_usePeopleSegmentation, true);
		local_nh.param("tracking_range_m", m_trackingRangeM, 0.3);

		it_ = new image_transport::ImageTransport(m_nodeHandle);
		m_peopleSegmentationImageSub.subscribe(*it_, "people_segmentation_image", 1);
		if (m_display==true) m_colorImageSub.subscribe(*it_, "colorimage", 1);
		m_facePositionSubscriber.subscribe(m_nodeHandle, "face_position_array_in", 1);

		sensor_msgs::Image::ConstPtr nullPtr;
		if (m_usePeopleSegmentation == true)
		{
			if (m_display == false)
			{
				m_syncInput2 = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >(2);
				m_syncInput2->connectInput(m_facePositionSubscriber, m_peopleSegmentationImageSub);
				m_syncInput2->registerCallback(boost::bind(&cobPeopleDetectionNodelet::inputCallback, this, _1, _2, nullPtr));
			}
			else
			{
				m_syncInput3 = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image, sensor_msgs::Image> >(3);
				m_syncInput3->connectInput(m_facePositionSubscriber, m_peopleSegmentationImageSub, m_colorImageSub);
				m_syncInput3->registerCallback(boost::bind(&cobPeopleDetectionNodelet::inputCallback, this, _1, _2, _3));
			}
		}
		else
		{
			if (m_display == true)
			{
				m_syncInput2 = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >(2);
				m_syncInput2->connectInput(m_facePositionSubscriber, m_colorImageSub);
				m_syncInput2->registerCallback(boost::bind(&cobPeopleDetectionNodelet::inputCallback, this, _1, nullPtr, _2));
			}
			else
			{
				m_facePositionSubscriber.registerCallback(boost::bind(&cobPeopleDetectionNodelet::inputCallback, this, _1, nullPtr, nullPtr));
			}
		}

		m_serviceServerDetectPeople = m_nodeHandle.advertiseService("detect_people", &cobPeopleDetectionNodelet::detectPeopleCallback, this);

		m_facePositionPublisher = m_nodeHandle.advertise<cob_msgs::DetectionArray>("face_position_array", 1);

		people_detection_image_pub_ = it_->advertise("people_detection_image", 1);

		std::cout << "cobPeopleDetectionNodelet initialized.\n";

		//ros::spin(); not necessary with nodelets
	}

	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image)
	{
		try
		{
		  image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
		  ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		  return ipa_Utils::RET_FAILED;
		}
		image = image_ptr->image;

		return ipa_Utils::RET_OK;
	}

	/// copies the data from src to dest
	/// @param src The new data which shall be copied into dest
	/// @param update If update is true, dest must contain the data which shall be updated
	unsigned long copyDetection(const cob_msgs::Detection& src, cob_msgs::Detection& dest, bool update=false)
	{
		// 2D image coordinates
		dest.mask.roi.x = src.mask.roi.x;           dest.mask.roi.y = src.mask.roi.y;
		dest.mask.roi.width = src.mask.roi.width;   dest.mask.roi.height = src.mask.roi.height;

		// 3D world coordinates
		const geometry_msgs::Point* pointIn = &(src.pose.pose.position);
		geometry_msgs::Point* pointOut = &(dest.pose.pose.position);
		pointOut->x = pointIn->x;  pointOut->y = pointIn->y;  pointOut->z = pointIn->z;

		// person ID
		if (update==true)
		{
			if (src.label!="No face")
			{
				if (src.label=="Unknown")
				{
					if (dest.label=="No face") dest.label = "Unknown";
				}
				else if (dest.label=="No face" || dest.label=="Unknown") dest.label = src.label;
			}
		}
		else dest.label = src.label;

		// time stamp, detector (color or range)
		if (update==true)
		{
			if (src.detector=="color") dest.detector = "color";
		}
		else dest.detector = src.detector;

		dest.header.stamp = ros::Time::now();

		return ipa_Utils::RET_OK;
	}

	template<typename T> inline T abs(T x) { return ((x<0) ? -x : x); }
	bool faceInList(const cob_msgs::Detection& detIn, int imageHeight, int imageWidth)
	{
		if (m_facePositionAccumulator.size() == 0) return false;

		// find the closest face found previously, if it is close enough, assign the new detection to it
		double minDistance = 1e50;
		size_t minIndex = 0;
		const geometry_msgs::Point* pointIn = &(detIn.pose.pose.position);
		for (int i=0; i<(int)m_facePositionAccumulator.size(); i++)
		{
			geometry_msgs::Point* point = &(m_facePositionAccumulator[i].pose.pose.position);
			double dx = abs(pointIn->x - point->x);
			double dy = abs(pointIn->y - point->y);
			double dz = abs(pointIn->z - point->z);
			double dist = dx*dx+dy*dy+dz*dz;

			if (dist < minDistance)
			{
				minDistance = dist;
				minIndex = i;
			}
		}
		// close enough?
		geometry_msgs::Point* point = &(m_facePositionAccumulator[minIndex].pose.pose.position);
		double dx = abs(pointIn->x - point->x);
		double dy = abs(pointIn->y - point->y);
		double dz = abs(pointIn->z - point->z);
		if (dx<m_trackingRangeM || dy<m_trackingRangeM || dz<m_trackingRangeM)
		{
			// close enough, update old face
			copyDetection(detIn, m_facePositionAccumulator[minIndex], true);
			return true;
		}

//		double du = abs(detIn.mask.roi.x - m_facePositionAccumulator[i].mask.roi.x)/imageWidth;
//		double dv = abs(detIn.mask.roi.y - m_facePositionAccumulator[i].mask.roi.y)/imageHeight;
//		double dwidth = abs(detIn.mask.roi.width - m_facePositionAccumulator[i].mask.roi.width)/imageWidth;
//		double dheight = abs(detIn.mask.roi.height - m_facePositionAccumulator[i].mask.roi.height)/imageHeight;

		// face was not found in the list
		return false;
	}

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_msgs::DetectionArray::ConstPtr& face_position_msg, const sensor_msgs::Image::ConstPtr& people_segmentation_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
	{
		// convert segmentation image to cv::Mat
		cv_bridge::CvImageConstPtr people_segmentation_image_ptr;
		cv::Mat people_segmentation_image;
		if (m_usePeopleSegmentation == true) convertColorImageMessageToMat(people_segmentation_image_msg, people_segmentation_image_ptr, people_segmentation_image);

		if (m_display) std::cout << "detections: " << face_position_msg->detections.size() << "\n";

		// source image size
		std::stringstream ss;
		ss << face_position_msg->header.frame_id;
		int width, height;
		ss >> width;
		ss >> height;

		// delete old face positions in list
		ros::Duration timeSpan(m_faceRedetectionTime);
		for (int i=0; i<(int)m_facePositionAccumulator.size(); i++)
		{
			if ((ros::Time::now()-m_facePositionAccumulator[i].header.stamp) > timeSpan) m_facePositionAccumulator.erase(m_facePositionAccumulator.begin()+i);
		}

		// secure this section with a mutex
		boost::timed_mutex::scoped_timed_lock lock(m_facePositionAccumulatorMutex, boost::posix_time::milliseconds(2000));
		if (lock.owns_lock() == false)
		{
			ROS_ERROR("cob_people_detection::cobPeopleDetectionNodelet: Mutex was not freed during response time.");
			return;
		}

		// publish face positions
		cob_msgs::DetectionArray facePositionMsg;
		for(int i=0; i<(int)face_position_msg->detections.size(); i++)
		{
			const cob_msgs::Detection* const detIn = &(face_position_msg->detections[i]);
			cv::Rect face;
			face.x = detIn->mask.roi.x;
			face.y = detIn->mask.roi.y;
			face.width = detIn->mask.roi.width;
			face.height = detIn->mask.roi.height;

			// check with people segmentation
			if (m_usePeopleSegmentation == true)
			{
				int numberBlackPixels = 0;
				//std::cout << "face: " << face.x << " " << face.y << " " << face.width << " " << face.height << "\n";
				for (int v=face.y; v<face.y+face.height; v++)
				{
					uchar* data = people_segmentation_image.ptr(v);
					for (int u=face.x; u<face.x+face.width; u++)
					{
						int index = 3*u;
						if (data[index]==0 && data[index+1]==0 && data[index+2]==0) numberBlackPixels++;
					}
				}
				int faceArea = face.height * face.width;
				double segmentedPeopleRatio = (double)(faceArea-numberBlackPixels)/(double)faceArea;

				//std::cout << "ratio: " << segmentedPeopleRatio << "\n";

				if ((detIn->detector=="color" && segmentedPeopleRatio < m_minSegmentedPeopleRatioColor) || (detIn->detector=="range" && segmentedPeopleRatio < m_minSegmentedPeopleRatioRange))
				{
					// False detection
					std::cout << "False detection\n";
					continue;
				}
			}

			// valid face detection
			std::cout << "Face detection\n";
			// check whether this face was found before and if it is new, whether it is a color face detection (range detection is not sufficient for a new face)
			if (faceInList(*detIn, height, width)==false && detIn->detector=="color")
			{
				std::cout << "\n***** New detection *****\n\n";
				cob_msgs::Detection detOut;
				copyDetection(*detIn, detOut, false);
				detOut.pose.header.frame_id = "head_tof_link";
				m_facePositionAccumulator.push_back(detOut);
			}
		}
		facePositionMsg.detections = m_facePositionAccumulator;
		facePositionMsg.header.stamp = ros::Time::now();
		m_facePositionPublisher.publish(facePositionMsg);

		if (m_display == true)
		{
			// convert image message to cv::Mat
			cv_bridge::CvImageConstPtr colorImagePtr;
			cv::Mat colorImage;
			convertColorImageMessageToMat(color_image_msg, colorImagePtr, colorImage);


			// display segmentation
			//if (m_usePeopleSegmentation == true)
			//{
				//cv::namedWindow("People Segmentation Image");
				//imshow("People Segmentation Image", people_segmentation_image);
			//}

			// display color image
			cv::namedWindow("People Detector and Tracker");
			for(int i=0; i<(int)m_facePositionAccumulator.size(); i++)
			{
				cv::Rect face;
				cob_msgs::Rect& faceRect = m_facePositionAccumulator[i].mask.roi;
				face.x = faceRect.x;    face.width = faceRect.width;
				face.y = faceRect.y;    face.height = faceRect.height;

				if (m_facePositionAccumulator[i].detector == "range")
					cv::rectangle(colorImage, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
				else
					cv::rectangle(colorImage, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

				if (m_facePositionAccumulator[i].label == "Unknown")
					// Distance to face class is too high
					cv::putText(colorImage, "Unknown", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				else if (m_facePositionAccumulator[i].label ==  "No face")
					// Distance to face space is too high
					cv::putText(colorImage, "No face", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				else
					// Face classified
					cv::putText(colorImage, m_facePositionAccumulator[i].label.c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
			}
			//cv::imshow("People Detector and Tracker", colorImage);
			//cv::waitKey(10);
			// publish image
			cv_bridge::CvImage cv_ptr;
			cv_ptr.image = colorImage;
			cv_ptr.encoding = "bgr8";
			people_detection_image_pub_.publish(cv_ptr.toImageMsg());
		}
	}

	bool detectPeopleCallback(cob_people_detection::DetectPeople::Request &req, cob_people_detection::DetectPeople::Response &res)
	{
		// secure this section with a mutex
		boost::timed_mutex::scoped_timed_lock lock(m_facePositionAccumulatorMutex, boost::posix_time::milliseconds(2000));
		if (lock.owns_lock() == false)
		{
			ROS_ERROR("cob_people_detection::cobPeopleDetectionNodelet: Mutex was not freed during response time.");
			return false;
		}
		res.people_list.detections = m_facePositionAccumulator;
		res.people_list.header.stamp = ros::Time::now();
		return true;
	}
};

};

 PLUGINLIB_DECLARE_CLASS(ipa_PeopleDetector, cobPeopleDetectionNodelet, ipa_PeopleDetector::cobPeopleDetectionNodelet, nodelet::Nodelet);

#endif // _PEOPLE_DETECTION_
