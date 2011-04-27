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

// external includes
#include "cob_vision_utils/GlobalDefines.h"


#include <sstream>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.h>

namespace ipa_PeopleDetector {


//####################
//#### node class ####
class CobPeopleDetectionNodelet : public nodelet::Nodelet
{
protected:
	//message_filters::Subscriber<sensor_msgs::PointCloud2> shared_image_sub_;	///< Shared xyz image and color image topic
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter people_segmentation_image_sub_;	///< Color camera image topic
	image_transport::SubscriberFilter color_image_sub_;	///< Color camera image topic
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >* sync_input_2_;
	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image, sensor_msgs::Image> >* sync_input_3_;
	message_filters::Subscriber<cob_msgs::DetectionArray> face_position_subscriber_;		///< receives the face messages from the face detector
	ros::Publisher face_position_publisher_;		///< publisher for the positions of the detected faces
	image_transport::Publisher people_detection_image_pub_;	///< topic for publishing the image containing the people positions
	ros::ServiceServer service_server_detect_people_; ///< Service server to request people detection


	ros::NodeHandle node_handle_;				///< ROS node handle

	std::vector<cob_msgs::Detection> face_position_accumulator_;	///< accumulates face positions over time
	boost::timed_mutex face_position_accumulator_mutex_;			///< secures write and read operations to face_position_accumulator_
	std::vector<std::map<std::string, double> > face_identification_votes_;	///< collects votes for all names ever assigned to each detection in face_position_accumulator_

	// parameters
	bool display_;								///< if on, several debug outputs are activated
	bool use_people_segmentation_;				///< enables the combination of face detections with the openni people segmentation
	double face_redetection_time_;				///< timespan during which a face is preserved in the list of tracked faces although it is currently not visible
	double min_segmented_people_ratio_color_;		///< the minimum area inside the face rectangle found in the color image that has to be covered with positive people segmentation results (from openni_tracker)
	double min_segmented_people_ratio_range_;		///< the minimum area inside the face rectangle found in the range image that has to be covered with positive people segmentation results (from openni_tracker)
	double tracking_range_m_;					///< maximum tracking manhattan distance for a face (in meters), i.e. a face can move this distance between two images and can still be tracked

public:

	CobPeopleDetectionNodelet()
	{
		it_ = 0;
		sync_input_2_ = 0;
		sync_input_3_ = 0;
	}

	~CobPeopleDetectionNodelet()
	{
		if (it_ != 0) delete it_;
		if (sync_input_2_ != 0) delete sync_input_2_;
		if (sync_input_3_ != 0) delete sync_input_3_;
	}

	/// Nodelet init function
	void onInit()
	{
		node_handle_ = getNodeHandle();

		// parameters
		std::cout << "\n---------------------------\nPeople Detection Parameters:\n---------------------------\n";
		node_handle_.param("display", display_, true);
		std::cout << "display = " << display_ << "\n";
		node_handle_.param("face_redetection_time", face_redetection_time_, 2.0);
		std::cout << "face_redetection_time = " << face_redetection_time_ << "\n";
		node_handle_.param("min_segmented_people_ratio_color", min_segmented_people_ratio_color_, 0.7);
		std::cout << "min_segmented_people_ratio_color = " << min_segmented_people_ratio_color_ << "\n";
		node_handle_.param("min_segmented_people_ratio_range", min_segmented_people_ratio_range_, 0.2);
		std::cout << "min_segmented_people_ratio_range = " << min_segmented_people_ratio_range_ << "\n";
		node_handle_.param("use_people_segmentation", use_people_segmentation_, true);
		std::cout << "use_people_segmentation = " << use_people_segmentation_ << "\n";
		node_handle_.param("tracking_range_m", tracking_range_m_, 0.3);
		std::cout << "tracking_range_m = " << tracking_range_m_ << "\n";

		// subscribers
		it_ = new image_transport::ImageTransport(node_handle_);
		people_segmentation_image_sub_.subscribe(*it_, "people_segmentation_image", 1);
		if (display_==true) color_image_sub_.subscribe(*it_, "colorimage", 1);
		face_position_subscriber_.subscribe(node_handle_, "face_position_array_in", 1);

		// input synchronization
		sensor_msgs::Image::ConstPtr nullPtr;
		if (use_people_segmentation_ == true)
		{
			if (display_ == false)
			{
				sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >(2);
				sync_input_2_->connectInput(face_position_subscriber_, people_segmentation_image_sub_);
				sync_input_2_->registerCallback(boost::bind(&CobPeopleDetectionNodelet::inputCallback, this, _1, _2, nullPtr));
			}
			else
			{
				sync_input_3_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image, sensor_msgs::Image> >(3);
				sync_input_3_->connectInput(face_position_subscriber_, people_segmentation_image_sub_, color_image_sub_);
				sync_input_3_->registerCallback(boost::bind(&CobPeopleDetectionNodelet::inputCallback, this, _1, _2, _3));
			}
		}
		else
		{
			if (display_ == true)
			{
				sync_input_2_ = new message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_msgs::DetectionArray, sensor_msgs::Image> >(2);
				sync_input_2_->connectInput(face_position_subscriber_, color_image_sub_);
				sync_input_2_->registerCallback(boost::bind(&CobPeopleDetectionNodelet::inputCallback, this, _1, nullPtr, _2));
			}
			else
			{
				face_position_subscriber_.registerCallback(boost::bind(&CobPeopleDetectionNodelet::inputCallback, this, _1, nullPtr, nullPtr));
			}
		}

		// services
		service_server_detect_people_ = node_handle_.advertiseService("detect_people", &CobPeopleDetectionNodelet::detectPeopleCallback, this);

		// publishers
		face_position_publisher_ = node_handle_.advertise<cob_msgs::DetectionArray>("face_position_array", 1);
		people_detection_image_pub_ = it_->advertise("people_detection_image", 1);

		std::cout << "CobPeopleDetectionNodelet initialized.\n";

		//ros::spin(); not necessary with nodelets
	}


	/// Converts a color image message to cv::Mat format.
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


	/// Copies the data from src to dest.
	/// @param src The new data which shall be copied into dest
	/// @param dst The new data src is copied into dest
	/// @param update If update is true, dest must contain the data which shall be updated
	/// @param updateIndex The index in face_identification_votes_ corresponding to the previous detection dest. Only necessary if update is true.
	/// @return Return code.
	unsigned long copyDetection(const cob_msgs::Detection& src, cob_msgs::Detection& dest, bool update=false, unsigned int updateIndex=UINT_MAX)
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
//			if (src.label!="No face")
//			{
//				if (src.label=="Unknown")
//				{
//					if (dest.label=="No face") dest.label = "Unknown";
//				}
//				else if (dest.label=="No face" || dest.label=="Unknown") dest.label = src.label;
//			}
			// update label history
			if (src.label!="No face" && src.label!="Unknown")
			{
				if (face_identification_votes_[updateIndex].find(src.label) == face_identification_votes_[updateIndex].end())
					face_identification_votes_[updateIndex][src.label] = 1.0;
				else
					face_identification_votes_[updateIndex][src.label] += 1.0;
			}

			// apply voting decay with time and find most voted label
			unsigned int maxCount = 0;
			dest.label = "Error";
			for (std::map<std::string, double>::iterator face_identification_votes_it=face_identification_votes_[updateIndex].begin(); face_identification_votes_it!=face_identification_votes_[updateIndex].end(); face_identification_votes_it++)
			{
				face_identification_votes_it->second *= 0.9;		// todo: parameter
				if (face_identification_votes_it->second > maxCount)
				{
					maxCount = face_identification_votes_it->second;
					dest.label = face_identification_votes_it->first;
				}
			}
		}
		else dest.label = src.label;

		// time stamp, detector (color or range)
//		if (update==true)
//		{
//			if (src.detector=="color") dest.detector = "color";
//		}
//		else dest.detector = src.detector;
		dest.detector = src.detector;

		dest.header.stamp = ros::Time::now();

		return ipa_Utils::RET_OK;
	}

<<<<<<< HEAD
	template<typename T> inline T abs(T x) { return ((x<0) ? -x : x); }
	bool faceInList(const cob_msgs::Detection& detIn, int imageHeight, int imageWidth)
=======
	inline double abs(double x) { return ((x<0) ? -x : x); }


	/// Computes the Euclidian distance of a recent faces detection to a current face detection.
	/// If the current face detection is outside the neighborhood of the previous detection, DBL_MAX is returned.
	/// @return The Euclidian distance of both faces or DBL_MAX.
	double computeFacePositionDistance(const cob_msgs::Detection& previous_detection, const cob_msgs::Detection& current_detection)
>>>>>>> f033bbab5a7b46c77ea716824eb2205edfee133f
	{
		const geometry_msgs::Point* point_1 = &(previous_detection.pose.pose.position);
		const geometry_msgs::Point* point_2 = &(current_detection.pose.pose.position);

		double dx = abs(point_1->x - point_2->x);
		double dy = abs(point_1->y - point_2->y);
		double dz = abs(point_1->z - point_2->z);

		// return a huge distance if the current face position is too far away from the recent
		if (dx>tracking_range_m_ || dy>tracking_range_m_ || dz>tracking_range_m_)
		{
			// new face position is too distant to recent position
			return DBL_MAX;
		}

		// return Euclidian distance if both faces are sufficiently close
		double dist = dx*dx+dy*dy+dz*dz;
		return dist;
	}


//	///
//	bool faceInList(const cob_msgs::Detection& det_in) //, int image_height, int image_width)
//	{
//		if (face_position_accumulator_.size() == 0) return false;
//
//		// find the closest face found previously, if it is close enough, assign the new detection to it
//		double minDistance = 1e50;
//		size_t minIndex = 0;
//		const geometry_msgs::Point* pointIn = &(det_in.pose.pose.position);
//		for (int i=0; i<(int)face_position_accumulator_.size(); i++)
//		{
//			geometry_msgs::Point* point = &(face_position_accumulator_[i].pose.pose.position);
//			double dx = abs(pointIn->x - point->x);
//			double dy = abs(pointIn->y - point->y);
//			double dz = abs(pointIn->z - point->z);
//			double dist = dx*dx+dy*dy+dz*dz;
//
//			if (dist < minDistance)
//			{
//				minDistance = dist;
//				minIndex = i;
//			}
//		}
//		// close enough?
//		geometry_msgs::Point* point = &(face_position_accumulator_[minIndex].pose.pose.position);
//		double dx = abs(pointIn->x - point->x);
//		double dy = abs(pointIn->y - point->y);
//		double dz = abs(pointIn->z - point->z);
//		if (dx>tracking_range_m_ || dy>tracking_range_m_ || dz>tracking_range_m_)
//		{
//			// face was not found in the list
//			return false;
//		}
//
//		// close enough, update old face
//		copyDetection(det_in, face_position_accumulator_[minIndex], true);
//		return true;
//	}

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_msgs::DetectionArray::ConstPtr& face_position_msg_in, const sensor_msgs::Image::ConstPtr& people_segmentation_image_msg, const sensor_msgs::Image::ConstPtr& color_image_msg)
	{
		// convert segmentation image to cv::Mat
		cv_bridge::CvImageConstPtr people_segmentation_image_ptr;
		cv::Mat people_segmentation_image;
		if (use_people_segmentation_ == true) convertColorImageMessageToMat(people_segmentation_image_msg, people_segmentation_image_ptr, people_segmentation_image);

		if (display_) std::cout << "detections: " << face_position_msg_in->detections.size() << "\n";

		// source image size
		std::stringstream ss;
		ss << face_position_msg_in->header.frame_id;
		int width, height;
		ss >> width;
		ss >> height;

		// delete old face positions in list
		ros::Duration timeSpan(face_redetection_time_);
		for (int i=0; i<(int)face_position_accumulator_.size(); i++)
		{
			if ((ros::Time::now()-face_position_accumulator_[i].header.stamp) > timeSpan)
			{
				face_position_accumulator_.erase(face_position_accumulator_.begin()+i);
				face_identification_votes_.erase(face_identification_votes_.begin()+i);
			}
		}

		// secure this section with a mutex
		boost::timed_mutex::scoped_timed_lock lock(face_position_accumulator_mutex_, boost::posix_time::milliseconds(2000));
		if (lock.owns_lock() == false)
		{
			ROS_ERROR("cob_people_detection::CobPeopleDetectionNodelet: Mutex was not freed during response time.");
			return;
		}

		// verify face detections with people segmentation if enabled -> only accept detected faces if a person is segmented at that position
		std::vector<int> face_detection_indices;	// index set for face_position_msg_in: contains those indices of detected faces which are supported by a person segmentation at the same location
		if (use_people_segmentation_ == true)
		{
			for(int i=0; i<(int)face_position_msg_in->detections.size(); i++)
			{
				const cob_msgs::Detection* const det_in = &(face_position_msg_in->detections[i]);
				cv::Rect face;
				face.x = det_in->mask.roi.x;
				face.y = det_in->mask.roi.y;
				face.width = det_in->mask.roi.width;
				face.height = det_in->mask.roi.height;

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

				// if (display_) std::cout << "ratio: " << segmentedPeopleRatio << "\n";
				if ((det_in->detector=="color" && segmentedPeopleRatio < min_segmented_people_ratio_color_) || (det_in->detector=="range" && segmentedPeopleRatio < min_segmented_people_ratio_range_))
				{
					// False detection
					if (display_) std::cout << "False detection\n";
				}
				else
				{
					// Push index of this detection into the list
					face_detection_indices.push_back(i);
				}
			}
		}
		else
		{
			for (unsigned int i=0; i<face_position_msg_in->detections.size(); i++)
				face_detection_indices.push_back(i);
		}

		// match current detections with previous detections
		// build distance matrix
		std::map<int, std::map<int, double> > distance_matrix;	// 1. index = face_position_accumulator_ index of previous detections, 2. index = index of current detections, content = spatial distance between the indexed faces
		std::map<int, std::map<int, double> >::iterator distance_matrix_it;
		for (unsigned int previous_det=0; previous_det<face_position_accumulator_.size(); previous_det++)
		{
			std::map<int, double> distance_row;
			for (unsigned int i=0; i<face_detection_indices.size(); i++)
				distance_row[face_detection_indices[i]] = computeFacePositionDistance(face_position_accumulator_[previous_det], face_position_msg_in->detections[face_detection_indices[i]]);
			distance_matrix[previous_det] = distance_row;
		}

		// find matching faces between previous and current detections
		unsigned int number_matchings = (face_position_accumulator_.size() < face_detection_indices.size()) ? face_position_accumulator_.size() : face_detection_indices.size();
		std::vector<bool> current_detection_has_matching(face_detection_indices.size(), false);		// index set for face_detection_indices: contains the indices of face_detection_indices and whether these faces could be matched to previously found faces
		for (unsigned int m=0; m<number_matchings; m++)
		{
			// find minimum matching distance between any two elements of the distance_matrix
			double min_dist = DBL_MAX;
			int previous_min_index, current_min_index;
			for (distance_matrix_it=distance_matrix.begin(); distance_matrix_it!=distance_matrix.end(); distance_matrix_it++)
			{
				for (std::map<int, double>::iterator distance_row_it=distance_matrix_it->second.begin(); distance_row_it!=distance_matrix_it->second.end(); distance_row_it++)
				{
					if (distance_row_it->second < min_dist)
					{
						min_dist = distance_row_it->second;
						previous_min_index = distance_matrix_it->first;
						current_min_index = distance_row_it->first;
					}
				}
			}

			// if there is no matching pair of detections interrupt the search for matchings at this point
			if (min_dist == DBL_MAX) break;

			// instantiate the matching
			copyDetection(face_position_msg_in->detections[current_min_index], face_position_accumulator_[previous_min_index], true, previous_min_index);
			// mark the respective entry in face_detection_indices as labeled
			for (unsigned int i=0; i<face_detection_indices.size(); i++)
				if (face_detection_indices[i] == current_min_index)
					current_detection_has_matching[i] = true;

			// delete the row and column of the found matching so that both detections cannot be involved in any further matching again
			distance_matrix.erase(previous_min_index);
			for (distance_matrix_it=distance_matrix.begin(); distance_matrix_it!=distance_matrix.end(); distance_matrix_it++)
				distance_matrix_it->second.erase(current_min_index);
		}

		// create new detections for the unmatched of the current detections
		for (unsigned int i=0; i<face_detection_indices.size(); i++)
		{
			if (current_detection_has_matching[i] == false)
			{
				const cob_msgs::Detection* const det_in = &(face_position_msg_in->detections[face_detection_indices[i]]);
				if (det_in->detector=="color")
				{
					// save in accumulator
					if (display_) std::cout << "\n***** New detection *****\n\n";
					cob_msgs::Detection det_out;
					copyDetection(*det_in, det_out, false);
					det_out.pose.header.frame_id = "head_tof_link";
					face_position_accumulator_.push_back(det_out);
					// remember label history
					std::map<std::string, double> new_identification_data;
					if (det_in->label!="No face" && det_in->label!="Unknown")
						new_identification_data[det_in->label] = 1.0;
					face_identification_votes_.push_back(new_identification_data);
				}
			}
		}

		// publish face positions
		cob_msgs::DetectionArray face_position_msg_out;
		face_position_msg_out.detections = face_position_accumulator_;
		face_position_msg_out.header.stamp = ros::Time::now();
		face_position_publisher_.publish(face_position_msg_out);

		// display
		if (display_ == true)
		{
			// convert image message to cv::Mat
			cv_bridge::CvImageConstPtr colorImagePtr;
			cv::Mat colorImage;
			convertColorImageMessageToMat(color_image_msg, colorImagePtr, colorImage);

			// display color image
			for(int i=0; i<(int)face_position_accumulator_.size(); i++)
			{
				cv::Rect face;
				cob_msgs::Rect& faceRect = face_position_accumulator_[i].mask.roi;
				face.x = faceRect.x;    face.width = faceRect.width;
				face.y = faceRect.y;    face.height = faceRect.height;

				if (face_position_accumulator_[i].detector == "range")
					cv::rectangle(colorImage, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 0, 255), 2, 8, 0);
				else
					cv::rectangle(colorImage, cv::Point(face.x, face.y), cv::Point(face.x + face.width, face.y + face.height), CV_RGB(0, 255, 0), 2, 8, 0);

				if (face_position_accumulator_[i].label == "Unknown")
					// Distance to face class is too high
					cv::putText(colorImage, "Unknown", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				else if (face_position_accumulator_[i].label ==  "No face")
					// Distance to face space is too high
					cv::putText(colorImage, "No face", cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 255, 0, 0 ), 2);
				else
					// Face classified
					cv::putText(colorImage, face_position_accumulator_[i].label.c_str(), cv::Point(face.x,face.y+face.height+25), cv::FONT_HERSHEY_PLAIN, 2, CV_RGB( 0, 255, 0 ), 2);
			}
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
		boost::timed_mutex::scoped_timed_lock lock(face_position_accumulator_mutex_, boost::posix_time::milliseconds(2000));
		if (lock.owns_lock() == false)
		{
			ROS_ERROR("cob_people_detection::CobPeopleDetectionNodelet: Mutex was not freed during response time.");
			return false;
		}
		res.people_list.detections = face_position_accumulator_;
		res.people_list.header.stamp = ros::Time::now();
		return true;
	}
};

};

 PLUGINLIB_DECLARE_CLASS(ipa_PeopleDetector, CobPeopleDetectionNodelet, ipa_PeopleDetector::CobPeopleDetectionNodelet, nodelet::Nodelet);

#endif // _PEOPLE_DETECTION_
