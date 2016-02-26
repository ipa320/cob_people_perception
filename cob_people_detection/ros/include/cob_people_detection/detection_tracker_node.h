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
 * \date Date of creation: 08.08.2012
 *
 * \brief
 * functions for tracking detections, e.g. recognized faces
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

#ifndef _DETECTION_TRACKER_
#define _DETECTION_TRACKER_

//##################
//#### includes ####

// standard includes
//--

// ROS includes
#include <ros/ros.h>
#include <ros/package.h>

// ROS message includes
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/DetectionArray.h>

// services
//#include <cob_people_detection/DetectPeople.h>

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

// boost
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>

// external includes
#include "cob_vision_utils/GlobalDefines.h"

#include <sstream>
#include <string>
#include <vector>

namespace ipa_PeopleDetector
{

class DetectionTrackerNode
{
protected:
	image_transport::ImageTransport* it_;
	image_transport::SubscriberFilter people_segmentation_image_sub_; ///< Color camera image topic

	message_filters::Synchronizer<message_filters::sync_policies::ApproximateTime<cob_perception_msgs::DetectionArray, sensor_msgs::Image> >* sync_input_2_;
	message_filters::Subscriber<cob_perception_msgs::DetectionArray> face_position_subscriber_; ///< receives the face messages from the face detector
	ros::Publisher face_position_publisher_; ///< publisher for the positions of the detected faces

	ros::NodeHandle node_handle_; ///< ROS node handle

	std::vector<cob_perception_msgs::Detection> face_position_accumulator_; ///< accumulates face positions over time
	boost::timed_mutex face_position_accumulator_mutex_; ///< secures write and read operations to face_position_accumulator_
	std::vector<std::map<std::string, double> > face_identification_votes_; ///< collects votes for all names (map index) ever assigned to each detection (vector index) in face_position_accumulator_

	// parameters
	bool debug_; ///< enables some debug outputs
	bool use_people_segmentation_; ///< enables the combination of face detections with the openni people segmentation
	double face_redetection_time_; ///< timespan during which a face is preserved in the list of tracked faces although it is currently not visible
	ros::Duration publish_currently_not_visible_detections_timespan_; ///< timespan during which a currently not visible face, which is though preserved in the list of tracked faces, is published as detection (in [0, face_redetection_time])
	double min_segmented_people_ratio_face_; ///< the minimum area inside the face rectangle found in the color image that has to be covered with positive people segmentation results (from openni_tracker)
	double min_segmented_people_ratio_head_; ///< the minimum area inside the head rectangle found in the depth image that has to be covered with positive people segmentation results (from openni_tracker)
	double tracking_range_m_; ///< maximum tracking manhattan distance for a face (in meters), i.e. a face can move this distance between two images and can still be tracked
	double face_identification_score_decay_rate_; ///< face identification score decay rate (0 < x < 1), i.e. the score for each label at a certain detection location is multiplied by this factor
	double min_face_identification_score_to_publish_; ///< minimum face identification score to publish (0 <= x < max_score), i.e. this score must be exceeded by a label at a detection location before the person detection is published (higher values increase robustness against short misdetections, but consider the maximum possible score max_score w.r.t. the face_identification_score_decay_rate: new_score = (old_score+1)*face_identification_score_decay_rate --> max_score = face_identification_score_decay_rate/(1-face_identification_score_decay_rate))
	bool fall_back_to_unknown_identification_; ///< if this is true, the unknown label will be assigned for the identification of a person if it has the highest score, otherwise, the last detection of a name will display as label even if there has been a detection of Unknown recently for that face
	bool display_timing_;

	bool rosbag_mode_; /// < true if data from a rosbag is used, to ignore deprecated timestamps

public:

	DetectionTrackerNode(ros::NodeHandle nh);

	~DetectionTrackerNode();

	/// Converts a color image message to cv::Mat format.
	unsigned long convertColorImageMessageToMat(const sensor_msgs::Image::ConstPtr& image_msg, cv_bridge::CvImageConstPtr& image_ptr, cv::Mat& image);

	/// Copies the data from src to dest.
	/// @param src The new data which shall be copied into dest
	/// @param dst The new data src is copied into dest
	/// @param update If update is true, dest must contain the data which shall be updated
	/// @param updateIndex The index in face_identification_votes_ corresponding to the previous detection dest. Only necessary if update is true.
	/// @return Return code.
	unsigned long copyDetection(const cob_perception_msgs::Detection& src, cob_perception_msgs::Detection& dest, bool update = false,
			unsigned int updateIndex = UINT_MAX);

	/// Computes the Euclidean distance of a recent faces detection to a current face detection.
	/// If the current face detection is outside the neighborhood of the previous detection, DBL_MAX is returned.
	/// @return The squared Euclidian distance of both faces or DBL_MAX.
	double computeFacePositionDistanceTrackingRange(const cob_perception_msgs::Detection& previous_detection, const cob_perception_msgs::Detection& current_detection);

	/// Computes the Euclidean distance of a recent faces detection to a current face detection.
	/// @return Always returns the Euclidian distance of both faces.
	double computeFacePositionDistance(const cob_perception_msgs::Detection& previous_detection, const cob_perception_msgs::Detection& current_detection);

	/// Removes multiple instances of a label by renaming the detections with lower score to Unknown.
	/// @return Return code.
	unsigned long removeMultipleInstancesOfLabel();

	unsigned long prepareFacePositionMessage(cob_perception_msgs::DetectionArray& face_position_msg_out, ros::Time image_recording_time, std::string frame_id);

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void inputCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_position_msg_in, const sensor_msgs::Image::ConstPtr& people_segmentation_image_msg);

};

}
;

#endif // _DETECTION_TRACKER_
