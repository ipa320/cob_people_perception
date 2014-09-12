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

//##################
//#### includes ####

#include <munkres/munkres.h>

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
#include <boost/lexical_cast.hpp>

// external includes
#include "cob_vision_utils/GlobalDefines.h"

#include <sstream>
#include <string>
#include <vector>

class EvalObj
{
public:
	ros::Time timestamp;
	std::vector<geometry_msgs::Point> positions;
	std::vector<std::string> labels;
	EvalObj()
	{
	}
	void print()
	{
		std::cout << "-----------------------------" << std::endl;
		std::cout << "Timestamp:\n" << timestamp << std::endl;
		std::cout << "Positions:" << std::endl;
		for (int i = 0; i < positions.size(); i++)
		{
			std::cout << positions[i].x << " " << positions[i].y << std::endl;
			;
		}
		std::cout << "Labels:" << std::endl;
		for (int i = 0; i < labels.size(); i++)
		{
			std::cout << labels[i] << std::endl;
			;
		}
		std::cout << "-----------------------------" << std::endl;
	}

	bool compare(EvalObj& cmp, std::string& gt, bool& this_corr, bool& cmp_corr)
	{
		if (cmp.timestamp != this->timestamp)
		{
			std::cout << "time discrepancy - skipping comparison" << std::endl;
			return false;
		}
		else
		{
			// find out matching detections by comparing position
			float dist = 100000;
			std::vector<int> index_arr;
			index_arr.resize(this->labels.size());
			if (this->labels.size() == 0)
			{
				std::cout << "empty label array...." << std::endl;
				return false;
			}
			if (cmp.labels.size() == 0)
			{
				std::cout << "empty label array...." << std::endl;
				return false;
			}
			for (int i = 0; i < this->labels.size(); i++)
			{
				for (int j = 0; j < cmp.labels.size(); j++)
				{
					//calc distance
					float tmp_dist = (cmp.positions[i].x - this->positions[j].x) * (cmp.positions[i].x - this->positions[j].y) + (cmp.positions[i].y - this->positions[j].y)
							* (cmp.positions[i].y - this->positions[j].y);
					if (tmp_dist < dist)
					{
						dist = tmp_dist;
						index_arr[i] = j;
					}
				}
			}
			//for(int p = 0;p<index_arr.size();++p)
			//{
			//std::cout<<index_arr[p]<<std::endl;
			//}
			//
			//  compare labels of matches
			this_corr = true;
			cmp_corr = true;
			for (int i; i < index_arr.size(); i++)
			{
				std::string a = boost::lexical_cast<std::string>(this->labels[i]);
				std::string b = boost::lexical_cast<std::string>(cmp.labels[index_arr[i]]);
				std::cout << a << ": " << gt.compare(a) << " - " << b << ": " << gt.compare(b) << std::endl;
				if (a.compare("UnknownHead") == 0 || b.compare("UnknownHead") == 0)
				{
					std::cout << "discarding" << std::endl;
					return false;
				}
				//
				if (a.compare(gt) != 0)
					this_corr = false;
				if (b.compare(gt) != 0)
					cmp_corr = false;

			}

		}
		return true;
	}
};

class TrackingEvalNode
{
public:

	EvalObj eval_rec_;
	EvalObj eval_track_;
	bool rec_track;
	bool rec_rec;
	std::string gt_name_;

	int correct_trackings_;
	int correct_recognitions_;
	int total_comparisons_;
	int tracking_improvement_;
	int tracking_decline_;
	int error_out_;

	ros::Subscriber recognition_subscriber_;
	ros::Subscriber tracking_subscriber_;
	ros::NodeHandle node_handle_;

	TrackingEvalNode(ros::NodeHandle nh) :
		node_handle_(nh), correct_trackings_(0), correct_recognitions_(0), total_comparisons_(0), error_out_(0), tracking_improvement_(0), tracking_decline_(0), rec_track(false),
				rec_rec(false)
	{

		// subscribers
		recognition_subscriber_ = node_handle_.subscribe("/cob_people_detection/face_recognizer/face_recognitions", 1, &TrackingEvalNode::recognitionCallback, this);
		tracking_subscriber_ = node_handle_.subscribe("/cob_people_detection/detection_tracker/face_position_array", 1, &TrackingEvalNode::trackingCallback, this);

		// parameters
		if (!node_handle_.getParam("/cob_people_detection/tracking_evaluator/person", gt_name_))
			std::cout << "PARAM NOT AVAILABLE" << std::endl;
		std::cout << "Groundtruth name: " << gt_name_ << "\n";

	}

	~TrackingEvalNode()
	{
	}

	double computeFacePositionDistance(const cob_perception_msgs::Detection& previous_detection, const cob_perception_msgs::Detection& current_detection)
	{
		const geometry_msgs::Point* point_1 = &(previous_detection.pose.pose.position);
		const geometry_msgs::Point* point_2 = &(current_detection.pose.pose.position);

		double dx = point_1->x - point_2->x;
		double dy = point_1->y - point_2->y;
		double dz = point_1->z - point_2->z;

		return sqrt(dx * dx + dy * dy + dz * dz);
	}

	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void trackingCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_position_msg_in)
	{
		//std::cout<<"Callback TRACKING running..."<<std::endl;

		EvalObj eval_obj;
		eval_obj.timestamp = face_position_msg_in->header.stamp;
		// put timestamp, position and label in vector
		//std::cout<<"----> tracking: det_size="<<face_position_msg_in->detections.size()<<std::endl;
		for (unsigned int i = 0; i < face_position_msg_in->detections.size(); i++)
		{
			if (face_position_msg_in->detections[i].label.compare("") != 0)
			{
				//std::cout<<"      t label="<<face_position_msg_in->detections[i].label<<std::endl;
				eval_obj.positions.push_back(face_position_msg_in->detections[i].pose.pose.position);
				eval_obj.labels.push_back(face_position_msg_in->detections[i].label);
			}
		}
		eval_track_ = eval_obj;
		rec_track = true;
		this->eval();
	}
	/// checks the detected faces from the input topic against the people segmentation and outputs faces if both are positive
	void recognitionCallback(const cob_perception_msgs::DetectionArray::ConstPtr& face_position_msg_in)
	{
		//std::cout<<"Callback running..."<<std::endl;

		EvalObj eval_obj;
		eval_obj.timestamp = face_position_msg_in->header.stamp;
		// put timestamp, position and label in vector
		//std::cout<<"----> recognition: det_size="<<face_position_msg_in->detections.size()<<std::endl;
		for (unsigned int i = 0; i < face_position_msg_in->detections.size(); i++)
		{
			if (face_position_msg_in->detections[i].label.compare("") != 0)
			{
				//std::cout<<"      r label="<<face_position_msg_in->detections[i].label<<std::endl;
				eval_obj.positions.push_back(face_position_msg_in->detections[i].pose.pose.position);
				eval_obj.labels.push_back(face_position_msg_in->detections[i].label);
			}
		}
		eval_rec_ = eval_obj;
		rec_rec = true;
		this->eval();
	}

	void eval()
	{
		if (rec_track == true && rec_rec == true)
		{
			//std::cout<<"time rec= "<<eval_rec_.timestamp<<std::endl;
			//std::cout<<"time track= "<<eval_track_.timestamp<<std::endl;
			if (eval_track_.timestamp == eval_rec_.timestamp)
			{
				//std::cout<<"EVAL-->"<<std::endl;
				bool rec_res, track_res;
				bool cmp_successful = eval_rec_.compare(eval_track_, gt_name_, rec_res, track_res);
				if (cmp_successful)
				{
					if (rec_res)
						correct_recognitions_++;
					if (track_res)
						correct_trackings_++;
					if (track_res && !rec_res)
						tracking_improvement_++;
					if (!track_res && rec_res)
						tracking_decline_++;
					if (!track_res && !rec_res)
						error_out_++;
					total_comparisons_++;
					this->print_result();
				}
				rec_rec = false;
				rec_rec = false;
			}
		}
	}
	void print_result()
	{
		std::cout << "-----------------------------------" << std::endl;
		std::cout << "total comparisons    : " << total_comparisons_ << std::endl;
		std::cout << "correctly recognized : " << ((float)correct_recognitions_ / (float)total_comparisons_) * 100 << " \%" << std::endl;
		std::cout << "correctly tracked    : " << ((float)correct_trackings_ / (float)total_comparisons_) * 100 << " \%" << std::endl;
		std::cout << "tracking improvement : " << ((float)tracking_improvement_ / (float)total_comparisons_) * 100 << " \%" << std::endl;
		std::cout << "tracking worsening   : " << ((float)tracking_decline_ / (float)total_comparisons_) * 100 << " \%" << std::endl;
		std::cout << "errors after tracking: " << (error_out_ / total_comparisons_) * 100 << " \%" << std::endl;
		std::cout << "-----------------------------------" << std::endl;
		//std::cout<<"-----------------------------------"<<std::endl;
		//std::cout<<"total comparisons    : "<<total_comparisons_<<std::endl;
		//std::cout<<"correctly recognized : "<<correct_recognitions_<<std::endl;
		//std::cout<<"correctly tracked    : "<<correct_trackings_<<std::endl;
		//std::cout<<"tracking improvement : "<<tracking_improvement_<<std::endl;
		//std::cout<<"tracking worsening   : "<<tracking_decline_<<std::endl;
		//std::cout<<"errors after tracking: "<<error_out_<<std::endl;
		//std::cout<<"-----------------------------------"<<std::endl;
	}
};

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "tracking_evaluator");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create FaceRecognizerNode class instance
	TrackingEvalNode tracking_eval_node(nh);

	// Create action nodes
	//DetectObjectsAction detect_action_node(object_detection_node, nh);
	//AcquireObjectImageAction acquire_image_node(object_detection_node, nh);
	//TrainObjectAction train_object_node(object_detection_node, nh);

	ros::spin();

	return 0;
}
