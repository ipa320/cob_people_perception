#ifndef __DETECTION_VIEWER_H__
#define __DETECTION_VIEWER_H__


#include <ros/ros.h>


// ROS message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cob_perception_msgs/DetectionArray.h>

#include <iostream>
#include <string.h>

// message_filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

// opencv
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv/highgui.h>

namespace ipa_PeopleDetector
{

class DetectionViewer
{
public:
	DetectionViewer(ros::NodeHandle nh);
	~DetectionViewer(void);

	typedef sensor_msgs::Image image_msgT;
	typedef cob_perception_msgs::DetectionArray det_msgT;
	typedef message_filters::sync_policies::ApproximateTime<image_msgT, det_msgT, det_msgT> SyncPolicy;

protected:
	void callback(const sensor_msgs::Image::ConstPtr &image_msg,
				  const cob_perception_msgs::DetectionArray::ConstPtr& head_msg,
				  const cob_perception_msgs::DetectionArray::ConstPtr& face_msg);

	ros::NodeHandle node_handle_;

	message_filters::Subscriber<image_msgT>* image_sub_;
	message_filters::Subscriber<det_msgT>* head_sub_;
	message_filters::Subscriber<det_msgT>* face_sub_;
	message_filters::Synchronizer<SyncPolicy>* sync_;
};

}

#endif //__DETECTION_VIEWER_H__
