
#ifndef __DLIB_FACE_DETECTION_H__
#define __DLIB_FACE_DETECTION_H__


#include <ros/ros.h>
#include <ros/package.h>

// ROS message
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>
#include <dlib_face_detection/RectArray.h>

#include <string.h>

// dlib
#include <dlib/image_processing/frontal_face_detector.h>
#include <dlib/image_processing.h>
#include <dlib/image_processing/render_face_detections.h>
#include <dlib/image_processing/generic_image.h>
#include <dlib/gui_widgets.h>
#include <dlib/image_io.h>
#include <dlib/opencv.h>
#include <dlib/all/source.cpp>

// opencv
#include <opencv/cv.h>
#include <opencv/ml.h>
#include <cv_bridge/cv_bridge.h>


class FaceDetectorNode
{
public:
	FaceDetectorNode(ros::NodeHandle nh);
	~FaceDetectorNode(void);

protected:
	void imgCallback(const sensor_msgs::Image::ConstPtr &image_msg);
	void headCallback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& head_img_msg);

	ros::NodeHandle node_handle_;
	ros::Publisher direct_face_pub_;
	ros::Publisher head_based_face_pub_;

	ros::Subscriber image_sub_;
	ros::Subscriber head_img_sub_;

	dlib::frontal_face_detector detector_;
	dlib::shape_predictor pose_model_;

	double scale_factor_;
};

#endif //__DLIB_FACE_DETECTION_H__
