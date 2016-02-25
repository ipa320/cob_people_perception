
#ifndef __HEAD_DETECTOR_NODE_H__
#define __HEAD_DETECTOR_NODE_H__

#include "cob_people_detection/head_detector.h"


#include <ros/ros.h>
#include <ros/package.h>

// ROS message
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <cob_perception_msgs/ColorDepthImageArray.h>

#include <iostream>
#include <string.h>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// opencv
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>

namespace ipa_PeopleDetector
{

class HeadDetectorNode
{
public:
	HeadDetectorNode(ros::NodeHandle nh);
	~HeadDetectorNode(void);

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;

protected:
	void pointcloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg);
	void convertPclMessageToMat(PointCloudT::Ptr& depth_lcoud, cv::Mat& depth_image, cv::Mat& color_image);
	void tfPublisher(geometry_msgs::PoseStamped pose, std::string label);

	tf::TransformListener tfl_;
	tf::TransformBroadcaster tfb_;

	ros::NodeHandle node_handle_;
	ros::Publisher head_pub_;
	ros::Publisher head_img_pub;
	ros::Subscriber pointcloud_sub_;

	HeadDetector head_detector_; ///< implementation of the head detector

	tf::Transform transform_;
	tf::StampedTransform stf_;

	std::string data_directory_;
	bool fill_unassigned_depth_values_; ///< fills the unassigned depth values in the depth image, must be true for a kinect sensor

	bool enable_tf_;

};

}
#endif //__HEAD_DETECTOR_NODE_H__
