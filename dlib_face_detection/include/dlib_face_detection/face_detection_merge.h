
#ifndef __FACE_DETECTION_MERGE_H__
#define __FACE_DETECTION_MERGE_H__


#include <ros/ros.h>

// ROS message
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/DetectionArray.h>
#include <dlib_face_detection/RectArray.h>

#include <string.h>

//pcl
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>

// tf
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

// message_filter
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace ipa_PeopleDetector
{

class FaceDetectionMerge
{
public:
	FaceDetectionMerge(ros::NodeHandle nh);
	~FaceDetectionMerge(void);

	typedef pcl::PointXYZRGB PointT;
	typedef pcl::PointCloud<PointT> PointCloudT;
	typedef sensor_msgs::PointCloud2 cloud_msgT;
	typedef dlib_face_detection::RectArray rect_msgT;
	typedef message_filters::sync_policies::ApproximateTime<cloud_msgT, rect_msgT, rect_msgT> SyncPolicy;

protected:
	void callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
					const dlib_face_detection::RectArray::ConstPtr& dFace_msg,
					const dlib_face_detection::RectArray::ConstPtr& hbFace_msg);

	void tfpublisher(const cob_perception_msgs::DetectionArray::ConstPtr detArray);

	tf::TransformListener tfl_;
	tf::TransformBroadcaster tfb_;

	ros::NodeHandle node_handle_;
	ros::Publisher face_pub_;

	message_filters::Subscriber<cloud_msgT>* cloud_sub_;
	message_filters::Subscriber<rect_msgT>* direct_face_sub_;
	message_filters::Subscriber<rect_msgT>* head_based_face_sub_;
	message_filters::Synchronizer<SyncPolicy>* sync_;

	tf::Transform transform_;
	tf::StampedTransform stf_;

	bool enable_tf_;
};

}

#endif //__FACE_DETECTION_MERGE_H__
