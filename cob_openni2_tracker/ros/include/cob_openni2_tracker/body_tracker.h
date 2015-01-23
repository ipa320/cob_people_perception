/*******************************************************************************
 *                                                                              *
 *   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
 *   Copyright (C) 2012 PrimeSense Ltd.                                         *
 *                                                                              *
 *******************************************************************************/

#ifndef _NITE_USER_VIEWER_H_
#define _NITE_USER_VIEWER_H_

#include <libnite2/NiTE.h>
#include <ros/ros.h>
#include <tf/tfMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <pcl/ros/conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <cob_perception_msgs/Skeleton.h>
#include <cob_perception_msgs/People.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <boost/thread/mutex.hpp>
#include <boost/timer.hpp>


#define MAX_DEPTH 10000
typedef std::map<std::string, nite::SkeletonJoint>JointMap;

class BodyTracker
{

public:

	BodyTracker(ros::NodeHandle nh);
	virtual ~BodyTracker();
	//PARAMS
	std::string tf_prefix_, rel_frame_, cam_frame_;
	openni::Device		m_device;


private:

	//ROS ELEMENTS
	ros::NodeHandle nh_;

	//TF ELEMENTS
	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster br_;
	ros::Publisher vis_pub_, pcl_pub_, skeleton_pub_, people_pub_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud_;
	BodyTracker(const BodyTracker&);
	BodyTracker& operator=(BodyTracker&);

	void runTracker();
	void Finalize();
	void init();
	void updateUserState(const nite::UserData& user, uint64_t ts);

	void publishJoints(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string joint_name,
			nite::SkeletonJoint joint, std::string tf_prefix, std::string rel_frame, int id);
	geometry_msgs::Pose convertNiteJointToMsgs(nite::SkeletonJoint joint);
	void drawLine(const double r, const double g, const double b, const double a,
			const nite::Point3f& pose_start, const nite::Point3f& pose_end );
	void drawCircle(const double r, const double g, const double b, const double a,
			const nite::Point3f& pose);
	void drawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData);
	//void drawUser(nite::UserTrackerFrameRef& userTrackerFrame);
	void drawCenterOfMass(nite::UserTracker* pUserTracker, const nite::UserData& user);
	void drawBoundingBox(const nite::UserData& user);
	void drawStatusLabel(nite::UserTracker* pUserTracker, const nite::UserData& user);
	void drawFrameId(int frameId);
	void drawPointCloud();
	void drawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color);
	void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame);

	std::list<nite::UserData>* tracked_users_;

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	//openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;
	openni::VideoStream depthSensor_;
	image_transport::Publisher image_pub_;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
	unsigned int marker_id_;

	bool g_drawSkeleton_;
	bool g_drawCenterOfMass_;
	bool g_drawStatusLabel_;
	bool g_drawBoundingBox_;
	bool g_drawBackground_;
	bool g_drawDepth_;
	bool g_drawFrameId_;

	int g_poseTimeoutToExit_;

	////
	static BodyTracker* ms_self;

};

#endif // _NITE_USER_VIEWER_H_
