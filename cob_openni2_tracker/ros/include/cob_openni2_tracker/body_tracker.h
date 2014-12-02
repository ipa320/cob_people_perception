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
#include <visualization_msgs/Marker.h>

#define MAX_DEPTH 10000
typedef std::map<std::string, nite::SkeletonJoint>JointMap;

class SampleViewer
{
public:
	SampleViewer(const char* strSampleName,ros::NodeHandle nh);
	virtual ~SampleViewer();

	virtual openni::Status Init(int argc, char **argv);
	virtual openni::Status Run();	//Does not return


protected:

	//ROS ELEMENTS
	ros::NodeHandle nh_;

	//PARAMS
	std::string tf_prefix_, rel_frame_;

	//TF ELEMENTS
	tf::TransformListener transform_listener_;
	tf::TransformBroadcaster br_;
	ros::Publisher vis_pub_, skeleton_pub_;

	virtual void Display();
	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image
    virtual void Finalize();
	virtual void OnKey(unsigned char key, int x, int y);


	virtual openni::Status InitOpenGL(int argc, char **argv);
	void InitOpenGLHooks();


	void publishJoints(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string joint_name,
			nite::SkeletonJoint joint, std::string tf_prefix, std::string rel_frame, int id);

	void drawLine (const double r, const double g, const double b, const double a,
			const nite::Point3f& pose_start, const nite::Point3f& pose_end );
	void drawCircle(const double r, const double g, const double b, const double a,
			const nite::Point3f& pose);
	//void drawPeople(JointMap& joints);

private:
	SampleViewer(const SampleViewer&);
	SampleViewer& operator=(SampleViewer&);

	static SampleViewer* ms_self;
	static void glutIdle();
	static void glutDisplay();

	static void glutKeyboard(unsigned char key, int x, int y);

	float				m_pDepthHist[MAX_DEPTH];
	char			m_strSampleName[ONI_MAX_STR];
	openni::RGB888Pixel*		m_pTexMap;
	unsigned int		m_nTexMapX;
	unsigned int		m_nTexMapY;

	openni::Device		m_device;
	nite::UserTracker* m_pUserTracker;

	nite::UserId m_poseUser;
	uint64_t m_poseTime;
	unsigned int marker_id_;

};

#endif // _NITE_USER_VIEWER_H_
