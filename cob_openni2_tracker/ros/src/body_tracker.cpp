/*******************************************************************************
 *                                                                              *
 *   PrimeSense NiTE 2.0 - User Viewer Sample                                   *
 *   Copyright (C) 2012 PrimeSense Ltd.                                         *
 *                                                                              *
 *******************************************************************************/


#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "cob_openni2_tracker/body_tracker.h"
#include <GL/glut.h>
#include <cob_openni2_tracker/NiteSampleUtilities.h>

//screen and texture measures
#define GL_WIN_SIZE_X	640
#define GL_WIN_SIZE_Y	512
#define TEXTURE_SIZE	256
#define MAX_USERS 10
#define MIN_NUM_CHUNKS(data_size, chunk_size)	((((data_size)-1) / (chunk_size) + 1))
#define MIN_CHUNKS_SIZE(data_size, chunk_size)	(MIN_NUM_CHUNKS(data_size, chunk_size) * (chunk_size))

#define USER_MESSAGE(msg) {\
		sprintf(userStatusLabels[user.getId()], "%s", msg);\
		printf("[%08" PRIu64 "] User #%d:\t%s\n", ts, user.getId(), msg);}

float Colors[][3] 										= {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
bool visibleUsers[MAX_USERS] 							= {false};
int colorCount 											= 3;
nite::SkeletonState skeletonStates[MAX_USERS] 			= {nite::SKELETON_NONE};
char userStatusLabels[MAX_USERS][100] 					= {{0}};
char generalMessage[100] 								= {0};
static std::string detector_ 							= "camera";

using namespace std;
using namespace message_filters;


BodyTracker::BodyTracker(ros::NodeHandle nh_priv)
:pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>), m_poseUser(0),transform_listener_(nh_), br_()
{
	marker_id_ = 0;

	nh_ = nh_priv;
	// Get Tracker Parameters
	if(!nh_priv.getParam("camera_frame_id", cam_frame_)){
		ROS_WARN("tf_prefix was not found on Param Server! See your launch file!");
		//return -1;
		nh_.shutdown();
		Finalize();
	}

	if(!nh_priv.getParam("tf_prefix", tf_prefix_)){
		ROS_WARN("tf_prefix was not found on Param Server! See your launch file!");
		nh_.shutdown();
		Finalize();
	}

	if(!nh_priv.getParam("relative_frame", rel_frame_)){
		ROS_WARN("relative_frame was not found on Param Server! See your launch file!");
		nh_.shutdown();
		Finalize();
	}

	nh_priv.param("drawSkeleton", drawSkeleton_, true);
	nh_priv.param("drawCenterOfMass", drawCenterOfMass_, true);
	nh_priv.param("drawStatusLabel", drawStatusLabel_, true);
	nh_priv.param("drawBoundingBox", drawBoundingBox_, true);
	nh_priv.param("drawBackground", drawBackground_, true);
	nh_priv.param("drawDepth", drawDepth_, true);
	nh_priv.param("drawFrameId", drawFrameId_, false);
	nh_priv.param("poseTimeoutToExit", poseTimeoutToExit_, 2000);

	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
	pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ> >("body_tracker_filter", 0);
	people_pub_ = nh_.advertise<cob_perception_msgs::People>("people", 0);

	ROS_INFO("Create BodyTracker.\n");
	m_pUserTracker = new nite::UserTracker;
	pcl_cloud_->points.clear();

	init();

	it_ = new image_transport::ImageTransport(nh_priv);
	image_sub_.registerCallback(boost::bind(&BodyTracker::imageCallback, this, _1));
	image_sub_.subscribe(*it_, "/camera/rgb/image_raw", 1);
	image_pub_ = it_->advertise("colorimage_out", 1);
}



void BodyTracker::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	cv_bridge::CvImageConstPtr cv_ptr;
	//ROS_INFO("publish");
	try
	{
		cv_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return;
	}

	cv::Mat color_image = cv_ptr->image;
	int height = color_image_msg->height;
	int width = color_image_msg->width;

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols > 60)

		if(!tracked_users_->empty())
			for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin(); iter_ != tracked_users_->end(); ++iter_)
			{
				if((*iter_).getCenterOfMass().x != 0 && (*iter_).getCenterOfMass().y && (*iter_).getCenterOfMass().z)
				{
					int max_x = width - (*iter_).getBoundingBox().max.x;
					int max_y = (*iter_).getBoundingBox().max.y;
					int min_x = width - (*iter_).getBoundingBox().min.x;
					int min_y = (*iter_).getBoundingBox().min.y;

					double center_x = (*iter_).getCenterOfMass().x;
					double center_y = (*iter_).getCenterOfMass().y;

					int r = 255*Colors[(*iter_).getId() % colorCount][0];
					int g = 255*Colors[(*iter_).getId() % colorCount][1];
					int b = 255*Colors[(*iter_).getId() % colorCount][2];

					if(drawBoundingBox_)
					{
						cv::rectangle(color_image, cv::Point(min_x, min_y), cv::Point(max_x, max_y) ,
								CV_RGB(255, 0, 0), 2);//drawBoundingBox((*iter_), color_image, image_width);
					}
					if(drawCenterOfMass_)
					{
						//cv::circle(color_image, cv::Point((int)center_x, (int) center_y), 10, CV_RGB(r, g, b));
//						cv::circle(color_image, cv::Point(100, 100), 10, CV_RGB(255, 0, 0));
					}
				}
			}
	cv::waitKey(10);
	// Output modified video stream

	try{
	image_pub_.publish(cv_ptr->toImageMsg());
	}catch(...)
	{
		ROS_INFO("ERROR !!");
	}

}

BodyTracker::~BodyTracker()
{

	ROS_INFO("shutdown nodelet");
	cv::destroyWindow(OPENCV_WINDOW);
	Finalize();
}

void BodyTracker::Finalize()
{

	ros::spinOnce();
	delete[] m_pTexMap_;
	delete m_pUserTracker;
	device_.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

void BodyTracker::init()
{
	m_pTexMap_ = NULL;

	openni::Status rc = openni::OpenNI::initialize();
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
		nh_.shutdown();
		return;
	}
	const char* device_Uri = openni::ANY_DEVICE;

	rc = device_.open(device_Uri);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to open device_\n%s\n", openni::OpenNI::getExtendedError());
		nh_.shutdown();
		return;
	}

	rc = depthSensor_.create(device_, openni::SENSOR_DEPTH);
	if (rc != openni::STATUS_OK)
	{
		printf("Failed to create a video stream \n%s\n", openni::OpenNI::getExtendedError());
		nh_.shutdown();
		return;
	}

	nite::NiTE::initialize();
	if (m_pUserTracker->create(&device_) != nite::STATUS_OK)
	{
		printf(" Get data for NiTE failed\n");
		nh_.shutdown();
		return;
	}
}

void BodyTracker::runTracker()
{
	while(nh_.ok())
	{

		nite::UserTrackerFrameRef userTrackerFrame;
		openni::VideoFrameRef depthFrame;

		nite::Status rc = m_pUserTracker->readFrame(&userTrackerFrame);
		if (rc != nite::STATUS_OK)
		{
			printf("GetNextData failed\n");
			return;
		}

		depthFrame = userTrackerFrame.getDepthFrame();

		if (m_pTexMap_ == NULL)
		{
			// Texture map init
			m_nTexMapX = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
			m_nTexMapY = MIN_CHUNKS_SIZE(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
			m_pTexMap_ = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
		}
		const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

		if (depthFrame.isValid() && drawDepth_)
		{
			calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
		}
		memset(m_pTexMap_, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));
		float factor[3] = {1, 1, 1};

		//construct pcl
		if (depthFrame.isValid() && drawDepth_)
		{
			const nite::UserId* pLabels = userLabels.getPixels();

			const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
			openni::RGB888Pixel* pTexRow = m_pTexMap_ + depthFrame.getCropOriginY() * m_nTexMapX;
			int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);
			float dX, dY, dZ;

			pcl_cloud_->resize(depthFrame.getHeight()*depthFrame.getWidth());
			pcl_cloud_->width = depthFrame.getWidth();
			pcl_cloud_->height = depthFrame.getHeight();

			for (int y = 0; y < depthFrame.getHeight(); ++y)
			{
				const openni::DepthPixel* pDepth = pDepthRow;
				openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

				for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
				{
					if (*pLabels == 0)  // determine color
					{
						if (!drawBackground_)
						{
							factor[0] = factor[1] = factor[2] = 0;
						}
						else
						{
							factor[0] = Colors[colorCount][0];
							factor[1] = Colors[colorCount][1];
							factor[2] = Colors[colorCount][2];
						}
					}
					else
					{
						factor[0] = Colors[*pLabels % colorCount][0];
						factor[1] = Colors[*pLabels % colorCount][1];
						factor[2] = Colors[*pLabels % colorCount][2];
					}

					if(*pDepth != 0)
						openni::CoordinateConverter::convertDepthToWorld(depthSensor_, (float)x, (float)y, (float)(*pDepth), &dX, &dY, &dZ);
					else
						dX = dY = dZ = 0.f;
					pcl::PointXYZRGB& point = pcl_cloud_->at(x,y);
					point.r = 255*factor[0];
					point.g = 255*factor[1];
					point.b = 255*factor[2];
					point.x = dZ/1000;
					point.y = -dX/1000;
					point.z = dY/1000;

					if (*pDepth != 0)
					{
						int nHistValue = m_pDepthHist[*pDepth];
						pTex->r = nHistValue*factor[0];
						pTex->g = nHistValue*factor[1];
						pTex->b = nHistValue*factor[2];

						factor[0] = factor[1] = factor[2] = 1;
					}
				}
				pDepthRow += rowSize;
				pTexRow += m_nTexMapX;
			}
		}

		drawPointCloud();

		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		tracked_users_ = new list<nite::UserData>();

		if(!tracked_users_->empty())
		{
			return;
		}

		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user, userTrackerFrame.getTimestamp());

			if(user.isNew())
			{
				m_pUserTracker->startSkeletonTracking(user.getId());
				unsigned int id = user.getId();
				//printf("new user id %u", (unsigned int) id);
				m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
			}
			else if(!user.isLost())
			{
				if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
				{
					drawSkeleton(m_pUserTracker, user);

					//				JointMap joints;
					//				joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
					//				joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
					//				joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
					//				joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
					//				joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
					//				joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
					//				joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
					//				joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
					//				joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
					//				joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
					//				joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
					//				joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
					//				joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
					//				joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
					//				joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));
					//
					//								for (JointMap::iterator it=joints.begin(); it!=joints.end(); ++it){
					//									publishJoints(nh_, br_, it->first, it->second, tf_prefix_, rel_frame_, user.getId());
					//								}
					//												if (g_drawStatusLabel_){
					//													drawStatusLabel(m_pUserTracker, user);
					//												}
					//												if (g_drawCenterOfMass_){
					//													drawCenterOfMass(m_pUserTracker, user);
					//												}
					//												if (g_drawBoundingBox_){
					//													drawBoundingBox(user);
					//												}
					//												if (g_drawSkeleton_){
					//													drawSkeleton(m_pUserTracker, user);
					//												}

					tracked_users_->insert(tracked_users_->end(), user);
				}
			}

			if (m_poseUser == 0 || m_poseUser == user.getId())
			{
				const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);

				if (pose.isEntered())
				{
					// Start timer
					sprintf(generalMessage, "In exit pose. Keep it for %d second%s to exit\n", poseTimeoutToExit_/1000, poseTimeoutToExit_/1000 == 1 ? "" : "s");
					printf("Counting down %d second to exit\n", poseTimeoutToExit_/1000);
					m_poseUser = user.getId();
					m_poseTime_ = userTrackerFrame.getTimestamp();
				}
				else if (pose.isExited())
				{
					memset(generalMessage, 0, sizeof(generalMessage));
					printf("Count-down interrupted\n");
					m_poseTime_ = 0;
					m_poseUser = 0;
				}
				else if (pose.isHeld())
				{
					if (userTrackerFrame.getTimestamp() - m_poseTime_ > poseTimeoutToExit_ * 1000)
					{
						printf("Timeout.");
					}
				}
			}
		}

		//publish tracked users
		std::vector<cob_perception_msgs::Person> detected_people;
		for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin(); iter_ != tracked_users_->end(); ++iter_){

			cob_perception_msgs::Person person;
			cob_perception_msgs::Skeleton skeleton;

			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_HEAD)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_NECK)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_HAND)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_HAND)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_TORSO)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_HIP)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_HIP)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_KNEE)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_LEFT_FOOT)));
			skeleton.joints.push_back(convertNiteJointToMsgs((*iter_).getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT)));

			char id[100];
			snprintf(id,100,"bodytrack %d", (*iter_).getId());
			string id_ = std::string(id);

			person.name = id_;
			person.detector = detector_;
			person.position.position.x = (*iter_).getCenterOfMass().x/1000.0;
			person.position.position.y = -(*iter_).getCenterOfMass().y/1000.0;
			person.position.position.z = (*iter_).getCenterOfMass().z/1000.0;

			person.skeleton = skeleton;

			unsigned int length =
					( person.position.position.x * person.position.position.x +
							person.position.position.y * person.position.position.y +
							person.position.position.z * person.position.position.z);

			if(length != 0)
				detected_people.push_back(person);
		}

		cob_perception_msgs::People array;
		array.header.stamp = ros::Time::now();
		array.header.frame_id = rel_frame_;
		array.people = detected_people;
		people_pub_.publish(array);

		/*
	if (g_drawFrameId_)
	{
		drawFrameId(userTrackerFrame.getFrameIndex());
	}

	if (g_generalMessage[0] != '\0')
	{
		char *msg = g_generalMessage;
		float colors[3] = {1, 0, 0};
		//rasterPos2i(100, 20);
		//To Do : print message, msg);
	}
		 */
	}
	Finalize();
}

void BodyTracker::updateUserState(const nite::UserData& user, uint64_t ts)
{
	if(user.isNew()){
		USER_MESSAGE("New User detected.");
	}
	else if(user.isVisible() && !visibleUsers[user.getId()]){
		printf("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	}
	else if(!user.isVisible() && visibleUsers[user.getId()]){
		printf("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	}
	else if(user.isLost()){
		USER_MESSAGE("Lost");
	}
	visibleUsers[user.getId()] = user.isVisible();

	if(skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking !")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking ...");
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}


void BodyTracker::publishJoints(ros::NodeHandle& nh, tf::TransformBroadcaster& br, std::string joint_name,
		nite::SkeletonJoint joint, std::string tf_prefix, std::string rel_frame, int id){

	if (joint.getPositionConfidence() > 0.0)
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(joint.getPosition().x/1000.0, -joint.getPosition().y/1000.0,
				joint.getPosition().z/1000.0));
		tf::Quaternion frame_rotation;
		frame_rotation.setEuler(0, 0, 0);
		transform.setRotation(frame_rotation);
		std::stringstream frame_id_stream;
		std::string frame_id;
		frame_id_stream << "/" << tf_prefix << "/user_" << id << "/" << joint_name;
		frame_id = frame_id_stream.str();
		// std::cout << frame_id << std::endl;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), rel_frame, frame_id));
	}
}

geometry_msgs::Pose BodyTracker::convertNiteJointToMsgs(nite::SkeletonJoint joint)
{
	geometry_msgs::Pose transform_msgs;

	if (joint.getPositionConfidence() > 0.0)
	{
		tf::Transform transform;
		transform.setOrigin(tf::Vector3(joint.getPosition().x/1000.0, -joint.getPosition().y/1000.0,
				joint.getPosition().z/1000.0));
		tf::Quaternion frame_rotation;
		frame_rotation.setEuler(0, 0, 0);
		transform.setRotation(frame_rotation);
		tf::poseTFToMsg(transform, transform_msgs);
	}
	return transform_msgs;
}


void BodyTracker::drawPointCloud()
{
	ros::Time time = ros::Time::now();
	uint64_t st = time.toNSec();
	pcl_cloud_->header.stamp = st;
	pcl_cloud_->header.frame_id = cam_frame_;

	pcl_pub_.publish(pcl_cloud_);
	pcl_cloud_->points.clear();
}
//TO DO
void BodyTracker::drawCenterOfMass(const nite::UserData& user)
{

	double x = user.getCenterOfMass().x;
	double y = user.getCenterOfMass().y;
	double z = user.getCenterOfMass().z;

	//ROS_INFO("%f %f %f", (float) x, (float) y, (float) z);
}

void BodyTracker::drawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{

	//To Do: ponints, coordinates + 3
}

void BodyTracker::drawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	drawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}


void BodyTracker::drawBoundingBox(const nite::UserData& user, cv::Mat& color_image, const int width)
{
	int max_x = width - user.getBoundingBox().max.x;
	int max_y = user.getBoundingBox().max.y;
	int min_x = width -user.getBoundingBox().min.x;
	int min_y = user.getBoundingBox().min.y;


	int r = 255*Colors[user.getId() % colorCount][0];
	int g = 255*Colors[user.getId() % colorCount][1];
	int b = 255*Colors[user.getId() % colorCount][2];

	cv::rectangle(color_image, cv::Point(min_x, min_y), cv::Point(max_x, max_y) ,
			CV_RGB(r, g, b), 2);
}

void BodyTracker::drawStatusLabel(const nite::UserData& user, cv::Mat& color_image, cv::Point& tag_coords)
{

	std::string tag_label = "User" + user.getId();
	std::cout << "The text tag reads: " << tag_label << "." << std::endl;
	cv::putText(color_image, tag_label, tag_coords, cv::FONT_HERSHEY_PLAIN, 3, CV_RGB(0,0,255), 2);
}

void BodyTracker::drawFrameId(int frameId)
{
	char buffer[80] = "";
	sprintf(buffer, "%d", frameId);

	//To Do: print string (frame id) , buffer.
}
/**
 * A helper function to draw a simple line in rviz.
 */
void BodyTracker::drawLine (const double r, const double g, const double b, const double a,
		const nite::Point3f& pose_start, const nite::Point3f& pose_end )
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = rel_frame_;
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	geometry_msgs::Point p;
	marker.id = marker_id_;
	p.x = pose_start.x/1000.0;
	p.y = pose_start.y/1000.0;
	p.z = pose_start.z/1000.0;
	marker.points.push_back(p);
	p.x =pose_end.x/1000.0;
	p.y = pose_end.y/1000.0;
	p.z = pose_end.z/1000.0;
	marker.points.push_back(p);
	marker.scale.x = 0.029;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.color.a = a;
	marker.lifetime = ros::Duration(0.01);
	vis_pub_.publish(marker);
	marker_id_++;
}

/*
 * A helper function to draw circles in rviz
 */

void BodyTracker::drawCircle(const double r, const double g, const double b, const double a,
		const nite::Point3f& pose){
	visualization_msgs::Marker m;
	m.header.stamp = ros::Time::now();
	m.action = visualization_msgs::Marker::ADD;
	m.header.frame_id = "/camra_link";
	m.type = m.SPHERE;
	m.id = marker_id_;
	m.pose.position.x = pose.x/1000.0;
	m.pose.position.y = pose.y/1000.0;
	m.pose.position.z = pose.z/1000.0;
	m.scale.x = .1;
	m.scale.y = .1;
	m.scale.z = .1;
	m.color.a = 1;
	m.color.r = 1;
	m.lifetime = ros::Duration(0.01);
	vis_pub_.publish(m);
	marker_id_++;
}

void BodyTracker::calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
{
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
	int width = depthFrame.getWidth();
	int height = depthFrame.getHeight();
	// Calculate the accumulative histogram (the yellow display...)
	memset(pHistogram, 0, histogramSize*sizeof(float));
	int restOfRow = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - width;

	unsigned int nNumberOfPoints = 0;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				pHistogram[*pDepth]++;
				nNumberOfPoints++;
			}
		}
		pDepth += restOfRow;
	}
	for (int nIndex=1; nIndex<histogramSize; nIndex++)
	{
		pHistogram[nIndex] += pHistogram[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<histogramSize; nIndex++)
		{
			pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
		}
	}
}
int main(int argc, char** argv)
{
	ros::init(argc, argv, "cob_body_tracker");
	ros::NodeHandle nh_priv("~");
	BodyTracker BodyTracker(nh_priv);

	ros::spin();
	return 0;

}
