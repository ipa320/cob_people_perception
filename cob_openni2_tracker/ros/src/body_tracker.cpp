/*
 *****************************************************************
 * Copyright (c) 2015 \n
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
 * ROS package name: cob_openni2_tracker
 *
 * \author
 * Author: Olha Meyer
 * \author
 * Supervised by: Richard Bormann
 *
 * \date Date of creation: 01.11.2014
 *
 * \brief
 * functions for detecting people within a depth image
 * current approach: read the current video flow and detect people using OpenNI and NiTE2
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

/*
 * Other Copyrights :
 *
 ********************************************************
 *                                                        *
 *   PrimeSense NiTE 2.0 - User Viewer Sample             *
 *   Copyright (C) 2012 PrimeSense Ltd.                   *
 *   (For a license distribution please see               *
 *     the package information):                          *
 **********************************************************/


#if (defined _WIN32)
#define PRIu64 "llu"
#else
#define __STDC_FORMAT_MACROS
#include <inttypes.h>
#endif

#include "sensor_msgs/PointCloud2.h"
#include "cob_openni2_tracker/body_tracker.h"
//#include <GL/glut.h>
#include <cob_openni2_tracker/NiteSampleUtilities.h>
#include <math.h>
#include <stdio.h>

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

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
bool visibleUsers[MAX_USERS] = {false};
int colorCount = 3;
nite::SkeletonState skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char userStatusLabels[MAX_USERS][100] = {{0}};
char generalMessage[100] = {0};
static std::string detector_ = "camera";

using namespace std;
using namespace message_filters;

/*
 * BodyTracker constructor: loads the parameter on the parameter server and
 * advertises subscribes and publishes.
 */
BodyTracker::BodyTracker(ros::NodeHandle nh_priv)
:pcl_cloud_(new pcl::PointCloud<pcl::PointXYZRGB>), tracked_users_(new list<nite::UserData>()), m_poseUser(0), transform_broadcaster_(), nh_(nh_priv)
{
	marker_id_ = 0;
	init_counter_color_image_ = 0;
	init_counter_point_cloud_ = 0;
	shutdown_ = false;

	it_ = 0;
	m_pTexMap_ = 0;
	m_pUserTracker = 0;

	//nh_ = nh_priv;
	// Get Tracker Parameters
	if(!nh_.getParam("depth_optical_frame_id", depth_optical_frame_)){
		ROS_WARN("depth_optical_frame_id was not found on Param Server! See your launch file!");
		nh_.shutdown();
		finalize();
	}

	if(!nh_.getParam("tf_prefix", tf_prefix_)){
		ROS_WARN("tf_prefix was not found on Param Server! See your launch file!");
		nh_.shutdown();
		finalize();
	}

//	if(!nh_.getParam("relative_frame", rel_frame_)){
//		ROS_WARN("relative_frame was not found on Param Server! See your launch file!");
//		nh_.shutdown();
//		finalize();
//	}

	std::cout << "\n---------------------------\nPeople Tracker Detection Parameters (CAMERA):\n---------------------------\n";
	//parameters from a YAML File
	nh_.param("drawSkeleton", drawSkeleton_, false);
	std::cout << "drawSkeleton = " << drawSkeleton_ << "\n";
	nh_.param("drawCenterOfMass", drawCenterOfMass_, false);
	std::cout << "drawCenterOfMass = " << drawCenterOfMass_ << "\n";
	nh_.param("drawUserName", drawUserName_, true);
	std::cout << "drawUserName = " << drawUserName_ << "\n";
	nh_.param("drawBoundingBox", drawBoundingBox_, true);
	std::cout << "drawBoundingBox = " << drawBoundingBox_ << "\n";
	nh_.param("drawBackground", drawBackground_, false);
	std::cout << "drawBackground = " << drawBackground_ << "\n";
	nh_.param("drawDepth", drawDepth_, false);
	std::cout << "drawDepth = " << drawDepth_ << "\n";
	nh_.param("drawFrames", drawFrames_, false);
	std::cout << "drawFrames = " << drawFrames_ << "\n";
	nh_.param("poseTimeoutToExit", poseTimeoutToExit_, 2000);
	std::cout << "poseTimeoutToExit = " << poseTimeoutToExit_ << "\n";
	bool standalone_without_camera_driver = false;
	nh_.param("standalone_without_camera_driver", standalone_without_camera_driver, false);
	std::cout << "standalone_without_camera_driver = " << standalone_without_camera_driver << "\n";

	tracked_users_ = new list<nite::UserData>();

	it_ = new image_transport::ImageTransport(nh_);
	image_sub_.registerCallback(boost::bind(&BodyTracker::imageCallback, this, _1));
	image_sub_.subscribe(*it_, "color_image_topic", 1);
	image_pub_ = it_->advertise("colorimage_out", 1);

	pcl_sub_ = nh_.subscribe("point_cloud_topic", 1, &BodyTracker::pointcloudCallback, this);

	ros::Time start_time = ros::Time::now();
	while(standalone_without_camera_driver==false)
	{
		if (init_counter_color_image_>3 && init_counter_point_cloud_>3)
			break;
		if ((ros::Time::now()-start_time).sec > 300)
		{
			ROS_ERROR("The camera driver does not seem to work properly, not enough image and point cloud messages received during the last 300s.");
			shutdown_ = true;
			return;
		}
	}

	pcl_sub_.shutdown();

	vis_pub_ = nh_.advertise<visualization_msgs::Marker>( "visualization_marker", 10);
	if (drawDepth_==true)
		pcl_pub_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("points_body_tracker", 1);	// original point cloud with all points that belong to one tracked person colored in a individual color
	people_pub_ = nh_.advertise<cob_perception_msgs::People>("people", 1);		// detections

	ROS_INFO("Create BodyTracker.\n");
	m_pUserTracker = new nite::UserTracker;
	pcl_cloud_->points.clear();

	init();
}


/*
 * A callback function for reading and modifying a color video stream.
 * Uses a list of currently tracked people.
 *
 * @param color_image_msg : received sensor message
 */
void BodyTracker::imageCallback(const sensor_msgs::ImageConstPtr& color_image_msg)
{
	init_counter_color_image_++;

	cv_bridge::CvImageConstPtr cv_ptr;

	try
	{
		cv_ptr = cv_bridge::toCvShare(color_image_msg, sensor_msgs::image_encodings::BGR8);
	}catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("PeopleDetection: cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat color_image_ = cv_ptr->image;
	int height = color_image_msg->height;
	int width = color_image_msg->width;

	//if the list of tracked people not empty, proceed drawing
	if(!tracked_users_->empty())
	{
		for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin();
				iter_ != tracked_users_->end(); ++iter_)
		{
			if((*iter_).getCenterOfMass().x != 0 && (*iter_).getCenterOfMass().y != 0 && (*iter_).getCenterOfMass().z != 0)
			{
				int max_x = /*width - */(*iter_).getBoundingBox().max.x;
				int max_y = (*iter_).getBoundingBox().max.y;
				int min_x = /*width - */(*iter_).getBoundingBox().min.x;
				int min_y = (*iter_).getBoundingBox().min.y;

				double center_x = (*iter_).getCenterOfMass().x;
				double center_y = (*iter_).getCenterOfMass().y;

				int r = 255*Colors[(*iter_).getId() % colorCount][0];
				int g = 255*Colors[(*iter_).getId() % colorCount][1];
				int b = 255*Colors[(*iter_).getId() % colorCount][2];

				if(drawBoundingBox_)
				{
					cv::rectangle(color_image_, cv::Point(min_x, min_y), cv::Point(max_x, max_y), CV_RGB(r, g, b), 2);
				}

				if(drawCenterOfMass_)
				{
					cv::circle(color_image_, cv::Point(100, 100), 10, CV_RGB(r, g, b));
				}
				if(drawUserName_)
				{
					//drawUserName((*iter_), color_image, cv::Point(100, 100));
				}
			}
		}
	}
	cv::waitKey(10);
	// Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());
}

void BodyTracker::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& pointcloud)
{
	// dummy function
	init_counter_point_cloud_++;
}

BodyTracker::~BodyTracker()
{
	//shutdown node
	finalize();
}

void BodyTracker::finalize()
{
	ros::spinOnce();
	if (it_ != 0)
		delete it_;
	if (m_pTexMap_ != 0)
		delete[] m_pTexMap_;
	if (m_pUserTracker)
		delete m_pUserTracker;
	if (device_.isValid() == true)
		device_.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

/*
 * Initializes OpenNI and NiTE, loads a camera device and creates a video stream.
 */
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

/*
 * A loop function to track people.
 *
 * Firstly, it gets the first snapshot of a depth frame,
 * does the segmentation of the scene and starts the UserTracker algorithm
 * to detect possible users, which are saved in a UserMap. Each User gets specific
 * information(i.a. skeleton, position of joints, detected status).
 *
 * Secondly, it composes a Point Cloud according to the Usermap information
 * and publishes it on a given topic.
 *
 * At last, it goes through all possible users and puts the visible once
 * in the tracked_users_ list:
 * - If the user becomes visible/tracked, it is added to the list.
 * - If the user is lost, it is erased from the list.
 * - If the user is already in the list, its data is updated.
 *
 */
void BodyTracker::runTracker()
{
	while(shutdown_ ==false && nh_.ok() && ros::ok())
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
						//determine color for users
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
//					point.x = dZ/1000;		// todo: Kinect might be different from Asus?
//					point.y = -dX/1000;
//					point.z = dY/1000;
					point.x = dX/1000.0;
					point.y = -dY/1000.0;
					point.z = dZ/1000.0;

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
		//publish pcl
		drawPointCloud();

		//go through possible users (all that have been detected)
		const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
		for (int i = 0; i < users.getSize(); ++i)
		{
			const nite::UserData& user = users[i];
			updateUserState(user, userTrackerFrame.getTimestamp());

			if(user.isNew())
			{
				m_pUserTracker->startSkeletonTracking(user.getId());
				m_pUserTracker->startPoseDetection(user.getId(), nite::POSE_CROSSED_HANDS);
			}
			else if(!user.isLost() && users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				if (drawSkeleton_)
					drawSkeleton(m_pUserTracker, user);

				if(drawFrames_)
				{
					drawFrames(user);
				}

				double length = sqrt(users[i].getCenterOfMass().x) + sqrt(users[i].getCenterOfMass().y) +
						sqrt(users[i].getCenterOfMass().z);
				//if the user is valid (no empty information)
				if(length != 0)
				{
					//if the user in the list and valid, prepare list for update and
					//erase user
					bool found = false;
					for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin();
							iter_ != tracked_users_->end(); ++iter_)
					{
						if((*iter_).getId() == user.getId())
						{
							found = true;
							tracked_users_->erase(iter_++);
							//printf("erase: [");
							//for (list<nite::UserData>::iterator iter =tracked_users_->begin();iter !=tracked_users_->end(); ++iter) {
							//printf("%i ,", (unsigned int) (*iter).getId());
							//}
						}
					}
					//list waits for update
					if(found)
					{
						//save updated data for a user in the list
						tracked_users_->insert(tracked_users_->begin(), users[i]);
					}
					//if the user is valid and has not been in the list jet,
					//insert the user to the list
					if(!found || tracked_users_->empty())
					{
						tracked_users_->insert(tracked_users_->end(), users[i]);
					}
				}
			}
			else if(user.isLost())
			{
				//delete user out of the list
				for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin();
						iter_ != tracked_users_->end(); ++iter_)
				{
					if((*iter_).getId() ==  user.getId())
						tracked_users_->erase(iter_++);
				}
			}

			if (m_poseUser == 0 || m_poseUser == user.getId())
			{
				const nite::PoseData& pose = user.getPose(nite::POSE_CROSSED_HANDS);
				if (pose.isEntered())
				{
					// Start timer
					sprintf(generalMessage, "In exit pose. Keep it for %d second%s to exit\n", poseTimeoutToExit_/1000, poseTimeoutToExit_/1000 == 1 ? "" : "s");
					printf("Waiting to start timeout %i \n", poseTimeoutToExit_/1000);
					m_poseUser = user.getId();
					m_poseTime_ = userTrackerFrame.getTimestamp();
				}
				else if (pose.isExited())
				{
					memset(generalMessage, 0, sizeof(generalMessage));
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
		publishTrackedUserMsg();
	}
}
/*
 * Publishes user message [@cob_perception_msgs::Person] on the given topic of [@people_pub] publisher.
 */
void BodyTracker::publishTrackedUserMsg()
{
	//publish tracked users
	std::vector<cob_perception_msgs::Person> detected_people;
	for(std::list<nite::UserData>::iterator iter_ = tracked_users_->begin(); iter_ != tracked_users_->end(); ++iter_)
	{
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
		snprintf(id,100,"person %d", (*iter_).getId());
		string id_ = std::string(id);

		person.name = id_;
		person.detector = detector_;
		person.position.position.x = (*iter_).getCenterOfMass().x/1000.0;
		person.position.position.y = -(*iter_).getCenterOfMass().y/1000.0;
		person.position.position.z = (*iter_).getCenterOfMass().z/1000.0;

		person.skeleton = skeleton;

		unsigned int length = sqrt(person.position.position.x) + sqrt(person.position.position.y) +
				sqrt(person.position.position.z);

		//save only valid data and no empty points
		if(length != 0)
			detected_people.push_back(person);
	}
	cob_perception_msgs::People array;
	array.header.stamp = ros::Time::now();
	array.header.frame_id = depth_optical_frame_;
	array.people = detected_people;
	people_pub_.publish(array);

}
/*
 * Updates and prints the user information.
 */
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

/*
 * Publishes joints on TF.
 */
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
		//frame_id_stream << "user_" << id << "-" << joint_name;
		frame_id = frame_id_stream.str();
		// std::cout << frame_id << std::endl;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), rel_frame, frame_id));
	}
}

/*
 * Converts a nite message to a geometry message.
 */
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

/*
 * Draws a Point Cloud. Resets the points of a [@pcl_cloud].
 */
void BodyTracker::drawPointCloud()
{
	sensor_msgs::PointCloud2 pc;
	pcl::toROSMsg(*pcl_cloud_, pc);

	ros::Time time = ros::Time::now();
	//uint64_t st = time.toNSec();
	pc.header.stamp = time;
	pc.header.frame_id = depth_optical_frame_;

	pcl_pub_.publish(pc);
	pcl_cloud_->points.clear();
}
/*
 * Draws the limbs of a person on an image view.
 */
void BodyTracker::drawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color)
{
	//TO DO: compose limbs for drawing function
}
/*
 * Draws a skeleton on an image view.
 */
void BodyTracker::drawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData)
{
	//TO DO: draw skeleton on image view
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

/*
 * Writes down a label on an image view
 */
void BodyTracker::drawUserName(const nite::UserData& user, cv::Mat& color_image, cv::Point& tag_coords)
{
	//
	//	std::string tag_label = "User" + user.getId();
	//	std::cout << "The text tag reads: " << tag_label << "." << std::endl;
	//	cv::putText(color_image, tag_label, tag_coords, cv::FONT_HERSHEY_PLAIN, 3, CV_RGB(0,0,255), 2);

	//TO DO: write down a label on image view

}
/*
 * Publishes frames and joints markers on TF.
 */
void BodyTracker::drawFrames(const nite::UserData& user)
{
	int r = 255*Colors[user.getId() % colorCount][0];
	int g = 255*Colors[user.getId() % colorCount][1];
	int b = 255*Colors[user.getId() % colorCount][2];

	JointMap joints;
	joints["head"] = (user.getSkeleton().getJoint(nite::JOINT_HEAD));
	joints["neck"] = (user.getSkeleton().getJoint(nite::JOINT_NECK));
	joints["left_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER));
	joints["right_shoulder"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER));
	joints["left_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW));
	joints["right_elbow"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW));
	joints["left_hand"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND));
	joints["right_hand"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND));
	joints["torso"] = (user.getSkeleton().getJoint(nite::JOINT_TORSO));
	joints["left_hip"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP));
	joints["right_hip"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP));
	joints["left_knee"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE));
	joints["right_knee"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE));
	joints["left_foot"] = (user.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT));
	joints["right_foot"] = (user.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT));

	for (JointMap::iterator it=joints.begin(); it!=joints.end(); ++it){
		publishJoints(nh_, transform_broadcaster_, it->first, it->second, tf_prefix_, depth_optical_frame_, user.getId());
	}

	drawCircle(r, g, b, joints["head"].getPosition());
	drawCircle(r, g, b, joints["right_hand"].getPosition());
	drawCircle(r, g, b, joints["left_hand"].getPosition());
	drawCircle(r, g, b, joints["right_foot"].getPosition());
	drawCircle(r, g, b, joints["left_foot"].getPosition());

	drawLine(r, g, b, joints["head"].getPosition(), joints["neck"].getPosition());
	drawLine(r, g, b, joints["neck"].getPosition(), joints["left_shoulder"].getPosition());
	drawLine(r, g, b, joints["neck"].getPosition(), joints["right_shoulder"].getPosition());
	drawLine(r, g, b, joints["right_shoulder"].getPosition(), joints["right_elbow"].getPosition());
	drawLine(r, g, b, joints["left_shoulder"].getPosition(), joints["left_elbow"].getPosition());
	drawLine(r, g, b, joints["right_elbow"].getPosition(), joints["right_hand"].getPosition());
	drawLine(r, g, b, joints["left_elbow"].getPosition(), joints["left_hand"].getPosition());
	drawLine(r, g, b, joints["neck"].getPosition(), joints["torso"].getPosition());
	drawLine(r, g, b, joints["torso"].getPosition(), joints["left_hip"].getPosition());
	drawLine(r, g, b, joints["torso"].getPosition(), joints["right_hip"].getPosition());
	drawLine(r, g, b, joints["right_hip"].getPosition(), joints["right_knee"].getPosition());
	drawLine(r, g, b, joints["left_hip"].getPosition(), joints["left_knee"].getPosition());
	drawLine(r, g, b, joints["right_knee"].getPosition(), joints["right_foot"].getPosition());
	drawLine(r, g, b, joints["left_knee"].getPosition(), joints["left_foot"].getPosition());

}

/**
 * A helper function to draw a simple line in rviz.
 */
void BodyTracker::drawLine (const double r, const double g, const double b,
		const nite::Point3f& pose_start, const nite::Point3f& pose_end )
{
	visualization_msgs::Marker marker;
	marker.header.frame_id = depth_optical_frame_;
	marker.header.stamp = ros::Time::now();
	marker.action = visualization_msgs::Marker::ADD;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	geometry_msgs::Point p;
	marker.id = marker_id_;
	p.x = pose_start.x/1000.0;
	p.y = -pose_start.y/1000.0;
	p.z = pose_start.z/1000.0;
	marker.points.push_back(p);
	p.x =pose_end.x/1000.0;
	p.y = pose_end.y/1000.0;
	p.z = pose_end.z/1000.0;
	marker.points.push_back(p);
	marker.scale.x = 0.019;
	marker.scale.y = 0.0;
	marker.scale.z = 0.0;
	marker.color.r = r;
	marker.color.g = g;
	marker.color.b = b;
	marker.lifetime = ros::Duration(0.01);
	vis_pub_.publish(marker);
	marker_id_++;
}

/*
 * A helper function to draw circles in rviz
 */

void BodyTracker::drawCircle(const double r, const double g, const double b,
		const nite::Point3f& pose){
	visualization_msgs::Marker m;
	m.header.stamp = ros::Time::now();
	m.action = visualization_msgs::Marker::ADD;
	m.header.frame_id = "/camra_link";
	m.type = m.SPHERE;
	m.id = marker_id_;
	m.pose.position.x = pose.x/1000.0;
	m.pose.position.y = -pose.y/1000.0;
	m.pose.position.z = pose.z/1000.0;
	m.scale.x = .1;
	m.scale.y = .1;
	m.scale.z = .1;
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
