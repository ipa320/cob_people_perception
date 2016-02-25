
#include <dlib_face_detection/face_detection_merge.h>

using namespace ipa_PeopleDetector;

FaceDetectionMerge::FaceDetectionMerge(ros::NodeHandle nh)
{
	node_handle_.param("enable_tf", enable_tf_, false);

	cloud_sub_ = new message_filters::Subscriber<cloud_msgT>(nh, "pointcloud_in", 10);
	direct_face_sub_ = new message_filters::Subscriber<rect_msgT>(nh, "/dlib_face_detection/direct_face_detections", 10);
	head_based_face_sub_ = new message_filters::Subscriber<rect_msgT>(nh, "/dlib_face_detection/head_based_face_detections", 10);

	sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(60), *cloud_sub_, *direct_face_sub_, *head_based_face_sub_);
	sync_->registerCallback(boost::bind(&FaceDetectionMerge::callback, this, _1, _2, _3));

	face_pub_ = node_handle_.advertise<cob_perception_msgs::DetectionArray>("/people_detections/face_detections", 1);

}

FaceDetectionMerge::~FaceDetectionMerge()
{

}

void FaceDetectionMerge::callback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
						const dlib_face_detection::RectArray::ConstPtr& dFace_msg,
						const dlib_face_detection::RectArray::ConstPtr& hbFace_msg)
{
	 // convert cloud_msg to pcl
	PointCloudT::Ptr cloud(new PointCloudT), cloud_transformed(new PointCloudT);
	pcl::fromROSMsg(*cloud_msg, *cloud);

	cob_perception_msgs::DetectionArray::Ptr detArray (new cob_perception_msgs::DetectionArray);
	detArray->header = cloud_msg->header;

	cob_perception_msgs::Detection detection;

	detection.header = cloud_msg->header;
	detection.pose.header = cloud_msg->header;
	detection.detector = "face";

	std::list<PointT> point_list;

	for (int i = 0; i < dFace_msg->rects.size(); ++i)
	{
		int x, y;
		// center of the detected face
		x = dFace_msg->rects[i].x + dFace_msg->rects[i].width/2;
		y = dFace_msg->rects[i].y + dFace_msg->rects[i].height/2;

		PointT point = cloud->at(x,y);
		point_list.push_back(point);

		detection.mask.roi.x = dFace_msg->rects[i].x;
		detection.mask.roi.y = dFace_msg->rects[i].y;
		detection.mask.roi.width = dFace_msg->rects[i].width;
		detection.mask.roi.height = dFace_msg->rects[i].height;

		detection.pose.pose.position.x = point.x;
		detection.pose.pose.position.y = point.y;
		detection.pose.pose.position.z = point.z;
		detection.pose.pose.orientation.x = 0;
		detection.pose.pose.orientation.y = 0;
		detection.pose.pose.orientation.z = 0;
		detection.pose.pose.orientation.w = 1;

		detArray->detections.push_back(detection);
	}


	for (int i = 0; i < hbFace_msg->rects.size(); ++i)
	{
		int x, y;
		// center of the detected face
		x = hbFace_msg->rects[i].x + hbFace_msg->rects[i].width/2;
		y = hbFace_msg->rects[i].y + hbFace_msg->rects[i].height/2;

		PointT point = cloud->at(x,y);

		bool det_flag = false;
		// check if the face was detected previously by direct face detection
		if (point_list.size() == 0)
			det_flag = true;
		else
		{
			std::list<PointT>::iterator it = point_list.begin();
			for (int i = 0; i < point_list.size(); ++i)
			{
				if (abs(point.x - it->x) > 0.1 && abs(point.z - it->z) > 0.1)
					det_flag = true;
				else
				{
					det_flag = false;
					break;
				}
				++it;
			}
		}

		if(det_flag)
		{

			PointT point = cloud->at(x,y);

			detection.mask.roi.x = hbFace_msg->rects[i].x;
			detection.mask.roi.y = hbFace_msg->rects[i].y;
			detection.mask.roi.width = hbFace_msg->rects[i].width;
			detection.mask.roi.height = hbFace_msg->rects[i].height;

			detection.pose.pose.position.x = point.x;
			detection.pose.pose.position.y = point.y;
			detection.pose.pose.position.z = point.z;
			detection.pose.pose.orientation.x = 0;
			detection.pose.pose.orientation.y = 0;
			detection.pose.pose.orientation.z = 0;
			detection.pose.pose.orientation.w = 1;

			detArray->detections.push_back(detection);
		}

	}

	face_pub_.publish(detArray);

	if (enable_tf_ )
		tfpublisher(detArray);

}

void FaceDetectionMerge::tfpublisher(const cob_perception_msgs::DetectionArray::ConstPtr detArray)
{
	geometry_msgs::PoseStamped pose_world;

	int k = 0;

	for (int i = 0; i < detArray->detections.size(); i++)
	{
		std::stringstream s;
		k += 1;
		s << "dlib_face_" << k;

		int a = 0;
		while (a < 5)
		{
			// transform pose to "/odom_combined" (world coordinates)
			try
			{
				ros::Time now = ros::Time(0);
				tfl_.waitForTransform("/odom_combined", detArray->header.frame_id, now, ros::Duration(0.5));
				tfl_.transformPose("/odom_combined", now, detArray->detections[i].pose, "/odom_combined", pose_world);
			}
			catch(tf::TransformException &e)
			{
				a++;
				ROS_ERROR(" %s", e.what());
				ros::Duration(0.2).sleep();
				continue;
			}

			pose_world.pose.orientation.x = 0;
			pose_world.pose.orientation.y = 0;
			pose_world.pose.orientation.z = 0;
			pose_world.pose.orientation.w = 1;

			break;
		}

		 // publish tf
		 transform_.setOrigin(tf::Vector3(pose_world.pose.position.x, pose_world.pose.position.y, pose_world.pose.position.z));
		 tf::Quaternion q;
		 q.setRPY(pose_world.pose.orientation.x, pose_world.pose.orientation.y, pose_world.pose.orientation.z);
		 transform_.setRotation(q);

		 tfb_.sendTransform(tf::StampedTransform(transform_, pose_world.header.stamp, pose_world.header.frame_id, s.str()));
	}

}


int main(int argc, char** argv) {
	ros::init(argc, argv, "dlib_face_detection");

	ros::NodeHandle nh("~");

	FaceDetectionMerge face_detection_merge(nh);

	ros::spin();

	return 0;
}

