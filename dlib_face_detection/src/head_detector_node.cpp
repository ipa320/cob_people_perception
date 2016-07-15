#include "dlib_face_detection/head_detector_node.h"

using namespace ipa_PeopleDetector;

HeadDetectorNode::HeadDetectorNode(ros::NodeHandle nh) :
	node_handle_(nh)
{
	data_directory_ = ros::package::getPath("cob_people_detection") + "/common/files/";

	// Parameters
	double depth_increase_search_scale; // The factor by which the search window is scaled between the subsequent scans
	int depth_drop_groups; // Minimum number (minus 1) of neighbor rectangles that makes up an object.
	int depth_min_search_scale_x; // Minimum search scale x
	int depth_min_search_scale_y; // Minimum search scale y

	node_handle_.param("data_directory", data_directory_, data_directory_);
	node_handle_.param("fill_unassigned_depth_values", fill_unassigned_depth_values_, true);
	node_handle_.param("depth_increase_search_scale", depth_increase_search_scale, 1.1);
	node_handle_.param("depth_drop_groups", depth_drop_groups, 68);
	node_handle_.param("depth_min_search_scale_x", depth_min_search_scale_x, 20);
	node_handle_.param("depth_min_search_scale_y", depth_min_search_scale_y, 20);
	node_handle_.param("enable_tf", enable_tf_, false);

	// initialize head detector
	head_detector_.init(data_directory_, depth_increase_search_scale, depth_drop_groups, depth_min_search_scale_x, depth_min_search_scale_y);

	// advertise topics
	head_pub_ = node_handle_.advertise<cob_perception_msgs::DetectionArray>("/people_detections/head_detections", 1);
	head_img_pub = node_handle_.advertise<cob_perception_msgs::ColorDepthImageArray>("/dlib_face_detection/head_detections", 1);

	// subscribe to sensor topic
	pointcloud_sub_ = nh.subscribe("pointcloud_in", 1, &HeadDetectorNode::pointcloudCallback, this);

	std::cout << "HeadDetectorNode initialized." << std::endl;

}


HeadDetectorNode::~HeadDetectorNode(void)
{
}

void HeadDetectorNode::tfPublisher(geometry_msgs::PoseStamped pose, std::string label)
{
	 transform_.setOrigin(tf::Vector3(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z));
	 tf::Quaternion q;
	 q.setRPY(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z);
	 transform_.setRotation(q);

	 tfb_.sendTransform(tf::StampedTransform(transform_, pose.header.stamp, pose.header.frame_id, label));
}


void HeadDetectorNode::pointcloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
	// convert incoming colored point cloud to cv::Mat images
	cv::Mat depth_image, color_image;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr depth_cloud (new pcl::PointCloud<pcl::PointXYZRGB>); // point cloud
	pcl::fromROSMsg(*cloud_msg, *depth_cloud);

	convertPclMessageToMat(depth_cloud, depth_image, color_image);

	// detect heads in the depth image
	std::vector<cv::Rect> head_bounding_boxes;
	head_detector_.detectRangeFace(depth_image, head_bounding_boxes, fill_unassigned_depth_values_);

	// publish head detections
	cob_perception_msgs::DetectionArray::Ptr detArray  (new cob_perception_msgs::DetectionArray);
	cob_perception_msgs::Detection det;

	detArray->header = cloud_msg->header;
	det.header = cloud_msg->header;
	det.pose.header = cloud_msg->header;
	det.detector = "head";

	cob_perception_msgs::ColorDepthImageArray::Ptr headImgArray (new cob_perception_msgs::ColorDepthImageArray);
	cob_perception_msgs::ColorDepthImage head_img;
	headImgArray->header = cloud_msg->header;

	cv_bridge::CvImage cv_ptr;

	int i = 0;
	for (int i = 0; i < head_bounding_boxes.size(); i++)
	{
		std::stringstream s;
		s << "dlib_head_" << i;
		det.label = s.str();

		PointT point = depth_cloud->at(head_bounding_boxes[i].x + head_bounding_boxes[i].width/2, head_bounding_boxes[i].y + head_bounding_boxes[i].height/2);

		det.pose.pose.position.x = point.x;
		det.pose.pose.position.y = point.y;
		det.pose.pose.position.z = point.z;

		det.pose.pose.orientation.x = 0;
		det.pose.pose.orientation.y = 0;
		det.pose.pose.orientation.z = 0;
		det.pose.pose.orientation.w = 1;

		det.mask.roi.x = head_bounding_boxes[i].x;
		det.mask.roi.y = head_bounding_boxes[i].y;
		det.mask.roi.width = head_bounding_boxes[i].width;
		det.mask.roi.height = head_bounding_boxes[i].height;

		detArray->detections.push_back(det);

		head_img.head_detection.x = head_bounding_boxes[i].x;
		head_img.head_detection.y = head_bounding_boxes[i].y;
		head_img.head_detection.width = head_bounding_boxes[i].width;
		head_img.head_detection.height = head_bounding_boxes[i].height;

		cv::Mat color_patch = color_image(head_bounding_boxes[i]);
		cv_ptr.image = color_patch;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		head_img.color_image = *(cv_ptr.toImageMsg());

		headImgArray->head_detections.push_back(head_img);

		if (enable_tf_)
			tfPublisher(det.pose, det.label);
	}

	head_pub_.publish(detArray);
	head_img_pub.publish(headImgArray);

}

void HeadDetectorNode::convertPclMessageToMat(PointCloudT::Ptr& depth_cloud, cv::Mat& depth_image, cv::Mat& color_image)
{
	depth_image.create(depth_cloud->height, depth_cloud->width, CV_32FC3);
	color_image.create(depth_cloud->height, depth_cloud->width, CV_8UC3);
	uchar* depth_image_ptr = (uchar*)depth_image.data;
	uchar* color_image_ptr = (uchar*)color_image.data;
	for (int v = 0; v < (int)depth_cloud->height; v++)
	{
		int depth_base_index = depth_image.step * v;
		int color_base_index = color_image.step * v;
		for (int u = 0; u < (int)depth_cloud->width; u++)
		{
			int depth_index = depth_base_index + 3 * u * sizeof(float);
			float* depth_data_ptr = (float*)(depth_image_ptr + depth_index);
			int color_index = color_base_index + 3 * u * sizeof(uchar);
			uchar* color_data_ptr = (uchar*)(color_image_ptr + color_index);
			pcl::PointXYZRGB point_xyz = depth_cloud->at(u, v);
			depth_data_ptr[0] = point_xyz.x;
			depth_data_ptr[1] = point_xyz.y;
			depth_data_ptr[2] = (isnan(point_xyz.z)) ? 0.f : point_xyz.z;
			color_data_ptr[0] = point_xyz.r;
			color_data_ptr[1] = point_xyz.g;
			color_data_ptr[2] = point_xyz.b;
		}
	}
}

//#######################
//#### main programm ####
int main(int argc, char** argv)
{
	// Initialize ROS, specify name of node
	ros::init(argc, argv, "head_detector");

	// Create a handle for this node, initialize node
	ros::NodeHandle nh;

	// Create HeadDetectorNode class instance
	HeadDetectorNode head_detector_node(nh);

	ros::spin();

	return 0;
}
