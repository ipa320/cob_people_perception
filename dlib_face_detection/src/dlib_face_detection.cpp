
#include <dlib_face_detection/dlib_face_detection.h>

using namespace ipa_PeopleDetector;

FaceDetectorNode::FaceDetectorNode(ros::NodeHandle nh)
{
	dlib::deserialize(ros::package::getPath("dlib_face_detection") + "/data/shape_predictor_68_face_landmarks.dat") >> pose_model_;

	node_handle_.param("scale_factor", scale_factor_, 2.0);

	detector_ = dlib::get_frontal_face_detector();

	head_img_sub_ = nh.subscribe<cob_perception_msgs::ColorDepthImageArray>("/dlib_face_detection/head_detections", 2,  &FaceDetectorNode::headCallback, this);
	image_sub_ = nh.subscribe<sensor_msgs::Image>("image_in", 2, &FaceDetectorNode::imgCallback, this);

	direct_face_pub_ = node_handle_.advertise<dlib_face_detection::RectArray>("/dlib_face_detection/direct_face_detections", 1);

	head_based_face_pub_ = node_handle_.advertise<dlib_face_detection::RectArray>("/dlib_face_detection/head_based_face_detections", 1);
}

FaceDetectorNode::~FaceDetectorNode()
{
}


// Prevent deleting memory twice, when using smart pointer
void voidDeleter(const sensor_msgs::Image* const )
{
}

void FaceDetectorNode::imgCallback(const sensor_msgs::Image::ConstPtr &image_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;

	// convert image_msg to cv::Mat
	color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	color_image = color_image_ptr->image;

	//convert cv::Mat to dlib format
	dlib::cv_image<dlib::bgr_pixel> cimg(color_image);

	std::vector<dlib::rectangle> dets = detector_(cimg);

	dlib_face_detection::RectArray::Ptr rectArray (new dlib_face_detection::RectArray);
	rectArray->header = image_msg->header;
	dlib_face_detection::Rect rect;

	for (int i = 0; i < dets.size(); ++i)
	{
		rect.x = dets[i].left();
		rect.y = dets[i].top();
		rect.width = dets[i].right()-dets[i].left();
		rect.height = dets[i].bottom()-dets[i].top();

		rectArray->rects.push_back(rect);
	}

	direct_face_pub_.publish(rectArray);
}


void FaceDetectorNode::headCallback(const cob_perception_msgs::ColorDepthImageArray::ConstPtr& head_img_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	sensor_msgs::ImageConstPtr msgPtr;
	cv::Mat head_img, head_img_scaled;

	dlib_face_detection::RectArray::Ptr rectArray (new dlib_face_detection::RectArray);
	rectArray->header = head_img_msg->header;
	dlib_face_detection::Rect rect;

	// use images from head detector to detect smaller faces
	for (int j = 0; j < head_img_msg->head_detections.size(); ++j)
	{
		//if (head_img_msg->head_detections[j].color_image.height > 140 || head_img_msg->head_detections[j].color_image.width > 140)
		//	continue;

		msgPtr = boost::shared_ptr<sensor_msgs::Image const>(&(head_img_msg->head_detections[j].color_image), voidDeleter);
		color_image_ptr = cv_bridge::toCvShare(msgPtr, sensor_msgs::image_encodings::BGR8);
		head_img = color_image_ptr->image.clone();

		cv::resize(head_img, head_img_scaled, cv::Size(), scale_factor_, scale_factor_, cv::INTER_LINEAR);

		dlib::cv_image<dlib::bgr_pixel> himg(head_img_scaled);

		std::vector<dlib::rectangle> dets = detector_(himg);

		if (dets.size() > 0)
		{
			rect.x = head_img_msg->head_detections[j].head_detection.x;
			rect.y = head_img_msg->head_detections[j].head_detection.y;
			rect.width = head_img_msg->head_detections[j].head_detection.width;
			rect.height = head_img_msg->head_detections[j].head_detection.height;

			rectArray->rects.push_back(rect);
		}
	}

	head_based_face_pub_.publish(rectArray);
}


int main(int argc, char** argv) {
	ros::init(argc, argv, "dlib_face_detection");

	ros::NodeHandle nh("~");

	FaceDetectorNode face_detector_node(nh);

	ros::spin();

	return 0;
}
