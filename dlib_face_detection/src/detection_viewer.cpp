#include <dlib_face_detection/detection_viewer.h>

using namespace ipa_PeopleDetector;

DetectionViewer::DetectionViewer(ros::NodeHandle nh)
{
	image_sub_ = new message_filters::Subscriber<image_msgT>(nh, "image_in", 10);
	head_sub_ = new message_filters::Subscriber<det_msgT>(nh, "/people_detections/head_detections", 10);
	face_sub_ = new message_filters::Subscriber<det_msgT>(nh, "/people_detections/face_detections", 10);

	sync_ = new message_filters::Synchronizer<SyncPolicy>(SyncPolicy(30), *image_sub_, *head_sub_, *face_sub_);
	sync_->registerCallback(boost::bind(&DetectionViewer::callback, this, _1, _2, _3));

	cv::namedWindow("dlib Detections", cv::WINDOW_AUTOSIZE );
	cv::startWindowThread();
}

DetectionViewer::~DetectionViewer()
{
}

void DetectionViewer::callback(const sensor_msgs::Image::ConstPtr &image_msg,
						const cob_perception_msgs::DetectionArray::ConstPtr& head_msg,
						const cob_perception_msgs::DetectionArray::ConstPtr& face_msg)
{
	cv_bridge::CvImageConstPtr color_image_ptr;
	cv::Mat color_image;

	// convert image_msg to cv::Mat
	color_image_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
	color_image = color_image_ptr->image;

	for (int i = 0; i < face_msg->detections.size(); ++i)
	{
		cob_perception_msgs::Detection det = face_msg->detections[i];
		cv::rectangle(color_image, cv::Point(det.mask.roi.x+5, det.mask.roi.y +5), cv::Point(det.mask.roi.x + det.mask.roi.width -5, det.mask.roi.y + det.mask.roi.height -5), CV_RGB(0, 255, 0), 2);
	}

	for(int i = 0; i < head_msg->detections.size(); ++i)
	{
		cob_perception_msgs::Detection det = head_msg->detections[i];
		cv::rectangle(color_image, cv::Point(det.mask.roi.x, det.mask.roi.y), cv::Point(det.mask.roi.x + det.mask.roi.width, det.mask.roi.y + det.mask.roi.height), CV_RGB(0, 0, 255), 2);
	}


	cv::imshow("dlib Detections", color_image);

}

int main(int argc, char** argv) {
	ros::init(argc, argv, "dlib_detection_viewer");

	ros::NodeHandle nh("~");
	DetectionViewer det_viewer(nh);

	ros::spin();

	return 0;
}
