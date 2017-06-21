#include <iostream>
#include <iomanip>
#include <fstream>

#include <opencv2/opencv.hpp>

#include <pcl/common/transform.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

class scene_publisher
{
public:

	// Constructor
	scene_publisher()
	{
		this->persp = 1;
		this->shot = 0;

		file = "-1";
		n_.param("/cob_people_detection/face_recognizer/file", file, file);
		std::cout << "input file: " << file << std::endl;

		scene_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1);
		img_pub_ = n_.advertise<sensor_msgs::Image>("/camera/rgb/image_color", 1);

	}

	// Destructor
	~scene_publisher()
	{
	}

	void process()
	{

		pc.clear();

		std::stringstream xml_stream, jpg_stream;
		xml_stream << file.c_str() << "/" << std::setw(3) << std::setfill('0') << persp << "_" << shot << "_d.xml";
		jpg_stream << file.c_str() << "/" << std::setw(3) << std::setfill('0') << persp << "_" << shot << "_c.bmp";

		std::cout << xml_stream.str() << " " << jpg_stream.str() << std::endl;

		//Load depth map
		cv::Mat dm = cv::Mat(640, 480, CV_64FC1);
		std::cout << xml_stream.str().c_str() << std::endl;
		std::cout << jpg_stream.str().c_str() << std::endl;
		cv::FileStorage fs(xml_stream.str().c_str(), cv::FileStorage::READ);
		fs["depthmap"] >> dm;
		fs.release();
		cv::Mat dm_cp, dm_cp_roi;
		//  dm.convertTo(dm_cp,CV_8UC1,255);
		//  cv::Rect roi=cv::Rect(320-100,240-100,200,200);
		//  cv::equalizeHist(dm_cp(roi),dm_cp_roi);
		//  cv::imwrite("/share/goa-tz/people_detection/debug/img_depth.jpg",dm_cp);
		// imshow("DM",dm_cp_roi);
		// cv::waitKey(10);


		//Load color map
		cv::Mat img;
		img = cv::imread(jpg_stream.str().c_str());
		img.convertTo(img, CV_8UC3);
		cv::resize(img, img, cv::Size(640, 480));
		//  cv::Mat img_cp,img_cp_roi;
		//  cv::cvtColor(img,img_cp,CV_RGB2GRAY);
		//  cv::equalizeHist(img_cp(roi),img_cp_roi);
		//  cv::imwrite("/share/goa-tz/people_detection/debug/img_rgb.jpg",img_cp);


		std::cout << "calculcating" << std::endl;

		//calculate pointcloud from depthmap

		pc.width = 640;
		pc.height = 480;
		cv::Mat cam_mat = (cv::Mat_<double>(3, 3) << 524.90160178307269, 0.0, 320.13543361773458, 0.0, 525.85226379335393, 240.73474482242005, 0.0, 0.0, 1.0);
		// compensate for kinect offset
		int index = 0;
		cv::Vec3f rvec, tvec;
		cv::Mat dist_coeff;

		tvec = cv::Vec3f(0.03, 0.0, 0.0);
		rvec = cv::Vec3f(0.0, 0.0, 0);
		dist_coeff = cv::Mat::zeros(1, 5, CV_32FC1);
		cv::Mat pc_trans = cv::Mat::zeros(640, 480, CV_64FC1);
		for (int r = 0; r < dm.rows; r++)
		{
			for (int c = 0; c < dm.cols; c++)
			{
				//pt  << c,r,1.0/525;
				//pt=cam_mat_inv*pt;

				cv::Point3f pt;
				pt.x = (c - 320);
				pt.y = (r - 240);
				pt.z = 525;

				double nrm = cv::norm(pt);
				pt.x /= nrm;
				pt.y /= nrm;
				pt.z /= nrm;

				pt = pt * dm.at<double>(r, c);
				std::vector<cv::Point3f> pt3_vec;
				pt3_vec.push_back(pt);
				std::vector<cv::Point2f> pt2_vec;

				cv::projectPoints(pt3_vec, rvec, tvec, cam_mat, dist_coeff, pt2_vec);

				int x_t, y_t;
				x_t = round(pt2_vec[0].x);
				y_t = round(pt2_vec[0].y);
				if (x_t < 0 || x_t > 640 || y_t < 0 || y_t > 480)
					continue;
				pc_trans.at<double>(y_t, x_t) = dm.at<double>(r, c);

			}
		}

		for (int ra = 0; ra < dm.rows; ra++)
		{
			for (int ca = 0; ca < dm.cols; ca++)
			{
				cv::Point3f pt;
				pt.x = (ca - 320);
				pt.y = (ra - 240);
				pt.z = 525;

				double nrm = cv::norm(pt);
				pt.x /= nrm;
				pt.y /= nrm;
				pt.z /= nrm;

				pt = pt * pc_trans.at<double>(ra, ca);

				pcl::PointXYZRGB point;
				point.x = (float)pt.x;
				point.y = (float)pt.y;
				point.z = (float)pt.z;

				uint32_t rgb = (static_cast<uint32_t>(img.at<cv::Vec3b>(ra, ca)[0]) << 16 | static_cast<uint32_t>(img.at<cv::Vec3b>(ra, ca)[1]) << 8
						| static_cast<uint32_t>(img.at<cv::Vec3b>(ra, ca)[2]));
				point.rgb = *reinterpret_cast<float*>(&rgb);
				pc.points.push_back(point);

				//pc.points[index].x=pt[0];
				//pc.points[index].y=pt[1];
				//pc.points[index].z=pt[2];
				//uint8_t r_c =  img.at<cv::Vec3b>(r,c)[0];
				//uint8_t g_c =  img.at<cv::Vec3b>(r,c)[1];
				//uint8_t b_c =  img.at<cv::Vec3b>(r,c)[2];
				//int32_t rgb_val = (r_c << 16) | (g_c << 8) | b_c;
				//pc.points[index].rgb = *(float *)(&rgb_val);
				index++;
			}
		}

		cv_bridge::CvImage cv_ptr;
		cv_ptr.image = img;
		cv_ptr.encoding = sensor_msgs::image_encodings::BGR8;
		out_img = *(cv_ptr.toImageMsg());

		pcl::toROSMsg(pc, out_pc2);

	}

	void publish()
	{
		out_pc2.header.frame_id = "/camera/";
		out_pc2.header.stamp = ros::Time::now();
		scene_pub_.publish(out_pc2);

		out_img.header.frame_id = "/camera/";
		out_img.header.stamp = ros::Time::now();
		img_pub_.publish(out_img);

		//debug output
		//  dm.convertTo(dm,CV_8UC1,255);
		//  cv::imshow("dm_raw",dm);
		//  cv::waitKey(0);
		//  img.convertTo(dm,CV_8UC3);
		//  cv::imshow("img_raw",img);
		//  cv::waitKey(0);


	}

	ros::NodeHandle n_;
	int persp;
	int shot;

protected:
	ros::Publisher scene_pub_;
	ros::Publisher img_pub_;
	sensor_msgs::PointCloud2 out_pc2;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	sensor_msgs::Image out_img;
	std::string file;

};

int main(int argc, char** argv)
{

	ros::init(argc, argv, "scene publisher");
	int persp_filter = 17;

	scene_publisher sp;

	ros::Rate loop_rate(1);
	while (ros::ok())
	{
		{
			sp.shot++;
			std::cout << "PERSPECTIVE:" << sp.persp << std::endl;
			if (sp.shot == 4)
			{
				sp.shot = 1;
				sp.persp++;
			}
			if (sp.persp == 18)
				break;

			if (sp.persp == persp_filter)
			{
				sp.process();
				sp.publish();
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
	}
}
