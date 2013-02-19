#include<iostream>
#include<fstream>

#include<opencv/cv.h>
#include<opencv/highgui.h>

#include<pcl/common/transform.h>
#include<pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

#include<sensor_msgs/PointCloud2.h>
#include<sensor_msgs/Image.h>
#include<sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
class scene_publisher
{
public:

	// Constructor
	scene_publisher()
	{

    file="-1";
    n_.param("/cob_people_detection/face_recognizer/file",file,file);
    std::cout<<"input file: "<<file<<std::endl;


		scene_pub_ = n_.advertise<sensor_msgs::PointCloud2>("/camera/depth_registered/points",1);
		img_pub_ = n_.advertise<sensor_msgs::Image>("/camera/rgb/image_color",1);

	}

	// Destructor
	~scene_publisher()
	{
	}


void process()
{


  std::stringstream xml_stream,jpg_stream;
  xml_stream<<file.c_str()<<"d.xml";
  jpg_stream<<file.c_str()<<"c.bmp";


  //Load depth map
  cv::Mat dm=cv::Mat(640,480,CV_64FC1);
  std::cout<<xml_stream.str().c_str()<<std::endl;
  std::cout<<jpg_stream.str().c_str()<<std::endl;
  cv::FileStorage fs(xml_stream.str().c_str(),cv::FileStorage::READ);
  fs["depthmap"]>> dm;
  fs.release();


  //Load color map
  cv::Mat img=cv::Mat(1280,960,CV_16UC3);
  img=cv::imread(jpg_stream.str().c_str());
  img.convertTo(img,CV_8UC3);
  cv::resize(img,img,cv::Size(640,480));



    std::cout<<"calculcating"<<std::endl;

  //calculate pointcloud from depthmap

  pc.width=640;
  pc.height=480;
  Eigen::Matrix3f cam_mat;
  Eigen::Vector3f pt;
  cam_mat << 524.90160178307269,0.0,312.13543361773458,0.0,525.85226379335393,254.73474482242005,0.0,0.0,1.0;
  Eigen::Matrix3f cam_mat_inv=cam_mat.inverse();
  int index=0;
  for(int r=0;r<dm.rows;r++)
  {
    for(int c=0;c<dm.cols;c++)
    {
    pt  << c,r,1.0;
    pt=cam_mat_inv*pt;
    pt[2]=dm.at<double>(r,c);

    pcl::PointXYZRGB point;
    point.x=(float)pt[0];
    point.y=(float)pt[1];
    point.z=(float)pt[2];
    uint32_t rgb = (static_cast<uint32_t>(img.at<cv::Vec3b>(r,c)[0]) << 16 |static_cast<uint32_t>(img.at<cv::Vec3b>(r,c)[1]) << 8 | static_cast<uint32_t>(img.at<cv::Vec3b>(r,c)[2]));
    point.rgb = *reinterpret_cast<float*>(&rgb);
    pc.points.push_back (point);

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

  pcl::toROSMsg(pc,out_pc2);

}

  void publish()
{
  out_pc2.header.frame_id="/camera/";
  out_pc2.header.stamp = ros::Time::now();
  scene_pub_.publish(out_pc2);

  out_img.header.frame_id="/camera/";
  out_img.header.stamp = ros::Time::now();
  img_pub_.publish(out_img);


  ////debug output
  //dm.convertTo(dm,CV_8UC1,255);
  //cv::imshow("dm_raw",dm);
  //cv::waitKey(0);
  //img.convertTo(dm,CV_8UC3);
  //cv::imshow("img_raw",img);
  //cv::waitKey(0);


}

	ros::NodeHandle n_;


protected:
	ros::Publisher scene_pub_;
	ros::Publisher img_pub_;
  sensor_msgs::PointCloud2 out_pc2;
  pcl::PointCloud<pcl::PointXYZRGB> pc;
  sensor_msgs::Image out_img;
  std::string file;




};

int main (int argc, char** argv)
{
	ros::init (argc, argv, "cylinder_client");


  scene_publisher sp;
  sp.process();

	ros::Rate loop_rate(1);
	while (ros::ok())
	{
    sp.publish();
		ros::spinOnce ();
		loop_rate.sleep();
	}
}



