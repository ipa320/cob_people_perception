
#include<opencv/cv.h>
#include<iostream>


#define KINECT 0
class VirtualCamera {
public:
  VirtualCamera (int camera_type);
  virtual ~VirtualCamera ();


 inline void set_extrinsics(cv::Vec3f& irot,cv::Vec3f& itrans){ rot=irot ; trans=itrans;};
 inline void rotate_cam(cv::Vec3f& irot){rot+=irot;};
 inline void translate_cam(cv::Vec3f& itrans){trans+=itrans;};

  void calc_extrinsics(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> obj_pts);
  void calc_intrinsics();

  void sample_pc(cv::Mat& pc,cv::Mat& img);
  void sample_pc(cv::Mat& pc_xyz,cv::Mat& pc_rgb,cv::Mat& img);

  cv::Mat cam_mat;
  cv::Mat dist_coeffs;





private:
  /* data */

  //intrinsics

  cv::Size sensor_size;
  cv::Size pixel_dim;
  cv::Point2f pp;
  double focal_length;

  //extrinsics
  cv::Vec3f rot;
  cv::Vec3f trans;


};
