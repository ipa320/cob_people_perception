#ifndef VIRTUALCAMERA_H_
#define VIRTUALCAMERA_H_
#include<opencv/cv.h>
#include<iostream>


class VirtualCamera {

public:

  enum TYPE
  {
    KINECT
  };

  VirtualCamera (TYPE cam_type);
  virtual ~VirtualCamera (){};


 inline void set_extrinsics(cv::Vec3f& irot,cv::Vec3f& itrans){ rot=irot ; trans=itrans;};
 inline void rotate_cam(cv::Vec3f& irot){rot+=irot;};
 inline void translate_cam(cv::Vec3f& itrans){trans+=itrans;};

  bool calc_extrinsics(std::vector<cv::Point3f> obj_pts,std::vector<cv::Point2f> img_pts, bool validity_check);
  void calc_intrinsics();

  void sample_pc(cv::Mat& pc,cv::Mat& img);
  void sample_pc(cv::Mat& pc_xyz,cv::Mat& pc_rgb,cv::Mat& img);

  cv::Mat cam_mat;
  cv::Mat dist_coeffs;





private:

  bool validity_check(std::vector<cv::Point3f>& obj_pts,std::vector<cv::Point2f>& img_pts,cv::Mat& rot,cv::Mat& trans);

  //intrinsics

  cv::Size sensor_size;
  cv::Size pixel_dim;
  cv::Point2f pp;
  double focal_length;

  //extrinsics
  cv::Vec3f rot;
  cv::Vec3f trans;


};
#endif 
