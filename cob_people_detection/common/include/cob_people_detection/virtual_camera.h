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
  void sample_pc(cv::Mat& pc_xyzPtr,cv::Mat& pc_rgbPtr,cv::Mat& img,cv::Mat& depth_map);
  void resample_pc_indirect(cv::Mat& src,cv::Mat& dst,cv::Mat& homo);
  bool calc_homography(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts,cv::Mat& homo);
  cv::Mat cam_mat;
  cv::Mat dist_coeffs;


  //extrinsics
  cv::Vec3f rot;
  cv::Vec3f trans;




  bool validity_check(std::vector<cv::Point3f>& obj_pts,std::vector<cv::Point2f>& img_pts,cv::Mat& rot,cv::Mat& trans);

  //intrinsics

private:
  cv::Size sensor_size;
  cv::Size pixel_dim;
  cv::Point2f pp;
  cv::Vec2f focal_length;



};
#endif 
