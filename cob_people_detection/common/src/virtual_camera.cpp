#include"cob_people_detection/virtual_camera.h"


VirtualCamera::VirtualCamera(int camera_type)
{
  // SENSOR PROPERTIES --KINECT
  if(camera_type==0){
  focal_length= 526;
  sensor_size=cv::Size(640,480);
  pixel_dim=cv::Size(0,0);
  dist_coeffs=(cv::Mat_<double>(1,5) << 0.0 , 0.0 , 0.0 , 0.0 ,0.0);
  }

  if(camera_type!=0)
  {
    std::cout<<"[VIRTUAL CAMERA] camera type not implemented - use KINECT\n";
  }


  //calculation of intrinsics
  pp=cv::Point(round(sensor_size.width),round(sensor_size.height));
  cam_mat=(cv::Mat_<double>(3,3) << focal_length , 0.0 , pp.x  , 0.0 , focal_length , pp.y  , 0.0 , 0.0 , 1);


  //initialization of extrinsics
  rot=cv::Vec3f(0.0,0.0,0.0);
  trans=cv::Vec3f(0.0,0.0,0.0);
}





//void VirtualCamera::set_extrinsics(cv::Vec3f& irot, cv::Vec3f& itrans)
//{
//  rot=irot;
//  trans=itrans;
//}


void VirtualCamera::calc_extrinsics(std::vector<cv::Point2f> img_pts, std::vector<cv::Point3f> obj_pts)
{
   // calculate object pose
   cv::solvePnP(obj_pts,img_pts,cam_mat,dist_coeffs,rot,trans);
   return;
}



void VirtualCamera::sample_pc(cv::Mat& pc_xyz,cv::Mat& pc_rgb,cv::Mat& img)
{
  if(pc_xyz.rows>1 && pc_xyz.cols >1)
  {
    pc_xyz=pc_xyz.reshape(3,1);
  }

   //project 3d points to virtual camera
   cv::Mat pc_proj(pc_xyz.size(),1,CV_32FC2);

   // calc reprojection diffs
   cv::projectPoints(pc_xyz,rot,trans,cam_mat,dist_coeffs,pc_proj);

   // assign color values to calculated image coordinates
   cv::Vec2f* pc_proj_ptr=pc_proj.ptr<cv::Vec2f>(0,0);
   cv::Vec3b* pc_rgb_ptr=pc_rgb.ptr<cv::Vec3b>(0,0);
   int ty,tx;

   cv::Mat occ_grid=cv::Mat::zeros(sensor_size,CV_8UC1);


   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);


       if (ty>0 && tx>0 && ty<sensor_size.height && tx<sensor_size.width && !isnan(ty) && !isnan(tx) && occ_grid.at<unsigned char>(ty,tx)<1)
       {
          img.at<cv::Vec3b>(ty,tx)=(*pc_rgb_ptr);
          occ_grid.at<unsigned char>(ty,tx)+=50;
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
      }

   return;
}
