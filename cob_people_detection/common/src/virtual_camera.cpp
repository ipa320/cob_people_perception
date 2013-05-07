#include"cob_people_detection/virtual_camera.h"
#include<opencv/highgui.h>



VirtualCamera::VirtualCamera(VirtualCamera::TYPE cam_type)
{
  // SENSOR PROPERTIES --KINECT
  switch(cam_type)
  {
    case VirtualCamera::KINECT:
      {
        focal_length[0]=524.90160178307269 ;
        focal_length[1]=525.85226379335393;
        sensor_size=cv::Size(640,480);
        pixel_dim=cv::Size(0,0);
        dist_coeffs=(cv::Mat_<double>(1,5) << 0.25852454045259377, -0.88621162461930914, 0.0012346117737001144, 0.00036377459304633028, 1.0422813597203011);
        //dist_coeffs=(cv::Mat_<double>(1,5) << 0,0,0,0,0);
        pp.x=312.13543361773458;
        pp.y=254.73474482242005;
      break;
      }
    default:
      {
        std::cout<<"[VIRTUAL CAMERA] camera type not implemented - use KINECT\n";
        break;
      }

  }

  //calculation of intrinsics
  //pp=cv::Point(round(sensor_size.width),round(sensor_size.height));
  cam_mat=(cv::Mat_<double>(3,3) << focal_length[0] , 0.0 , pp.x  , 0.0 , focal_length[1] , pp.y  , 0.0 , 0.0 , 1);

  //initialization of extrinsics
  rot=cv::Vec3f(0.0,0.0,0.0);
  trans=cv::Vec3f(0.0,0.0,0.0);
}



//bool VirtualCamera::calibrate(std::vector<cv::Mat>& img_vec,cv::Size& pattern_size,double square_length)
//{
//
//
//
//
//}


//void VirtualCamera::set_extrinsics(cv::Vec3f& irot, cv::Vec3f& itrans)
//{
//  rot=irot;
//  trans=itrans;
//}

bool VirtualCamera::calc_homography(std::vector<cv::Point2f> src_pts, std::vector<cv::Point2f> dst_pts,cv::Mat& homo)
{

   homo =cv::findHomography(src_pts,dst_pts,CV_LMEDS);

}

bool VirtualCamera::calc_extrinsics( std::vector<cv::Point3f> obj_pts,std::vector<cv::Point2f> img_pts,bool check_model)
{
   // calculate object pose
  cv::Mat temp_rot,temp_trans;

  
//  double* ptr=cam_mat.ptr<double>(0,0);
//  for(int i=0;i<9;i++)
//  {
//    std::cout<<"CAM "<<*ptr<<std::endl;
//    ++ptr;
//  }
//
//  for(int j =0;j<4;j++)
//  {
//    std::cout<<"OBJ"<<obj_pts[j].x<<" , "<<obj_pts[j].y<<" , "<<obj_pts[j].z<<std::endl;
//    std::cout<<"IMG"<<img_pts[j].x<<" , "<<img_pts[j].y<<std::endl;
//  }


  cv::solvePnP(obj_pts,img_pts,cam_mat,dist_coeffs,temp_rot,temp_trans,false);

  if(check_model==true)
  {
    if (!validity_check(obj_pts,img_pts,temp_rot,temp_trans))
    {
      std::cout<<"\n[VIRTUAL CAMERA] invalid extrinsics\n"<<std::endl;
      return false;
    }
  }

    rot=temp_rot;
    trans=temp_trans;
    //trans[1]=0;
    //trans[2]=0;
    //trans[2]=-1.0;

   return true;

}



void VirtualCamera::sample_pc_NEW(cv::Mat& pc_xyzPtr,cv::Mat& pc_rgbPtr,cv::Mat& img,cv::Mat& depth_map)
{
  int channels=pc_rgbPtr.channels();

  cv::Mat pc_xyz,pc_rgb;
  pc_xyzPtr.copyTo(pc_xyz);
  pc_rgbPtr.copyTo(pc_rgb);
  if(pc_xyz.rows>1 && pc_xyz.cols >1)
  {
    pc_xyz=pc_xyz.reshape(3,1);
  }

   //project 3d points to virtual camera
   cv::Mat pc_proj(pc_xyz.rows*pc_xyz.cols,1,CV_32FC2);

   // calc reprojection diffs
   cv::projectPoints(pc_xyz,rot,trans,cam_mat,dist_coeffs,pc_proj);

   cv::Vec3f* pc_ptr=pc_xyzPtr.ptr<cv::Vec3f>(0,0);
   cv::Vec2f* pc_proj_ptr=pc_proj.ptr<cv::Vec2f>(0,0);
   int ty,tx;



   if(channels==3)
   {
    cv::add(img,0,img);
    cv::add(depth_map,0,depth_map);
   // assign color values to calculated image coordinates
   cv::Vec3b* pc_rgb_ptr=pc_rgb.ptr<cv::Vec3b>(0,0);


   cv::Mat occ_grid=cv::Mat::ones(sensor_size,CV_32FC3);
   cv::Mat img_cum=cv::Mat::zeros(sensor_size,CV_32FC3);
   cv::Vec3f occ_inc=cv::Vec3f(1,1,1);
   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);


       if (ty>1 && tx>1 && ty<sensor_size.height-1 && tx<sensor_size.width-1 && !isnan(ty) && !isnan(tx) )
       {
            img_cum.at<cv::Vec3b>(ty,tx)+=(*pc_rgb_ptr);
            //test interpolation method++++++++++++++++++++++++
            img_cum.at<cv::Vec3f>(ty+1,tx)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty-1,tx)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty,tx-1)+=(*pc_rgb_ptr);
            img_cum.at<cv::Vec3f>(ty,tx+1)+=(*pc_rgb_ptr);
            occ_grid.at<cv::Vec3f>(ty,tx)+=  occ_inc;
            occ_grid.at<cv::Vec3f>(ty+1,tx)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty-1,tx)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty,tx+1)+=occ_inc;
            occ_grid.at<cv::Vec3f>(ty,tx-1)+=occ_inc;
            //+++++++++++++++++++++++++++++

            depth_map.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
       pc_ptr++;
      }
   img_cum=img_cum / occ_grid;
   img_cum.convertTo(img,CV_8UC3);
   }


   if(channels==1)
   {
   // assign color values to calculated image coordinates
    cv::add(img,0,img);
    cv::add(depth_map,0,depth_map);
   unsigned char* pc_rgb_ptr=pc_rgb.ptr<unsigned char>(0,0);

   cv::Mat occ_grid=cv::Mat::ones(sensor_size,CV_32FC1);
   cv::Mat img_cum=cv::Mat::zeros(sensor_size,CV_32FC1);
   cv::Mat occ_grid2=cv::Mat::ones(sensor_size,CV_32FC1);
   for(int i=0;i<pc_proj.rows;++i)
     {
       cv::Vec2f txty=*pc_proj_ptr;
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);


       if (ty>1 && tx>1 && ty<sensor_size.height-1 && tx<sensor_size.width-1 && !isnan(ty) && !isnan(tx) )
       {
            //if((depth_map.at<cv::Vec3f>(ty,tx)[2]==0) || (depth_map.at<cv::Vec3f>(ty,tx)[2]>(*pc_ptr)[2]))
            //{
            img.at<unsigned char>(ty,tx)=(*pc_rgb_ptr);
            occ_grid2.at<float>(ty,tx)=0.0;
            //test interpolation method++++++++++++++++++++++++
            img_cum.at<float>(ty+1,tx)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty-1,tx)+=(*pc_rgb_ptr);
            img_cum.at<float>(ty,tx-1)+=(*pc_rgb_ptr);
             img_cum.at<float>(ty,tx+1)+=(*pc_rgb_ptr);
             img_cum.at<float>(ty+1,tx+1)+=(*pc_rgb_ptr);
             img_cum.at<float>(ty-1,tx-1)+=(*pc_rgb_ptr);
             img_cum.at<float>(ty-1,tx+1)+=(*pc_rgb_ptr);
             img_cum.at<float>(ty+1,tx-1)+=(*pc_rgb_ptr);
             occ_grid.at<float>(ty,tx)+=  1;
             occ_grid.at<float>(ty+1,tx)+=1;
             occ_grid.at<float>(ty-1,tx)+=1;
             occ_grid.at<float>(ty,tx+1)+=1;
             occ_grid.at<float>(ty,tx-1)+=1;
             occ_grid.at<float>(ty+1,tx+1)+=1;
             occ_grid.at<float>(ty-1,tx-1)+=1;
             occ_grid.at<float>(ty-1,tx+1)+=1;
             occ_grid.at<float>(ty+1,tx-1)+=1;
            //+++++++++++++++++++++++++++++
            depth_map.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
            //}
       }
       pc_rgb_ptr++;
       pc_proj_ptr++;
       pc_ptr++;
      }

   occ_grid=occ_grid;
   img_cum=img_cum / (occ_grid.mul(occ_grid2)-1);
   img_cum.convertTo(img_cum,CV_8UC1);
   cv::add(img,img_cum,img);


   }

   return;
}

//void VirtualCamera::sample_pc(cv::Mat& pc_xyzPtr,cv::Mat& pc_rgbPtr,cv::Mat& img,cv::Mat& depth_map)
//{
//
//
//  int channels=pc_rgbPtr.channels();
//
//  cv::Mat pc_xyz,pc_rgb;
//  pc_xyzPtr.copyTo(pc_xyz);
//  pc_rgbPtr.copyTo(pc_rgb);
//  if(pc_xyz.rows>1 && pc_xyz.cols >1)
//  {
//    pc_xyz=pc_xyz.reshape(3,1);
//  }
//
//   //project 3d points to virtual camera
//   cv::Mat pc_proj(pc_xyz.rows*pc_xyz.cols,1,CV_32FC2);
//
//   // calc reprojection diffs
//   cv::projectPoints(pc_xyz,rot,trans,cam_mat,dist_coeffs,pc_proj);
//
//   cv::Vec3f* pc_ptr=pc_xyzPtr.ptr<cv::Vec3f>(0,0);
//   cv::Vec2f* pc_proj_ptr=pc_proj.ptr<cv::Vec2f>(0,0);
//   int ty,tx;
//
//
//
//   if(channels==3)
//   {
//   cv::Mat occ_grid=cv::Mat::zeros(sensor_size,CV_8UC1);
//    cv::add(img,0,img);
//    cv::add(depth_map,0,depth_map);
//   // assign color values to calculated image coordinates
//   cv::Vec3b* pc_rgb_ptr=pc_rgb.ptr<cv::Vec3b>(0,0);
//   for(int i=0;i<pc_proj.rows;++i)
//     {
//       cv::Vec2f txty=*pc_proj_ptr;
//       tx=(int)round(txty[0]);
//       ty=(int)round(txty[1]);
//
//
//       if (ty>0 && tx>0 && ty<sensor_size.height && tx<sensor_size.width && !isnan(ty) && !isnan(tx) )
//       {
//         if(occ_grid.at<unsigned char>(ty,tx)>0) int a=0;// img.at<cv::Vec3b>(ty,tx)=cv::Vec3b(0,0,255);
//         else
//          {
//            img.at<cv::Vec3b>(ty,tx)=(*pc_rgb_ptr);
//            depth_map.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
//          }
//            occ_grid.at<unsigned char>(ty,tx)+=50;
//       }
//       pc_rgb_ptr++;
//       pc_proj_ptr++;
//       pc_ptr++;
//      }
//   }
//
//
//   if(channels==1)
//   {
//   // assign color values to calculated image coordinates
//    cv::add(img,0,img);
//    cv::add(depth_map,0,depth_map);
//
//    cv::Mat  img_cum=cv::Mat::zeros(sensor_size,CV_32FC1);
//    cv::Mat  occ_grid=cv::Mat::ones(sensor_size,CV_32FC1);
//   unsigned char* pc_rgb_ptr=pc_rgb.ptr<unsigned char>(0,0);
//
//  // declare pointers
//  // c= center, l=left, r=right , a=above, b=below
//   float* c_ptr,l_ptr,lb_ptr,la_ptr,r_ptr,rb_ptr,ra_ptr,b_ptr,a_ptr;
//
//  cv::Vec2f inc_vec=cv::Vec2f(0,1);
//
//   for(int i=0;i<pc_proj.rows;++i)
//     {
//       cv::Vec2f txty=*pc_proj_ptr;
//       tx=(int)round(txty[0]);
//       ty=(int)round(txty[1]);
//
//
//       if (ty>1 && tx>1 && ty<sensor_size.height -1 && tx<sensor_size.width -1 && !isnan(ty) && !isnan(tx) )
//       {
//         //if(occ_grid.at<unsigned char>(ty,tx)>0) int a=0;// img.at<cv::Vec3b>(ty,tx)=cv::Vec3b(0,0,255);
//         //else
//          //{
//            //if((depth_map.at<cv::Vec3f>(ty,tx)[2]==0) || (depth_map.at<cv::Vec3f>(ty,tx)[2]>(*pc_ptr)[2]))
//            //{
//
//          // initialize pointer mask
//            c_ptr=img_cum.ptr<float>(ty,tx);
//            l_ptr=c_ptr-1;
//            r_ptr=c_ptr+1;
//
//            a_ptr=c_ptr-img_cum.cols;
//            b_ptr=c_ptr+img_cum.cols;
//
//            la_ptr=a_ptr-1;
//            ra_ptr=a_ptr+1;
//
//            lb_ptr=b_ptr-1;
//            rb_ptr=b_ptr+1;
//
//
//            img.at<unsigned char>(ty,tx)=(*pc_rgb_ptr);
//            occ_grid.at<float>(ty,tx)=0;
//            depth_map.at<cv::Vec3f>(ty,tx)=((*pc_ptr));
//
//            inc_vec[0]=(*pc_rgb_ptr)
//            //add grayvalue to mask
//            *l_ptr+=  inc_vec;
//            *r_ptr+=  inc_vec;
//            *a_ptr+=  inc_vec;
//            *b_ptr+=  inc_vec;
//            *la_ptr+= inc_vec;
//            *lb_ptr+= inc_vec;
//            *ra_ptr+= inc_vec;
//            *rb_ptr+= inc_vec;
//
//            //}
//          //}
//            //occ_grid.at<unsigned char>(ty,tx)+=50;
//       }
//       pc_rgb_ptr++;
//       pc_proj_ptr++;
//       pc_ptr++;
//      }
//
//   //std::vector channels;
//   //cv::split(img_cum,vector_channels);
//   //channels[0]=channels[0] / (channels[1].mul(occ_grid)-1);
//   //channels[0].convertTo(channels[0],CV_8UC1);
//   //img+=channels[0];
//
//   }
//   return;
//}
void VirtualCamera::sample_point(cv::Point3f& p_xyz,cv::Point2f& p_uv)
{



   // calc reprojection diffs
   //cv::Mat m_xyz=(cv::Mat)p_xyz;
   //cv::Mat m_uv= (cv::Mat)p_uv;
   //std::cout<<"m xyz"<<m_xyz<<std::endl;
    std::vector<cv::Point3f> m_xyz;
    std::vector<cv::Point2f> m_uv;
    m_xyz.push_back(p_xyz);
   cv::projectPoints(m_xyz,rot,trans,cam_mat,dist_coeffs,m_uv);
   p_uv=m_uv[0];
   return;
}


bool VirtualCamera::validity_check(std::vector<cv::Point3f>& obj_pts,std::vector<cv::Point2f>& img_pts,cv::Mat& rot,cv::Mat& trans)
{

  double val_limit=5.0;
  cv::Mat proj_pts;
  cv::projectPoints(obj_pts,rot,trans,cam_mat,dist_coeffs,proj_pts);
  cv::Point2f* proj_pts_ptr=proj_pts.ptr<cv::Point2f>(0,0);

  for(int i=0;i<img_pts.size();++i)
  {
    if(img_pts[i].x- (*proj_pts_ptr).x >val_limit || img_pts[i].y - (*proj_pts_ptr).y > val_limit) return false;
    proj_pts_ptr++;
  }

  return true;

}
