#include<cob_people_detection/face_normalizer.h>
using namespace cv;
FaceNormalizer::FaceNormalizer():scale_(1.0),
                                epoch_ctr(0),
                                debug_path_("/share/goa-tz/people_detection/debug/"),
                                debug_(true)
{
  std::string nose_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_nose.xml";
  nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
  nose_storage_=cvCreateMemStorage(0);

  std::string eye_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_eye.xml";
              eye_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_path.c_str(),0,0,0);
              eye_storage_=cvCreateMemStorage(0);

 //std::string eye_l_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
  std::string eye_l_path="/home/goa-tz/data/haarcascade_lefteye_2splits.xml";
 //std::string eye_l_path="/home/goa-tz/git/care-o-bot/cob_people_perception/cob_people_detection/common/files/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
              eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
              eye_l_storage_=cvCreateMemStorage(0);

  //std::string eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
  std::string eye_r_path="/home/goa-tz/data/haarcascade_righteye_2splits.xml";
              eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
              eye_r_storage_=cvCreateMemStorage(0);

  std::string mouth_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_mouth.xml";
              mouth_cascade_=(CvHaarClassifierCascade*) cvLoad(mouth_path.c_str(),0,0,0);
              mouth_storage_=cvCreateMemStorage(0);
}


FaceNormalizer::~FaceNormalizer(){
	cvReleaseHaarClassifierCascade(&mouth_cascade_);
	cvReleaseMemStorage(&mouth_storage_);
	cvReleaseHaarClassifierCascade(&nose_cascade_);
	cvReleaseMemStorage(&nose_storage_);
	cvReleaseHaarClassifierCascade(&eye_l_cascade_);
	cvReleaseMemStorage(&eye_l_storage_);
	cvReleaseHaarClassifierCascade(&eye_r_cascade_);
	cvReleaseMemStorage(&eye_r_storage_);
};

void FaceNormalizer::set_norm_face(int& size)

{
  norm_size_.height=size;
  norm_size_.width=size;


  f_norm_img_.lefteye.x=0.4*norm_size_.width;
  f_norm_img_.lefteye.y=0.3*norm_size_.height;

  f_norm_img_.righteye.x=0.6*norm_size_.width;
  f_norm_img_.righteye.y=0.3*norm_size_.height;

  f_norm_img_.mouth.x=0.5*norm_size_.width;
  f_norm_img_.mouth.y=0.85*norm_size_.height;

  f_norm_img_.nose.x=0.5*norm_size_.width;
  f_norm_img_.nose.y=0.4*norm_size_.height;


  // reset detections
  f_det_img_.lefteye.x=-1;
  f_det_img_.lefteye.y=-1;

  f_det_img_.righteye.x=-1;
  f_det_img_.righteye.y=-1;

  f_det_img_.mouth.x=-1;
  f_det_img_.mouth.y=-1;


  f_det_img_.nose .x=-1;
  f_det_img_.nose .y=-1;

}


bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Mat& depth,int & rows,cv::Vec2f& offset)
{

  img.copyTo(img_);
  depth.copyTo(depth_);

  offset_[0]=offset[0];
  offset_[1]=offset[1];

  epoch_ctr++;
  //
  //norm size from input image
  set_norm_face(rows);

  if(debug_)
  {
  cv::Mat temp_mat;
   cv::cvtColor(img,temp_mat,CV_BGR2RGB);
    dump_img(temp_mat,"0_original");

  }

  //geometric normalization
  if(!normalize_geometry_depth(img,depth)) return false;
  //if(debug_)dump_img(img,"4_geometry");
  ////resizing
  //cv::resize(img,img,norm_size_,0,0);
  //if(debug_)dump_img(img,"1_resized");

  //// radiometric normalization
  //if(!normalize_radiometry(img)) return false;
  //dump_img(img,"2_radiometry");




  return true;
}
bool FaceNormalizer::normalizeFace( cv::Mat& img,int & rows)
{
  //norm size ffrom input image
  set_norm_face(rows);

  if(debug_)
  {
    cv::cvtColor(img,img,CV_BGR2RGB);
    dump_img(img,"0_original");
  }

  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"1_resized");

  // radiometric normalization
  if(!normalize_radiometry(img)) return false;
  dump_img(img,"2_radiometry");


  //geometric normalization
  if(!normalize_geometry(img)) return false;
  if(debug_)dump_img(img,"4_geometry");


  epoch_ctr++;
  return true;
}

bool FaceNormalizer::normalize_radiometry(cv::Mat& img)
{
  cv::Mat v_channel;
  extractVChannel(img,v_channel);

  dct(v_channel);
  subVChannel(img,v_channel);

  return true;
}

void FaceNormalizer::extractVChannel( cv::Mat& img,cv::Mat& V)
{
  if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2].copyTo(V);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;

}

void FaceNormalizer::subVChannel(cv::Mat& img,cv::Mat& V)
{
  
  if(debug_)cv::cvtColor(img,img,CV_RGB2HSV);
  else cv::cvtColor(img,img,CV_BGR2HSV);

  std::vector<cv::Mat> channels;
  cv::split(img,channels);
  channels[2]=V;
  cv::merge(channels,img);

  if(debug_)cv::cvtColor(img,img,CV_HSV2RGB);
  else cv::cvtColor(img,img,CV_HSV2BGR);

  return;

}


void FaceNormalizer::eqHist(cv::Mat& img)
{

  cv::equalizeHist(img,img);

}

void FaceNormalizer::dct(cv::Mat& img)
{
// Dct conversion on logarithmic image

  img.convertTo(img,CV_32FC1);
  cv::Scalar mu=cv::mean(img);
  double C_00=log(mu.val[0])*sqrt(img.cols*img.rows);

  cv::log(img,img);
  cv::dct(img,img);

  //---------------------------------------
  if(debug_)std::cout<<"C_00 a priori="<<img.at<float>(0,0)<<std::endl;
  if(debug_)std::cout<<"C_00 a post="<<C_00<<std::endl;
  img.at<float>(0,0)=C_00;
  img.at<float>(0,1)=0;
  //--------------------------------------

  cv::idct(img,img);
  cv::exp(img,img);
  img.convertTo(img,CV_8UC1);

}




bool FaceNormalizer::normalize_geometry_depth(cv::Mat& img,cv::Mat& depth)
{



  // detect features
  if(!features_from_color(img_))
  {
    std::cout<<" NO COLOR FEATS"<<std::endl;
    return false;
  }
   if(debug_)dump_features(img_);

  //std::string path_root="/share/goa-tz/people_detection/debug/scenes/scene";
  //std::string path= path_root;
  //path.append(boost::lexical_cast<std::string>(epoch_ctr));
  //save_scene(depth_,img_,offset_,path);
  //return false;

   ident_face();

   if(!features_from_depth(depth_)) return false;
   //dyn_norm_face();


   int xoffset=round(offset_[0]);
   int yoffset=round(offset_[1]);



  f_det_img_.print();
  f_norm_img_.print();
  f_det_xyz_.print();

  f_det_img_.add_offset  (xoffset,yoffset);
  f_norm_img_.add_offset (xoffset,yoffset);

  //calculate transformation


   cv::Mat trans,rot;


   //camera matrix
   double fx=526.37013657;
   double fy=526.37013657;
   double cy=259.01834898;
   double cx=313.68782938;
   cv::Mat cam_mat=(cv::Mat_<double>(3,3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

   calcPnP(cam_mat,rot,trans);

   if(debug_)
   {
     std::cout<<"Rotation:\n"<<std::endl;
     std::cout<<rot.at<double>(0,0)<<" , "<<rot.at<double>(0,1)<<" , "<<rot.at<double>(0,2)<<std::endl;
     std::cout<<"Translation:\n"<<std::endl;
     std::cout<<trans.at<double>(0,0)<<" , "<<trans.at<double>(0,1)<<" , "<<trans.at<double>(0,2)<<std::endl;
   }

   cv::Mat img_res;
   resample_direct(cam_mat,rot,trans,img_res);
  //cv::Mat rot_test=(cv::Mat_<double>(3,1) << 0 , 0 , 0);
  //cv::Mat trans_test=(cv::Mat_<double>(3,1) << 0 , -0.03 , 0);
  //resample_direct(cam_mat,rot_test,trans_test,img_res);

   return true;
}
void FaceNormalizer::calcPnP(cv::Mat& cam_mat,cv::Mat& rot,cv::Mat& trans)
{
   std::vector<cv::Point3f> object_points;
   std::vector<cv::Point2f> img_points;
   f_norm_img_.as_vector(img_points);
   f_det_xyz_.as_vector(object_points);

   // calculate object pose
   cv::Mat dist_coeffs=(cv::Mat_<double>(5,1)<< 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000);
   cv::solvePnP(object_points,img_points,cam_mat,dist_coeffs,rot,trans);
   return;


}

void FaceNormalizer::resample_direct(cv::Mat& cam_mat,cv::Mat& rot,cv::Mat& trans,cv::Mat& res)
{
   std::vector<cv::Point3f> object_vec;
   int nan_ctr=0;
   cv::Point3f temp;
   for(int i=0;i<depth_.rows;++i)
   {
     for(int j=0;j<depth_.cols;++j)
     {
         temp = (cv::Point3f)depth_.at<cv::Point3f>(i,j);
         kin2xyz(temp);

       object_vec.push_back(temp);
      if(isnan(depth_.at<cv::Vec3f>(i,j)[0])) nan_ctr++;
     }
   }
   std::cout<<"[FaceNormalizer] # nan before"<<nan_ctr<<std::endl;



   //project 3d points to virtual camera
   cv::Mat dist_coeffs=(cv::Mat_<double>(5,1)<< 0.00000000, 0.00000000, 0.00000000, 0.00000000, 0.00000000);
   cv::Mat object_proj;
   //std::vector<cv::Vec2f> reproj_feat;
   // calc reprojection diffs
   cv::projectPoints(object_vec,rot,trans,cam_mat,dist_coeffs,object_proj);


   bool i_debug = true;
   if(debug_){

   std::vector<cv::Point2f> img_points;
   std::vector<cv::Point3f> object_points;
   f_norm_img_.as_vector(img_points);
   f_det_xyz_.as_vector(object_points);
   std::vector<cv::Vec2f> reproj_feat;
   cv::projectPoints(object_points,rot,trans,cam_mat,dist_coeffs,reproj_feat);
    std::cout<<"reprojection results"<<std::endl;
    std::cout<< "c: "<<img_points[0].x<<" - "<<reproj_feat[0][0]<<std::endl;
    std::cout<< "r: "<<img_points[0].y<<" - "<<reproj_feat[0][1]<<std::endl;
    std::cout<<"-----------------------------------------------------"<<std::endl;
    std::cout<<"reprojection results"<<std::endl;
    std::cout<< "c: "<<img_points[1].x<<" - "<<reproj_feat[1][0]<<std::endl;
    std::cout<< "r: "<<img_points[1].y<<" - "<<reproj_feat[1][1]<<std::endl;
    std::cout<<"-----------------------------------------------------"<<std::endl;
    std::cout<<"reprojection results"<<std::endl;
    std::cout<< "c: "<<img_points[2].x<<" - "<<reproj_feat[2][0]<<std::endl;
    std::cout<< "r: "<<img_points[2].y<<" - "<<reproj_feat[2][1]<<std::endl;
    std::cout<<"-----------------------------------------------------"<<std::endl;
    std::cout<<"reprojection results"<<std::endl;
    std::cout<< "c: "<<img_points[3].x<<" - "<<reproj_feat[3][0]<<std::endl;
    std::cout<< "r: "<<img_points[3].y<<" - "<<reproj_feat[3][1]<<std::endl;
    std::cout<<"-----------------------------------------------------"<<std::endl;


   }



   // assign color values to calculated image coordinates
   cv::Vec2f* img_ptr=object_proj.ptr<cv::Vec2f>(0,0);
   int r,c,ty,tx;

   cv::Mat img_proj=cv::Mat::ones(img_.rows,img_.cols,CV_8UC1)*255;
   cv::Mat img_proj_rgb=cv::Mat::zeros(img_.rows,img_.cols,CV_8UC3);
   cv::Mat depth_proj=cv::Mat::ones(img_.rows,img_.cols,CV_32FC1)*1000;
   cv::Mat occ_grid=cv::Mat::zeros(img_.rows,img_.cols,CV_8UC1);

   cv::Mat img_gray;
   cv::cvtColor(img_,img_gray,CV_BGR2GRAY);

  nan_ctr=0;
   for(int i=0;i<object_proj.rows;++i)
     {
       cv::Vec2f txty=*img_ptr;
       //cv::Vec2f trtc=object_proj.at<cv::Vec2f>(i,0);
       tx=(int)round(txty[0]);
       ty=(int)round(txty[1]);

       tx-=offset_[0];
       ty-=offset_[1];


        //calculate row and column
        r=floor(i/img_.cols);
        c=i % img_.cols;

       if (ty>0 && tx>0 && ty<img_.rows && tx<img_.cols && !isnan(ty) && !isnan(tx))
       {
         //img_proj.at<cv::Vec3f>(tr,tc)=img.at<cv::Vec3f>(r,c);
         //std::cout<<"tr="<<tr<<" tc="<<tc<<" - "<<"r="<<r<<" c="<<c<<std::endl;
        if(occ_grid.at<unsigned char>(ty,tx)<1 )
        //if((occ_grid.at<int>(tr,tc)<1) || (depth_proj.at<float>(tr,tc)> depth_.at<cv::Vec3f>(r,c)[2]))
       //std::cout<<tr<<" "<<tc<<" "<<r<<" "<<c<<std::endl;
       {
          img_proj.at<unsigned char>(ty,tx)=img_gray.at<unsigned char>(r,c);
          img_proj_rgb.at<cv::Vec3b>(ty,tx)=img_.at<cv::Vec3b>(r,c);
          depth_proj.at<float>(ty,tx)=depth_.at<cv::Vec3f>(r,c)[2];
          occ_grid.at<unsigned char>(ty,tx)+=50;
       }
       }
       else
       {
        //std::cout<<"ERROR: "<<tr<<" - "<<tc<<std::endl;
        nan_ctr++;
       }

       img_ptr++;
      }

   std::cout<<"[FaceNormalizer] # nan after"<<nan_ctr<<std::endl;

   dump_img(img_proj,"projected");
   dump_img(img_proj_rgb,"projected RGB");
   dump_img(occ_grid,"occ grid");
   dump_img(depth_proj,"depth proj");

   return;


}


bool FaceNormalizer::normalize_geometry(cv::Mat& img)
{

  // detect features
  if(!features_from_color(img))return false;

   dyn_norm_face();
  //calculate transformation
   cv::Mat trafo(2,3,CV_32FC1);
   get_transform_affine(trafo);

   if(debug_)dump_features(img);

  //warping
   cv::Mat warped = cv::Mat(img.rows,img.cols,img.type());
   cv::warpAffine(img,warped,trafo,norm_size_ );


  warped.copyTo(img);

  return true;

}

bool FaceNormalizer::features_from_color(cv::Mat& img_color)
{
  if(!detect_feature(img_color,f_det_img_.nose,PP_NOSE))
  {
    if(debug_)std::cout<<"no nose"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.lefteye,PP_EYE_L))
  {
    if(debug_)std::cout<<"no eye_l"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.righteye,PP_EYE_R))
  {
    if(debug_)std::cout<<"no eye_r"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.mouth,PP_MOUTH))
  {
    if(debug_)std::cout<<"no mouth"<<std::endl;
     return false;
  }


  if(debug_)
  {
    std::cout<<"detected lefteye "<<f_det_img_.lefteye.x<<" - " << f_det_img_.lefteye.y<<std::endl;
    std::cout<<"detected righteye "<<f_det_img_.righteye.x<<" - " << f_det_img_.righteye.y<<std::endl;
    std::cout<<"detected nose "<<f_det_img_.nose.x<<" - " << f_det_img_.nose.y<<std::endl;
    std::cout<<"detected mouth "<<f_det_img_.mouth.x<<" - " << f_det_img_.mouth.y<<std::endl;
  }
  return true;
}

bool FaceNormalizer::features_from_depth(cv::Mat& depth)
{

  //pick 3D points from pointcloud
  //cv::Vec3f m=depth.at<cv::Vec3f>((int)depth.rows/2,(int)depth.cols/2);
  f_det_xyz_.nose=      depth.at<cv::Vec3f>(f_det_img_.nose.y,f_det_img_.nose.x)               ;
  f_det_xyz_.mouth=     depth.at<cv::Vec3f>(f_det_img_.mouth.y,f_det_img_.mouth.x)            ;
  f_det_xyz_.lefteye=   depth.at<cv::Vec3f>(f_det_img_.lefteye.y,f_det_img_.lefteye.x)      ;
  f_det_xyz_.righteye=  depth.at<cv::Vec3f>(f_det_img_.righteye.y,f_det_img_.righteye.x)   ;
  //f_det_xyz_.nose=depth.at<cv::Vec3f>(f_det_img_.nose.x,f_det_img_.nose.y)               ;
  //f_det_xyz_.mouth=depth.at<cv::Vec3f>(f_det_img_.mouth.x,f_det_img_.mouth.y)            ;
  //f_det_xyz_.lefteye=depth.at<cv::Vec3f>(f_det_img_.lefteye.x,f_det_img_.lefteye.y)      ;
  //f_det_xyz_.righteye=depth.at<cv::Vec3f>(f_det_img_.righteye.x,f_det_img_.righteye.y)   ;
  kin2xyz(f_det_xyz_.nose);
  kin2xyz(f_det_xyz_.mouth);
  kin2xyz(f_det_xyz_.lefteye);
  kin2xyz(f_det_xyz_.righteye);



  if(debug_)
  {
    std::cout<<"LEFTEYE: "<<f_det_xyz_.lefteye.x<<" "<<f_det_xyz_.lefteye.y<<" "<<f_det_xyz_.lefteye.z<<std::endl;
    std::cout<<"RIGTHEYE: "<<f_det_xyz_.righteye.x<<" "<<f_det_xyz_.righteye.y<<" "<<f_det_xyz_.righteye.z<<std::endl;
    std::cout<<"Nose: "<<f_det_xyz_.nose.x<<" "<<f_det_xyz_.nose.y<<" "<<f_det_xyz_.nose.z<<std::endl;
    std::cout<<"Mouth: "<<f_det_xyz_.mouth.x<<" "<<f_det_xyz_.mouth.y<<" "<<f_det_xyz_.mouth.z<<std::endl;
  }
  if(!f_det_xyz_.valid()) return false;


  return true;
}

bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Point2f& coords,int code)
{

  CvSeq* seq;
  cv::Vec2f offset;

  if(code==PP_NOSE)
  {
    offset<< (0,0);
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));
  }


  if(code==PP_EYE_L)
  {
    offset[0]=0;
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,0,f_det_img_.nose.x,f_det_img_.nose.y));
    //showImg(sub_img,"eye_l");
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_EYE_R)
  {
    offset[0]=((int)f_det_img_.nose.x);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(f_det_img_.nose.x,0,img.cols-f_det_img_.nose.x-1,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.1,1,0,cvSize(5,5));

  }

  if(code==PP_MOUTH)
  {
    offset[0]=0;
    offset[1]=(int)f_det_img_.nose.y;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,f_det_img_.nose.y,img.cols,img.rows-f_det_img_.nose.y-1));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(15,15));

  }

    if(seq->total ==0) return false;
    Rect* seq_det=(Rect*)cvGetSeqElem(seq,0);
    coords.x=(float)seq_det->x+seq_det->width/2+offset[0];
    coords.y=(float)seq_det->y+seq_det->height/2+offset[1];

    return true;
}


void FaceNormalizer::dyn_norm_face()
{

  //measured values x

  //eye base
  cv::Vec2f base=(cv::Vec2f)f_det_img_.righteye-(cv::Vec2f)f_det_img_.lefteye;
  cv::normalize(base,base);
  double a=cv::norm((cv::Vec2f)f_det_img_.lefteye,(cv::Vec2f)f_det_img_.righteye,cv::NORM_L2);
  cv::Vec2f dummy=(cv::Vec2f)f_det_img_.lefteye+(a*0.5*base);

  double b =cv::norm(dummy,(cv::Vec2f)f_det_img_.nose,cv::NORM_L2);
  double c=cv::norm((cv::Vec2f)f_det_img_.nose,(cv::Vec2f)f_det_img_.mouth,cv::NORM_L2);

  double s1=a/b;
  double s2=b/c;

  //norm values
  f_norm_img_.nose.y=f_norm_img_.lefteye.y+(f_norm_img_.righteye.x-f_norm_img_.lefteye.x)/s1;
  f_norm_img_.mouth.y=f_norm_img_.nose.y+(f_norm_img_.nose.y-f_norm_img_.lefteye.y)/s2;


  return;
}

void FaceNormalizer::ident_face()
{
  f_norm_img_.lefteye=f_det_img_.lefteye;
  f_norm_img_.righteye=f_det_img_.righteye;
  f_norm_img_.mouth=f_det_img_.mouth;
  f_norm_img_.nose=f_det_img_.nose;
}




void FaceNormalizer::transformPerspective(cv::Mat& trafo)
{

  cv::Point2f src[4],dst[4];
  src[0]=Point2f(f_det_img_.lefteye.x,f_det_img_.lefteye.y);
  src[1]=Point2f(f_det_img_.righteye.x,f_det_img_.righteye.y);
  src[2]=Point2f(f_det_img_.mouth.x,f_det_img_.mouth.y);
  src[3]=Point2f(f_det_img_.nose.x,f_det_img_.nose.y);

  dst[0]=Point2f(f_norm_img_.lefteye.x,f_norm_img_.lefteye.y);
  dst[1]=Point2f(f_norm_img_.righteye.x,f_norm_img_.righteye.y);
  dst[2]=Point2f(f_norm_img_.mouth.x,f_norm_img_.mouth.y);
  dst[3]=Point2f(f_norm_img_.nose.x,f_norm_img_.nose.y);
  trafo=cv::getPerspectiveTransform(src,dst);
}

void FaceNormalizer::get_transform_affine(cv::Mat& trafo)
{
  cv::Point2f src[3],dst[3];

  src[0]=Point2f(f_det_img_.lefteye.x,f_det_img_.lefteye.y);
  src[1]=Point2f(f_det_img_.righteye.x,f_det_img_.righteye.y);
  src[2]=Point2f(f_det_img_.mouth.x,f_det_img_.mouth.y);

  dst[0]=Point2f(f_norm_img_.lefteye.x,f_norm_img_.lefteye.y);
  dst[1]=Point2f(f_norm_img_.righteye.x,f_norm_img_.righteye.y);
  dst[2]=Point2f(f_norm_img_.mouth.x,f_norm_img_.mouth.y);


  trafo = cv::getAffineTransform(src,dst);





}


void FaceNormalizer:: showImg(cv::Mat& img,std::string window_name){
  cv::namedWindow(window_name,CV_WINDOW_AUTOSIZE);
  cv::imshow(window_name,img);
  cv::waitKey(0);
    }

void FaceNormalizer::dump_img(cv::Mat& data,std::string name){
  std::string filename =debug_path_;
  filename.append(boost::lexical_cast<std::string>(epoch_ctr));
  filename.append("_");
  filename.append(name);
  filename.append(".jpg");

  cv::imwrite(filename,data);
}

void FaceNormalizer::dump_features(cv::Mat& img)
{

  cv::Mat img2;
  img.copyTo(img2);
  IplImage ipl_img=(IplImage)img2;
   cv::circle(img2,cv::Point(f_det_img_.nose.x, f_det_img_.nose.y),5,CV_RGB(255,0,0));
   cv::circle(img2,cv::Point(f_det_img_.mouth.x,f_det_img_.mouth.y),5,CV_RGB(0,255,0));
   cv::circle(img2,cv::Point(f_det_img_.lefteye.x,f_det_img_.lefteye.y),5,CV_RGB(255,255,0));
   cv::circle(img2,cv::Point(f_det_img_.righteye.x,f_det_img_.righteye.y),5,CV_RGB(255,0,255));
   dump_img(img2,"3_features");
}

bool FaceNormalizer::save_scene(cv::Mat& depth,cv::Mat& color,cv::Vec2f& offset,std::string path)
{
  std::string depth_path,color_path;
  color_path=path;
  color_path.append(".jpg");
  std::cout<<"color path=" <<color_path<<std::endl;
  depth_path=path;
  depth_path.append(".xml");
  cv::FileStorage fs(depth_path,FileStorage::WRITE);
  fs << "depth"<<depth;
  fs << "color"<<color;
  fs << "offset_row"<<offset[1];
  fs << "offset_col"<<offset[0];
  fs.release();

  imwrite(color_path,color);
}

bool FaceNormalizer::read_scene(cv::Mat& depth, cv::Mat& color,cv::Vec2f& offset,std::string path)
{
  cv::FileStorage fs(path,FileStorage::READ);
  fs["depth"]>> depth;
  fs["color"]>> color;
  fs["offset_row"]>> offset[1];
  fs["offset_col"]>> offset[0];
  fs.release();
}


void FaceNormalizer::kin2xyz(cv::Point3f& vec)
{
  double temp=vec.y;
  vec.x*=-1;
  vec.y=vec.z;
  vec.z=temp;
}


int main(int argc, const char *argv[])
{
  FaceNormalizer fn;
  cv::Mat depth,img;
  cv::Vec2f offset;
  std::string i_path="/share/goa-tz/people_detection/debug/scenes/scene";
  i_path.append(argv[1]);
  i_path.append(".xml");

  std::cout<<"[FaceNormalizer] reading scene..."<<std::endl;
  fn.read_scene(depth,img,offset,i_path);

  std::cout<<"[FaceNormalizer] normalizing face..."<<std::endl;
  fn.normalizeFace(img,depth,img.rows,offset);

  return 0;
}
