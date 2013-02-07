#include<cob_people_detection/face_normalizer.h>
#include<pcl/common/transform.h>

using namespace cv;
FaceNormalizer::FaceNormalizer(): epoch_ctr(0),
                                  debug_(true),
                                  //HOME
                                  //debug_path_("/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/"),
                                  //IPA
                                  debug_path_("/share/goa-tz/people_detection/debug/"),
                                  kinect(VirtualCamera::KINECT)
{
  bool home=false;

  std::string eye_r_path,eye_path,eye_l_path,nose_path,mouth_path;
  if(home)
  {
    eye_r_path="/usr/local/share/OpenCV/haarcascades/haarcascade_mcs_righteye.xml";
    eye_path="/usr/local/share/OpenCV/haarcascades/haarcascade_eye.xml";
    eye_l_path="/usr/local/share/OpenCV/haarcascades/haarcascade_mcs_lefteye.xml";
    nose_path="/usr/local/share/OpenCV/haarcascades/haarcascade_mcs_nose.xml";
    mouth_path="/usr/local/share/OpenCV/haarcascades/haarcascade_mcs_mouth.xml";

  }
  else
  {
     eye_r_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_righteye.xml";
     eye_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
     eye_l_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_lefteye.xml";
     nose_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_nose.xml";
     mouth_path="/usr/share/OpenCV-2.3.1/haarcascades/haarcascade_mcs_mouth.xml";
  }
  eye_r_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_r_path.c_str(),0,0,0);
  eye_r_storage_=cvCreateMemStorage(0);

  eye_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_path.c_str(),0,0,0);
  eye_storage_=cvCreateMemStorage(0);

  eye_l_cascade_=(CvHaarClassifierCascade*) cvLoad(eye_l_path.c_str(),0,0,0);
  eye_l_storage_=cvCreateMemStorage(0);

  nose_cascade_=(CvHaarClassifierCascade*) cvLoad(nose_path.c_str(),0,0,0);
  nose_storage_=cvCreateMemStorage(0);

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

void FaceNormalizer::set_norm_face(cv::Size& input_size)
{

  f_norm_img_.lefteye.x=0.25     *input_size.width     ;
  f_norm_img_.lefteye.y=0.25      *input_size.height     ;

  f_norm_img_.righteye.x=0.75    *input_size.width     ;
  f_norm_img_.righteye.y=0.25     *input_size.height     ;

  f_norm_img_.mouth.x=0.5        *input_size.width     ;
  f_norm_img_.mouth.y=0.85       *input_size.height     ;

  f_norm_img_.nose.x=0.5         *input_size.width     ;
  f_norm_img_.nose.y=0.4         *input_size.height     ;


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


bool FaceNormalizer::captureScene( cv::Mat& img,cv::Mat& depth,cv::Vec2f& offset)
{
  input_size_=cv::Size(img.cols,img.rows);



  std::cout<<"SAVING SCENE"<<std::endl;
  //std::string path_root="/home/tom/git/care-o-bot/cob_people_perception/cob_people_detection/debug/scene";
  std::string path_root="/share/goa-tz/people_detection/normalization/scenes/scene";
  std::string path= path_root;
  path.append(boost::lexical_cast<std::string>(epoch_ctr));
  save_scene(depth,img,offset,path);
  epoch_ctr++;

  return true;
}
bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Mat& depth,cv::Size& norm_size,cv::Vec2f& offset,cv::Mat& depth_res)
{
  // set members to current values
  norm_size_=norm_size;
  input_size_=cv::Size(img.cols,img.rows);


  bool valid = true; // Flag only returned true if all steps have been completed successfully

  offset_[0]=offset[0];
  offset_[1]=offset[1];

  epoch_ctr++;

  //norm size from input image
  set_norm_face(input_size_);

  if(debug_)
  {
  cv::Mat temp_mat;
   cv::cvtColor(img,temp_mat,CV_BGR2RGB);
    dump_img(temp_mat,"0_originalRGBD");

  }

  //geometric normalization
  //TODO : TEMPORARY DISABLING GEOM NORM
  if(!normalize_geometry_depth(img,depth)) valid=false ;
  if(debug_)dump_img(img,"1_geometryRGBD");
  //if(debug_)std::cout<<"1 - normalized geometry"<<std::endl;

  if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);

  if(valid)despeckle<unsigned char>(img,img);
  //reducing the depth map
  processDM(depth,depth_res);
  if(valid)despeckle<float>(depth_res,depth_res);

  if(debug_ && valid)dump_img(img,"2_despeckle");
  //if(debug_ && valid)std::cout<<"2 - filtered"<<std::endl;

  //resizing
  cv::resize(img,img,norm_size_,0,0);
  cv::resize(depth,depth,norm_size_,0,0);
  cv::resize(depth_res,depth_res,norm_size_,0,0);
  if(debug_)dump_img(img,"3_resized");
  //if(debug_)std::cout<<"3 - resized"<<std::endl;

  if(img.channels()==3)cv::cvtColor(img,img,CV_RGB2GRAY);
  // radiometric normalization
  if(!normalize_radiometry(img)) valid=false;
  if(debug_)dump_img(img,"4_radiometry");
  //if(debug_)std::cout<<"4 - normalized geometry"<<std::endl;

  




  return valid;
}
bool FaceNormalizer::normalizeFace( cv::Mat& img,cv::Size& norm_size)
{
  // set members to current values
  norm_size_=norm_size;
  input_size_=cv::Size(img.cols,img.rows);


  bool valid = true; // Flag only returned true if all steps have been completed successfully

  //norm size ffrom input image
  set_norm_face(input_size_);

  if(debug_)
  {
    if(img.channels() == 3)
    {
    cv::cvtColor(img,img,CV_BGR2RGB);
    }
    dump_img(img,"0_original");
  }


  //geometric normalization
  if(!normalize_geometry(img,FaceNormalizer::AFFINE)) valid= false;
  if(debug_)dump_img(img,"1_geometryRGB");

  if(img.channels()==3)
  {
  cv::cvtColor(img,img,CV_BGR2GRAY);
  }

  //resizing
  cv::resize(img,img,norm_size_,0,0);
  if(debug_)dump_img(img,"2_resized");

  // radiometric normalization
  if(!normalize_radiometry(img)) valid= false;
  if(debug_)dump_img(img,"3_radiometry");

  epoch_ctr++;
  return valid;
}

bool FaceNormalizer::normalize_radiometry(cv::Mat& img)
{

  dct(img);
  //logAbout(img);

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
  cv::resize(img,img,cv::Size(img.cols*2,img.rows*2));

  float mask_arr[]={-1, -1, -1 , -1 , 9 , -1, -1 , -1 ,-1};
  cv::Mat mask=cv::Mat(3,3,CV_32FC1,mask_arr);
  cv::filter2D(img,img,-1,mask);
  img.convertTo(img,CV_32FC1);
  cv::Scalar mu=cv::mean(img);
  double C_00=log(mu.val[0])*sqrt(img.cols*img.rows);

  img=img+1;
  cv::log(img,img);
  cv::dct(img,img);

  //---------------------------------------
  //if(debug_)std::cout<<"C_00 a priori="<<img.at<float>(0,0)<<std::endl;
  //if(debug_)std::cout<<"C_00 a post="<<C_00<<std::endl;
  img.at<float>(0,0)=C_00;
  img.at<float>(0,1)=0;
  //--------------------------------------

  cv::idct(img,img);
  cv::normalize(img,img,0,255,cv::NORM_MINMAX);
  cv::resize(img,img,cv::Size(img.cols/2,img.rows/2));

  img.convertTo(img,CV_8UC1);

  //cv::equalizeHist(img,img);

}

void FaceNormalizer::logAbout(cv::Mat& img)
{
  img.convertTo(img,CV_32FC1);
  float mask_arr[]={-1, -1, -1 , -1 , 9 , -1, -1 , -1 ,-1};
  cv::Mat mask=cv::Mat(3,3,CV_32FC1,mask_arr);
  cv::filter2D(img,img,-1,mask);
  img=img+1;
  cv::log(img,img);
  cv::convertScaleAbs(img,img);
  cv::normalize(img,img,0,255,cv::NORM_MINMAX);
  img.convertTo(img,CV_8UC1);
}




bool FaceNormalizer::normalize_geometry_depth(cv::Mat& img,cv::Mat& depth)
{	


  // detect features
  if(!features_from_color(img))
  {
    //std::cout<<" NO COLOR FEATS"<<std::endl;
    return false;
  }
   if(debug_)dump_features(img);


   //ident_face();
   dyn_norm_face();

   if(!features_from_depth(depth)) return false;




   Eigen::Vector3f x_new,y_new,z_new,lefteye;

   lefteye<<f_det_xyz_.lefteye.x,f_det_xyz_.lefteye.y,f_det_xyz_.lefteye.z;
   x_new<<f_det_xyz_.righteye.x-f_det_xyz_.lefteye.x,f_det_xyz_.righteye.y-f_det_xyz_.lefteye.y,f_det_xyz_.righteye.z-f_det_xyz_.lefteye.z;
   y_new<<f_det_xyz_.mouth.x-f_det_xyz_.nose.x,f_det_xyz_.mouth.y-f_det_xyz_.nose.y,f_det_xyz_.mouth.z-f_det_xyz_.nose.z;
   x_new.normalize();
   y_new.normalize();

   //x_new<<1,0,0;
   //y_new<<0,1,0;
   z_new=x_new.cross(y_new);
   //lefteye<<0,0,0;
   std::cout<<"new x \n"<<x_new<<std::endl;
   std::cout<<"new y \n"<<y_new<<std::endl;
   std::cout<<"new z \n"<<z_new<<std::endl;
   Eigen::Affine3f trafo;
   //Eigen::Affine3f trafo=Eigen::Affine3f::Identity();
   pcl::getTransformationFromTwoUnitVectorsAndOrigin(y_new,z_new,lefteye,trafo);
   //trafo.setIdentity();
   

   cv::Vec3f* ptr=depth.ptr<cv::Vec3f>(0,0);
   Eigen::Vector3f pt;

   for(int i=0;i<img.total();i++)
   {
     pt<<(*ptr)[0],(*ptr)[1],(*ptr)[2];
     //pt=trafo.inverse().rotation()*pt;
     pt=trafo*pt;
    (*ptr)[0]=pt[0];
    (*ptr)[1]=pt[1];
    (*ptr)[2]=pt[2];
     ptr++;
   }
     




   int xoffset=round(offset_[0]);
   int yoffset=round(offset_[1]);




  f_det_img_.add_offset  (xoffset,yoffset);
  f_norm_img_.add_offset (xoffset,yoffset);


  std::cout<< xoffset<<" "<<yoffset<<std::endl;

  //calculate difference in PnP
  cv::Vec3f rot_orig, rot_norm;

  

  //if(!kinect.calc_extrinsics(f_det_xyz_.as_vector(),f_det_img_.as_vector(),false))  return false;
  //rot_orig=kinect.trans;

  //if(!kinect.calc_extrinsics(f_det_xyz_.as_vector(),f_norm_img_.as_vector(),false))  return false;
  //rot_norm=kinect.trans;


  
  std::cout<<"relative Position= "<<rot_orig[0]<<" , "<<rot_orig[1]<<" , "<<rot_orig[2]<<" , "<<std::endl;


  cv::Mat res=cv::Mat::zeros(480,640,CV_8UC3);
  cv::Mat dmres=cv::Mat::zeros(480,640,CV_32FC3);
  kinect.sample_pc(depth,img,res,dmres);

  cv::Rect crop(xoffset,yoffset,img.cols,img.rows);
  res(crop).copyTo(img);
  cv::Mat dmcrop;
  dmres(crop).copyTo(depth);

  if(debug_)dump_img(res,"virtual_full");


   return true;
}


bool FaceNormalizer::normalize_geometry(cv::Mat& img,FaceNormalizer::TRAFO model)
{
  return true;

  // detect features
  if(!features_from_color(img))return false;
  if(debug_)dump_features(img);
  dyn_norm_face();


  //calculate transformation
   cv::Mat trafo(2,3,CV_32FC1);
   cv::Mat warped = cv::Mat(img.rows,img.cols,img.type());
  switch (model)
  {
    case FaceNormalizer::AFFINE:
      {
   get_transform_affine(trafo);
   cv::warpAffine(img,warped,trafo,cv::Size(img.cols,img.rows),cv::INTER_LINEAR,cv::BORDER_CONSTANT,cv::Scalar(100,100,100));
   cv::resize(warped,warped,norm_size_);
   break;
      }
    case FaceNormalizer::PERSPECTIVE:
      {
   get_transform_perspective(trafo);
   cv::warpPerspective(img,warped,trafo,norm_size_ );
   break;
      }
 }


  warped.copyTo(img);

  return true;

}

bool FaceNormalizer::features_from_color(cv::Mat& img_color)
{
  if(!detect_feature(img_color,f_det_img_.nose,FACE::NOSE))
  {
    if(debug_)std::cout<<"no nose"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.lefteye,FACE::LEFTEYE))
  {
    if(debug_)std::cout<<"no eye_l"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.righteye,FACE::RIGHTEYE))
  {
    if(debug_)std::cout<<"no eye_r"<<std::endl;
     return false;
  }
  if(!detect_feature(img_color,f_det_img_.mouth,FACE::MOUTH))
  {
    if(debug_)std::cout<<"no mouth"<<std::endl;
     return false;
  }


  if(debug_)
  {
    std::cout<<"detected image features:\n";
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
  f_det_xyz_.nose=      depth.at<cv::Vec3f>(f_det_img_.nose.y,f_det_img_.nose.x)               ;
  f_det_xyz_.mouth=     depth.at<cv::Vec3f>(f_det_img_.mouth.y,f_det_img_.mouth.x)            ;
  f_det_xyz_.lefteye=   depth.at<cv::Vec3f>(f_det_img_.lefteye.y,f_det_img_.lefteye.x)      ;
  f_det_xyz_.righteye=  depth.at<cv::Vec3f>(f_det_img_.righteye.y,f_det_img_.righteye.x)   ;



  if(debug_)
  {
    std::cout<<"Coordinates of features in pointcloud:"<<std::endl;
    std::cout<<"LEFTEYE: "<<f_det_xyz_.lefteye.x<<" "<<f_det_xyz_.lefteye.y<<" "<<f_det_xyz_.lefteye.z<<std::endl;
    std::cout<<"RIGTHEYE: "<<f_det_xyz_.righteye.x<<" "<<f_det_xyz_.righteye.y<<" "<<f_det_xyz_.righteye.z<<std::endl;
    std::cout<<"NOSE: "<<f_det_xyz_.nose.x<<" "<<f_det_xyz_.nose.y<<" "<<f_det_xyz_.nose.z<<std::endl;
    std::cout<<"MOUTH: "<<f_det_xyz_.mouth.x<<" "<<f_det_xyz_.mouth.y<<" "<<f_det_xyz_.mouth.z<<std::endl;
  }
  if(!f_det_xyz_.valid()) return false;


  return true;
}

bool FaceNormalizer::detect_feature(cv::Mat& img,cv::Point2f& coords,FACE::TYPE type)
{

  //  determine scale of search pattern
  double scale=norm_size_.width/160.0;
  

  CvSeq* seq;
  cv::Vec2f offset;

  switch(type)
  {

    case FACE::NOSE:
  {
    offset =cv::Vec2f(0,0);
    IplImage ipl_img=(IplImage)img;
     seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cv::Size(15*scale,15*scale));
     //seq=cvHaarDetectObjects(&ipl_img,nose_cascade_,nose_storage_,1.3,2,CV_HAAR_DO_CANNY_PRUNING,cv::Size(15*scale,15*scale));
     break;
  }


    case FACE::LEFTEYE:
  {
    offset[0]=0;
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,0,f_det_img_.nose.x,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_l_cascade_,eye_l_storage_,1.1,1,0,cvSize(20*scale,10*scale));
     break;

  }

    case FACE::RIGHTEYE:
  {
    offset[0]=((int)f_det_img_.nose.x);
    offset[1]=0;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(f_det_img_.nose.x,0,img.cols-f_det_img_.nose.x-1,f_det_img_.nose.y));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,eye_r_cascade_,eye_r_storage_,1.1,1,0,cvSize(20*scale,10*scale));
     break;

  }

    case FACE::MOUTH:
  {
    offset[0]=0;
    offset[1]=(int)f_det_img_.nose.y;
    cv::Mat sub_img=img.clone();
    sub_img=sub_img(cvRect(0,f_det_img_.nose.y,img.cols,img.rows-f_det_img_.nose.y-1));
    IplImage ipl_img=(IplImage)sub_img;
     seq=cvHaarDetectObjects(&ipl_img,mouth_cascade_,mouth_storage_,1.3,4,CV_HAAR_DO_CANNY_PRUNING,cvSize(30*scale,15*scale));
     break;

  }
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




void FaceNormalizer::get_transform_perspective(cv::Mat& trafo)
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


void FaceNormalizer:: showImg(cv::Mat& img,std::string window_name)
{
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

  return true;
}

bool FaceNormalizer::read_scene(cv::Mat& depth, cv::Mat& color,cv::Vec2f& offset,std::string path)
{
  cv::FileStorage fs(path,FileStorage::READ);
  fs["depth"]>> depth;
  fs["color"]>> color;
  fs["offset_row"]>> offset[1];
  fs["offset_col"]>> offset[0];
  fs.release();
  return true;
}



void FaceNormalizer::processDM(cv::Mat& dm_xyz,cv::Mat& dm)
{
  //reducing to depth ( z - coordinate only)
  if(dm_xyz.channels()==3)
  {
  std::vector<cv::Mat> cls;
  cv::split(dm_xyz,cls);
  dm=cls[2];
  }
  else if (dm_xyz.channels()==1)
  {
    dm=dm_xyz;
  }

  //reduce z values
  float minval=1000.0;
  float mean=0.0;
  int mean_ctr=0;
  for(int r=0;r<dm.rows;r++)
  {
    for(int c=0;c<dm.cols;c++)
    {
      if(dm.at<float>(r,c)!=0)
      {
        mean+=dm.at<float>(r,c);
        mean_ctr ++;
      }
      if(dm.at<float>(r,c) < minval &&dm.at<float>(r,c)!=0)
        minval =dm.at<float>(r,c);
    }

  }
  mean=(mean/mean_ctr )- minval;
  for(int r=0;r<dm.rows;r++)
  {
    for(int c=0;c<dm.cols;c++)
    {
      if(dm.at<float>(r,c)!=0)
        dm.at<float>(r,c) -=minval;

    }

  }


}


bool FaceNormalizer::get_feature_correspondences( cv::Mat& img, cv::Mat& depth,std::vector<cv::Point2f>& img_pts,std::vector<cv::Point3f>& obj_pts)
{

  features_from_color(img);
  features_from_depth(depth);
  img_pts=f_det_img_.as_vector();
  obj_pts=f_det_xyz_.as_vector();

}
